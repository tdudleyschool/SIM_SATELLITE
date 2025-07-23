#include "PropulsionSystem.hh"
#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>
#include <algorithm>

#include "../../Propulsion/src/Propulsion_System_PIC2D.cpp"
#include "../../Propulsion/src/hall_thruster_PIC2D.cpp"

#ifdef _WIN32
  #include <ws2tcpip.h>
#else
  #include <arpa/inet.h>
#endif

PropulsionSystem::PropulsionSystem(const std::string& tk_ip_, int tk_port_,
                                   const std::string& eps_ip_, int eps_port_,
                                   const std::string& rb_ip_, int rb_port_,
                                   int listen_port_)
    : Actor("PropulsionSystem"),
      tk_ip(tk_ip_), tk_port(tk_port_),
      eps_ip(eps_ip_), eps_port(eps_port_),
      rb_ip(rb_ip_), rb_port(rb_port_),
      listen_port(listen_port_), server_fd(INVALID_SOCKET) 
      
{
    propulsion = Propulsion_System_PIC2D(2.0, 2.0, 2.0, 0.5, 1.0);
    ref_ori[0] = 0;
    ref_ori[1] = 0;
    ref_ori[2] = -1;
    //setting up reference orientation
    propulsion.set_all_thruster_ref_ori(ref_ori);
    propulsion.initialize_all_HET_sim();
    propulsion.turn_all_on();

    for(int i = 0; i < 7; i++) {
        input_V[i] = 0.0;
        input_mass_flow[i] = 0.0;
    }
}

void PropulsionSystem::initializeNetwork() {
    #ifdef _WIN32
        WSADATA wsa;
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
            std::cerr << "[PropulsionSystem] WSAStartup failed.\n";
            exit(1);
        }
    #endif

    // Connect to Timekeeper
    socket_t tkSock = socket(AF_INET, SOCK_STREAM, 0);
    if (tkSock == INVALID_SOCKET) {
        std::cerr << "[PropulsionSystem] Failed to create socket for Timekeeper.\n";
        exit(1);
    }
    sockaddr_in tk_server{};
    tk_server.sin_family = AF_INET;
    tk_server.sin_port = htons(tk_port);

    #ifdef _WIN32
        InetPton(AF_INET, tk_ip.c_str(), &tk_server.sin_addr);
    #else
        inet_pton(AF_INET, tk_ip.c_str(), &tk_server.sin_addr);
    #endif

    while (connect(tkSock, (sockaddr*)&tk_server, sizeof(tk_server)) == SOCKET_ERROR) {
        std::cerr << "[PropulsionSystem] Waiting to connect to Timekeeper...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    sockets.push_back(tkSock);
    std::cout << "[PropulsionSystem] Connected to Timekeeper.\n";

    // Connect to EPS
    socket_t epsSock = socket(AF_INET, SOCK_STREAM, 0);
    if (epsSock == INVALID_SOCKET) {
        std::cerr << "[PropulsionSystem] Failed to create socket for EPS.\n";
        exit(1);
    }
    sockaddr_in eps_server{};
    eps_server.sin_family = AF_INET;
    eps_server.sin_port = htons(eps_port);

    #ifdef _WIN32
        InetPton(AF_INET, eps_ip.c_str(), &eps_server.sin_addr);
    #else
        inet_pton(AF_INET, eps_ip.c_str(), &eps_server.sin_addr);
    #endif

    while (connect(epsSock, (sockaddr*)&eps_server, sizeof(eps_server)) == SOCKET_ERROR) {
        std::cerr << "[PropulsionSystem] Waiting to connect to EPS...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    sockets.push_back(epsSock);
    std::cout << "[PropulsionSystem] Connected to EPS.\n";

    // Connect to RidgedBodyModule
    socket_t rbSock = socket(AF_INET, SOCK_STREAM, 0);
    if (rbSock == INVALID_SOCKET) {
        std::cerr << "[" << name << "] Failed to create socket for RidgedBodyModule.\n";
        exit(1);
    }

    sockaddr_in rb_server{};
    rb_server.sin_family = AF_INET;
    rb_server.sin_port = htons(rb_port);
    
    #ifdef _WIN32
    InetPton(AF_INET, rb_ip.c_str(), &rb_server.sin_addr);
    #else
    inet_pton(AF_INET, rb_ip.c_str(), &rb_server.sin_addr);
    #endif

    while (connect(rbSock, (sockaddr*)&rb_server, sizeof(rb_server)) == SOCKET_ERROR) {
        std::cerr << "[" << name << "] Waiting to connect to RidgedBodyModule...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    sockets.push_back(rbSock);
    std::cout << "[" << name << "] Connected to RidgedBodyModule.\n";


    // Send ready messages to both
    Message readyMsg;
    readyMsg.sender = name;
    readyMsg.type = "ready_for_initialization";
    sendMessage(readyMsg);

    // Setup listener for incoming connections
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in listen_addr{};
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_port = htons(listen_port);
    listen_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(server_fd, (sockaddr*)&listen_addr, sizeof(listen_addr)) == SOCKET_ERROR ||
        listen(server_fd, 5) == SOCKET_ERROR) {
        std::cerr << "[PropulsionSystem] Failed to bind/listen on port " << listen_port << "\n";
        exit(1);
    }

    std::cout << "[PropulsionSystem] Listening on port " << listen_port << "\n";

    // Accept connections in separate thread
    std::thread(&PropulsionSystem::acceptConnections, this).detach();

    configureBehaviors();
}

void PropulsionSystem::acceptConnections() {
    while (running) {
        sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);
        socket_t new_sock = accept(server_fd, (sockaddr*)&client_addr, &addr_len);
        if (new_sock == INVALID_SOCKET) continue;

        std::string buffer;
        char ch;
        while (true) {
            int ret = recv(new_sock, &ch, 1, 0);
            if (ret <= 0) {
                closesocket(new_sock);
                break;
            }
            if (ch == '\n') break;
            buffer.push_back(ch);
        }

        if (buffer.empty()) {
            closesocket(new_sock);
            continue;
        }

        Message msg = deserializeMessage(buffer);
        std::cout << "[PropulsionSystem] Incoming message: " << msg.type << " from " << msg.sender << "\n";
        sockets.push_back(new_sock);

        if (msg.type == "init_message") {
            // Initialize rotation matrix
            if (msg.fields.find("rotation_matrix") != msg.fields.end()) {
                std::string matStr = msg.fields.at("rotation_matrix");
                std::stringstream ss(matStr);
                std::string val;
                int i = 0;

                while (getline(ss, val, ',') && i < 9) {
                    R_matrix[i / 3][i % 3] = std::stod(val);  // Fill row-major
                    i++;
                }
            }
            // Initialize satellite position
            if (msg.fields.find("sat_position") != msg.fields.end()) {
                string arrayStr = msg.fields.at("sat_position");
                stringstream ss(arrayStr);
                string val;
                int i = 0;
                while (getline(ss, val, ',') && i < 3) {
                    sat_pos[i] = stod(val);
                    i++;
                }
            }
            std::cout << "[" << name << "] Received initialization message from RidgedBodyModule.\n";
            std::cout << "[" << name << "] Sending ready message... \n";
            Message readyMsg;
            readyMsg.sender = name;
            readyMsg.type = "ready";
            sendMessage(readyMsg);

        }
    }
}

void PropulsionSystem::configureBehaviors() {
    setBehavior([this](const Message& msg) {
        std::cout << "[PropulsionSystem] Message from " << msg.sender << ": " << msg.type << "\n";

        try {
            //If tick then update motor based off of internal state
            if (msg.type == "tick") {
                double dt = stod(msg.fields.at("dt"));

                //Process and output 
                for (int i = 0; i < 7; i++){
                    propulsion.run_step_HET_sim(i, input_mass_flow[i], input_V[i]);
                }

                // === APPLY THRUSTER FORCE ===
                double net_force[3], force_pos[3];
                propulsion.get_all_force(net_force, force_pos);
                propulsion.update_tankmass();

                //Send this netforce at the position to forcetorque tracker
                //Send Torque To Force Torque Tracker
                Message thruster_force_message;
                thruster_force_message.sender = name;
                thruster_force_message.type = "add_force_at";
                // Serialize to comma-separated string aka converting array into csv format
                string net_force_msg;
                string net_f_pos_msg;
                for (int i = 0; i < 3; ++i) {
                    net_force_msg += to_string(net_force[i]);
                    net_f_pos_msg += to_string(force_pos[i]);
                    if (i < 2) {
                        net_force_msg += ","; // Don't add comma after last
                        net_f_pos_msg += ","; // Don't add comma after last
                    }
                }

                //Sending voltage as a field
                std::cout << "Force Sent=" << net_force_msg << "\n";
                std::cout << "At Position=" << net_f_pos_msg << "\n";
                thruster_force_message.fields["thruster_net_force"] = net_force_msg;
                thruster_force_message.fields["thruster_net_position"] = net_f_pos_msg;

                sendMessage(thruster_force_message);

            }
            //from eps, ACS:input manipulate internal state
            if (msg.type == "THRUSTER:input") {
                //update the internal input state that being the voltage and the massflow
                if (msg.fields.find("input_voltage") != msg.fields.end()) {
                    string arrayStr = msg.fields.at("input_voltage");
                    stringstream ss(arrayStr);
                    string val;
                    int i = 0;
                    while (getline(ss, val, ',') && i < 7) {
                        input_V[i] = stod(val);
                        i++;
                    }
                }
                if (msg.fields.find("input_mass_flow") != msg.fields.end()) {
                    string arrayStr = msg.fields.at("input_mass_flow");
                    stringstream ss(arrayStr);
                    string val;
                    int i = 0;
                    while (getline(ss, val, ',') && i < 7) {
                        input_mass_flow[i] = stod(val);
                        i++;
                    }
                }

                std::cout << "[PropulsionSystem] Voltages: " << input_V[0] << "[0], " << input_V[1] << "[1], " << input_V[2] << "[2] \n";
                std::cout << "[PropulsionSystem] Mass Flow: " << input_mass_flow[0] << "[0], " << input_mass_flow[1] << "[1], " << input_mass_flow[2] << "[2] \n";
            }
            //Field is specifically so that position an orientation are alighed with ridged body
            if (msg.type == "RidgedBody:Update" ) {
                //processing message. rotation matrix direcly comes form ridged body
                if (msg.fields.find("rotation_matrix") != msg.fields.end()) {
                    std::string matStr = msg.fields.at("rotation_matrix");
                    std::stringstream ss(matStr);
                    std::string val;
                    int i = 0;

                    while (getline(ss, val, ',') && i < 9) {
                        R_matrix[i / 3][i % 3] = std::stod(val);  // Fill row-major
                        i++;
                    }
                }
                //processing sattelite position
                if (msg.fields.find("sat_position") != msg.fields.end()) {
                    string arrayStr = msg.fields.at("sat_position");
                    stringstream ss(arrayStr);
                    string val;
                    int i = 0;
                    while (getline(ss, val, ',') && i < 3) {
                        sat_pos[i] = stod(val);
                        i++;
                    }
                }
                //update internal state of propulsion
                propulsion.update_all_pos_ori(sat_pos, R_matrix);
            }
            if (msg.type == "init_message") {
                // Initialize rotation matrix
                if (msg.fields.find("rotation_matrix") != msg.fields.end()) {
                    std::string matStr = msg.fields.at("rotation_matrix");
                    std::stringstream ss(matStr);
                    std::string val;
                    int i = 0;

                    while (getline(ss, val, ',') && i < 9) {
                        R_matrix[i / 3][i % 3] = std::stod(val);  // Fill row-major
                        i++;
                    }
                }
                // Initialize satellite position
                if (msg.fields.find("sat_position") != msg.fields.end()) {
                    string arrayStr = msg.fields.at("sat_position");
                    stringstream ss(arrayStr);
                    string val;
                    int i = 0;
                    while (getline(ss, val, ',') && i < 3) {
                        sat_pos[i] = stod(val);
                        i++;
                    }
                }
                std::cout << "[" << name << "] Received initialization message from RidgedBodyModule.\n";
                std::cout << "[" << name << "] Sending ready message... \n";
                Message readyMsg;
                readyMsg.sender = name;
                readyMsg.type = "ready";
                sendMessage(readyMsg);

            }

        } catch (const std::exception& e) {
            std::cerr << "[PropulsionSystem] Exception in behavior: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[PropulsionSystem] Unknown exception in behavior.\n";
        }
    });
}

void PropulsionSystem::run() {
    initializeNetwork();

    while (running) {
        Message incoming = readIncomingMessage();
        if (incoming.type.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        {
            std::lock_guard<std::mutex> lock(mailboxMutex);
            mailbox.push(incoming);
        }
        mailboxCV.notify_one();

        Message msg = receiveMessage();
        if (!msg.type.empty()) {
            handleMessage(msg);
        }
    }

    closeAllSockets();

#ifdef _WIN32
    WSACleanup();
#endif
}
