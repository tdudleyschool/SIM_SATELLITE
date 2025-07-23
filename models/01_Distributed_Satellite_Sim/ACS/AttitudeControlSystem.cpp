#include "AttitudeControlSystem.hh"
#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>
#include <algorithm>

#include "../../Attitude_Control/src/Attitude_Control_System.cpp"
#include "../../Attitude_Control/src/control_wheels.cpp"
#include "../../Attitude_Control/src/motor.cpp"


#ifdef _WIN32
  #include <ws2tcpip.h>
#else
  #include <arpa/inet.h>
#endif

double integrate(double prev, double rate, double delta) {
    return prev + rate * delta;
}


AttitudeControlSystem::AttitudeControlSystem(const std::string& tk_ip_, int tk_port_,
                                             const std::string& eps_ip_, int eps_port_,
                                             const std::string& rb_ip_, int rb_port_,
                                             int listen_port_)
    : Actor("attitudecontrolsystem"),
      tk_ip(tk_ip_), tk_port(tk_port_),
      eps_ip(eps_ip_), eps_port(eps_port_),
      rb_ip(rb_ip_), rb_port(rb_port_),
      listen_port(listen_port_), server_fd(INVALID_SOCKET) 
{
//=======================================//
//====ATTITUDE CONTROL VARIABLE SETUP====//
//=======================================//

//Update The ACS Based On Several State Variables
    for(int i = 0; i < 3; i++) {
        I[i] = 0.0;
        w[i] = 0.0;
        dI[i] = 0.0;
        dw[i] = 0.0;

        //initialize internalstate input
        input_V[i] = 0.0;
    }
}

void AttitudeControlSystem::initializeNetwork() {
    #ifdef _WIN32
        WSADATA wsa;
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
            std::cerr << "[AttitudeControlSystem] WSAStartup failed.\n";
            exit(1);
        }
    #endif

    // Connect to Timekeeper
    socket_t tkSock = socket(AF_INET, SOCK_STREAM, 0);
    if (tkSock == INVALID_SOCKET) {
        std::cerr << "[AttitudeControlSystem] Failed to create socket for Timekeeper.\n";
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
        std::cerr << "[AttitudeControlSystem] Waiting to connect to Timekeeper...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    sockets.push_back(tkSock);
    std::cout << "[AttitudeControlSystem] Connected to Timekeeper.\n";

    // Connect to EPS
    socket_t epsSock = socket(AF_INET, SOCK_STREAM, 0);
    if (epsSock == INVALID_SOCKET) {
        std::cerr << "[AttitudeControlSystem] Failed to create socket for EPS.\n";
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
        std::cerr << "[AttitudeControlSystem] Waiting to connect to EPS...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    sockets.push_back(epsSock);
    std::cout << "[AttitudeControlSystem] Connected to EPS.\n";

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

    // Send ready messages to all
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
        std::cerr << "[AttitudeControlSystem] Failed to bind/listen on port " << listen_port << "\n";
        exit(1);
    }

    std::cout << "[AttitudeControlSystem] Listening on port " << listen_port << "\n";

    // Accept connections in separate thread
    std::thread(&AttitudeControlSystem::acceptConnections, this).detach();

    configureBehaviors();
}

void AttitudeControlSystem::acceptConnections() {
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
        std::cout << "[AttitudeControlSystem] Incoming message: " << msg.type << " from " << msg.sender << "\n";
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

            std::cout << "[" << name << "] Received initialization message from RidgedBodyModule.\n";
            std::cout << "[" << name << "] Sending ready message... \n";
            // Send ready message to Timekeeper
            Message readyMsg;
            readyMsg.sender = name;
            readyMsg.type = "ready";
            sendMessage(readyMsg);
        }

    }
}

void AttitudeControlSystem::configureBehaviors() {
    setBehavior([this](const Message& msg) {
        std::cout << "[AttitudeControlSystem] Message from " << msg.sender << ": " << msg.type << "\n";

        try {
            //If tick then update motor based off of internal state
            if (msg.type == "tick") {
                double dt = stod(msg.fields.at("dt"));

                //Process and output torque
                for(int i = 0; i < 3; i++) {
                    ACS.motor_clock(i, input_V[i]);
                }

                ACS.update_all_ori(R_matrix); //or position ori but this suffices
                ACS.updateAll_I(I_prev);
                ACS.updateAll_omega(w_prev);

                ACS.state_deriv_getALL_dI(dI);
                ACS.state_deriv_getALL_Alpha(dw);

                for (int i = 0; i < 3; i++) {
                    I[i] = integrate(I_prev[i], dI[i], dt);
                    w[i] = integrate(w_prev[i], dw[i], dt);
                }

                double torque[3];
                ACS.get_total_torque(torque);

                //Send Torque To Force Torque Tracker
                Message torque_message;
                torque_message.sender = name;
                torque_message.type = "add_torque";
                // Serialize to comma-separated string aka converting array into csv format
                string torqe_msg;
                for (int i = 0; i < 3; ++i) {
                    torqe_msg += to_string(torque[i]);
                    if (i < 2) torqe_msg += ","; // Don't add comma after last
                }

                //Sending voltage as a field
                std::cout << "Torque Sent=" << torqe_msg << "\n";
                torque_message.fields["input_torque"] = torqe_msg;

                sendMessage(torque_message);
            }
            //from eps, ACS:input manipulate internal state
            if (msg.type == "ACS:input") {
                //update the internal input state just being voltage
            if (msg.fields.find("input_voltage") != msg.fields.end()) {
                string arrayStr = msg.fields.at("input_voltage");
                stringstream ss(arrayStr);
                string val;
                int i = 0;
                while (getline(ss, val, ',') && i < 3) {
                    input_V[i] = stod(val);
                    i++;
                }
            }

                std::cout << "[AttitudeControlSystem] Voltages: " << input_V[0] << "[0], " << input_V[1] << "[1], " << input_V[2] << "[2] \n";
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

                std::cout << "[" << name << "] Received initialization message from RidgedBodyModule.\n";
                std::cout << "[" << name << "] Sending ready message... \n";
                // Send ready message to Timekeeper
                Message readyMsg;
                readyMsg.sender = name;
                readyMsg.type = "ready";
                sendMessage(readyMsg);
            }

        } catch (const std::exception& e) {
            std::cerr << "[AttitudeControlSystem] Exception in behavior: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[AttitudeControlSystem] Unknown exception in behavior.\n";
        }
    });
}

void AttitudeControlSystem::run() {
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
