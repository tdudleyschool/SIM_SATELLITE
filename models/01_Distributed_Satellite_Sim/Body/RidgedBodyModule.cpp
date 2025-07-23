#include "RidgedBodyModule.hh"
#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>
#include <algorithm>

#include "../../Ridged_Body/src/Satellite_Box.cpp"
#include "../../Ridged_Body/src/Ridged_Body.cpp"

#ifdef _WIN32
  #include <ws2tcpip.h>
#else
  #include <arpa/inet.h>
#endif

void integrate(double& val, double integ_val, double dt) {
    double interval = integ_val * dt;
    val = val + interval;
}

RidgedBodyModule::RidgedBodyModule(const std::string& timekeeper_ip_, int timekeeper_port_,
                                   const std::string& mission_ip_, int mission_port_,
                                   int listen_port_)
    : Actor("RidgedBodyModule"),
      mission_ip(mission_ip_), mission_port(mission_port_),
      timekeeper_ip(timekeeper_ip_), timekeeper_port(timekeeper_port_),
      listen_port(listen_port_), server_fd(INVALID_SOCKET) 
{
    for(int i = 0; i < 3; i++) {
        sat_x[i] = 0.0;
        sat_v[i] = 0.0;
        sat_a[i] = 0.0;
        sat_w[i] = 0.0;
    }
    Sattelite_Body.get_ref_k_vec(sat_ori);
    Sattelite_Body.initialize_body(1.0, 0.5);
    Sattelite_Body.initialize_motion(sat_x, sat_v, sat_a);
    Sattelite_Body.get_R(R_matrix);
    Sattelite_Body.get_w(sat_w);
    Sattelite_Body.get_Qori(sat_q);
}


void RidgedBodyModule::initializeNetwork() {
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::cerr << "[RidgedBodyModule] WSAStartup failed.\n";
        exit(1);
    }
#endif

    // Connect to MissionProcessor
    socket_t missionSock = socket(AF_INET, SOCK_STREAM, 0);
    if (missionSock == INVALID_SOCKET) {
        std::cerr << "[RidgedBodyModule] Failed to create socket for MissionProcessor.\n";
        exit(1);
    }
    sockaddr_in mission_server{};
    mission_server.sin_family = AF_INET;
    mission_server.sin_port = htons(mission_port);
#ifdef _WIN32
    InetPton(AF_INET, mission_ip.c_str(), &mission_server.sin_addr);
#else
    inet_pton(AF_INET, mission_ip.c_str(), &mission_server.sin_addr);
#endif
    while (connect(missionSock, (sockaddr*)&mission_server, sizeof(mission_server)) == SOCKET_ERROR) {
        std::cerr << "[RidgedBodyModule] Waiting to connect to MissionProcessor...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    sockets.push_back(missionSock);
    std::cout << "[RidgedBodyModule] Connected to MissionProcessor.\n";
    //since we connect to mission processor first, we can assume that it is ready
    MISSIONPROCESSOR_ready = true;

    // Connect to Timekeeper
    socket_t timekeeperSock = socket(AF_INET, SOCK_STREAM, 0);
    if (timekeeperSock == INVALID_SOCKET) {
        std::cerr << "[RidgedBodyModule] Failed to create socket for Timekeeper.\n";
        exit(1);
    }
    sockaddr_in timekeeper_server{};
    timekeeper_server.sin_family = AF_INET;
    timekeeper_server.sin_port = htons(timekeeper_port);
#ifdef _WIN32
    InetPton(AF_INET, timekeeper_ip.c_str(), &timekeeper_server.sin_addr);
#else
    inet_pton(AF_INET, timekeeper_ip.c_str(), &timekeeper_server.sin_addr);
#endif
    while (connect(timekeeperSock, (sockaddr*)&timekeeper_server, sizeof(timekeeper_server)) == SOCKET_ERROR) {
        std::cerr << "[RidgedBodyModule] Waiting to connect to Timekeeper...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    sockets.push_back(timekeeperSock);
    std::cout << "[RidgedBodyModule] Connected to Timekeeper.\n";

    // Setup listener for incoming connections
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in listen_addr{};
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_port = htons(listen_port);
    listen_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(server_fd, (sockaddr*)&listen_addr, sizeof(listen_addr)) == SOCKET_ERROR ||
        listen(server_fd, 5) == SOCKET_ERROR) {
        std::cerr << "[RidgedBodyModule] Failed to bind/listen on port " << listen_port << "\n";
        exit(1);
    }

    std::cout << "[RidgedBodyModule] Listening on port " << listen_port << "\n";
    std::thread(&RidgedBodyModule::acceptConnections, this).detach();
    configureBehaviors();
}

void RidgedBodyModule::acceptConnections() {
    std::thread acceptThread([this]() {
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
                    new_sock = INVALID_SOCKET;
                    break;
                }
                if (ch == '\n') break;
                buffer.push_back(ch);
            }

            if (buffer.empty() || new_sock == INVALID_SOCKET) continue;

            Message msg = deserializeMessage(buffer);
            std::cout << "[RidgedBodyModule] Incoming message: " << msg.type << " from " << msg.sender << "\n";

            if (msg.type == "ready" && msg.sender == "forcetorquetracker") {
                sockets.push_back(new_sock);
                std::cout << "[RidgedBodyModule] Registered connection from " << msg.sender << "\n";
                FFT_ready = true;
                Message readyMsg;
                if (MISSIONPROCESSOR_ready && ACS_ready && PROPULSION_ready) {
                    std::cout << "[RidgedBodyModule] All systems ready. Sending initialization message.\n";
                    sendInitializationMessage();
                } else {
                    std::cout << "[RidgedBodyModule] Waiting for all systems to be ready...\n";
                }
                std::cout << "[RidgedBodyModule] Sending ready \n";
                readyMsg.sender = name;
                readyMsg.type = "ready";
                sendMessage(readyMsg);
            } else if (msg.type == "ready_for_initialization") {
                std::cout << "[RidgedBodyModule] Received ready for initialization from " << msg.sender << "\n";
                if (msg.sender == "attitudecontrolsystem") {
                    ACS_ready = true;
                }
                else if (msg.sender == "PropulsionSystem") {
                    PROPULSION_ready = true;
                }
                else {
                    std::cout << "[RidgedBodyModule] Unexpected ready message from: " << msg.sender << ".\n";   
                }
                
                if (FFT_ready && MISSIONPROCESSOR_ready && ACS_ready && PROPULSION_ready) {
                    std::cout << "[RidgedBodyModule] All systems ready. Sending initialization message.\n";
                    sendInitializationMessage();
                } else {
                    std::cout << "[RidgedBodyModule] Waiting for all systems to be ready...\n";
                }
            } else {
                std::cout << "[RidgedBodyModule] Unexpected message from: " << msg.sender << "\n";
                //closesocket(new_sock);
                //continue;
            }

        }
    });
    acceptThread.detach();
}

void RidgedBodyModule::sendInitializationMessage() {
    std::cout << "[RidgedBodyModule] All systems ready. Preparing initialization message.\n";
    //send initialization conditions
    Message initializeMessage;
    initializeMessage.sender = name;
    initializeMessage.type = "init_message";

    // Serialize to comma-separated string aka converting array into csv format
    string pos_msg;
    string vel_msg;
    string accel_msg;
    string ori_msg;//pointing forward
    string w_msg;
    for (int i = 0; i < 3; ++i) {
        pos_msg += to_string(sat_x[i]);
        vel_msg += to_string(sat_v[i]);
        accel_msg += to_string(sat_a[i]);
        ori_msg += to_string(sat_x[i]);
        w_msg += to_string(sat_x[i]);
        if (i < 2) {
            pos_msg += ",";
            vel_msg += ",";
            accel_msg += ",";
            ori_msg += ",";
            w_msg += ",";
        }
    }

    //Sending voltage as a field
    initializeMessage.fields["body_pos"] = pos_msg;
    initializeMessage.fields["body_vel"] = vel_msg;
    initializeMessage.fields["body_acc"] = accel_msg;
    initializeMessage.fields["body_ori"] = ori_msg;
    initializeMessage.fields["body_omega"] = w_msg;

    //setting rotation matrix as field
    // Convert rotation matrix to flat string to flat string
    std::stringstream ss;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            ss << R_matrix[i][j];
            if (!(i == 2 && j == 2)) ss << ","; // Avoid trailing comma
        }
    }

    initializeMessage.fields["rotation_matrix"] = ss.str();

    std::cout << "[RidgedBodyModule] Sending initialization message to all connected modules.\n";
    // Send the initialization message to all connected sockets

    sendMessage(initializeMessage);
}

void RidgedBodyModule::configureBehaviors() {
    setBehavior([this](const Message& msg) {
        std::cout << "[RidgedBodyModule] Message from " << msg.sender << ": " << msg.type << "\n";
        if(msg.type == "tick") {
            double dt = stod(msg.fields.at("dt"));

            Sattelite_Body.update_force(sat_Force[0], sat_Force[1], sat_Force[2]);
            Sattelite_Body.update_torque(sat_Torque[0], sat_Torque[1], sat_Torque[2]);

            //state derivative for translational motion
            Sattelite_Body.state_deriv_getAccel(sat_a);
            for(int i = 0; i < 3; i++) {
                integrate(sat_x[i], sat_v[i], dt);
                integrate(sat_v[i], sat_a[i], dt);
            }
            Sattelite_Body.update_v(sat_v[0], sat_v[1], sat_v[2]);
            Sattelite_Body.update_pos(sat_x[0], sat_x[1], sat_x[2]);
            
            //state derivatives for rotational motion
            Sattelite_Body.get_w(sat_w);
            Sattelite_Body.get_Qori(sat_q);

            Sattelite_Body.state_deriv_get_alpha(sat_dw);
            for (int i = 0; i < 3; i++) {
                integrate(sat_w[i], sat_dw[i], dt);
            }
            Sattelite_Body.update_w(sat_w[0], sat_w[1], sat_w[2]);
            Sattelite_Body.state_deriv_getQori(sat_dq);

            for(int i = 0; i < 4; i++) {
                integrate(sat_q[i], sat_dq[i], 0.5*dt); //0.5*dt for quaterniod
            }

            Sattelite_Body.update_Qori(sat_q);
            Sattelite_Body.get_R(R_matrix);

            //Output Everything Positional Wise
            std::cout << "[RidgedBodyModule] Position: (" << sat_x[0] << ", " << sat_x[1] << ", " << sat_x[2] << ") "
                      << "Velocity: (" << sat_v[0] << ", " << sat_v[1] << ", " << sat_v[2] << ") "
                      << "Acceleration: (" << sat_a[0] << ", " << sat_a[1] << ", " << sat_a[2] << ") "
                      << "Angular Velocity: (" << sat_w[0] << ", " << sat_w[1] << ", " << sat_w[2] << ")\n";

            Message stateMsg;
            stateMsg.sender = name;
            stateMsg.type = "RidgedBody:Update";

            // Serialize position, velocity, acceleration, angular velocity
            std::string pos_str, vel_str, accel_str, omega_str;
            for (int i = 0; i < 3; ++i) {
                pos_str += std::to_string(sat_x[i]);
                vel_str += std::to_string(sat_v[i]);
                accel_str += std::to_string(sat_a[i]);
                omega_str += std::to_string(sat_w[i]);
                if (i < 2) {
                    pos_str += ",";
                    vel_str += ",";
                    accel_str += ",";
                    omega_str += ",";
                }
            }

            // Serialize rotation matrix
            std::stringstream rot_ss;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    rot_ss << R_matrix[i][j];
                    if (!(i == 2 && j == 2)) rot_ss << ",";
                }
            }

            stateMsg.fields["position"] = pos_str;
            stateMsg.fields["velocity"] = vel_str;
            stateMsg.fields["acceleration"] = accel_str;
            stateMsg.fields["angular_velocity"] = omega_str;
            stateMsg.fields["rotation_matrix"] = rot_ss.str();

            sendMessage(stateMsg);
        }
    
        if(msg.type == "update_force_torque") {
            
            if (msg.fields.find("net_force") != msg.fields.end()) {
                string arrayStr = msg.fields.at("net_force");
                stringstream ss(arrayStr);
                string val;
                int i = 0;
                while (getline(ss, val, ',') && i < 3) {
                    sat_Force[i] = stod(val);
                    i++;
                }
            }

            if (msg.fields.find("net_torque") != msg.fields.end()) {
                string arrayStr = msg.fields.at("net_torque");
                stringstream ss(arrayStr);
                string val;
                int i = 0;
                while (getline(ss, val, ',') && i < 3) {
                    sat_Torque[i] = stod(val);
                    i++;
                }
            }
        }
    });
}


void RidgedBodyModule::run() {
    initializeNetwork();

    while (running) {
    Message incoming = readIncomingMessage();
    if (incoming.type.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
    }

    std::cout << "[" << name << "] Received message of type: " << incoming.type << "\n";
    handleMessage(incoming);  // Process it directly
}


    closeAllSockets();

#ifdef _WIN32
    WSACleanup();
#endif
}
