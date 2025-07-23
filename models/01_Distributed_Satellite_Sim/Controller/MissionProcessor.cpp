#include "MissionProcessor.hh"
#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>
#include <algorithm>
#include <cmath>

#ifdef _WIN32
  #include <ws2tcpip.h>
#else
  #include <arpa/inet.h>
#endif

MissionProcessor::MissionProcessor(const std::string& timekeeper_ip, int timekeeper_port, int listen_port_)
    : Actor("missionprocessor"),
      tk_ip(timekeeper_ip),
      tk_port(timekeeper_port),
      listen_port(listen_port_),
      server_fd(INVALID_SOCKET),
      low_voltage_enabled(false) {}

MissionProcessor::~MissionProcessor() {
    stop();
}

void MissionProcessor::initializeNetwork() {
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::cerr << "[MissionProcessor] WSAStartup failed.\n";
        exit(1);
    }
#endif

    // Connect to Timekeeper
    socket_t tkSock = socket(AF_INET, SOCK_STREAM, 0);
    if (tkSock == INVALID_SOCKET) {
        std::cerr << "[MissionProcessor] Failed to create socket for Timekeeper.\n";
        exit(1);
    }

    sockaddr_in tk_server{};
    tk_server.sin_family = AF_INET;
    tk_server.sin_port = htons(tk_port);

#ifdef _WIN32
    if (InetPton(AF_INET, tk_ip.c_str(), &tk_server.sin_addr) != 1) {
        std::cerr << "[MissionProcessor] Invalid Timekeeper IP.\n";
        closesocket(tkSock);
        exit(1);
    }
#else
    if (inet_pton(AF_INET, tk_ip.c_str(), &tk_server.sin_addr) <= 0) {
        std::cerr << "[MissionProcessor] Invalid Timekeeper IP.\n";
        closesocket(tkSock);
        exit(1);
    }
#endif

    while (connect(tkSock, (sockaddr*)&tk_server, sizeof(tk_server)) == SOCKET_ERROR) {
        std::cerr << "[MissionProcessor] Waiting to connect to Timekeeper...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    sockets.push_back(tkSock);

    std::cout << "[MissionProcessor] Connected to Timekeeper.\n";

    // Listen for downstream actor connections (e.g., EPS)
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == INVALID_SOCKET) {
        std::cerr << "[MissionProcessor] Failed to create listen socket.\n";
        exit(1);
    }

    sockaddr_in listen_addr{};
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_port = htons(listen_port);
    listen_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(server_fd, (sockaddr*)&listen_addr, sizeof(listen_addr)) == SOCKET_ERROR) {
        std::cerr << "[MissionProcessor] Bind failed on listen port.\n";
        exit(1);
    }

    if (listen(server_fd, 5) == SOCKET_ERROR) {
        std::cerr << "[MissionProcessor] Listen failed on listen port.\n";
        exit(1);
    }

    std::cout << "[MissionProcessor] Listening for downstream actor connections on port " << listen_port << "\n";

    //ridged body connects to mission processor no need to send it this message


    // Start accepting downstream connections on a separate thread
    std::thread acceptThread(&MissionProcessor::acceptDownstreamConnections, this);
    acceptThread.detach();

    configureBehaviors();
}

void MissionProcessor::sendReadyToTimekeeper() {
    Message ready;
    ready.sender = getName();
    ready.type = "ready";
    ready.fields["receiver"] = "timekeeper";
    sendMessage(ready);
    std::cout << "[" << getName() << "] Sent ready message to Timekeeper\n";
}

void MissionProcessor::acceptDownstreamConnections() {
    while (running) {
        sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);
        socket_t new_sock = accept(server_fd, (sockaddr*)&client_addr, &addr_len);
        if (new_sock == INVALID_SOCKET) {
            std::cerr << "[MissionProcessor] Accept failed or stopped.\n";
            break;
        }

        // Read initial 'ready' message from the connected actor
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

        if (msg.type == "ready") {
            std::string actorName = msg.sender;
            std::transform(actorName.begin(), actorName.end(), actorName.begin(), ::tolower);

            {
                std::lock_guard<std::mutex> lock(downstreamMutex);
                downstreamConnections[actorName] = new_sock;
                sockets.push_back(new_sock);
            }

            std::cout << "[MissionProcessor] Connected downstream actor '" << actorName << "'\n";
        } else if (msg.type == "init_message") {
            // Handle initialization message from RidgedBodyModule
            if (msg.fields.find("body_pos") != msg.fields.end()) {
                std::string posStr = msg.fields.at("body_pos");
                std::stringstream ss(posStr);
                std::string val;
                int i = 0;
                while (getline(ss, val, ',') && i < 3) {
                    pos[i] = std::stod(val);
                    i++;
                }
            }

            if (msg.fields.find("body_vel") != msg.fields.end()) {
                std::string velStr = msg.fields.at("body_vel");
                std::stringstream ss(velStr);
                std::string val;
                int i = 0;
                while (getline(ss, val, ',') && i < 3) {
                    vel[i] = std::stod(val);
                    i++;
                }
            }

            if (msg.fields.find("body_acc") != msg.fields.end()) {
                std::string accStr = msg.fields.at("body_acc");
                std::stringstream ss(accStr);
                std::string val;
                int i = 0;
                while (getline(ss, val, ',') && i < 3) {
                    acc[i] = std::stod(val);
                    i++;
                }
            }

            if (msg.fields.find("rotation_matrix") != msg.fields.end()) {
                std::string matStr = msg.fields.at("rotation_matrix");
                std::stringstream ss(matStr);
                std::string val;
                int i = 0;
                while (getline(ss, val, ',') && i < 9) {
                    R_matrix[i / 3][i % 3] = std::stod(val);  // Fill row-major
                    i++;
                }
            } else {
                std::cout << "[MissionProcessor] Unexpected Message from: " << msg.sender << "\n";
                //closesocket(new_sock);
                //continue;
            }

            // Set controller mode based on initialization
            controller_mode = true;
            input_mode = false;

            std::cout << "[" << name << "] Received initialization message from RidgedBodyModule.\n";
            std::cout << "[" << name << "] Sending ready message... \n";

            
            //place this elsewhere
            sendReadyToTimekeeper();
        }
    }
}

void MissionProcessor::configureBehaviors() {
    setBehavior([this](const Message& msg) {
        if (msg.type == "tick") {
            std::cout << "[" << name << "] Received tick, sending load messages.\n";

            // different operatioons depending on mode
            double dt = stod(msg.fields.at("dt"));

            if(input_mode) {
                // read input from user

                //for now just input for one thruster and for one wheel
                double input_thruster_voltage = 300.0; // Placeholder for user input
                double input_thruster_mass_flow = 0.45; // Placeholder for user input
                double input_wheel_voltage[3] = { 5.0, 5.0, 5.0 }; // Placeholder for user input
                
                for(int i = 0; i < 7; ++i) {
                    input_prop_V[i] = input_thruster_voltage;
                    input_mass_flow[i] = input_thruster_mass_flow;
                }

                input_acs_V[0] = input_wheel_voltage[0];
            }
            else if (controller_mode) {

                //Thrust Controller
                //Intended Input But Static For Now: Desired Acceleration
                double desired_acc_magnitude = 3.0; // Placeholder for desired acceleration

                // PID controller for thrusters (acceleration magnitude control)
                static double thruster_prev_error = 0.0;
                static double thruster_integral = 0.0;
                double thruster_Kp = 200.0; // Tune as needed
                double thruster_Ki = 5.0;
                double thruster_Kd = 20.0;

                // Calculate current acceleration magnitude
                double current_acc_magnitude = std::sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
                double thruster_error = desired_acc_magnitude - current_acc_magnitude;
                thruster_integral += thruster_error * dt;
                double thruster_derivative = (thruster_error - thruster_prev_error) / dt;

                // PID output for voltage (simple proportional mapping)
                double thruster_voltage = thruster_Kp * thruster_error + thruster_Ki * thruster_integral + thruster_Kd * thruster_derivative;
                thruster_prev_error = thruster_error;

                // Clamp voltage to valid range
                if (thruster_voltage < 0.0) thruster_voltage = 0.0;
                if (thruster_voltage > 600.0) thruster_voltage = 600.0;

                // Mass flow proportional to voltage (simple model)
                double thruster_mass_flow = 0.45 * (thruster_voltage / 600.0);

                // Update input_prop_V and input_mass_flow arrays for 7 thrusters
                for (int i = 0; i < 7; ++i) {
                    input_prop_V[i] = thruster_voltage;
                    input_mass_flow[i] = thruster_mass_flow;
                }


                // Wheel Controller
                // Intended Input But Static For Now: Desired Yaw, Pitch, Roll
                double desired_yaw = 0.1;   // radians, placeholder
                double desired_pitch = 0.2; // radians, placeholder
                double desired_roll = 0.3;  // radians, placeholder

                // ZYX Euler to rotation matrix
                double cy = cos(desired_yaw);
                double sy = sin(desired_yaw);
                double cp = cos(desired_pitch);
                double sp = sin(desired_pitch);
                double cr = cos(desired_roll);
                double sr = sin(desired_roll);

                double desired_R[3][3];
                desired_R[0][0] = cy * cp;
                desired_R[0][1] = cy * sp * sr - sy * cr;
                desired_R[0][2] = cy * sp * cr + sy * sr;
                desired_R[1][0] = sy * cp;
                desired_R[1][1] = sy * sp * sr + cy * cr;
                desired_R[1][2] = sy * sp * cr - cy * sr;
                desired_R[2][0] = -sp;
                desired_R[2][1] = cp * sr;
                desired_R[2][2] = cp * cr;

                // Compute R_error = R_matrix^T * desired_R
                double R_error[3][3] = {0};
                for (int i = 0; i < 3; ++i)
                    for (int j = 0; j < 3; ++j)
                        for (int k = 0; k < 3; ++k)
                            R_error[i][j] += R_matrix[k][i] * desired_R[k][j]; // Transpose(R_matrix) * desired_R

                // Skew-symmetric part (approximate rotation error vector)
                double error[3];
                error[0] = 0.5 * (R_error[2][1] - R_error[1][2]); // error around i_hat (motor[0])
                error[1] = 0.5 * (R_error[0][2] - R_error[2][0]); // error around j_hat (motor[1])
                error[2] = 0.5 * (R_error[1][0] - R_error[0][1]); // error around k_hat (motor[2])

                // PID controller parameters (placeholder)
                static double prev_error[3] = {0.0, 0.0, 0.0};
                static double integral[3] = {0.0, 0.0, 0.0};
                double Kp = 10.0, Ki = 0.1, Kd = 1.0;
                double dt = 1.0; // Assume 1s tick for now

                for (int i = 0; i < 3; ++i) {
                    integral[i] += error[i] * dt;
                    double derivative = (error[i] - prev_error[i]) / dt;
                    input_acs_V[i] = Kp * error[i] + Ki * integral[i] + Kd * derivative;
                    prev_error[i] = error[i];
                }

            }

            //========================
            //Prepare THRUSTER message
            //========================
            Message thruster_msg;
            thruster_msg.sender = name;
            thruster_msg.type = "turn_load_on";
            thruster_msg.fields["load_type"] = "THRUSTER";
            thruster_msg.fields["num_of_loads"] = "7";

            // Build comma-separated voltage and mass flow strings from input_prop_V and input_mass_flow arrays
            std::ostringstream voltageStream, massFlowStream;
            for (int i = 0; i < 7; ++i) {
                voltageStream << input_prop_V[i];
                if (i < 6) voltageStream << ",";
            }
            for (int i = 0; i < 7; ++i) {
                massFlowStream << input_mass_flow[i];
                if (i < 6) massFlowStream << ",";
            }

            thruster_msg.fields["input_voltage"] = voltageStream.str();
            thruster_msg.fields["input_mass_flow"] = massFlowStream.str();
            thruster_msg.fields["active_nodes"] = "1,1,1,1,1,1,1";

            sendMessage(thruster_msg);

            //========================
            //Prepare ACS message           
            //========================

            Message acs_msg;
            acs_msg.sender = name;
            acs_msg.type = "turn_load_on";
            acs_msg.fields["load_type"] = "ACS";
            acs_msg.fields["num_of_loads"] = "3";

            // Build comma-separated voltage string from input_acs_voltage array
            std::ostringstream acsVoltageStream;
            for (int i = 0; i < 3; ++i) {
                acsVoltageStream << input_acs_V[i];
                if (i < 2) acsVoltageStream << ",";
            }
            acs_msg.fields["input_voltage"] = acsVoltageStream.str();
            acs_msg.fields["active_nodes"] = "1,1,1";
            sendMessage(acs_msg);

            // Send LOW_BUS only once
            if (!low_voltage_enabled) {
                Message lv_msg;
                lv_msg.sender = name;
                lv_msg.type = "turn_load_on";
                lv_msg.fields["load_type"] = "LOW_BUS";
                lv_msg.fields["num_of_loads"] = "3";
                lv_msg.fields["active_nodes"] = "1,1,1";
                sendMessage(lv_msg);

                low_voltage_enabled = true;
                std::cout << "[" << name << "] Sent LOW_BUS turn-on message \n";
            }

            std::cout << "[" << name << "] Sent THRUSTER and ACS turn-on messages to all downstream actors.\n";
        } 
        if (msg.type == "init_message") {
            // Handle initialization message from RidgedBodyModule
            if (msg.fields.find("body_pos") != msg.fields.end()) {
                std::string posStr = msg.fields.at("body_pos");
                std::stringstream ss(posStr);
                std::string val;
                int i = 0;
                while (getline(ss, val, ',') && i < 3) {
                    pos[i] = std::stod(val);
                    i++;
                }
            }

            if (msg.fields.find("body_vel") != msg.fields.end()) {
                std::string velStr = msg.fields.at("body_vel");
                std::stringstream ss(velStr);
                std::string val;
                int i = 0;
                while (getline(ss, val, ',') && i < 3) {
                    vel[i] = std::stod(val);
                    i++;
                }
            }

            if (msg.fields.find("body_acc") != msg.fields.end()) {
                std::string accStr = msg.fields.at("body_acc");
                std::stringstream ss(accStr);
                std::string val;
                int i = 0;
                while (getline(ss, val, ',') && i < 3) {
                    acc[i] = std::stod(val);
                    i++;
                }
            }

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

            // Set controller mode based on initialization
            controller_mode = true;
            input_mode = false;

            std::cout << "[" << name << "] Received initialization message from RidgedBodyModule.\n";
            std::cout << "[" << name << "] Sending ready message... \n";

            
            //place this elsewhere
            sendReadyToTimekeeper();
        }

        if (msg.type == "RidgedBody:Update") {
            if (msg.fields.find("position") != msg.fields.end()) {
                string arrayStr = msg.fields.at("position");
                stringstream ss(arrayStr);
                string val;
                int i = 0;
                while (getline(ss, val, ',') && i < 3) {
                    pos[i] = stod(val);
                    i++;
                }
            }
            if (msg.fields.find("velocity") != msg.fields.end()) {
                string arrayStr = msg.fields.at("velocity");
                stringstream ss(arrayStr);
                string val;
                int i = 0;
                while (getline(ss, val, ',') && i < 3) {
                    vel[i] = stod(val);
                    i++;
                }
            }
            if (msg.fields.find("acceleration") != msg.fields.end()) {
                string arrayStr = msg.fields.at("acceleration");
                stringstream ss(arrayStr);
                string val;
                int i = 0;
                while (getline(ss, val, ',') && i < 3) {
                    acc[i] = stod(val);
                    i++;
                }
            }
            if (msg.fields.find("angular_velocity") != msg.fields.end()) {
                string arrayStr = msg.fields.at("angular_velocity");
                stringstream ss(arrayStr);
                string val;
                int i = 0;
                while (getline(ss, val, ',') && i < 3) {
                    // Assuming angular velocity is stored in vel for simplicity
                    vel[i] = stod(val);
                    i++;
                }
            }
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
            std::cout << "[" << name << "] Received state update from RidgedBodyModule.\n";
        }
    });
}



void MissionProcessor::run() {
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
