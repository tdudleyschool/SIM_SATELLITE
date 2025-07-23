#include "ForceTorqueTracker.hh"
#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>
#include <algorithm>

#include "../../Recources/src/force_torque_tracker.cpp"
#include "../../Recources/src/functions.cpp"

#ifdef _WIN32
  #include <ws2tcpip.h>
#else
  #include <arpa/inet.h>
#endif

using namespace std;

ForceTorqueTracker::ForceTorqueTracker(const std::string& timekeeper_ip_, int timekeeper_port_,
                                       const std::string& att_ip_, int att_port_,
                                       const std::string& prop_ip_, int prop_port_,
                                       const std::string& ridgedbody_ip_, int ridgedbody_port_,
                                       int listen_port_)
    : Actor("forcetorquetracker"),
      timekeeper_ip(timekeeper_ip_), timekeeper_port(timekeeper_port_),
      att_ip(att_ip_), att_port(att_port_),
      prop_ip(prop_ip_), prop_port(prop_port_),
      ridgedbody_ip_(ridgedbody_ip_), ridgedbody_port_(ridgedbody_port_),
      listen_port(listen_port_), server_fd(INVALID_SOCKET) {}


void ForceTorqueTracker::initializeNetwork() {
    #ifdef _WIN32
        WSADATA wsa;
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
            std::cerr << "[ForceTorqueTracker] WSAStartup failed.\n";
            exit(1);
        }
    #endif

    // Connect to AttitudeControlSystem
    socket_t attSock = socket(AF_INET, SOCK_STREAM, 0);
    if (attSock == INVALID_SOCKET) {
        std::cerr << "[ForceTorqueTracker] Failed to create socket for AttitudeControlSystem.\n";
        exit(1);
    }
    sockaddr_in att_server{};
    att_server.sin_family = AF_INET;
    att_server.sin_port = htons(att_port);

    #ifdef _WIN32
        InetPton(AF_INET, att_ip.c_str(), &att_server.sin_addr);
    #else
        inet_pton(AF_INET, att_ip.c_str(), &att_server.sin_addr);
    #endif

    while (connect(attSock, (sockaddr*)&att_server, sizeof(att_server)) == SOCKET_ERROR) {
        std::cerr << "[ForceTorqueTracker] Waiting to connect to AttitudeControlSystem...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    sockets.push_back(attSock);
    std::cout << "[ForceTorqueTracker] Connected to AttitudeControlSystem.\n";

    // Connect to PropulsionSystem
    socket_t propSock = socket(AF_INET, SOCK_STREAM, 0);
    if (propSock == INVALID_SOCKET) {
        std::cerr << "[ForceTorqueTracker] Failed to create socket for PropulsionSystem.\n";
        exit(1);
    }
    sockaddr_in prop_server{};
    prop_server.sin_family = AF_INET;
    prop_server.sin_port = htons(prop_port);

    #ifdef _WIN32
        InetPton(AF_INET, prop_ip.c_str(), &prop_server.sin_addr);
    #else
        inet_pton(AF_INET, prop_ip.c_str(), &prop_server.sin_addr);
    #endif

    while (connect(propSock, (sockaddr*)&prop_server, sizeof(prop_server)) == SOCKET_ERROR) {
        std::cerr << "[ForceTorqueTracker] Waiting to connect to PropulsionSystem...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    sockets.push_back(propSock);
    std::cout << "[ForceTorqueTracker] Connected to PropulsionSystem.\n";

    // Connect to RidgedBodyModule
    socket_t rbSock = socket(AF_INET, SOCK_STREAM, 0);
    if (rbSock == INVALID_SOCKET) {
        std::cerr << "[ForceTorqueTracker] Failed to create socket for RidgedBody.\n";
        exit(1);
    }
    sockaddr_in rb_server{};
    rb_server.sin_family = AF_INET;
    rb_server.sin_port = htons(ridgedbody_port_);

    #ifdef _WIN32
    InetPton(AF_INET, ridgedbody_ip_.c_str(), &rb_server.sin_addr);
    #else
    inet_pton(AF_INET, ridgedbody_ip_.c_str(), &rb_server.sin_addr);
    #endif

    while (connect(rbSock, (sockaddr*)&rb_server, sizeof(rb_server)) == SOCKET_ERROR) {
        std::cerr << "[ForceTorqueTracker] Waiting to connect to RidgedBodyModule...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    sockets.push_back(rbSock);
    std::cout << "[ForceTorqueTracker] Connected to RidgedBodyModule.\n";

    // Connect to Timekeeper
    socket_t timeSock = socket(AF_INET, SOCK_STREAM, 0);
    if (timeSock == INVALID_SOCKET) {
        std::cerr << "[ForceTorqueTracker] Failed to create socket for Timekeeper.\n";
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

    while (connect(timeSock, (sockaddr*)&timekeeper_server, sizeof(timekeeper_server)) == SOCKET_ERROR) {
        std::cerr << "[ForceTorqueTracker] Waiting to connect to Timekeeper...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    sockets.push_back(timeSock);
    std::cout << "[ForceTorqueTracker] Connected to Timekeeper.\n";


    // Send ready
    Message readyMsg;
    readyMsg.sender = name;
    readyMsg.type = "ready";
    sendMessage(readyMsg);


    // Setup listener for incoming connections
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in listen_addr{};
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_port = htons(listen_port);
    listen_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(server_fd, (sockaddr*)&listen_addr, sizeof(listen_addr)) == SOCKET_ERROR ||
        listen(server_fd, 5) == SOCKET_ERROR) {
        std::cerr << "[ForceTorqueTracker] Failed to bind/listen on port " << listen_port << "\n";
        exit(1);
    }

    std::cout << "[ForceTorqueTracker] Listening on port " << listen_port << "\n";

    std::thread(&ForceTorqueTracker::acceptConnections, this).detach();

    configureBehaviors();
}

void ForceTorqueTracker::acceptConnections() {
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
        std::cout << "[ForceTorqueTracker] Incoming message: " << msg.type << " from " << msg.sender << "\n";
        sockets.push_back(new_sock);
    }
}

void ForceTorqueTracker::configureBehaviors() {
    setBehavior([this](const Message& msg) {
        std::cout << "[ForceTorqueTracker] Message from " << msg.sender << ": " << msg.type << "\n";

        try {
            //If tick then update motor based off of internal state
            if (msg.type == "tick") {
                //either i could have this send each time both torque and force are updated or by ticks. here ill do by ticks
                //only thing tick dose is send message here. in other verisons perhabs tick may not be needed

                                //Send Torque To Force Torque Tracker
                Message force_torque_message;
                force_torque_message.sender = name;
                force_torque_message.type = "update_force_torque";
                // Serialize to comma-separated string aka converting array into csv format
                string force_msg;
                string torque_msg;
                for (int i = 0; i < 3; ++i) {
                    force_msg += to_string(curr_force[i]);
                    torque_msg += to_string(curr_torque[i]);
                    if (i < 2) {
                        force_msg += ","; // Don't add comma after last
                        torque_msg += ","; // Don't add comma after last
                    }
                }

                //Sending voltage as a field
                std::cout << "force Sent=" << force_msg << "\n";
                std::cout << "torque Sent=" << torque_msg << "\n";
                force_torque_message.fields["net_force"] = force_msg;
                force_torque_message.fields["net_torque"] = torque_msg;

                sendMessage(force_torque_message);
            }
            if (msg.type == "add_torque") {

                if(!updated_torque) {
                    double input_torque[3];

                    if (msg.fields.find("input_torque") != msg.fields.end()) {
                        string arrayStr = msg.fields.at("input_torque");
                        stringstream ss(arrayStr);
                        string val;
                        int i = 0;
                        while (getline(ss, val, ',') && i < 3) {
                            input_torque[i] = stod(val);
                            i++;
                        }
                    }

                    Satellite_Forces.addTorque(input_torque);

                    updated_torque = true;
                }

                
            }

            if (msg.type == "add_force_at") {
                //locks making so you can't update one without the other
                if(!updated_force) {
                    double input_force[3];
                    double input_pos[3];

                    if (msg.fields.find("thruster_net_force") != msg.fields.end()) {
                        string arrayStr = msg.fields.at("thruster_net_force");
                        stringstream ss(arrayStr);
                        string val;
                        int i = 0;
                        while (getline(ss, val, ',') && i < 3) {
                            input_force[i] = stod(val);
                            i++;
                        }
                    }

                    if (msg.fields.find("thruster_net_position") != msg.fields.end()) {
                        string arrayStr = msg.fields.at("thruster_net_position");
                        stringstream ss(arrayStr);
                        string val;
                        int i = 0;
                        while (getline(ss, val, ',') && i < 3) {
                            input_pos[i] = stod(val);
                            i++;
                        }
                    }

                    //input this into tracker
                    Satellite_Forces.addForce(input_force, input_pos);

                    updated_force = true;
                }
            }
            if (updated_torque && updated_force) {
                //we use gards to update the torques and forces all at once
                //Note: Id like to see how completly asynchronous one basically one that just upates force and torque separatly runs

                //either i could have this send each time both torque and force are updated or by ticks. here ill do by ticks
                Satellite_Forces.getNetForce(curr_force);
                Satellite_Forces.getNetTorque(curr_torque);
                Satellite_Forces.resetValues();
                updated_force = false;
                updated_torque = false;
            }

        } catch (const std::exception& e) {
            std::cerr << "[ForceTorqueTracker] Exception in behavior: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[ForceTorqueTracker] Unknown exception in behavior.\n";
        }
    });
}

void ForceTorqueTracker::run() {
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
