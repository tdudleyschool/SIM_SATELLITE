#include "MissionProcessor.hh"
#include <iostream>
#include <thread>
#include <chrono>

#ifdef _WIN32
  #include <ws2tcpip.h>
#else
  #include <arpa/inet.h>
#endif

MissionProcessor::MissionProcessor(const std::string& timekeeper_ip, int timekeeper_port, int listen_port_)
    : Actor("MissionProcessor"),
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
    InetPton(AF_INET, tk_ip.c_str(), &tk_server.sin_addr);
#else
    inet_pton(AF_INET, tk_ip.c_str(), &tk_server.sin_addr);
#endif

    while (connect(tkSock, (sockaddr*)&tk_server, sizeof(tk_server)) == SOCKET_ERROR) {
        std::cerr << "[MissionProcessor] Waiting to connect to Timekeeper...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    sockets.push_back(tkSock);

    std::cout << "[MissionProcessor] Connected to Timekeeper.\n";

    // Send ready message
    Message readyMsg;
    readyMsg.sender = name;
    readyMsg.type = "ready";
    sendMessage(readyMsg);
    std::cout << "[MissionProcessor] Sent ready message to Timekeeper.\n";

    // Start listening for downstream actor connections (e.g., EPS)
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in listen_addr{};
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_port = htons(listen_port);
    listen_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(server_fd, (sockaddr*)&listen_addr, sizeof(listen_addr)) == SOCKET_ERROR ||
        listen(server_fd, 5) == SOCKET_ERROR) {
        std::cerr << "[MissionProcessor] Failed to bind/listen on port " << listen_port << "\n";
        exit(1);
    }

    std::cout << "[MissionProcessor] Listening for downstream actor connections on port " << listen_port << "\n";
    std::thread(&MissionProcessor::acceptDownstreamConnections, this).detach();
}

void MissionProcessor::acceptDownstreamConnections() {
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

        if (buffer.empty()) continue;

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
        } else {
            closesocket(new_sock);
        }
    }
}

void MissionProcessor::configureBehaviors() {
    setBehavior([this](const Message& msg) {
        if (msg.type != "tick") return;

        std::cout << "[MissionProcessor] Received tick, sending load-on commands...\n";

        Message thruster_msg;
        thruster_msg.sender = name;
        thruster_msg.type = "turn_load_on";
        thruster_msg.fields = {
            {"load_type", "THRUSTER"},
            {"num_of_loads", "7"},
            {"input_voltage", "600.0,600.0,600.0,600.0,600.0,600.0,600.0"},
            {"input_mass_flow", "0.45"},
            {"active_nodes", "1,1,1,1,1,1,1"}
        };

        Message acs_msg;
        acs_msg.sender = name;
        acs_msg.type = "turn_load_on";
        acs_msg.fields = {
            {"load_type", "ACS"},
            {"num_of_loads", "3"},
            {"input_voltage", "10.0,10.0,10.0"},
            {"active_nodes", "1,1,1"}
        };

        Message lv_msg;
        if (!low_voltage_enabled) {
            lv_msg.sender = name;
            lv_msg.type = "turn_load_on";
            lv_msg.fields = {
                {"load_type", "LOW_BUS"},
                {"num_of_loads", "3"},
                {"active_nodes", "1,1,1"}
            };
            low_voltage_enabled = true;
        }

        std::lock_guard<std::mutex> lock(downstreamMutex);
        for (const auto& [actor, sock] : downstreamConnections) {
            sendMessage(actor, thruster_msg);
            sendMessage(actor, acs_msg);
            if (!lv_msg.type.empty()) {
                sendMessage(actor, lv_msg);
            }
        }

        std::cout << "[MissionProcessor] Load-on messages sent to downstream actors.\n";
    });
}

void MissionProcessor::run() {
    initializeNetwork();
    configureBehaviors();

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

