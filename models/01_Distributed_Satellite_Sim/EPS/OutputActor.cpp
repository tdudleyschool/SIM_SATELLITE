#include "OutputActor.hh"
#include <iostream>
#include <thread>
#include <chrono>

#ifdef _WIN32
  #include <ws2tcpip.h>
#else
  #include <arpa/inet.h>
#endif

OutputActor::OutputActor(int port_)
    : Actor("output"), port(port_), server_fd(INVALID_SOCKET), client_fd(INVALID_SOCKET) {}

void OutputActor::initializeNetwork() {
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::cerr << "[Output] WSAStartup failed.\n";
        exit(1);
    }
#endif

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == INVALID_SOCKET) {
        std::cerr << "[Output] Failed to create socket.\n";
        exit(1);
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(server_fd, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        std::cerr << "[Output] Bind failed.\n";
        exit(1);
    }

    listen(server_fd, 1);
}

void OutputActor::run() {
    initializeNetwork();

    std::cout << "[Output] Waiting for Summer to connect...\n";

    client_fd = accept(server_fd, nullptr, nullptr);
    if (client_fd == INVALID_SOCKET) {
        std::cerr << "[Output] Accept failed.\n";
        running = false;
        return;
    }
    sockets.push_back(client_fd);

    std::cout << "[Output] Summer connected.\n";

    setBehavior([&](const Message& msg) {
        if (msg.type == "sum") {
            std::cout << "[Output] Received sum: " << msg.fields.at("value") << "\n";
        } else {
            std::cout << "[Output] Received message type: " << msg.type << "\n";
        }
    });

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


