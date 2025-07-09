#include "Timekeeper.hh"
#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>

#ifdef _WIN32
  #include <ws2tcpip.h>  // already included in Actor.hh, but safe here if used directly
#else
  #include <arpa/inet.h>
#endif

TimekeeperActor::TimekeeperActor(int port_)
    : Actor("Timekeeper"), port(port_) {}

void TimekeeperActor::initializeNetwork() {
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::cerr << "[Timekeeper] WSAStartup failed.\n";
        exit(1);
    }
#endif

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == INVALID_SOCKET) {
        std::cerr << "[Timekeeper] Failed to create socket.\n";
        exit(1);
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(server_fd, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        std::cerr << "[Timekeeper] Bind failed.\n";
        exit(1);
    }

    listen(server_fd, 1);
}

void TimekeeperActor::waitForLogger() {
    while (client_fd == INVALID_SOCKET && running) {
        std::cout << "[Timekeeper] Waiting for logger to connect...\n";
        client_fd = accept(server_fd, nullptr, nullptr);
        if (client_fd == INVALID_SOCKET) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    std::cout << "[Timekeeper] Logger connected.\n";

    // Now use this accepted socket for communication
    sock = client_fd;
}

void TimekeeperActor::sendTickLoop() {
    int tick = 0;
    auto nextTick = std::chrono::steady_clock::now();

    while (running) {
        nextTick += std::chrono::milliseconds(25);

        Message tickMsg;
        tickMsg.sender = name;
        tickMsg.content = "tick:" + std::to_string(tick++);
        sendMessage(tickMsg);  // Sends locally and over TCP

        std::this_thread::sleep_until(nextTick);
    }
}

void TimekeeperActor::run() {
    initializeNetwork();
    waitForLogger();

    bool logger_ready = false;

    // Set behavior to listen for logger ready confirmation
    setBehavior([&](const Message& msg) {
        if (msg.content == "logger:ready") {
            std::cout << "[Timekeeper] Logger is ready. Starting tick loop.\n";
            logger_ready = true;
        }
    });

    // Wait for logger to confirm readiness
    while (!logger_ready && running) {
        // Use new base class function to read message from TCP socket and push to mailbox
        Message incoming = readIncomingMessage();
        if (incoming.content.empty()) {
            // Connection closed or error
            running = false;
            break;
        }

        sendMessage(incoming); // Push incoming message to mailbox for local processing
        Message msg = receiveMessage();
        handleMessage(msg);
    }

    if (running) {
        sendTickLoop();
    }

    if (client_fd != INVALID_SOCKET) {
        closesocket(client_fd);
        client_fd = INVALID_SOCKET;
    }
    if (server_fd != INVALID_SOCKET) {
        closesocket(server_fd);
        server_fd = INVALID_SOCKET;
    }
#ifdef _WIN32
    WSACleanup();
#endif
}
