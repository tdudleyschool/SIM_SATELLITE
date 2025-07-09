#include "Logger.hh"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>

#ifdef _WIN32
  #include <ws2tcpip.h>  // already included in Actor.hh, but safe here if used directly
#else
  #include <arpa/inet.h>
#endif

LoggerActor::LoggerActor(const std::string& ip_, int port_)
    : Actor("Logger"), ip(ip_), port(port_) {}

void LoggerActor::initializeNetwork() {
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::cerr << "[Logger] WSAStartup failed.\n";
        exit(1);
    }
#endif

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == INVALID_SOCKET) {
        std::cerr << "[Logger] Failed to create socket.\n";
        exit(1);
    }

    sockaddr_in server{};
    server.sin_family = AF_INET;
    server.sin_port = htons(port);

#ifdef _WIN32
    if (InetPton(AF_INET, ip.c_str(), &server.sin_addr) != 1) {
        std::cerr << "[Logger] Invalid IP address.\n";
        closesocket(sock);
        exit(1);
    }
#else
    if (inet_pton(AF_INET, ip.c_str(), &server.sin_addr) <= 0) {
        std::cerr << "[Logger] Invalid IP address.\n";
        closesocket(sock);
        exit(1);
    }
#endif

    // Try connecting to Timekeeper until successful
    while (connect(sock, (sockaddr*)&server, sizeof(server)) == SOCKET_ERROR) {
        std::cerr << "[Logger] Waiting to connect to Timekeeper...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "[Logger] Connected to Timekeeper.\n";

    // Immediately notify Timekeeper that Logger is ready
    Message readyMsg;
    readyMsg.sender = name;
    readyMsg.content = "logger:ready";
    sendMessage(readyMsg);
}

void LoggerActor::run() {
    initializeNetwork();

    setBehavior([&](const Message& msg) {
        if (msg.content.rfind("tick:", 0) == 0) {
            std::cout << "[Logger] Received: " << msg.content << "\n";
        }
    });

    while (running) {
        Message incoming = readIncomingMessage();
        if (incoming.content.empty()) {
            std::cerr << "[Logger] Connection closed or error.\n";
            running = false;
            break;
        }

        sendMessage(incoming);  // push to mailbox
        Message msg = receiveMessage();
        handleMessage(msg);
    }

    if (sock != INVALID_SOCKET) {
        closesocket(sock);
        sock = INVALID_SOCKET;
    }
#ifdef _WIN32
    WSACleanup();
#endif
}
