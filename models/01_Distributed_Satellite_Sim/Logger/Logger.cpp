#include "Logger.hh"
#include <iostream>
#include <thread>
#include <chrono>

#ifdef _WIN32
  #include <ws2tcpip.h>
#else
  #include <arpa/inet.h>
#endif

LoggerActor::LoggerActor(const std::string& ip_, int port_)
    : Actor("logger"), ip(ip_), port(port_) {}

void LoggerActor::initializeNetwork() {
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::cerr << "[Logger] WSAStartup failed.\n";
        exit(1);
    }
#endif

    socket_t loggerSock = socket(AF_INET, SOCK_STREAM, 0);
    if (loggerSock == INVALID_SOCKET) {
        std::cerr << "[Logger] Failed to create socket.\n";
        exit(1);
    }

    sockaddr_in server{};
    server.sin_family = AF_INET;
    server.sin_port = htons(port);

#ifdef _WIN32
    if (InetPton(AF_INET, ip.c_str(), &server.sin_addr) != 1) {
        std::cerr << "[Logger] Invalid IP address.\n";
        closesocket(loggerSock);
        exit(1);
    }
#else
    if (inet_pton(AF_INET, ip.c_str(), &server.sin_addr) <= 0) {
        std::cerr << "[Logger] Invalid IP address.\n";
        closesocket(loggerSock);
        exit(1);
    }
#endif

    while (connect(loggerSock, (sockaddr*)&server, sizeof(server)) == SOCKET_ERROR) {
        std::cerr << "[Logger] Waiting to connect to Timekeeper...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    sockets.push_back(loggerSock);

    std::cout << "[Logger] Connected to Timekeeper.\n";

    // Send ready message using new format
    Message readyMsg;
    readyMsg.sender = name;
    readyMsg.type = "ready";
    sendMessage(readyMsg);
}

void LoggerActor::run() {
    initializeNetwork();

    setBehavior([&](const Message& msg) {
        if (msg.type == "tick") {
            std::cout << "[Logger] Received tick count: " << msg.fields.at("count") << "\n";
        } else {
            std::cout << "[Logger] Received message type: " << msg.type << "\n";
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
