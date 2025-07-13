#include "SummerActor.hh"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>

#ifdef _WIN32
  #include <ws2tcpip.h>
#else
  #include <arpa/inet.h>
#endif

SummerActor::SummerActor(const std::string& timekeeper_ip, int timekeeper_port,
                         const std::string& output_ip_, int output_port_)
    : Actor("summer"), tk_ip(timekeeper_ip), tk_port(timekeeper_port),
      output_ip(output_ip_), output_port(output_port_), output_sock(INVALID_SOCKET) {}

void SummerActor::initializeNetwork() {
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::cerr << "[Summer] WSAStartup failed.\n";
        exit(1);
    }
#endif

    // Connect to Timekeeper
    socket_t tkSock = socket(AF_INET, SOCK_STREAM, 0);
    if (tkSock == INVALID_SOCKET) {
        std::cerr << "[Summer] Failed to create socket for Timekeeper.\n";
        exit(1);
    }

    sockaddr_in tk_server{};
    tk_server.sin_family = AF_INET;
    tk_server.sin_port = htons(tk_port);

#ifdef _WIN32
    if (InetPton(AF_INET, tk_ip.c_str(), &tk_server.sin_addr) != 1) {
        std::cerr << "[Summer] Invalid Timekeeper IP.\n";
        closesocket(tkSock);
        exit(1);
    }
#else
    if (inet_pton(AF_INET, tk_ip.c_str(), &tk_server.sin_addr) <= 0) {
        std::cerr << "[Summer] Invalid Timekeeper IP.\n";
        closesocket(tkSock);
        exit(1);
    }
#endif

    while (connect(tkSock, (sockaddr*)&tk_server, sizeof(tk_server)) == SOCKET_ERROR) {
        std::cerr << "[Summer] Waiting to connect to Timekeeper...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    sockets.push_back(tkSock);

    std::cout << "[Summer] Connected to Timekeeper.\n";

    // Connect to Output
    connectToOutput();
}

void SummerActor::connectToOutput() {
    socket_t outputSock = socket(AF_INET, SOCK_STREAM, 0);
    if (outputSock == INVALID_SOCKET) {
        std::cerr << "[Summer] Failed to create socket for Output.\n";
        exit(1);
    }

    sockaddr_in output_server{};
    output_server.sin_family = AF_INET;
    output_server.sin_port = htons(output_port);

#ifdef _WIN32
    if (InetPton(AF_INET, output_ip.c_str(), &output_server.sin_addr) != 1) {
        std::cerr << "[Summer] Invalid Output IP.\n";
        closesocket(outputSock);
        exit(1);
    }
#else
    if (inet_pton(AF_INET, output_ip.c_str(), &output_server.sin_addr) <= 0) {
        std::cerr << "[Summer] Invalid Output IP.\n";
        closesocket(outputSock);
        exit(1);
    }
#endif

    while (connect(outputSock, (sockaddr*)&output_server, sizeof(output_server)) == SOCKET_ERROR) {
        std::cerr << "[Summer] Waiting to connect to Output...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    sockets.push_back(outputSock);

    std::cout << "[Summer] Connected to Output.\n";

    // Send ready message to Timekeeper (over first socket)
    Message readyMsg;
    readyMsg.sender = name;
    readyMsg.content = "summer:ready";
    sendMessage(readyMsg);
}

void SummerActor::run() {
    initializeNetwork();

    setBehavior([&](const Message& msg) {
        std::cout << "[Summer] Handling message: '" << msg.content << "'\n";  // Debug print on handle
    
        if (msg.content.rfind("tick:", 0) == 0) {  // starts with "tick:"
            try {
                std::string numberPart = msg.content.substr(5); // remove "tick:"
                int64_t val = std::stoll(numberPart);
                sum += val;
                std::cout << "[Summer] Updated sum: " << sum << "\n";
    
                // Optionally send sum to OutputActor
                Message outMsg;
                outMsg.sender = name;
                outMsg.content = std::to_string(sum);
                sendMessage(outMsg);
            } catch (const std::exception& e) {
                std::cout << "[Summer] Failed to parse number from tick message: " << e.what() << "\n";
            }
        } else {
            std::cout << "[Summer] Ignoring non-tick message: '" << msg.content << "'\n";
        }
    });
    

    while (running) {
        Message incoming = Message{};
        while (running && incoming.content.empty()) {
            incoming = readIncomingMessage();
            if (incoming.content.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            } else {
                std::cout << "[Summer] Received raw message: '" << incoming.content << "'\n";  // Debug raw incoming
            }
        }

        if (!running) break;

        if (incoming.content.empty()) {
            std::cerr << "[Summer] Connection closed or error.\n";
            running = false;
            break;
        }

        {
            std::lock_guard<std::mutex> lock(mailboxMutex);
            mailbox.push(incoming);
        }
        mailboxCV.notify_one();

        Message msg = receiveMessage();
        if (!msg.content.empty()) {
            handleMessage(msg);
        } else {
            std::cerr << "[Summer] Received empty message after popping from mailbox!\n";
        }
    }

    closeAllSockets();

#ifdef _WIN32
    WSACleanup();
#endif
}
