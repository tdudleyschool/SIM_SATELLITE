#include "ModularActor.hh"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>

#ifdef _WIN32
  #include <ws2tcpip.h>
#else
  #include <arpa/inet.h>
#endif

// Constructor: store connection info and initialize actor name
ModularActor::ModularActor(const std::string& timekeeper_ip, int timekeeper_port,
                           const std::vector<std::pair<std::string, int>>& outboundTargets,
                           int listenPort)
    : Actor("modular"), tk_ip(timekeeper_ip), tk_port(timekeeper_port),
      listen_port(listenPort), outboundConnections(outboundTargets) 
{
    // Initialize trigger flags (reset to false) COSTOMIZE
    for (const auto& msg : triggerMessages) {
        triggerFlags[msg] = false;
    }
}

// Function: connects to Timekeeper + any outbound connections
void ModularActor::connectToActors() {
    // Connect to timekeeper
    socket_t tkSock = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in tk_addr{};
    tk_addr.sin_family = AF_INET;
    tk_addr.sin_port = htons(tk_port);

    #ifdef _WIN32
        InetPton(AF_INET, tk_ip.c_str(), &tk_addr.sin_addr);
    #else
        inet_pton(AF_INET, tk_ip.c_str(), &tk_addr.sin_addr);
    #endif

    while (connect(tkSock, (sockaddr*)&tk_addr, sizeof(tk_addr)) == SOCKET_ERROR) {
        std::cerr << "[ModularActor] Waiting to connect to Timekeeper...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    sockets.push_back(tkSock);
    std::cout << "[ModularActor] Connected to Timekeeper.\n";

    // Connect to additional actors
    for (const auto& [ip, port] : outboundConnections) {
        socket_t sock = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);

        #ifdef _WIN32
                InetPton(AF_INET, ip.c_str(), &addr.sin_addr);
        #else
                inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);
        #endif
        
        while (connect(sock, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
            std::cerr << "[ModularActor] Waiting to connect to " << ip << ":" << port << "\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        sockets.push_back(sock);
        std::cout << "[ModularActor] Connected to " << ip << ":" << port << "\n";
    }
}

// Function: accepts incoming actor connections (if listening is enabled)
void ModularActor::listenForActors() {
    if (listen_port <= 0) return;

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(listen_port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(server_fd, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        std::cerr << "[ModularActor] Bind failed on port " << listen_port << "\n";
        return;
    }

    listen(server_fd, 5);
    std::cout << "[ModularActor] Listening for connections on port " << listen_port << "\n";

    std::thread([&]() {
        while (running) {
            socket_t client_fd = accept(server_fd, nullptr, nullptr);
            if (client_fd != INVALID_SOCKET) {
                std::cout << "[ModularActor] Inbound connection accepted.\n";
                sockets.push_back(client_fd);
            }
        }
    }).detach();
}

// Function: sets all message handling behaviors
void ModularActor::configureBehaviors() {
    setBehavior([&](const Message& msg) {
        std::cout << "[ModularActor] Handling message: '" << msg.content << "'\n";

        // Example: Update accumulator on tick message
        if (msg.content.rfind("tick:", 0) == 0) {
            try {
                int64_t val = std::stoll(msg.content.substr(5));
                valueAccumulator += val;
                std::cout << "[ModularActor] New accumulator value: " << valueAccumulator << "\n";
            } catch (...) {
                std::cerr << "[ModularActor] Failed to parse tick value.\n";
            }
        }

        // Example: Fire-and-forget on keyword
        if (msg.content == "report") {
            Message reply;
            reply.sender = name;
            reply.content = "accumulator:" + std::to_string(valueAccumulator);
            sendMessage(reply);
        }

        // Example: Ask and receive (non-yielding)
        if (msg.content == "query") {
            Message ask;
            ask.sender = name;
            ask.content = "get:status";
            sendAsk(ask, [&](const Message& response) {
                std::cout << "[ModularActor] Got async response: " << response.content << "\n";
            });
        }

        // Message-triggered conditional behavior
        if (triggerFlags.find(msg.content) != triggerFlags.end()) {
            triggerFlags[msg.content] = true;
        }

        // Check if all trigger flags are true
        bool allReady = true;
        for (const auto& [key, state] : triggerFlags) {
            if (!state) {
                allReady = false;
                break;
            }
        }

        if (allReady) {
            std::cout << "[ModularActor] All trigger messages received. Performing special behavior.\n";

            // Insert special action here
            Message special;
            special.sender = name;
            special.content = "triggered:action";
            sendMessage(special);

            // Reset flags
            for (auto& [key, state] : triggerFlags) {
                state = false;
            }
        }
    });
}

// Function: initializes all sockets
void ModularActor::initializeNetwork() {
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::cerr << "[ModularActor] WSAStartup failed.\n";
        exit(1);
    }
#endif

    connectToActors();     // Connect to remote actors
    listenForActors();     // Optionally accept connections

    // Notify timekeeper after setup
    Message readyMsg;
    readyMsg.sender = name;
    readyMsg.content = name + ":ready";
    sendMessage(readyMsg);
}

// Main loop: message processing loop
void ModularActor::run() {
    initializeNetwork();
    configureBehaviors();

    while (running) {
        Message incoming = Message{};
        while (running && incoming.content.empty()) {
            incoming = readIncomingMessage();
            if (incoming.content.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            } else {
                std::cout << "[ModularActor] Received raw: '" << incoming.content << "'\n";
            }
        }

        if (!running) break;

        {
            std::lock_guard<std::mutex> lock(mailboxMutex);
            mailbox.push(incoming);
        }
        mailboxCV.notify_one();

        Message msg = receiveMessage();
        if (!msg.content.empty()) {
            handleMessage(msg);
        }
    }

    closeAllSockets();

#ifdef _WIN32
    WSACleanup();
#endif
}
