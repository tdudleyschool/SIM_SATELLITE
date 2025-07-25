#include "Timekeeper.hh"
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>

#ifdef _WIN32
  #include <ws2tcpip.h>
#else
  #include <arpa/inet.h>
#endif

TimekeeperActor::TimekeeperActor(int port_, const std::vector<std::string>& expectedActors)
    : Actor("timekeeper"), port(port_), server_fd(INVALID_SOCKET)
{
    for (const auto& actorName : expectedActors) {
        readyMap[actorName] = false;
    }
}

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

    listen(server_fd, 5);
}

void TimekeeperActor::acceptConnections() {
    std::cout << "[Timekeeper] Waiting for connections...\n";

    while (running && std::count_if(readyMap.begin(), readyMap.end(),
                                   [](auto& p) { return !p.second; }) > 0) {
        sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);
        socket_t new_sock = accept(server_fd, (sockaddr*)&client_addr, &addr_len);
        if (new_sock == INVALID_SOCKET) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        std::cout << "[Timekeeper] New connection accepted.\n";
        sockets.push_back(new_sock);

        // Wait for ready message from this actor
        bool identified = false;
        while (running && !identified) {
            Message msg;
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
            if (buffer.empty() || new_sock == INVALID_SOCKET) break;

            msg = deserializeMessage(buffer);

                        // ...existing code...
            if (msg.type == "ready") {
                std::string actorName = msg.sender;
                std::transform(actorName.begin(), actorName.end(), actorName.begin(), ::tolower);
            
                if (readyMap.find(actorName) != readyMap.end()) {
                    readyMap[actorName] = true;
                    identified = true;
                    std::cout << "[Timekeeper] Actor '" << actorName << "' is ready.\n";
                } else {
                    std::cerr << "[Timekeeper] Unknown actor '" << actorName << "' connected, closing socket.\n";
                    closesocket(new_sock);
                    sockets.pop_back();
                    new_sock = INVALID_SOCKET;
                    break;
                }
            } else {
                std::cout << "[Timekeeper] Received '" << msg.type << "' from '" << msg.sender
                          << "'. Waiting for 'ready' message to proceed.\n";
                //std::this_thread::sleep_for(std::chrono::milliseconds(100));
                //closesocket(new_sock);
                //sockets.pop_back();
                //new_sock = INVALID_SOCKET;
                //break;
            }
            // ...existing code...
        }
    }
}

bool TimekeeperActor::waitForAllReady() {
    while (running) {
        for (auto s : sockets) {
            if (s == INVALID_SOCKET) {
                std::cerr << "[Timekeeper] A connected socket disconnected. Stopping.\n";
                running = false;
                return false;
            }
        }

        bool allReady = std::all_of(readyMap.begin(), readyMap.end(),
                                    [](auto& p) { return p.second; });
        if (allReady) return true;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

void TimekeeperActor::sendTickLoop() {
    int tick = 0;
    auto nextTick = std::chrono::steady_clock::now();

    while (running) {
        nextTick += std::chrono::milliseconds(25);

        Message tickMsg;
        tickMsg.sender = name;
        tickMsg.type = "tick";
        tickMsg.fields["count"] = std::to_string(tick++);
        tickMsg.fields["dt"] = "0.025";  // 25 milliseconds in seconds

        sendMessage(tickMsg);

        std::cout << "[Timekeeper] Sent tick #" << tickMsg.fields["count"] << "\n";

        std::this_thread::sleep_until(nextTick);
    }
}

void TimekeeperActor::run() {
    initializeNetwork();

    acceptConnections();

    if (!waitForAllReady()) {
        std::cerr << "[Timekeeper] Not all actors became ready, stopping.\n";
        stop();
        return;
    }

    std::cout << "[Timekeeper] All actors are ready. Starting tick loop.\n";

    setBehavior([&](const Message& msg) {
        std::cout << "[Timekeeper] Received message from " << msg.sender << ": type=" << msg.type << "\n";
    });

    sendTickLoop();

    closeAllSockets();

#ifdef _WIN32
    WSACleanup();
#endif
}
