#include "Actor.hh"
#include <iostream>
#include <cstring>

std::atomic<MessageID> Actor::globalMessageID{1};

// STATIC: Serialization
std::string Actor::serializeMessage(const Message& msg) {
    return msg.sender + "|" + std::to_string(msg.id) + "|" +
           std::to_string(msg.replyTo) + "|" + msg.content + "\n";
}

// STATIC: Deserialization
Message Actor::deserializeMessage(const std::string& raw) {
    Message msg;
    size_t pos1 = raw.find('|');
    if (pos1 == std::string::npos) return msg;
    msg.sender = raw.substr(0, pos1);

    size_t pos2 = raw.find('|', pos1 + 1);
    if (pos2 == std::string::npos) return msg;
    msg.id = std::stoull(raw.substr(pos1 + 1, pos2 - pos1 - 1));

    size_t pos3 = raw.find('|', pos2 + 1);
    if (pos3 == std::string::npos) return msg;
    msg.replyTo = std::stoull(raw.substr(pos2 + 1, pos3 - pos2 - 1));

    msg.content = raw.substr(pos3 + 1);
    if (!msg.content.empty() && msg.content.back() == '\n') {
        msg.content.pop_back();
    }
    return msg;
}

Actor::Actor(std::string name_) : name(std::move(name_)), running(false) {}
Actor::~Actor() { stop(); }

void Actor::start() {
    running = true;
    worker = std::thread(&Actor::run, this);
}

void Actor::stop() {
    running = false;
    mailboxCV.notify_all();
    if (worker.joinable())
        worker.join();
    closeAllSockets();
#ifdef _WIN32
    WSACleanup();
#endif
}

void Actor::closeAllSockets() {
    for (auto s : sockets) {
        if (s != INVALID_SOCKET) {
            closesocket(s);
        }
    }
    sockets.clear();
}

void Actor::sendMessage(const Message& msg) {
    {
        std::lock_guard<std::mutex> lock(mailboxMutex);
        mailbox.push(msg);
    }
    mailboxCV.notify_one();

    if (!sockets.empty()) {
        if (!sendNetworkMessage(msg)) {
            std::cerr << "[" << name << "] Failed to send network message.\n";
        }
    }
}

void Actor::sendAsk(const Message& msg, std::function<void(const Message&)> callback) {
    Message out = msg;
    out.id = generateMessageID();
    {
        std::lock_guard<std::mutex> lock(mailboxMutex);
        pendingResponses[out.id] = std::move(callback);
        mailbox.push(out);
    }
    mailboxCV.notify_one();

    if (!sockets.empty()) {
        if (!sendNetworkMessage(out)) {
            std::cerr << "[" << name << "] Failed to send ask message over network.\n";
        }
    }
}

Message Actor::receiveMessage() {
    std::unique_lock<std::mutex> lock(mailboxMutex);
    mailboxCV.wait(lock, [&] { return !mailbox.empty() || !running; });

    if (!mailbox.empty()) {
        Message msg = mailbox.front();
        mailbox.pop();
        return msg;
    }
    return Message{};
}

void Actor::handleMessage(const Message& msg) {
    if (msg.replyTo != 0) {
        std::function<void(const Message&)> cb;
        {
            std::lock_guard<std::mutex> lock(mailboxMutex);
            auto it = pendingResponses.find(msg.replyTo);
            if (it != pendingResponses.end()) {
                cb = it->second;
                pendingResponses.erase(it);
            }
        }
        if (cb) {
            cb(msg);
            return;
        }
    }

    if (behavior) {
        behavior(msg);
    }
}

void Actor::setBehavior(std::function<void(const Message&)> newBehavior) {
    behavior = std::move(newBehavior);
}

std::string Actor::getName() const {
    return name;
}

MessageID Actor::generateMessageID() {
    return globalMessageID++;
}

Message Actor::readIncomingMessage() {
    for (auto& s : sockets) {
        if (s == INVALID_SOCKET) continue;

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(s, &readfds);
        timeval timeout{0, 0};
        int ret = select(static_cast<int>(s + 1), &readfds, nullptr, nullptr, &timeout);
        if (ret < 0) continue;
        if (ret == 0) continue;

        if (FD_ISSET(s, &readfds)) {
            std::string buffer;
            char ch;
            while (true) {
                int r = recv(s, &ch, 1, 0);
                if (r <= 0) {
                    closesocket(s);
                    s = INVALID_SOCKET;
                    break;
                }
                if (ch == '\n') break;
                buffer.push_back(ch);
            }
            if (!buffer.empty()) {
                return Actor::deserializeMessage(buffer); // Explicit now
            }
        }
    }
    return Message{};
}

bool Actor::sendNetworkMessage(const Message& msg) {
    std::string data = Actor::serializeMessage(msg);
    bool anySuccess = false;

    for (auto& s : sockets) {
        if (s == INVALID_SOCKET) continue;

        size_t sent = 0;
        bool thisSocketSuccess = true;

        while (sent < data.size()) {
            int ret = send(s, data.c_str() + sent, static_cast<int>(data.size() - sent), 0);
            if (ret == SOCKET_ERROR || ret == 0) {
                std::cerr << "[" << name << "] Socket send failed, closing socket.\n";
                closesocket(s);
                s = INVALID_SOCKET;
                thisSocketSuccess = false;
                break;
            }
            sent += ret;
        }

        if (thisSocketSuccess) anySuccess = true;
    }
    return anySuccess;
}