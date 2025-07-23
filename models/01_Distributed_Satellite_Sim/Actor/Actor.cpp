#include "Actor.hh"
#include <iostream>
#include <cstring>
#include <sstream>

// Static member init
atomic<MessageID> Actor::globalMessageID{1};

// Serialize Message to string format:
// sender|type|id|replyTo|key1=val1;key2=val2;\n
string Actor::serializeMessage(const Message& msg) {
    string fields_str = msg.flattenFields();
    return msg.sender + "|" + msg.type + "|" +
           std::to_string(msg.id) + "|" + std::to_string(msg.replyTo) + "|" +
           fields_str + "\n";
}

// Deserialize string to Message struct
Message Actor::deserializeMessage(const string& raw) {
    Message msg;
    size_t pos1 = raw.find('|');
    if (pos1 == string::npos) return msg;
    msg.sender = raw.substr(0, pos1);

    size_t pos2 = raw.find('|', pos1 + 1);
    if (pos2 == string::npos) return msg;
    msg.type = raw.substr(pos1 + 1, pos2 - pos1 - 1);

    size_t pos3 = raw.find('|', pos2 + 1);
    if (pos3 == string::npos) return msg;
    try {
        msg.id = std::stoull(raw.substr(pos2 + 1, pos3 - pos2 - 1));
    } catch (...) {
        msg.id = 0;
    }

    size_t pos4 = raw.find('|', pos3 + 1);
    if (pos4 == string::npos) return msg;
    try {
        msg.replyTo = std::stoull(raw.substr(pos3 + 1, pos4 - pos3 - 1));
    } catch (...) {
        msg.replyTo = 0;
    }

    string fields_str = raw.substr(pos4 + 1);
    if (!fields_str.empty() && fields_str.back() == '\n') {
        fields_str.pop_back();
    }

    // Parse fields: key=value;key2=value2;
    size_t start = 0;
    while (start < fields_str.size()) {
        size_t eq_pos = fields_str.find('=', start);
        if (eq_pos == string::npos) break;
        size_t semi_pos = fields_str.find(';', eq_pos);
        if (semi_pos == string::npos) break;

        string key = fields_str.substr(start, eq_pos - start);
        string val = fields_str.substr(eq_pos + 1, semi_pos - eq_pos - 1);

        msg.fields[key] = val;

        start = semi_pos + 1;
    }

    return msg;
}

Actor::Actor(string name_) : name(std::move(name_)), running(false) {}
Actor::~Actor() { stop(); }

void Actor::start() {
    running = true;
    worker = thread(&Actor::run, this);
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
        lock_guard<mutex> lock(mailboxMutex);
        mailbox.push(msg);
    }
    mailboxCV.notify_one();

    if (!sockets.empty()) {
        if (!sendNetworkMessage(msg)) {
            cerr << "[" << name << "] Failed to send network message.\n";
        }
    }
}

#include <string>

void Actor::sendMessage(const Message& msg, socket_t sock) {
    std::string serialized = serializeMessage(msg) + "\n";

    size_t totalSent = 0;
    size_t toSend = serialized.size();
    const char* data = serialized.c_str();

    while (totalSent < toSend) {
        int sent = send(sock, data + totalSent, static_cast<int>(toSend - totalSent), 0);
        if (sent == SOCKET_ERROR || sent == 0) {
            std::cerr << "[" << name << "] Failed to send message on socket.\n";
            // Optionally handle disconnect here (close socket etc.)
            break;
        }
        totalSent += sent;
    }
}


void Actor::sendAsk(const Message& msg, function<void(const Message&)> callback) {
    Message out = msg;
    out.id = generateMessageID();
    {
        lock_guard<mutex> lock(mailboxMutex);
        pendingResponses[out.id] = move(callback);
        mailbox.push(out);
    }
    mailboxCV.notify_one();

    if (!sockets.empty()) {
        if (!sendNetworkMessage(out)) {
            cerr << "[" << name << "] Failed to send ask message over network.\n";
        }
    }
}

Message Actor::receiveMessage() {
    unique_lock<mutex> lock(mailboxMutex);
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
        function<void(const Message&)> cb;
        {
            lock_guard<mutex> lock(mailboxMutex);
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

void Actor::setBehavior(function<void(const Message&)> newBehavior) {
    behavior = move(newBehavior);
}

string Actor::getName() const {
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
        if (ret < 0) {
            // select error, just continue
            continue;
        }
        if (ret == 0) continue; // no data

        if (FD_ISSET(s, &readfds)) {
            std::string buffer;
            char ch;
            while (true) {
                int r = recv(s, &ch, 1, 0);
                if (r == 0) {
                    // Connection closed by peer, mark socket invalid but don't close here immediately
                    std::cerr << "[Actor] Connection closed by peer on socket " << s << "\n";
                    closesocket(s);
                    s = INVALID_SOCKET;
                    break;
                }
                if (r < 0) {
#ifdef _WIN32
                    int err = WSAGetLastError();
                    if (err == WSAEWOULDBLOCK || err == WSAEINTR) {
                        // Non-fatal, try again or break from inner loop
                        break;
                    } else {
                        std::cerr << "[Actor] recv error " << err << " on socket " << s << "\n";
                        closesocket(s);
                        s = INVALID_SOCKET;
                        break;
                    }
#else
                    if (errno == EAGAIN || errno == EINTR) {
                        break;
                    } else {
                        std::cerr << "[Actor] recv error " << errno << " on socket " << s << "\n";
                        closesocket(s);
                        s = INVALID_SOCKET;
                        break;
                    }
#endif
                }
                if (ch == '\n') break;
                buffer.push_back(ch);
            }

            if (!buffer.empty()) {
                return deserializeMessage(buffer);
            }
        }
    }
    return Message{};
}


bool Actor::sendNetworkMessage(const Message& msg) {
    string data = serializeMessage(msg);
    bool anySuccess = false;

    for (auto& s : sockets) {
        if (s == INVALID_SOCKET) continue;

        size_t sent = 0;
        bool thisSocketSuccess = true;

        while (sent < data.size()) {
            int ret = send(s, data.c_str() + sent, static_cast<int>(data.size() - sent), 0);
            if (ret == SOCKET_ERROR || ret == 0) {
                cerr << "[" << name << "] Socket send failed, closing socket.\n";
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
