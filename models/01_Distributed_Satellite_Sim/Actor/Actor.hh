#ifndef ACTOR_HH
#define ACTOR_HH

#include <string>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>
#include <unordered_map>
#include <vector>
#include <sstream>
#include <iostream>

#ifndef NOMINMAX
#define NOMINMAX
#endif

#define WIN32_LEAN_AND_MEAN

#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #include <windows.h>
  #pragma comment(lib, "ws2_32.lib")
  using socket_t = SOCKET;
#else
  #include <sys/socket.h>
  #include <arpa/inet.h>
  #include <unistd.h>
  using socket_t = int;
  #define INVALID_SOCKET -1
  #define SOCKET_ERROR -1
  #define closesocket close
#endif

using MessageID = uint64_t;
using namespace std;

struct Message {
    string sender;
    string type;
    unordered_map<string, string> fields;
    MessageID id = 0;
    MessageID replyTo = 0;

    // Serialize fields as key=value;key2=value2;...
    string flattenFields() const {
        string result;
        for (const auto& [k, v] : fields) {
            result += k + "=" + v + ";";
        }
        return result;
    }
};

class Actor {
public:
    Actor(string name);
    virtual ~Actor();

    void start();
    void stop();

    void sendMessage(const Message& msg); // broadcast to all connections
    
    // New overload to send to a specific socket
    void sendMessage(const Message& msg, socket_t sock);
    
    void sendAsk(const Message& msg, std::function<void(const Message&)> callback);

    void setBehavior(std::function<void(const Message&)> newBehavior);
    std::string getName() const;

    static std::string serializeMessage(const Message& msg);
    static Message deserializeMessage(const std::string& raw);

protected:
    virtual void initializeNetwork() = 0;
    virtual void run() = 0;

    Message readIncomingMessage();
    bool sendNetworkMessage(const Message& msg);
    Message receiveMessage();
    void handleMessage(const Message& msg);

    string name;
    atomic<bool> running;
    thread worker;

    queue<Message> mailbox;
    mutex mailboxMutex;
    condition_variable mailboxCV;

    function<void(const Message&)> behavior;

    static atomic<MessageID> globalMessageID;
    unordered_map<MessageID, function<void(const Message&)>> pendingResponses;

    MessageID generateMessageID();

    vector<socket_t> sockets;
    void closeAllSockets();
};

#endif
