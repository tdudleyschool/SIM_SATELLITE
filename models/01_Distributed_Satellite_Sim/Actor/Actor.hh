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

// TCP cross-platform
#define NOMINMAX
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
    string content;
    MessageID id = 0;
    MessageID replyTo = 0;
};

class Actor {
public:
    Actor(string name);
    virtual ~Actor();

    void start();
    void stop();

    // Sends message locally and also over TCP if connected
    void sendMessage(const Message& msg);

    // Ask pattern, uses mailbox + callback
    void sendAsk(const Message& msg, std::function<void(const Message&)> callback);

    void setBehavior(std::function<void(const Message&)> newBehavior);
    std::string getName() const;

protected:
    virtual void initializeNetwork() = 0;
    virtual void run() = 0;

    // Read raw data from socket, parse, return Message object
    Message readIncomingMessage();

    // Send serialized Message over TCP socket
    bool sendNetworkMessage(const Message& msg);

    // Receive Message from local mailbox queue
    Message receiveMessage();

    // Handles behavior callbacks and async replies
    void handleMessage(const Message& msg);

    string name;
    atomic<bool> running;
    thread worker;

    queue<Message> mailbox;
    mutex mailboxMutex;
    condition_variable mailboxCV;

    function<void(const Message&)> behavior;

    // Ask-response tracking
    static atomic<MessageID> globalMessageID;
    unordered_map<MessageID, function<void(const Message&)>> pendingResponses;

    MessageID generateMessageID();

    // TCP socket used for communication
    socket_t sock = INVALID_SOCKET;
};

#endif

