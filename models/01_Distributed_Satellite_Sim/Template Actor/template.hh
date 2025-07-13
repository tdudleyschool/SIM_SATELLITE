#ifndef TEMPLATE_ACTOR_HH
#define TEMPLATE_ACTOR_HH

#ifndef MODULAR_ACTOR_HH
#define MODULAR_ACTOR_HH

#include "Actor.hh"
#include <string>
#include <vector>
#include <unordered_map>

/*
 * ModularActor is a customizable Actor with built-in support for:
 * - Connecting to one or more remote actors
 * - Listening for one or more incoming connections
 * - Supporting behavior triggers based on combinations of messages
 */
class ModularActor : public Actor {
public:
    // Constructor parameters:
    // - timekeeper_ip/timekeeper_port: the required timekeeper connection
    // - outboundTargets: vector of (ip, port) pairs to actively connect to
    // - listenPort: optional port for accepting incoming actor connections
    ModularActor(const std::string& timekeeper_ip, int timekeeper_port,
                 const std::vector<std::pair<std::string, int>>& outboundTargets,
                 int listenPort = -1); // -1 disables listening

protected:
    void initializeNetwork() override;
    void run() override;

    // Modular connection setup
    void connectToActors();   // actively connects to remote actors
    void listenForActors();   // accepts incoming connections

    // Modular behavior setup
    void configureBehaviors();  // user-defined conditions + behavior triggers

private:
    std::string tk_ip;
    int tk_port;
    int listen_port;
    std::vector<std::pair<std::string, int>> outboundConnections;

    socket_t server_fd = INVALID_SOCKET;  // for listening
    std::unordered_map<std::string, bool> triggerFlags;  // stores message trigger state

    // Internal state tracking (user can add their own)
    int64_t valueAccumulator = 0;  // example of internal computation

    // Example trigger messages (customize)
    std::vector<std::string> triggerMessages = {"msgA", "msgB", "msgC"};
};

#endif
