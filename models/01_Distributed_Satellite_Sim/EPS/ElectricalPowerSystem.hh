#ifndef ELECTRICAL_POWER_SYSTEM_ACTOR_HH
#define ELECTRICAL_POWER_SYSTEM_ACTOR_HH

#include "../Actor/Actor.hh"
#include <string>
#include <vector>
#include <unordered_map>

#include "../../EPS/Battery.hh"
#include "../../EPS/bus.hh"
#include "../../EPS/solar_cell.hh"
#include "../../EPS/Solar_Power_System.hh"

/*
 * ModularActor is a customizable Actor with built-in support for:
 * - Connecting to one or more remote actors
 * - Listening for one or more incoming connections
 * - Supporting behavior triggers based on combinations of messages
 */2
class EPS_Module : public Actor {
public:
    // Constructor parameters:
    // - timekeeper_ip/timekeeper_port: the required timekeeper connection
    // - outboundTargets: vector of (ip, port) pairs to actively connect to
    // - listenPort: optional port for accepting incoming actor connections
    EPS_Module(const std::string& timekeeper_ip, int timekeeper_port,
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

    //Iternal State Variables For EPS
    //--EPS--//
    //PPE has 3 batteries
    battery EPS_bat[3];

    //2 buses high and low
    bus High_bus;
    bus Low_bus;

    //solar array system
    solar_array sol_arrays[2];
    solar_cell sol_cell[2];

    // Example trigger messages (customize) For EPS we do need it from some of the other systems to get electricity
    // std::vector<std::string> triggerMessages = {"msgA", "msgB", "msgC"};
};

#endif
