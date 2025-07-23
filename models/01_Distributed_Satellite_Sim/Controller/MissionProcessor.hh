#ifndef MISSIONPROCESSOR_HH
#define MISSIONPROCESSOR_HH

#include "../Actor/Actor.hh"
#include <unordered_map>
#include <vector>
#include <mutex>

class MissionProcessor : public Actor {
public:
    MissionProcessor(const std::string& timekeeper_ip, int timekeeper_port, int listen_port);
    ~MissionProcessor();

protected:
    void initializeNetwork() override;
    void run() override;

private:
    std::string tk_ip;
    int tk_port;

    int listen_port;
    socket_t server_fd = INVALID_SOCKET;

    // Map actor names to sockets (for downstream actors like EPS)
    std::unordered_map<std::string, socket_t> downstreamConnections;
    std::mutex downstreamMutex;

    // For tracking readiness from Timekeeper
    bool readySent = false;
    bool low_voltage_enabled = false;
    bool controller_mode = false;
    bool input_mode = true;

    //Internal States For Controllers
    double pos[3];
    double vel[3];
    double acc[3];
    double R_matrix[3][3];

    //internal inputs
    double input_acs_V[3];
    double input_prop_V[7];
    double input_mass_flow[7];

    
    // Thread to accept incoming connections from downstream actors
    void acceptDownstreamConnections();

    // Behavior configuration
    void configureBehaviors();

    // Helpers
    void sendReadyToTimekeeper();
    void handleTick(const Message& msg);
};

#endif
