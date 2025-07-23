#ifndef TIMEKEEPER_HH
#define TIMEKEEPER_HH

#include "../Actor/Actor.hh"
#include <unordered_map>

class TimekeeperActor : public Actor {
public:
    TimekeeperActor(int port, const std::vector<std::string>& expectedActors);

protected:
    void initializeNetwork() override;
    void run() override;

private:
    int port;
    socket_t server_fd = INVALID_SOCKET;

    std::unordered_map<std::string, bool> readyMap; // track ready status per actor

    void acceptConnections();
    void sendTickLoop();
    bool waitForAllReady();
};

#endif
