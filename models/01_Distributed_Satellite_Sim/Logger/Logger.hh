#ifndef LOGGER_HH
#define LOGGER_HH

#include "../Actor/Actor.hh"

class LoggerActor : public Actor {
public:
    LoggerActor(const std::string& ip, int port);

protected:
    void initializeNetwork() override;
    void run() override;

private:
    std::string ip;
    int port;
};

#endif

