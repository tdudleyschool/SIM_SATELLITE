#ifndef TIMEKEEPER_HH
#define TIMEKEEPER_HH

#include "../Actor/Actor.hh"

class TimekeeperActor : public Actor {
    public:
        TimekeeperActor(int port);
    
    protected:
        void initializeNetwork() override;
        void run() override;
    
    private:
        int port;
        socket_t server_fd = INVALID_SOCKET;
        socket_t client_fd = INVALID_SOCKET;
    
        void waitForLogger();
        void sendTickLoop();
};

#endif