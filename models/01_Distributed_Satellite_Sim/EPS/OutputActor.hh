#ifndef OUTPUT_HH
#define OUTPUT_HH

#include "../Actor/Actor.hh"

class OutputActor : public Actor {
public:
    OutputActor(int port);

protected:
    void initializeNetwork() override;
    void run() override;

private:
    int port;
    socket_t server_fd = INVALID_SOCKET;
    socket_t client_fd = INVALID_SOCKET;
};

#endif


