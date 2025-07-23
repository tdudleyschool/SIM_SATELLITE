#ifndef SUMMER_HH
#define SUMMER_HH

#include "../Actor/Actor.hh"

class SummerActor : public Actor {
public:
    SummerActor(const std::string& timekeeper_ip, int timekeeper_port,
                const std::string& output_ip, int output_port);

protected:
    void initializeNetwork() override;
    void run() override;

private:
    std::string tk_ip;
    int tk_port;
    std::string output_ip;
    int output_port;

    socket_t output_sock = INVALID_SOCKET;

    int64_t sum = 0;

    void connectToOutput();
};

#endif
