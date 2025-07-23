#pragma once
#include "../Actor/Actor.hh"
#include <string>
#include "../../Attitude_Control/include/Attitude_Control_System.hh"

class AttitudeControlSystem : public Actor {
public:
    AttitudeControlSystem(const std::string& tk_ip, int tk_port,
                          const std::string& eps_ip, int eps_port,
                          const std::string& rb_ip_, int rb_port_,
                          int listen_port);
    void run() override;

private:
    void initializeNetwork();
    void acceptConnections();

    std::string tk_ip;
    int tk_port;

    std::string eps_ip;
    int eps_port;

    std::string rb_ip;
    int rb_port;

    int listen_port;
    socket_t server_fd;

    //Variables For ACS
    Attitude_Control_System ACS;
    double I[3];
    double w[3];
    double I_prev[3];
    double w_prev[3];
    double R_matrix[3][3];

    double dI[3];
    double dw[3];

    //Internal State Inputs
    double input_V[3];

    void configureBehaviors();

};
