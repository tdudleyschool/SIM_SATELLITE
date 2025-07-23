#pragma once
#include "../Actor/Actor.hh"
#include <string>

#include "../../Propulsion/include/Propulsion_System_PIC2D.hh"

class PropulsionSystem : public Actor {
public:
    PropulsionSystem(const std::string& tk_ip, int tk_port,
                          const std::string& eps_ip, int eps_port,
                          const std::string& rb_ip_, int rb_port_,
                          int listen_port);
    void run() override;

private:
    void initializeNetwork();
    void acceptConnections();
    void configureBehaviors();

    //ALL Connections
    std::string tk_ip; //timekeeper
    int tk_port;

    std::string eps_ip; //eps
    int eps_port;

    std::string rb_ip; //ridged body
    int rb_port;

    int listen_port; //listening port
    socket_t server_fd;

    


    //Propulsion System
    Propulsion_System_PIC2D propulsion;
    double ref_ori[3];

    //internal state inputs
    double input_V[7];
    double input_mass_flow[7];

    //physical states
    double R_matrix[3][3];
    double sat_pos[3];

};
