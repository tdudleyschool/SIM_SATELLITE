#ifndef RidgedBodyModule_HH
#define RidgedBodyModule_HH

#include "../Actor/Actor.hh"
#include <string>

#include "../../Ridged_Body/include/Ridged_Body.hh"

class RidgedBodyModule : public Actor {
public:
    RidgedBodyModule(const std::string& timekeeper_ip_, int timekeeper_port_,
                    const std::string& mission_ip_, int mission_port_,
                    int listen_port_);

    void run() override;

private:

    std::string mission_ip;
    int mission_port;

    std::string timekeeper_ip;
    int timekeeper_port;

    int listen_port;
    socket_t server_fd;

    void initializeNetwork();
    void acceptConnections();
    void configureBehaviors();
    void sendInitializationMessage();

    //SATELLITE BODY
    ridged_body Sattelite_Body;
    double sat_x[3];
    double sat_v[3];
    double sat_a[3]; //state

    double sat_w[3];
    double sat_ori[3]; //forward
    double sat_q[4];
    double sat_dw[3];
    double sat_dq[4];
    double R_matrix[3][3];

    //internal input states
    double sat_Torque[3]; //state
    double sat_Force[3];


    //Initialization Flags yealds until these have been sent
    bool FFT_ready = false;
    bool ACS_ready = false;
    bool PROPULSION_ready = false;
    bool MISSIONPROCESSOR_ready = true;
};

#endif // RidgedBodyModule_HH
