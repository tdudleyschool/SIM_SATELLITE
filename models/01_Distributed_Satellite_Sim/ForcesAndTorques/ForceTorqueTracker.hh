#ifndef FORCE_TORQUE_TRACKER_MODULE
#define FORCE_TORQUE_TRACKER_MODULE

#pragma once
#include "../Actor/Actor.hh"
#include <string>

#include "../../Recources/include/force_torque_tracker.hh"
#include "../../Recources/include/functions.hh"

class ForceTorqueTracker : public Actor {
public:
    ForceTorqueTracker(const std::string& timekeeper_ip_, int timekeeper_port_,
                       const std::string& att_ip, int att_port,
                       const std::string& prop_ip, int prop_port,
                       const std::string& ridgedbody_ip, int ridgedbody_port,
                       int listen_port);
    void run() override;

private:
    void initializeNetwork();
    void acceptConnections();
    void configureBehaviors();

    std::string timekeeper_ip;
    int timekeeper_port;

    std::string att_ip;
    int att_port;

    std::string prop_ip;
    int prop_port;

    std::string ridgedbody_ip_;
    int ridgedbody_port_;

    int listen_port;
    socket_t server_fd;

    //internal variables
    force_torque_tracker Satellite_Forces;

    double curr_force[3];
    double curr_torque[3];

    //flags to update both force and torque at the same time
    bool updated_force = false;
    bool updated_torque = false;
};

#endif
