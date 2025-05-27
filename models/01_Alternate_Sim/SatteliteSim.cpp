#include "SatteliteSim.hh"
#include <iostream>
#include "../Attitude_Control/src/Attitude_Control_System.cpp"
#include "../Attitude_Control/src/motor.cpp"
#include "../Attitude_Control/src/control_wheels.cpp"

using namespace std;

ACS_Sim::ACS_Sim(double timestep, double max_time)
    : Sim_Object(timestep, max_time) {}

void ACS_Sim::initialize() {
    for (int i = 0; i < 3; i++) {
        I[i] = 0;
        w[i] = 0;
        dI[i] = 0;
        dw[i] = 0;
    }
    ACS.motor_clock(1, 120);
    ACS.motor_contClock(2, 120);
    ACS.updateAll_I(I);
    ACS.updateAll_omega(w);
}

void ACS_Sim::update(double delta) {
    for (int i = 0; i < 3; i++){
        I_prev[i] = I[i];
        w_prev[i] = w[i];
    }
    double R[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    ACS.update_all_ori(R);
    ACS.updateAll_I(I_prev);
    ACS.updateAll_omega(w_prev);
    ACS.motor_clock(1, 120);
    ACS.motor_contClock(2, 120);

    ACS.state_deriv_getALL_dI(dI);
    ACS.state_deriv_getALL_Alpha(dw);

    for (int i = 0; i < 3; i++) {
        I[i] = I_prev[i] + dI[i]*delta;
        w[i] = w_prev[i] + dw[i]*delta;
    }

    ACS.updateAll_I(I);
    ACS.updateAll_omega(w);
    double torque[3];

    ACS.get_total_torque(torque);
    
    cout << "===== \n";
    cout << "Current Each Motor Draws: " << I[0] << ", " << I[1] << ", " << I[2] << "\n";
    cout << "Total Angular Velocity: " << w[0] << " rad/s, " << w[1] << " rad/s, " << w[2] << " rad/s\n";
    cout << "Total Torque : " << torque[0] << " Nm, " << torque[1] << " Nm, " << torque[2] << " Nm\n";
    cout << "===== \n";
}