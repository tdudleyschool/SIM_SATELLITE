#include "SatteliteSim.hh"
#include <iostream>
#include <algorithm>
#include <cmath>
#include "../Attitude_Control/src/Attitude_Control_System.cpp"
#include "../Attitude_Control/src/motor.cpp"
#include "../Attitude_Control/src/control_wheels.cpp"
#include "../Ridged_Body/src/Ridged_Body.cpp"
#include "../Ridged_Body/src/Satellite_Box.cpp"

using namespace std;

void normalizeVector(double vec[3]) {
    // Step 1: Compute magnitude
    double magnitude = sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);

    // Step 2: Avoid division by zero
    if (magnitude == 0.0f) return;

    // Step 3: Divide each component by magnitude
    vec[0] /= magnitude;
    vec[1] /= magnitude;
    vec[2] /= magnitude;
}

ACS_Sim::ACS_Sim(double timestep, double max_time)
    : Sim_Object(timestep, max_time) {}

void ACS_Sim::initialize() {
    for (int i = 0; i < 3; i++) {
        I[i] = 0;
        w[i] = 0;
        dI[i] = 0;
        dw[i] = 0;

        //PID controller
        Kp[i] = 100.0;
        Ki[i] = 200.0;
        Kd[i] = 10.0;
        prev_error[i] = 0;
    }
    ACS.motor_clock(1, 120);
    ACS.motor_contClock(2, 400);
    ACS.updateAll_I(I);
    ACS.updateAll_omega(w);

    target_vec[0] = 1;
    target_vec[1] = 1;
    target_vec[2] = 1;

    normalizeVector(target_vec);

    //Body initialization
    Sattelite_Body.initialize_body(2.0, 2.0);
    Sattelite_Body.get_R(R_matrix);
    Sattelite_Body.get_w(Sat_w);
    Sattelite_Body.get_Qori(Sat_q);
}

void ACS_Sim::update(double delta) {
    //PID start of loop error analysis
    double error[3];
    double curr_j_axis[3];

    // - derivative here the integral and perportion are already done in .hh
    double derivative[3];

    Sattelite_Body.get_ref_j_vec(curr_j_axis);
    for (int i = 0; i < 3; i++) {
        error[i] = target_vec[i] - curr_j_axis[i];
    }

    // - convert error into reference, then split to i', j', k' vectors differntly getting the error each motor made
    Sattelite_Body.convert_to_ref_frame(error);

    // - now do teh PID for integral derivative, and proportion in the reference frame
    double V_in[3];
    for (int i = 0; i < 3; i++) {
        integral[i] = integral[i] + error[i]*delta;
        derivative[i] = (error[i] - prev_error[i])/delta;
        prev_error[i] = error[i];


        V_in[i] = Kp[i]*error[i] + Ki[i]*integral[i] + Kd[i]*derivative[i]; 
        V_in[i] = clamp(V_in[i], -120.0, 120.0);
        //Adjust the motor voltage
        ACS.motor_clock(i, V_in[i]);
    }
    
    //MOTIR SIMULATION
    for (int i = 0; i < 3; i++){
        I_prev[i] = I[i];
        w_prev[i] = w[i];
    }


    ACS.update_all_ori(R_matrix);
    ACS.updateAll_I(I_prev);
    ACS.updateAll_omega(w_prev);

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

    //Ridged body
    Sattelite_Body.update_torque(torque[0], torque[1], torque[2]);
    Sattelite_Body.get_w(Sat_w);
    Sattelite_Body.get_Qori(Sat_q);

    // -derivatives & integration
    //  - angular component
    Sattelite_Body.state_deriv_get_alpha(Sat_dw);
    for (int i = 0; i < 3; i++) {
        Sat_w[i] = Sat_w[i] + delta * Sat_dw[i];
    }
    Sattelite_Body.update_w(Sat_w[0], Sat_w[1], Sat_w[2]);
    //  - quaterniod component to get ridged body
    Sattelite_Body.state_deriv_getQori(Sat_dq);

    for (int i = 0; i < 4; i++){
        Sat_q[i] = Sat_q[i] + delta * Sat_dq[i] * 0.5;
    }
    //updates the ridged body as well.
    Sattelite_Body.update_Qori(Sat_q);

    
    cout << "===== \n";
    cout << "Current Each Motor Draws: " << I[0] << ", " << I[1] << ", " << I[2] << "\n";
    cout << "Total Angular Velocity: " << w[0] << " rad/s, " << w[1] << " rad/s, " << w[2] << " rad/s\n";
    cout << "Total Torque : " << torque[0] << " Nm, " << torque[1] << " Nm, " << torque[2] << " Nm\n";
    cout << "==============Values For Controller============= \n \n";
    cout << "Input Voltage in Each Motor: " << V_in[0] << " Volts, " << V_in[1] << " Volts, " << V_in[2] << " Volts \n";
    cout << "Total Y Direction : " << curr_j_axis[0] << " , " << curr_j_axis[1] << " , " << curr_j_axis[2] << " \n";
    cout << "===== \n";
}
