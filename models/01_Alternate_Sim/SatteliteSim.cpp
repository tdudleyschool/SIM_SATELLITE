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

//
#include <cmath>
#include <iostream>

constexpr double RAD2DEG = 180.0 / PI;

// Dot product
double dot(const double* a, const double* b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

double dot_product(const double* a, const double* b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

double norm(const double* v) {
    return std::sqrt(dot_product(v, v));
}


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

double angle_between_vectors(const double* A, const double* B) {
    double dotProd = dot(A, B);
    double normA = norm(A);
    double normB = norm(B);

    if (normA < 1e-15 || normB < 1e-15) {
        // Handle zero-length vector case: angle undefined, return 0 or NaN
        return 0.0;
    }

    double val = std::abs(dotProd) / (normA * normB);

    // Clamp val to [-1, 1] to avoid NaN due to floating point errors
    if (val > 1.0) val = 1.0;
    if (val < -1.0) val = -1.0;

    return std::acos(val);  // angle in radians, [0, pi]
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
        Kp[i] = 1.0;
        Ki[i] = 0.05;
        Kd[i] = 0.3;
        prev_error[i] = 0;
    }

    target_vec[0] = 1;
    target_vec[1] = 1;
    target_vec[2] = 1;

    normalizeVector(target_vec);

    //Body initialization
    Sattelite_Body.initialize_body(1, 0.5);
    Sattelite_Body.get_R(R_matrix);
    Sattelite_Body.get_w(Sat_w);
    Sattelite_Body.get_Qori(Sat_q);
}

void ACS_Sim::update(double delta) {
    double curr_j_axis[3];
    double target_ref_vec[3] = {target_vec[0], target_vec[1], target_vec[2]};
    double error[3], derivative[3];
    double vec_errors[3];

    // Normalize input target vector
    normalizeVector(target_vec);
    
    // Convert target vector to satellite's reference frame
    Sattelite_Body.convert_to_ref_frame(target_ref_vec);
    normalizeVector(target_ref_vec);

    // Get current Y-axis of the satellite (in reference frame)
    Sattelite_Body.get_ref_j_vec(curr_j_axis);
    normalizeVector(curr_j_axis);

    // Define local axis directions (reference frame)
    double x_axis[3] = {1, 0, 0};
    double y_axis[3] = {0, 1, 0};
    double z_axis[3] = {0, 0, 1};

    double target_x[3] = {target_ref_vec[0], 0, 0};
    double target_y[3] = {0, target_ref_vec[1], 0};
    double target_z[3] = {0, 0, target_ref_vec[2]};
    double curr_x[3] = {curr_j_axis[0], 0, 0};
    double curr_y[3] = {0, curr_j_axis[1], 0};
    double curr_z[3] = {0, 0, curr_j_axis[2]};

    // Compute angle error (in radians) between current and target vectors around local axes
    //error[0] = angle_error_around_axis(curr_j_axis, target_ref_vec, x_axis);  // x'
    //error[1] = angle_error_around_axis(curr_j_axis, target_ref_vec, y_axis);  // y'
    //error[2] = angle_error_around_axis(curr_j_axis, target_ref_vec, z_axis);  // z'

    for (int i = 0; i < 3; i++) {
        error[i] = target_vec[i] - curr_j_axis[i];
    }

    double error_around_axis[3];


    //Error x,y -> Motor_z
    error_around_axis[2] = error[0] * error[1]; //error around z
    //Error y,z -> Motor_x
    error_around_axis[0] = - error[1] * error[2]; //error around x
    //Error z,x -> Motor_y
    error_around_axis[1] = error[2] * error[0]; //error around y

    // Print raw angle errors for debugging
    std::cout << "Angle Errors (rad): X: " << error_around_axis[0] //e_x
              << ", Y: " << error_around_axis[1] //e_y
              << ", Z: " << error_around_axis[2] << "\n"; // e_z

    // PID loop
    double V_in[3];

    for (int i = 0; i < 3; i++) {
        //error[i] = error[i] / 3.14;
        integral[i] = clamp(integral[i] + error_around_axis[i] * delta, -2.0, 2.0);
        derivative[i] = (error_around_axis[i] - prev_error[i]) / delta;
        prev_error[i] = error_around_axis[i];
        

        V_in[i] = Kp[i]*error_around_axis[i] + Ki[i]*integral[i] + Kd[i]*derivative[i];

        cout << "<<<<<<<<<< == " << V_in[i] << "== >>>>>>>>>>> \n";
        V_in[i] = clamp(V_in[i], -10.0, 10.0);  // Adjust clamp as needed

        ACS.motor_clock(i, V_in[i]);
    }

    // Log controller output
    std::cout << "Input Voltages: X: " << V_in[0]
              << ", Y: " << V_in[1]
              << ", Z: " << V_in[2] << "\n";

    // === Motor Physics ===
    for (int i = 0; i < 3; i++) {
        I_prev[i] = I[i];
        w_prev[i] = w[i];
    }


    ACS.update_all_ori(R_matrix);
    ACS.updateAll_I(I_prev);
    ACS.updateAll_omega(w_prev);

    ACS.state_deriv_getALL_dI(dI);
    ACS.state_deriv_getALL_Alpha(dw);

    for (int i = 0; i < 3; i++) {
        I[i] = I_prev[i] + dI[i] * delta;
        w[i] = w_prev[i] + dw[i] * delta;
    }

    //ACS.updateAll_I(I);
    //ACS.updateAll_omega(w);

    double torque[3];
    ACS.get_total_torque(torque);

    // === Apply to Rigid Body ===
    Sattelite_Body.update_torque(torque[0], torque[1], torque[2]);
    Sattelite_Body.get_w(Sat_w);
    Sattelite_Body.get_Qori(Sat_q);
    Sattelite_Body.state_deriv_get_alpha(Sat_dw);

    for (int i = 0; i < 3; i++) {
        Sat_w[i] += delta * Sat_dw[i];
    }

    Sattelite_Body.update_w(Sat_w[0], Sat_w[1], Sat_w[2]);
    Sattelite_Body.state_deriv_getQori(Sat_dq);

    for (int i = 0; i < 4; i++) {
        Sat_q[i] += 0.5 * delta * Sat_dq[i];
    }

    Sattelite_Body.update_Qori(Sat_q);
    Sattelite_Body.get_R(R_matrix);
    
    cout << "===== \n";
    cout << "Current Each Motor Draws: " << I[0] << ", " << I[1] << ", " << I[2] << "\n";
    cout << "Total Angular Velocity: " << w[0] << " rad/s, " << w[1] << " rad/s, " << w[2] << " rad/s\n";
    cout << "Total Torque : " << torque[0] << " Nm, " << torque[1] << " Nm, " << torque[2] << " Nm\n";
    cout << "==============Values For Controller============= \n \n";
    cout << "Input Voltage in Each Motor: " << V_in[0] << " Volts, " << V_in[1] << " Volts, " << V_in[2] << " Volts \n";
    cout << "Target Vector: " << target_vec[0] << " , " << target_vec[1] << " , " << target_vec[2] << " \n";
    //cout << "Target ref vec: " << target_ref_vec[0] << ", " << target_ref_vec[1] << ", " << target_ref_vec[2] << "\n";
    cout << "Total Y Direction : " << curr_j_axis[0] << " , " << curr_j_axis[1] << " , " << curr_j_axis[2] << " \n";
    cout << "===== \n";
}
