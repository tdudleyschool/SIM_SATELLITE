#ifndef WIRE_HH
#define WIRE_HH



#include <iostream>

class wire {
private:
    // Physical properties
    double length, diameter, rho, mu0, epsilon0, h, r, A;

    // RLC values
    double R, L, C;

    // Voltage and current state
    double V, Vc;
    double I_in, I_out, I;

public:
    wire();
    wire(double length, double diameter);

    void initialize_wire_metrics(double length, double diameter);

    // Input modes
    void input_voltage(double voltage);                  // For voltage source
    void input_current(double currentIn);                // For current source

    // ODE helpers for voltage-driven mode
    void get_state_dirv(double& dI, double& dVc, double Vt); 
    void update_states(double curr, double vol_c);       

    // Current-driven mode
    void update_voltage_from_curr(double current, double prev_current, double dt);

    // Accessors
    double get_current();
    double get_voltage();
    double get_R();
    double get_L();
    double get_C();
};



#endif
