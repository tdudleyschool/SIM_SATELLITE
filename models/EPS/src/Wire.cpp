#include "../include/Wire.hh"
#include <cmath>
#include <iostream>

// Fix macro
#define PI 3.14159265358979323846

using namespace std;

wire::wire() {
    // Default values
    length = 1.0;
    diameter = 0.001;
    rho = 1.68e-8;
    mu0 = 4 * PI * 1e-7;
    epsilon0 = 8.85e-12;
    h = 0.01; // reasonable default for distance to ground (in meters)

    r = diameter / 2.0;
    A = PI * r * r;

    R = rho * length / A + 10.0;
    L = mu0 * length * (log(2 * length / r) - 1);
    C = (2 * PI * epsilon0 * length) / log(2 * h / r);

    V = 0.0;
    Vc = 0.0;
    I = 0.0;

    cout << "R = " << R << " Ohms\n";
    cout << "L = " << L << " H\n";
    cout << "C = " << C << " F\n";
}

wire::wire(double l, double d) {
    length = l;
    diameter = d;
    rho = 1.68e-8;
    mu0 = 4 * PI * 1e-7;
    epsilon0 = 8.85e-12;
    h = 0.01;

    r = diameter / 2.0;
    A = PI * r * r;

    R = rho * length / A + 10.0;
    L = mu0 * length * (log(2 * length / r) - 1);
    C = (2 * PI * epsilon0 * length) / log(2 * h / r);

    V = 0.0;
    Vc = 0.0;
    I = 0.0;
}

void wire::initialize_wire_metrics(double l, double d) {
    length = l;
    diameter = d;
    r = diameter / 2.0;
    A = PI * r * r;

    R = rho * length / A;
    L = mu0 * length * (log(2 * length / r) - 1);
    C = (2 * PI * epsilon0 * length) / log(2 * h / r);
}

void wire::input_voltage(double voltage) {
    V = voltage;
}

void wire::input_current(double currentIn) {
    I = currentIn;
}

void wire::get_state_dirv(double& dI, double& dVc, double Vt) {
    dI = (Vt - R * I - Vc) / L;
    dVc = I / C;
}

void wire::update_states(double curr, double vol_c) {
    I = curr;
    Vc = vol_c;
}

void wire::update_voltage_from_curr(double current, double prev_current, double dt) {
    double di_dt = (current - prev_current) / dt;
    Vc += dt * current / C;  // Integrate from input current
    I = current;             // Set internal current
    V = L * di_dt + R * I + Vc;
}

double wire::get_current() {
    return I;
}

double wire::get_voltage() {
    return V;
}

double wire::get_R() { return R; }
double wire::get_L() { return L; }
double wire::get_C() { return C; }