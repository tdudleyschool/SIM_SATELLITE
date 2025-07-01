#include "../include/Wire.hh"
#include <iostream>
#include <fstream>

using namespace std;

// RK4 integration for voltage-driven test
void test_voltage_source(wire& w, double V_step, double dt, double total_time) {
    cout << "=== Voltage Source Test (RK4) ===" << endl;
    ofstream out("voltage_source_output.csv");
    out << "time,current,voltageC\n";

    double t = 0.0;
    double I = 0.0;
    double Vc = 0.0;

    while (t < total_time) {
        double dI1, dVc1, dI2, dVc2, dI3, dVc3, dI4, dVc4;

        w.update_states(I, Vc);
        w.get_state_dirv(dI1, dVc1, V_step);

        w.update_states(I + 0.5 * dt * dI1, Vc + 0.5 * dt * dVc1);
        w.get_state_dirv(dI2, dVc2, V_step);

        w.update_states(I + 0.5 * dt * dI2, Vc + 0.5 * dt * dVc2);
        w.get_state_dirv(dI3, dVc3, V_step);

        w.update_states(I + dt * dI3, Vc + dt * dVc3);
        w.get_state_dirv(dI4, dVc4, V_step);

        I  += dt / 6.0 * (dI1 + 2*dI2 + 2*dI3 + dI4);
        Vc += dt / 6.0 * (dVc1 + 2*dVc2 + 2*dVc3 + dVc4);

        w.update_states(I, Vc);

        out << t << "," << I << "," << Vc << "\n";
        t += dt;
    }

    out.close();
    cout << "Voltage test complete. Output written to voltage_source_output.csv\n";
}

// RK4-like update for current-driven test
void test_current_source(wire& w, double I_step, double dt, double total_time) {
    cout << "\n=== Current Source Test (RK4 style) ===" << endl;
    ofstream out("current_source_output.csv");
    out << "time,current,voltage\n";

    double t = 0.0;
    double prev_I = 0.0;
    double I = I_step;
    double Vc = 0.0;

    while (t < total_time) {
        // RK4 for Vc
        double k1 = I / w.get_C();
        double k2 = I / w.get_C(); // constant I, so all ks same
        double k3 = I / w.get_C();
        double k4 = I / w.get_C();

        Vc += dt / 6.0 * (k1 + 2*k2 + 2*k3 + k4);

        double di_dt = (I - prev_I) / dt;
        double V = w.get_L() * di_dt + w.get_R() * I + Vc;

        prev_I = I;

        out << t << "," << I << "," << V << "\n";
        t += dt;
    }

    out.close();
    cout << "Current test complete. Output written to current_source_output.csv\n";
}

int main() {
    wire test_wire(1.0, 0.001);

    double dt = 1e-9;         // Better timestep
    double total_time = 0.0001; // 5 microseconds

    test_voltage_source(test_wire, 5.0, dt, total_time);

    wire test_wire2(1.0, 0.001);

    test_current_source(test_wire2, 0.01, dt, total_time);

    return 0;
}
