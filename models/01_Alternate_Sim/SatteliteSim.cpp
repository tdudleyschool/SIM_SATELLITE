#include "SatteliteSim.hh"
#include <iostream>

using namespace std;

SatteliteSim::SatteliteSim(double timestep, double max_time)
    : Sim_Object(timestep, max_time), value(0.0) {}

void SatteliteSim::initialize() {
    value = 100.0;
    cout << "Current Sim Value: " << value << "\n";
}

void SatteliteSim::update(double dt) {
    value += 10.0*dt;
    cout << "Custome update value: " << value << "\n";
}