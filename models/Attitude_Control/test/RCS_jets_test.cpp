//g++ src/RCS_jets.cpp test/RCS_jets_test.cpp -o RCS_program
#include <iostream>
#include "../include/RCS_jets.hh"

int main() {
    ReactionControlThrusters rcs(1.0, 1.0, 1.0, 0.1); // 1m cube, 0.1 N jets
    rcs.commandTorque({0.05, 0.0, 0.0}); // Request roll torque

    auto states = rcs.getFiringStates();
    for (size_t i = 0; i < states.size(); ++i) {
        std::cout << "Thruster " << i << " firing: " << (states[i] ? "YES" : "NO") << "\n";
    }

    auto torque = rcs.getNetTorque();
    std::cout << "Net Torque: (" << torque[0] << ", " << torque[1] << ", " << torque[2] << ")\n";
}
