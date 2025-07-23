#include "ElectricalPowerSystem.hh"
#include <iostream>

int main() {
    // Timekeeper at 127.0.0.1:9000
    // MissionProcessor at 127.0.0.1:9101
    // EPS listens on port 9103 for ACS and Propulsion

    EPS_Module eps("127.0.0.1", 9000, "127.0.0.1", 9101, 9103);
    eps.start();

    std::cout << "[Main] EPS Module running. Press Enter to stop...\n";
    std::cin.get();

    eps.stop();
    std::cout << "[Main] EPS Module stopped.\n";

    return 0;
}
