#include "AttitudeControlSystem.hh"
#include <iostream>

int main() {
    AttitudeControlSystem actor("127.0.0.1", 9000, "127.0.0.1", 9103, "127.0.0.1", 9500, 9200); // Timekeeper, EPS, ridged body, Self port
    actor.start();

    std::cout << "[Main] AttitudeControlSystem running. Press Enter to stop...\n";
    std::cin.get();

    actor.stop();
    std::cout << "[Main] AttitudeControlSystem stopped.\n";
    return 0;
}
