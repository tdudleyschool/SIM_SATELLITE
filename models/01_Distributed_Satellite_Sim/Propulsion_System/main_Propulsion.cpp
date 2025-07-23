#include "PropulsionSystem.hh"
#include <iostream>

int main() {
    PropulsionSystem actor("127.0.0.1", 9000, "127.0.0.1", 9103, "127.0.0.1", 9500, 9300); // Timekeeper, EPS, ridged body, Self port
    actor.start();

    std::cout << "[Main] PropulsionSystem running. Press Enter to stop...\n";
    std::cin.get();

    actor.stop();
    std::cout << "[Main] PropulsionSystem stopped.\n";
    return 0;
}
