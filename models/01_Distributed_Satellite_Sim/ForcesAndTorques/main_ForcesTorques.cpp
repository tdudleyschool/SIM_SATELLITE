#include "ForceTorqueTracker.hh"
#include <iostream>

int main() {
    ForceTorqueTracker actor("127.0.0.1", 9000, "127.0.0.1", 9200, "127.0.0.1", 9300, "127.0.0.1", 9500 ,9400); // timekeeper, Attitude, Propulsion, Self port
    actor.start();

    std::cout << "[Main] ForceTorqueTracker running. Press Enter to stop...\n";
    std::cin.get();

    actor.stop();
    std::cout << "[Main] ForceTorqueTracker stopped.\n";
    return 0;
}
