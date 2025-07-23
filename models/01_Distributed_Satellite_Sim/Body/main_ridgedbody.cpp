#include "RidgedBodyModule.hh"
#include <iostream>

int main() {
    RidgedBodyModule actor("127.0.0.1", 9000, "127.0.0.1", 9101, 9500); // timekeeper, MissionProcessor, listening
    actor.start();

    std::cout << "[Main] RidgedBodyModule running. Press Enter to stop...\n";
    std::cin.get();

    actor.stop();
    std::cout << "[Main] RidgedBodyModule stopped.\n";
    return 0;
}
