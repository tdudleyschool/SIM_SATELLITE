#include "MissionProcessor.hh"
#include <iostream>

int main() {
    // Connect to timekeeper at 127.0.0.1:9000, listen on 9101 for downstream actors
    MissionProcessor mp("127.0.0.1", 9000, 9101);
    mp.start();

    std::cout << "[Main] MissionProcessor running. Press Enter to stop...\n";
    std::cin.get();

    mp.stop();
    std::cout << "[Main] MissionProcessor stopped.\n";

    return 0;
}

