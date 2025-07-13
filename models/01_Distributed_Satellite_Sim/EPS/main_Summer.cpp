// main_summer.cpp
#include "SummerActor.hh"
#include <iostream>

int main() {
    SummerActor summer("127.0.0.1", 9000,  // timekeeper
                       "127.0.0.1", 9100); // output
    summer.start();

    std::cout << "[Main] Summer running. Press Enter to stop...\n";
    std::cin.get();

    summer.stop();
    std::cout << "[Main] Summer stopped.\n";
    return 0;
}

