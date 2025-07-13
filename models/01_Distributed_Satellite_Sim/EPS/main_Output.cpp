// main_output.cpp
#include "OutputActor.hh"
#include <iostream>

int main() {
    OutputActor output(9100);
    output.start();

    std::cout << "[Main] Output running. Press Enter to stop...\n";
    std::cin.get();

    output.stop();
    std::cout << "[Main] Output stopped.\n";
    return 0;
}

