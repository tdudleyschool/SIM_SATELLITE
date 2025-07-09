#include "Timekeeper.hh"
#include <iostream>

int main() {
    TimekeeperActor tk(9000);
    tk.start();

    std::cout << "[Main] Timekeeper running. Press Enter to stop...\n";
    std::cin.get();  // Wait for user to press Enter

    tk.stop();
    std::cout << "[Main] Timekeeper stopped.\n";

    return 0;
}