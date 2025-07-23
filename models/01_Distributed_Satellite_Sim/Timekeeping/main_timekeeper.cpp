// main_timekeeper.cpp
#include "Timekeeper.hh"
#include <iostream>

int main() {
    std::vector<std::string> expected = { "missionprocessor", "eps_module", "propulsionsystem", "attitudecontrolsystem", "ridgedbodymodule", "forcetorquetracker" };
    TimekeeperActor tk(9000, expected);
    tk.start();

    std::cout << "[Main] Timekeeper running. Press Enter to stop...\n";
    std::cin.get();

    tk.stop();
    std::cout << "[Main] Timekeeper stopped.\n";
    return 0;
}
