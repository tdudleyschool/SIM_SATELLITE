#include "../include/HET_simulation_2D_PIC.hh"
#include <iostream>

//g++ src/HET_simulation_2D_PIC.cpp test/hall_thruster_2test.cpp -o het_sim

int main() {
    std::cout << "Starting Hall Thruster Simulation...\n";

    HallThrusterSimulator sim;

    sim.initialize();

    std::cout << "Initialization complete. Running simulation...\n";

    sim.runSimulation();

    std::cout << "Simulation finished. Writing output...\n";

    sim.outputResults();

    std::cout << "Done.\n";

    return 0;
}