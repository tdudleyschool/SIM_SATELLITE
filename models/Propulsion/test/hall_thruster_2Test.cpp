#include "../include/hall_thruster_2D_PIC.hh"

int main() {
    HallThrusterSimulator sim;

    sim.initialize();

    sim.runSimulation();

    sim.outputResults();

    return 0;
}