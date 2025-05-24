#include "SatteliteSim.hh"

int main() {
    SatteliteSim sim(0.0025, 2.0);
    sim.run();
    return 0;
}