#ifndef FERNANDEZ_HET_SIM_HH
#define FERNANDEZ_HET_SIM_HH

#include <vector>
#include <random>
#include <fstream>

// ============================
// Simulation Domain
// ============================
class SimulationDomain {
public:
    int Nx, Nz;
    double dx, dz, x_min, x_max, z_min, z_max;
    std::vector<std::vector<double>> xGrid, zGrid;

    SimulationDomain(int Nx_in, int Nz_in, double xL, double xR, double zB, double zT);
    void initializeGrid();
};

// ============================
// Electron Fluid
// ============================
class ElectronFluid {
public:
    std::vector<std::vector<double>> Te, ne, ue;
    std::vector<std::vector<double>> ue_x, ue_z;
    std::vector<std::vector<double>> Te_temp;
    double alphaBohm = 1.0 / 16.0;

    void initialize(const SimulationDomain& domain);
    void updateElectronTemperature(double dt);
    void updateElectronVelocity(const std::vector<std::vector<double>>& Ex,
                                 const std::vector<std::vector<double>>& Ez,
                                 const std::vector<std::vector<double>>& Bz);
};

// ============================
// Electric Field and Magnetic Field
// ============================
class ElectricField {
public:
    std::vector<std::vector<double>> phi, Ex, Ez;
    std::vector<std::vector<double>> Bz;

    void computePotentialFromBoltzmann(const std::vector<std::vector<double>>& Te,
                                       const std::vector<std::vector<double>>& ne);
    void computeElectricField(double dx, double dz);
    void initializeMagneticField(const SimulationDomain& domain);
};

// ============================
// Ion Particle-In-Cell
// ============================
struct Ion {
    double x, z, vx, vz, weight;
};

class IonPIC {
public:
    std::vector<Ion> ions;

    void initialize(const SimulationDomain& domain);
    void pushParticles(const std::vector<std::vector<double>>& Ez,
                       const SimulationDomain& domain, double dt);
    void applyDomainBounds(const SimulationDomain& domain);
};

// ============================
// Neutral Particle-In-Cell
// ============================
struct Neutral {
    double x, z, vx, vz;
};

class NeutralPIC {
public:
    std::vector<Neutral> neutrals;
    std::default_random_engine rng;

    static const int MAX_NEUTRALS = 100000;

    NeutralPIC();
    void injectNeutrals(double rate, double Tgas, const SimulationDomain& domain);
    void moveNeutrals(double dt);
};

// ============================
// Ionization Module
// ============================
class Ionization {
public:
    double Ei = 12.1;
    std::default_random_engine rng;
    std::uniform_real_distribution<double> rand;

    Ionization();
    double crossSection(double Te);
    void performIonization(const std::vector<std::vector<double>>& Te,
                           const std::vector<std::vector<double>>& ne,
                           NeutralPIC& neutrals, IonPIC& ions,
                           const SimulationDomain& domain, double dt);
};

// ============================
// Boundary Conditions
// ============================
class BoundaryConditions {
public:
    void applyToTe(std::vector<std::vector<double>>& Te);
    void applyToPhi(std::vector<std::vector<double>>& phi, double);
    void injectNeutralsAtInlet(NeutralPIC& neutrals, const SimulationDomain& domain);
};

// ============================
// Thrust Calculator
// ============================
class ThrustCalculator {
public:
    std::vector<double> thrustHistory;

    void computeThrust(const IonPIC& ions, double& thrustOut, int& countOut);
};

// ============================
// Hall Thruster Simulator
// ============================
class HallThrusterSimulator {
private:
    SimulationDomain domain;
    ElectronFluid electrons;
    ElectricField field;
    IonPIC ions;
    NeutralPIC neutrals;
    Ionization ionizer;
    BoundaryConditions boundaries;
    ThrustCalculator thrustCalc;
    double currentTime = 0.0;
    double dt = 1e-8;
    int maxSteps = 100000;

public:
    HallThrusterSimulator();
    void initialize();
    void runSimulation();
    void outputResults();
};

#endif // FERNANDEZ_HET_SIM_HH
