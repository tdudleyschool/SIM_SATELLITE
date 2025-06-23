#ifndef HALL_THRUSTER_HH_2D_PIC
#define HALL_THRUSTER_HH_2D_PIC

#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <iostream>

using namespace std;

// ======= HEADER FILES (.hh) =======

// SimulationDomain.hh
class SimulationDomain {
public:
    int Nx, Nz;
    double dx, dz, x_min, x_max, z_min, z_max;
    vector<vector<double>> radialGrid, axialGrid;

    SimulationDomain(int Nx_in, int Nz_in, double xL, double xR, double zB, double zT);
    void initializeGrid();
};

class MagneticField {
public:
    vector<vector<double>> Bz, Bx;
    MagneticField(const SimulationDomain& domain);
    void loadFromFEMMFile(const string& filename);
    void initializeAnalytic();
};

// ElectricField.hh
class ElectricField {
public:
    vector<vector<double>> phi, Ex, Ez;
    void computeFromElectronDensity(const vector<vector<double>>&, const vector<vector<double>>&);
    void computePotentialLaplace(const SimulationDomain &domain,
        const BoundaryConditions &boundaries,
        int maxIter = 500, double tol = 1e-6);
    void updateField();
};

// ElectronFluid.hh
class ElectronFluid {
public:
    vector<vector<double>> ne, Te, ue;
    double alphaBohm = 1.0 / 16.0;

    void initialize(const SimulationDomain& domain);
    void updateElectronVelocity(const ElectricField& E, const MagneticField& B);
    void solveElectronTemperature(double dt);
};

// IonPIC.hh
struct Ion {
    double x, z, vx, vz, weight;
};
class IonPIC {
public:
    vector<Ion> ions;
    void initializeFromProfile(const SimulationDomain& domain);
    void pushParticles(const ElectricField& E, double dt);
    void pushParticles(const ElectricField& E, const SimulationDomain& domain, double dt);
    void handleWallCollisions();
    vector<vector<double>> depositToGrid();
};

// NeutralPIC.hh
struct Neutral {
    double x, z, vx, vz;
};
class NeutralPIC {
public:
    vector<Neutral> neutrals;
    void injectNeutrals(double rate, double Tgas, const SimulationDomain& domain);
    void advanceNeutrals(double dt);
    void handleWallRebound();
};

// Ionization.hh
class Ionization {
public:
    double Ei = 12.1;
    double crossSection(double Te);
    void performIonization(ElectronFluid&, NeutralPIC&, IonPIC&, double dt);
};

// BoundaryConditions.hh
class BoundaryConditions {
public:
    void applyToElectronTemperature(ElectronFluid&);
    void applyToPotential(ElectricField&);
};

// ThrustCalculator.hh
class ThrustCalculator {
public:
    vector<double> thrustHistory;
    double computeThrust(const IonPIC&, double);
};

// HallThrusterSimulator.hh
class HallThrusterSimulator {
private:
    SimulationDomain domain;
    MagneticField B;
    ElectricField E;
    ElectronFluid electrons;
    IonPIC ions;
    NeutralPIC neutrals;
    Ionization ionizer;
    BoundaryConditions boundaries;
    ThrustCalculator thrustCalc;
    double currentTime = 0.0;
    double dt = 0.025;
    int maxSteps = 100000;

public:
    HallThrusterSimulator();
    void initialize();
    void runSimulation();
    void outputResults();
};

#endif
