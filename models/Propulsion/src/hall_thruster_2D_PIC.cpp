#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <iostream>
#include <numeric>
#include <random>
#include <chrono>
#include <thread>

#include "../include/hall_thruster_2D_PIC.hh"

// ============================
// SimulationDomain
// ============================
SimulationDomain::SimulationDomain(int Nx_in, int Nz_in, double xL, double xR, double zB, double zT)
    : Nx(Nx_in), Nz(Nz_in), x_min(xL), x_max(xR), z_min(zB), z_max(zT) {
    dx = (x_max - x_min) / Nx;
    dz = (z_max - z_min) / Nz;
    radialGrid.resize(Nx, std::vector<double>(Nz));
    axialGrid.resize(Nx, std::vector<double>(Nz));
    initializeGrid();
}

void SimulationDomain::initializeGrid() {
    for (int i = 0; i < Nx; ++i) {
        for (int j = 0; j < Nz; ++j) {
            radialGrid[i][j] = x_min + i * dx;
            axialGrid[i][j] = z_min + j * dz;
        }
    }
}

// ============================
// MagneticField
// ============================
MagneticField::MagneticField(const SimulationDomain& domain) {
    Bz.resize(domain.Nx, std::vector<double>(domain.Nz, 0.0));
    Bx.resize(domain.Nx, std::vector<double>(domain.Nz, 0.0));
    initializeAnalytic();
}

void MagneticField::initializeAnalytic() {
    for (auto& row : Bz) std::fill(row.begin(), row.end(), 100.0);
    for (auto& row : Bx) std::fill(row.begin(), row.end(), 0.0);
}

void MagneticField::loadFromFEMMFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) return;

    for (int i = 0; i < Bz.size(); ++i) {
        for (int j = 0; j < Bz[0].size(); ++j) {
            double bx, bz;
            file >> bx >> bz;
            Bx[i][j] = bx;
            Bz[i][j] = bz;
        }
    }
}
// ============================
// ElectricField
// ============================
void ElectricField::computeFromElectronDensity(const std::vector<std::vector<double>>& ne, const std::vector<std::vector<double>>& Te) {
    int Nx = ne.size();
    int Nz = ne[0].size();
    phi.resize(Nx, std::vector<double>(Nz, 0.0));
    Ex.resize(Nx, std::vector<double>(Nz, 0.0));
    Ez.resize(Nx, std::vector<double>(Nz, 0.0));

    for (int i = 1; i < Nx - 1; ++i) {
        for (int j = 1; j < Nz - 1; ++j) {
            phi[i][j] = (Te[i][j] / 1.0) * log(ne[i][j] + 1e-10);
        }
    }
}

void ElectricField::computePotentialLaplace(const SimulationDomain& domain, const BoundaryConditions& boundaries, int maxIter=500, double tol=1e-6) {
    int Nx = domain.Nx;
    int Nz = domain.Nz;
    phi.resize(Nx, std::vector<double>(Nz, 0.0));
    Ex.resize(Nx, std::vector<double>(Nz, 0.0));
    Ez.resize(Nx, std::vector<double>(Nz, 0.0));

    // Initialize phi with boundary conditions first
    boundaries.applyToPotential(*this);

    // Initialize interior phi to average (optional)
    for (int i = 1; i < Nx - 1; ++i) {
        for (int j = 0; j < Nz; ++j) {
            phi[i][j] = 150.0; // mid-value initial guess
        }
    }

    for (int iter = 0; iter < maxIter; ++iter) {
        double max_diff = 0.0;
        for (int i = 1; i < Nx - 1; ++i) {
            for (int j = 1; j < Nz - 1; ++j) {
                double old_phi = phi[i][j];
                phi[i][j] = 0.25 * (phi[i+1][j] + phi[i-1][j] + phi[i][j+1] + phi[i][j-1]);
                double diff = fabs(phi[i][j] - old_phi);
                if(diff > max_diff) max_diff = diff;
            }
        }
        boundaries.applyToPotential(*this);

        if(max_diff < tol) break;
    }
}


void ElectricField::updateField() {
    int Nx = phi.size();
    int Nz = phi[0].size();
    for (int i = 1; i < Nx - 1; ++i) {
        for (int j = 1; j < Nz - 1; ++j) {
            Ex[i][j] = -(phi[i+1][j] - phi[i-1][j]) / 2.0;
            Ez[i][j] = -(phi[i][j+1] - phi[i][j-1]) / 2.0;
        }
    }
}

// ============================
// ElectronFluid
// ============================
void ElectronFluid::initialize(const SimulationDomain& domain) {
    ne.resize(domain.Nx, std::vector<double>(domain.Nz, 1e17));
    Te.resize(domain.Nx, std::vector<double>(domain.Nz, 5.0));
    ue.resize(domain.Nx, std::vector<double>(domain.Nz, 0.0));
}

void ElectronFluid::updateElectronVelocity(const ElectricField& E, const MagneticField& B) {
    int Nx = ne.size();
    int Nz = ne[0].size();
    ue.resize(Nx, std::vector<double>(Nz, 0.0));

    for (int i = 0; i < Nx; ++i) {
        for (int j = 0; j < Nz; ++j) {
            double mu_eff = alphaBohm / (B.Bz[i][j] + 1e-6);
            ue[i][j] = -mu_eff * (E.Ex[i][j]);
        }
    }
}

void ElectronFluid::solveElectronTemperature(double dt) {
    int Nx = Te.size();
    int Nz = Te[0].size();
    for (int i = 1; i < Nx - 1; ++i) {
        for (int j = 1; j < Nz - 1; ++j) {
            double laplacian = Te[i+1][j] + Te[i-1][j] + Te[i][j+1] + Te[i][j-1] - 4 * Te[i][j];
            Te[i][j] += dt * (0.01 * laplacian - 0.05 * Te[i][j]); // simplistic model
        }
    }
}

// ============================
// IonPIC
// ============================
void IonPIC::initializeFromProfile(const SimulationDomain& domain) {
    ions.clear();
    for (int i = 0; i < 1000; ++i) {
        Ion ion;
        ion.x = domain.x_min + (domain.x_max - domain.x_min) * (rand() / double(RAND_MAX));
        ion.z = domain.z_min;
        ion.vx = 0.0;
        ion.vz = 0.0;
        ion.weight = 1.0;
        ions.push_back(ion);
    }
}

void IonPIC::pushParticles(const ElectricField& E, const SimulationDomain& domain, double dt) {
    int Nx = E.Ez.size();
    int Nz = E.Ez[0].size();

    for (auto& ion : ions) {
        int i = static_cast<int>((ion.x - domain.x_min) / domain.dx);
        int j = static_cast<int>((ion.z - domain.z_min) / domain.dz);

        if (i >= 1 && i < Nx - 1 && j >= 1 && j < Nz - 1) {
            double Ez_local = E.Ez[i][j];
            double qm = 1e5; 
            ion.vz += qm * Ez_local * dt;
        }

        ion.z += ion.vz * dt;
    }
}

void IonPIC::handleWallCollisions() {
    for (auto& ion : ions) {
        if (ion.z < 0) {
            ion.z = 0;     // Keep inside domain
            ion.vz = -ion.vz * 0.5; // Some energy loss on bounce
        } else if (ion.z > 1.0) {
            ion.z = 1.0;
            ion.vz = -ion.vz * 0.5;
        }
    }
}

std::vector<std::vector<double>> IonPIC::depositToGrid() {
    int Nx = 50, Nz = 100;
    std::vector<std::vector<double>> rho(Nx, std::vector<double>(Nz, 0.0));
    for (const auto& ion : ions) {
        int i = static_cast<int>((ion.x) * Nx);
        int j = static_cast<int>((ion.z) * Nz);
        if (i >= 0 && i < Nx && j >= 0 && j < Nz) {
            rho[i][j] += ion.weight;
        }
    }
    return rho;
}

// ============================
// NeutralPIC
// ============================
void NeutralPIC::injectNeutrals(double rate, double Tgas, const SimulationDomain& domain) {
    int N_inject = static_cast<int>(rate);
    std::default_random_engine rng;
    std::uniform_real_distribution<double> pos_dist(0.0, 1.0);
    std::normal_distribution<double> vel_dist(0.0, sqrt(Tgas));

    for (int i = 0; i < N_inject; ++i) {
        Neutral n;
        n.x = domain.x_min + pos_dist(rng) * (domain.x_max - domain.x_min);
        n.z = domain.z_min;
        n.vx = vel_dist(rng);
        n.vz = fabs(vel_dist(rng));  // Make neutral velocity positive in axial (z) direction
        neutrals.push_back(n);
    }
}

void NeutralPIC::advanceNeutrals(double dt) {
    for (auto& n : neutrals) {
        n.x += n.vx * dt;
        n.z += n.vz * dt;
    }
}

void NeutralPIC::handleWallRebound() {
    for (auto& n : neutrals) {
        if (n.z < 0) {
            n.z = 0;
            n.vz = -n.vz * 0.5;
        } else if (n.z > 1.0) {
            n.z = 1.0;
            n.vz = -n.vz * 0.5;
        }
    }
}

// ============================
// Ionization
// ============================
double Ionization::crossSection(double Te) {
    return 1e-20 * exp(-Ei / Te);
}

void Ionization::performIonization(ElectronFluid& e, NeutralPIC& n, IonPIC& i, double dt) {
    std::default_random_engine rng;
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    double sigma = crossSection(5.0); // assume Te=5eV
    double P_ionize = 1.0 - exp(-sigma * dt);

    for (auto it = n.neutrals.begin(); it != n.neutrals.end(); ) {
        if (dist(rng) < P_ionize) {
            Ion ion;
            ion.x = it->x;
            ion.z = it->z;
            ion.vx = 0.0;
            ion.vz = 0.0;
            ion.weight = 1.0;
            i.ions.push_back(ion);
            it = n.neutrals.erase(it);
        } else {
            ++it;
        }
    }
}

// ============================
// BoundaryConditions
// ============================
void BoundaryConditions::applyToElectronTemperature(ElectronFluid& e) {
    for (int j = 0; j < e.Te[0].size(); ++j) {
        e.Te[0][j] = 5.0;
        e.Te.back()[j] = 5.0;
    }
}

void BoundaryConditions::applyToPotential(ElectricField& E) {
    for (int j = 0; j < E.phi[0].size(); ++j) {
        E.phi[0][j] = 300.0;      // Set high potential at left boundary
        E.phi.back()[j] = 0.0;    // Set low potential at right boundary
    }
}

// ============================
// ThrustCalculator
// ============================
double ThrustCalculator::computeThrust(const IonPIC& ions, double dt) {
    double thrust = 0.0;
    double ion_mass = 2.18e-25;  // Xenon ion mass in kg (adjust if needed)

    for (const auto& ion : ions.ions) {
        thrust += ion.weight * ion_mass * ion.vz / dt;
    }
    thrustHistory.push_back(thrust);
    return thrust;
}

// ============================
// HallThrusterSimulator
// ============================
HallThrusterSimulator::HallThrusterSimulator()
    : domain(50, 100, 0.0, 0.05, 0.0, 0.10), B(domain) {}

void HallThrusterSimulator::initialize() {
    electrons.initialize(domain);
    ions.initializeFromProfile(domain);
    E.phi.resize(domain.Nx, std::vector<double>(domain.Nz, 0.0));
}

void HallThrusterSimulator::runSimulation() {
    using namespace std::chrono;

    const int outputInterval = 1;  // output every step
    const int targetLoopTimeMs = 25;  // 40 Hz
    bool thrusterOn = true;

    for (int step = 0; step < maxSteps; ++step) {
        auto start_time = high_resolution_clock::now();

        if (step % 100 == 0) {
            std::cout << "Step " << step << " / " << maxSteps << std::endl;
        }

        if (thrusterOn) {
            E.computePotentialLaplace(domain, boundaries);

            E.updateField();

            electrons.updateElectronVelocity(E, B);
            electrons.solveElectronTemperature(dt);

            neutrals.injectNeutrals(100, 300.0, domain);
            neutrals.advanceNeutrals(dt);
            neutrals.handleWallRebound();

            ionizer.performIonization(electrons, neutrals, ions, dt);

            ions.pushParticles(E, domain, dt);
            ions.handleWallCollisions();
        }

        double thrust = thrustCalc.computeThrust(ions, dt);

        if (step % outputInterval == 0) {
            std::cout << "Time: " << currentTime << " s, Thrust: " << thrust << " N\n";
            // Debug info:
            std::cout << "Phi Left Boundary: " << E.phi[0][domain.Nz / 2] 
                      << ", Right Boundary: " << E.phi[domain.Nx - 1][domain.Nz / 2] << std::endl;
            std::cout << "Ez center: " << E.Ez[domain.Nx / 2][domain.Nz / 2] << std::endl;
        }

        currentTime += dt;

        auto elapsed = duration_cast<milliseconds>(high_resolution_clock::now() - start_time);
        int sleep_time = targetLoopTimeMs - static_cast<int>(elapsed.count());
        if (sleep_time > 0) {
            std::this_thread::sleep_for(milliseconds(sleep_time));
        }
    }
}

void HallThrusterSimulator::outputResults() {
    std::ofstream out("thrust_output.txt");
    for (const auto& T : thrustCalc.thrustHistory) {
        out << T << "\n";
    }
    out.close();
}
