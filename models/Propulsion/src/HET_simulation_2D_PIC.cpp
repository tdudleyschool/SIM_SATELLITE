// ============================
// Implementation for Fernandez-style HET Simulation
// ============================

#include "../include/HET_simulation_2D_PIC.hh"
#include <cmath>
#include <random>
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <cstdlib>  // for rand()
#include <algorithm>

// ----------------------------
// SimulationDomain 
// ----------------------------

// Constructor: initialize domain size and compute grid spacing
SimulationDomain::SimulationDomain(int Nx_in, int Nz_in, double xL, double xR, double zB, double zT)
    : Nx(Nx_in), Nz(Nz_in), x_min(xL), x_max(xR), z_min(zB), z_max(zT) {
    
    dx = (x_max - x_min) / static_cast<double>(Nx);
    dz = (z_max - z_min) / static_cast<double>(Nz);

    // Resize grid containers
    xGrid.resize(Nx, std::vector<double>(Nz, 0.0));
    zGrid.resize(Nx, std::vector<double>(Nz, 0.0));

    // Populate grid values
    initializeGrid();
}

// Fill xGrid and zGrid with spatial coordinates
void SimulationDomain::initializeGrid() {
    for (int i = 0; i < Nx; ++i) {
        for (int j = 0; j < Nz; ++j) {
            xGrid[i][j] = x_min + i * dx;
            zGrid[i][j] = z_min + j * dz;
        }
    }
}


// ----------------------------
// ElectronFluid
// ----------------------------

// Initialize electron temperature, density, and velocity across the grid
void ElectronFluid::initialize(const SimulationDomain& domain) {
    Te.resize(domain.Nx, std::vector<double>(domain.Nz, 5.0));
    ne.resize(domain.Nx, std::vector<double>(domain.Nz, 1e17));

    for (int i = 0; i < domain.Nx; ++i) {
        for (int j = 0; j < domain.Nz; ++j) {
            double x_frac = static_cast<double>(i) / domain.Nx;
            double z_frac = static_cast<double>(j) / domain.Nz;

            // Add gradients in both x and z directions
            Te[i][j] = 5.0 + 5.0 * x_frac + 2.0 * z_frac;  
            ne[i][j] = 1e17 * (1.0 - 0.5 * x_frac - 0.2 * z_frac);
        }
    }
}


// Update Te using a simplified RK4-like diffusion approximation
void ElectronFluid::updateElectronTemperature(double dt) {
    int Nx = Te.size();
    int Nz = Te[0].size();
    if (Te_temp.size() != Nx) {
        Te_temp.resize(Nx, std::vector<double>(Nz, 0.0));
    }

    for (int i = 1; i < Nx - 1; ++i) {
        for (int j = 1; j < Nz - 1; ++j) {
            double laplacian =
                Te[i + 1][j] + Te[i - 1][j] +
                Te[i][j + 1] + Te[i][j - 1] -
                4.0 * Te[i][j];

            Te_temp[i][j] = Te[i][j] + dt * (0.01 * laplacian - 0.05 * Te[i][j]);
        }
    }

    std::swap(Te, Te_temp);
}

// Calculate electron drift velocity using Ex, Ez, and Bz components
// Here ue_x and ue_z are computed, so ue needs to store 2D vector velocities
void ElectronFluid::updateElectronVelocity(const std::vector<std::vector<double>>& Ex,
                                           const std::vector<std::vector<double>>& Ez,
                                           const std::vector<std::vector<double>>& Bz) {
    int Nx = Ex.size();
    int Nz = Ex[0].size();

    // We'll create temporary ue_x and ue_z
    std::vector<std::vector<double>> ue_x(Nx, std::vector<double>(Nz, 0.0));
    std::vector<std::vector<double>> ue_z(Nx, std::vector<double>(Nz, 0.0));

    for (int i = 0; i < Nx; ++i) {
        for (int j = 0; j < Nz; ++j) {
            double Bz_local = Bz[i][j];
            if (fabs(Bz_local) < 1e-9) Bz_local = 1e-9;  // avoid divide by zero

            // Effective mobility from Bohm diffusion coefficient
            double mu_eff = alphaBohm / Bz_local;

            // Electron velocity components from E and B:
            // For simplicity, assume drift only along E-field direction divided by Bz
            // Realistically velocity perpendicular to B is E × B drift:
            // u_e = mu_eff * (E × B) / |B|^2 but with B along z, E in x,z plane:
            // E × B = (Ex, Ez, 0) × (0, 0, Bz) = (Ez*Bz, -Ex*Bz, 0)
            // so u_e_x = mu_eff * Ez
            // u_e_z = - mu_eff * Ex

            ue_x[i][j] = mu_eff * Ez[i][j];     // velocity in x due to Ez × Bz
            ue_z[i][j] = -mu_eff * Ex[i][j];    // velocity in z due to Ex × Bz
        }
    }

    // Store magnitude or split ue into components - depends on your data structure
    // If ue is just one vector, store magnitude for now:
    ue.resize(Nx, std::vector<double>(Nz, 0.0));
    for (int i = 0; i < Nx; ++i) {
        for (int j = 0; j < Nz; ++j) {
            ue[i][j] = std::sqrt(ue_x[i][j]*ue_x[i][j] + ue_z[i][j]*ue_z[i][j]);
        }
    }
}


// ----------------------------
// ElectricField
// ----------------------------
const double kTe_over_e = 1.0;  // Use 1.0 for normalized units or replace with (k_B / e)

// Compute electric potential using Boltzmann relation: φ = (kTe/e) * ln(ne)
void ElectricField::computePotentialFromBoltzmann(const std::vector<std::vector<double>>& Te,
                                                  const std::vector<std::vector<double>>& ne) {
    int Nx = Te.size();
    int Nz = Te[0].size();
    phi.resize(Nx, std::vector<double>(Nz, 0.0));

    for (int i = 0; i < Nx; ++i) {
        for (int j = 0; j < Nz; ++j) {
            // Use local Te for more accuracy; add small value to ne to avoid log(0)
            phi[i][j] = (Te[i][j]) * std::log(ne[i][j] + 1e-10);
        }
    }

    //Debug
    //std::cout << "phi left-center: " << phi[0][Nz/2] << ", phi right-center: " << phi[Nx-1][Nz/2] << ", phi center: " << phi[Nx/2][Nz/2] << std::endl;

}

void ElectricField::computeElectricField(double dx, double dz) {
    int Nx = phi.size();
    int Nz = phi[0].size();

    Ex.resize(Nx, std::vector<double>(Nz, 0.0));
    Ez.resize(Nx, std::vector<double>(Nz, 0.0));

    //Debug
    //std::cout << "dx = " << dx << ", dz = " << dz << std::endl;

    // Central differences for interior points
    for (int i = 1; i < Nx - 1; ++i) {
        for (int j = 1; j < Nz - 1; ++j) {
            Ex[i][j] = -(phi[i + 1][j] - phi[i - 1][j]) / (2.0 * dx);
            Ez[i][j] = -(phi[i][j + 1] - phi[i][j - 1]) / (2.0 * dz);
        }
    }

    // Forward/backward difference for Ez at z-boundaries
    for (int i = 0; i < Nx; ++i) {
        Ez[i][0] = -(phi[i][1] - phi[i][0]) / dz;               // forward diff at bottom boundary
        Ez[i][Nz - 1] = -(phi[i][Nz - 1] - phi[i][Nz - 2]) / dz; // backward diff at top boundary
    }

    // Forward/backward difference for Ex at x-boundaries
    for (int j = 0; j < Nz; ++j) {
        Ex[0][j] = -(phi[1][j] - phi[0][j]) / dx;               // forward diff at left boundary
        Ex[Nx - 1][j] = -(phi[Nx - 1][j] - phi[Nx - 2][j]) / dx; // backward diff at right boundary
    }

    // Debug prints near center grid point
    //int i_center = Nx / 2;
    //int j_center = Nz / 2;
    
    //Debug
    //std::cout << "phi near center (z-1, z, z+1): "
    //          << phi[i_center][j_center - 1] << ", "
    //          << phi[i_center][j_center] << ", "
    //          << phi[i_center][j_center + 1] << std::endl;
    //std::cout << "Ez at center: " << Ez[i_center][j_center] << std::endl;
}


void ElectricField::initializeMagneticField(const SimulationDomain& domain) {
    Bz.resize(domain.Nx, std::vector<double>(domain.Nz, 0.0));
    for (int i = 0; i < domain.Nx; ++i) {
        for (int j = 0; j < domain.Nz; ++j) {
            double x = domain.xGrid[i][j];
            double z = domain.zGrid[i][j];
            Bz[i][j] = 0.01;
        }   
    }
}

// ----------------------------
// IonPIC phi
// ----------------------------

// Initialize a uniform population of ions near the inlet (z = 0)
void IonPIC::initialize(const SimulationDomain& domain) {
    ions.clear();

    int num_ions = 1000;  // Adjust for simulation resolution
    for (int i = 0; i < num_ions; ++i) {
        Ion ion;

        ion.x = domain.x_min + (domain.x_max - domain.x_min) * (std::rand() / (double)RAND_MAX);
        ion.z = domain.z_min;  // Inject from bottom of the domain
        ion.vx = 0.0;
        ion.vz = 0.0;
        ion.weight = 1.0;

        ions.push_back(ion);
    }
}

// Push ions using Ez field: F = qE -> a = qE/m -> update vz and z
void IonPIC::pushParticles(const std::vector<std::vector<double>>& Ez,
                           const SimulationDomain& domain, double dt) {
    int Nx = Ez.size();
    int Nz = Ez[0].size();

    for (auto& ion : ions) {
        int i = static_cast<int>((ion.x - domain.x_min) / domain.dx);
        int j = static_cast<int>((ion.z - domain.z_min) / domain.dz);

        if (i >= 0 && i < Nx && j >= 0 && j < Nz) {
            double Ez_local = Ez[i][j];
            const double qm = 7.3e5; // q/m for Xe+
            ion.vz += qm * Ez_local * dt;
        }

        ion.z += ion.vz * dt;
    }

    // No bounce back – let ions exit at z > z_max
}


void IonPIC::applyDomainBounds(const SimulationDomain& domain) {
    ions.erase(std::remove_if(ions.begin(), ions.end(), [&](const Ion& ion) {
        return (ion.z > domain.z_max ||  // allow exit
                ion.z < domain.z_min || ion.x < domain.x_min || ion.x > domain.x_max);
    }), ions.end());
}


// ----------------------------
// NeutralPIC
// ----------------------------

// Inject neutrals from the bottom boundary (z = z_min) with thermal spread
NeutralPIC::NeutralPIC() : rng(std::random_device{}()) {}

void NeutralPIC::injectNeutrals(double rate, double Tgas, const SimulationDomain& domain) {
    int N_inject = static_cast<int>(rate);
    std::uniform_real_distribution<double> pos_dist(0.0, 1.0);
    std::normal_distribution<double> vel_dist(0.0, std::sqrt(Tgas));

    for (int i = 0; i < N_inject && neutrals.size() < MAX_NEUTRALS; ++i) {
        Neutral n;
        n.x = domain.x_min + pos_dist(rng) * (domain.x_max - domain.x_min);
        n.z = domain.z_min;
        n.vx = vel_dist(rng);
        n.vz = std::abs(vel_dist(rng));
        neutrals.push_back(n);
    }
}

void NeutralPIC::moveNeutrals(double dt) {
    for (auto& n : neutrals) {
        n.x += n.vx * dt;
        n.z += n.vz * dt;
    }

    // Remove neutrals that leave the domain
    neutrals.erase(std::remove_if(neutrals.begin(), neutrals.end(), [&](const Neutral& n) {
        return (n.z > 0.05 || n.z < 0.0 || n.x > 0.1 || n.x < 0.0);
    }), neutrals.end());

    // Cap size
    if (neutrals.size() > MAX_NEUTRALS) {
        neutrals.resize(MAX_NEUTRALS);
    }
}


// ----------------------------
// Ionization
// ----------------------------

// Empirical ionization cross-section as a function of electron temperature (eV)
// Here we use a placeholder exponential decay model
Ionization::Ionization() : rng(std::random_device{}()), rand(0.0, 1.0) {}

double Ionization::crossSection(double Te) {
    if (Te <= 0.0) return 0.0;
    double base_sigma = 1e-20 * std::exp(-Ei / Te);
    return base_sigma * 1e10;  // artificially boost by 10,000 for testing
}

// Perform stochastic ionization using local electron properties and Monte Carlo sampling
void Ionization::performIonization(const std::vector<std::vector<double>>& Te,
    const std::vector<std::vector<double>>& ne,
    NeutralPIC& neutrals, IonPIC& ions,
    const SimulationDomain& domain, double dt) {

    int Nx = Te.size();
    int Nz = Te[0].size();

    for (auto it = neutrals.neutrals.begin(); it != neutrals.neutrals.end();) {
        int i = static_cast<int>((it->x - domain.x_min) / domain.dx);
        int j = static_cast<int>((it->z - domain.z_min) / domain.dz);

        i = std::max(0, std::min(i, Nx - 1));
        j = std::max(0, std::min(j, Nz - 1));

        double Te_local = Te[i][j];
        double ne_local = ne[i][j];

        double sigma = crossSection(Te_local);

        // Original ionization probability
        double P_ionize = 1.0 - std::exp(-sigma * ne_local * dt);

        // Enforce a small minimum ionization probability to ensure some ions form
        const double minIonProb = 1e-6;  // ~1 in a million chance per neutral per step
        if (P_ionize < minIonProb) {
            P_ionize = minIonProb;
        }

        if (rand(rng) < P_ionize) {
            //std::cout << "Ionizing neutral at (" << it->x << "," << it->z << "), Te=" << Te_local
            //  << ", ne=" << ne_local << ", sigma=" << sigma << ", P_ionize=" << P_ionize << "\n";
            ions.ions.push_back(Ion{it->x, it->z, 0.0, 0.0, 1.0});
            *it = neutrals.neutrals.back();
            neutrals.neutrals.pop_back();
        }
        else {
            ++it;
        }
    }
}



// ----------------------------
// BoundaryConditions
// ----------------------------

// Apply Dirichlet boundary conditions for electron temperature
void BoundaryConditions::applyToTe(std::vector<std::vector<double>>& Te) {
    int Nx = Te.size();
    int Nz = Te[0].size();

    for (int j = 0; j < Nz; ++j) {
        Te[0][j]      = 5.0; // Left boundary (anode)
        Te[Nx - 1][j] = 5.0; // Right boundary (exit or wall)
    }
}

// Apply Dirichlet boundary conditions for electrostatic potential (phi)
void BoundaryConditions::applyToPhi(std::vector<std::vector<double>>& phi, double volt) {
    int Nx = phi.size();
    int Nz = phi[0].size();

    // Left and right boundaries (x boundaries)
    for (int j = 0; j < Nz; ++j) {
        phi[0][j] = volt;    // Anode (high potential)
        phi[Nx - 1][j] = 0.0; // Cathode/exit (ground)
    }

    // Top and bottom boundaries (z boundaries)
    for (int i = 0; i < Nx; ++i) {
        phi[i][0] = volt;        // Bottom boundary potential (example)
        phi[i][Nz - 1] = volt;   // Top boundary potential (example)
    }
}


// Inject neutrals at the anode/inlet (z = 0 plane)
void BoundaryConditions::injectNeutralsAtInlet(NeutralPIC& neutrals, const SimulationDomain& domain) {
    std::default_random_engine rng(std::random_device{}());
    std::uniform_real_distribution<double> x_dist(domain.x_min, domain.x_max);
    std::normal_distribution<double> vz_dist(0.0, 300.0);  // Maxwellian approx.

    const int N_inject = 100;  // number of neutrals per step

    for (int i = 0; i < N_inject; ++i) {
        Neutral n;
        n.x  = x_dist(rng);      // Spread across inlet width
        n.z  = domain.z_min;     // Always start at z = 0
        n.vx = 0.0;
        n.vz = std::abs(vz_dist(rng));  // Ensure flow is forward (+z)
        neutrals.neutrals.push_back(n);
    }
}


// ----------------------------
// ThrustCalculator
// ----------------------------
// Compute total thrust from all ions using 1D momentum summation
void ThrustCalculator::computeThrust(const IonPIC& ions, double& thrustOut, int& countOut) {
    thrustOut = 0.0;
    countOut = 0;

    const double ionMass = 2.18e-25;
    const double thrustPlane = 0.049;
    const double scalingFactor = 1e19;  // Adjust as needed

    for (const auto& ion : ions.ions) {
        if (ion.z > thrustPlane && ion.vz > 0) {
            thrustOut += ion.weight * ionMass * ion.vz;
            countOut++;
        }
    }

    thrustOut *= scalingFactor;
    thrustHistory.push_back(thrustOut);
}

// ----------------------------
// HallThrusterSimulator
// ----------------------------

// Constructor - set defaults (could be adjusted later)
HallThrusterSimulator::HallThrusterSimulator()
    : domain(100, 50, 0.0, 0.1, 0.0, 0.05),  // example grid sizes and domain extents
      currentTime(0.0),
      dt(5e-7),
      maxSteps(100000)
{ 
    // You could do further initialization here if needed
}

// Initialize all modules and simulation state
void HallThrusterSimulator::initialize() {
    std::cout << "Initializing domain...\n";
    domain.initializeGrid();

    std::cout << "Initializing electrons...\n";
    electrons.initialize(domain);

    std::cout << "Initializing ions...\n";
    ions.initialize(domain);

    std::cout << "Initializing magnetic field...\n";
    field.initializeMagneticField(domain);

    std::cout << "Computing electric potential and applying boundary conditions...\n";
    field.computePotentialFromBoltzmann(electrons.Te, electrons.ne);
    boundaries.applyToPhi(field.phi, 0);          // <-- apply BC here BEFORE computing E field
    field.computeElectricField(domain.dx, domain.dz);

    std::cout << "Applying boundary conditions (Te)...\n";
    boundaries.applyToTe(electrons.Te);

    std::cout << "Injecting initial neutrals...\n";
    boundaries.injectNeutralsAtInlet(neutrals, domain);

    std::cout << "Initialization complete.\n";
}

// Main simulation loop
void HallThrusterSimulator::runSimulation() {
    std::cout << "Running debug simulation with fixed Ez and phi boundaries...\n";
    using clock = std::chrono::steady_clock;
    auto next_step = clock::now();

    for (int step = 0; step < maxSteps; ++step) {
        next_step += std::chrono::milliseconds(25);  // 40 Hz loop

        // Electron fluid updates (not affecting Ez in this test)
        electrons.updateElectronTemperature(dt);
        electrons.updateElectronVelocity(field.Ex, field.Ez, field.Bz);

        if (step < 1000) {
            boundaries.applyToPhi(field.phi, 300);
        } else {
            boundaries.applyToPhi(field.phi, 0);
        }

        field.computeElectricField(domain.dx, domain.dz);

        neutrals.injectNeutrals(500, 900.0, domain);
        neutrals.moveNeutrals(dt);

        ionizer.performIonization(electrons.Te, electrons.ne, neutrals, ions, domain, dt);
        ions.pushParticles(field.Ez, domain, dt);
        ions.applyDomainBounds(domain);

        boundaries.applyToTe(electrons.Te);

        double thrust = 0.0;
        int count = 0;
        thrustCalc.computeThrust(ions, thrust, count);

        currentTime += dt;

        if (step % 1 == 0) {
            std::cout << "Step " << step 
                      << ", Time " << currentTime 
                      << ", Thrusting Ions: " << count 
                      << ", Thrust: " << thrust << " N\n";
            outputResults();
        }

        std::this_thread::sleep_until(next_step);
    }

    std::cout << "Simulation complete.\n";
}


void HallThrusterSimulator::outputResults() {
    static std::ofstream thrustFile("thrust_history.txt", std::ios::app);
    thrustFile << currentTime << "\t" << thrustCalc.thrustHistory.back() << "\n";
}