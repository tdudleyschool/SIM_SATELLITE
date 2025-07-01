#include <cmath>
#include <iostream>
#include "../include/hall_thruster_PIC2D.hh"

//other files for linking
#include "HET_simulation_2D_PIC.cpp"

#define ORANGE "\033[38;5;208m" //orange color
#define RESET "\033[0m"
#define WARN false

using namespace std;

HET_PIC2D::HET_PIC2D() 
    : domain(100, 50, 0.0, 0.1, 0.0, 0.05),  // example grid sizes and domain extents
      dt(5e-7)
{
    ref_pos.insert(0, 0, 0);
    ref_ori.insert(0, 0, 0);
    R_matrix.insert(1, 0, 0,
                0, 1, 0,
                0, 0, 1);

    //flags
    ref_pos_set = false;
    ref_ori_set = false;
    R_matrix_set = false;
    pos_set = false;
    ori_set = false;
}

//[[Set Refrence Position Function]]//
void HET_PIC2D::set_refrence_pos(double r_pos[3])
    //Description:      Initializes position in refrence to satellite position as if it were attached
    //Preconditions:    Array[3]/Eigen Vector3 variable with x, y, z corrdinates relative to satellite center
    //Postconditions:   new refrence position
{
    ref_pos.insert(r_pos[0], r_pos[1], r_pos[2]);
    pos.insert(0, 0, 0);
    pos = pos + ref_pos;
    ref_pos_set = true;
}
void HET_PIC2D::set_refrence_pos(double x, double y, double z)
{
    ref_pos.insert(x, y, z);
    pos.insert(0, 0, 0);
    pos = pos + ref_pos;
    ref_pos_set = true;
}
void HET_PIC2D::set_refrence_pos(Vector3d r_pos)
{
    ref_pos = r_pos;
    pos.insert(0, 0, 0);
    pos = pos + ref_pos;
    ref_pos_set = true;
}
//-----------------------------------//

//[[Set Refrence Orientation Function]]//
void HET_PIC2D::set_refrence_ori(double r_ori[3])
    //Description:      Initializes orientation (direction) in refrence to satellite orientation as if it were attached
    //Preconditions:    Array[3]/Eigen Vector3 with the x, y, z corrdinates representing direction relative to satellite direction
    //Postconditions:   new refrence orientation
{
    ref_ori.insert(r_ori[0], r_ori[1], r_ori[2]);
    ori.insert(0, 0, 0);
    ori = ori + ref_ori;
    ref_ori_set = true;
}
void HET_PIC2D::set_refrence_ori(double x, double y, double z)
{
    ref_ori.insert(x, y, z);
    ori.insert(0, 0, 0);
    ori = ori + ref_ori;
    ref_ori_set = true;  
}
void HET_PIC2D::set_refrence_ori(Vector3d r_ori)
{
    ref_ori = r_ori;
    ori.insert(0, 0, 0);
    ori = ori + ref_ori;
    ref_ori_set = true;
}
//-----------------------------------//

//[[Update Rotation Matrix Funnction]]//
void HET_PIC2D::update_R_matrix(double R[3][3])
    //Description:      updates rotation matrix. intended to get rotation matrix from satellite body
    //Preconditions:    either 2d 3x3 array or an Eigen matrix representing rotation matrix
    //Postconditions:   updated rotation matrix
{
    R_matrix.insert(R[0][0], R[0][1], R[0][2],
                R[1][0], R[1][1], R[1][2],
                R[2][0], R[2][1], R[2][2]);
    R_matrix_set = true;
}
void HET_PIC2D::update_R_matrix(const Matrix3d& R)
{
    R_matrix = R;
    R_matrix_set = true;
}
//-----------------------------------//

//[[Update Position Funnction]]//
void HET_PIC2D::update_pos(double ref[3], double R[3][3])
    //Description:      updates global position of thruster based on satellite updated center position
    //Preconditions:    if update_R_matrix is used then preconditions are an array/Eigen vector of the satellite center
    //                  if update_R_matrix is NOT used then requires array/Eigen vector representing satellite center and 3x3array/Eigen matrix for rotation matrix
    //Postconditions:   global position updated
{
    if(R_matrix_set && WARN){
        cerr << ORANGE << "Warning: " << RESET << "Rotation matrix was already set during this cycle. Ensure the values are not being set twice.\n \n";
    }
    Vector3d refrence;

    refrence.insert(ref[0], ref[1], ref[2]);
    R_matrix.insert(R[0][0], R[0][1], R[0][2],
                R[1][0], R[1][1], R[1][2],
                R[2][0], R[2][1], R[2][2]);
    
    pos = refrence + R_matrix*ref_pos;
    pos_set = true;
}
void HET_PIC2D::update_pos(Vector3d ref, const Matrix3d& R)
{
    if(R_matrix_set && WARN){
        cerr << ORANGE << "Warning: " << RESET << "Rotation matrix was already set during this cycle. Ensure the values are not being set twice.\n \n";
    }
    R_matrix = R;
    pos = ref + R_matrix*ref_pos;
    pos_set = true;
}
void HET_PIC2D::update_pos(double ref[3])
{
    if (!R_matrix_set && WARN) {
        cerr << ORANGE << "Warning: " << RESET << "Rotation matrix was not updated in this cycle. The function (update_pos) requires the most recent rotation matrix values for proper use.\n \n";
    }
    Vector3d refrence;
    refrence << ref[0], ref[1], ref[2];
    pos = refrence + R_matrix*ref_pos;
    pos_set = true;
}
void HET_PIC2D::update_pos(Vector3d ref)
{
    if (!R_matrix_set && WARN) {
        cerr << ORANGE << "Warning: " << RESET << "Rotation matrix was not updated in this cycle. The function (update_pos) requires the most recent rotation matrix values for proper use.\n \n";
    }
    pos = ref + R_matrix*ref_pos;
    pos_set = true;
}
//-----------------------------------//

//[[Update Orientation Funnction]]//
void HET_PIC2D::update_ori(double R[3][3])
    //Description:      updates global orienttation of thruster based on satellite updated orientation
    //Preconditions:    if update_R_matrix is NOT used then requires 3x3array/Eigen matrix for rotation matrix. Otherwise None
    //Postconditions:   global orientation updated
{
    if(R_matrix_set && WARN){
        cerr << ORANGE << "Warning: " << RESET << "Rotation matrix was already set during this cycle. Ensure the values are not being set twice.\n \n";
    }
    R_matrix.insert(R[0][0], R[0][1], R[0][2],
                R[1][0], R[1][1], R[1][2],
                R[2][0], R[2][1], R[2][2]);
    ori = R_matrix.inverse() * ref_ori;
    ori_set = true;
}
void HET_PIC2D::update_ori(const Matrix3d& R)
{
    if(R_matrix_set && WARN){
        cerr << ORANGE << "Warning: " << RESET << "Rotation matrix was already set during this cycle. Ensure the values are not being set twice.\n \n";
    }
    R_matrix = R;
    ori = R_matrix.inverse() * ref_ori;
    ori_set = true;
}
void HET_PIC2D::update_ori()
{
    if (!R_matrix_set && WARN) {
        cerr << ORANGE << "Warning: " << RESET << "Rotation matrix was not updated in this cycle. The function (update_ori) requires the most recent rotation matrix values for proper use."<< '\n' << '\n';
    }
    R_matrix.inverse() * ref_ori;
    ori_set = true;
}
//-----------------------------------//

//[[Update Position Orientation Funnction]]//
void HET_PIC2D::update_pos_ori(double ref[3], double R[3][3])
    //Description:      updates global position and orienttation of thruster based on satellite updated position and orientation
    //Preconditions:    if update_R_matrix is used then preconditions are an array/Eigen vector of the satellite center *(ref)
    //                  if update_R_matrix is NOT used then requires array/Eigen vector representing satellite center *(ref) and 3x3array/Eigen matrix for rotation matrix *(R)
    //Postconditions:   global orientation and position updated
{
    if(R_matrix_set && WARN){
        cerr << ORANGE << "Warning: " << RESET << "Rotation matrix was already set during this cycle. Ensure the values are not being set twice.\n \n";
    }
    Vector3d refrence;

    refrence.insert(ref[0], ref[1], ref[2]);
    R_matrix.insert(R[0][0], R[0][1], R[0][2],
                R[1][0], R[1][1], R[1][2],
                R[2][0], R[2][1], R[2][2]);
    
    pos = refrence + R_matrix*ref_pos;
    ori = R_matrix.inverse() * ref_ori;
    R_matrix_set = true;
    pos_set = true;
    ori_set = true;
}
void HET_PIC2D::update_pos_ori(Vector3d ref, const Matrix3d& R)
{
    if(R_matrix_set && WARN){
        cerr << ORANGE << "Warning: " << RESET << "Rotation matrix was already set during this cycle. Ensure the values are not being set twice.\n \n";
    }
    R_matrix = R;

    pos = ref + R_matrix*ref_pos;
    ori = R_matrix.inverse() * ref_ori;
    R_matrix_set = true;
    pos_set = true;
    ori_set = true;
}
void HET_PIC2D::update_pos_ori(double ref[3])
{
    if (!R_matrix_set && WARN) {
        cerr << ORANGE << "Warning: " << RESET << "Rotation matrix was not updated in this cycle. The function (update_ori) requires the most recent rotation matrix values for proper use."<< '\n' << '\n';
    }
    Vector3d refrence;
    refrence.insert(ref[0], ref[1], ref[2]);

    pos = refrence + R_matrix*ref_pos;
    ori = R_matrix.inverse() * ref_ori;
    pos_set = true;
    ori_set = true;
}
void HET_PIC2D::update_pos_ori(Vector3d ref)
{
    if (!R_matrix_set && WARN) {
        cerr << ORANGE << "Warning: " << RESET << "Rotation matrix was not updated in this cycle. The function (update_ori) requires the most recent rotation matrix values for proper use." << '\n' << '\n';
    }
    pos = ref + R_matrix*ref_pos;
    ori = R_matrix.inverse() * ref_ori;
    pos_set = true;
    ori_set = true;
}
//-----------------------------------//

//[[Get Position Function]]//
void HET_PIC2D::get_pos(double return_pos[3])
    //Description:      gets position of satellite
    //Preconditions:    None
    //Postconditions:   reciver gets the position array/Eigen vector
{
    return_pos[0] = pos[0];
    return_pos[1] = pos[1];
    return_pos[2] = pos[2];
}
Vector3d HET_PIC2D::get_pos()
{
    return pos;
}
//-----------------------------------//

//[[Get Orientation Function]]//
void HET_PIC2D::get_ori(double return_ori[3])
    //Description:      gets orientation of satellite
    //Preconditions:    None
    //Postconditions:   reciver gets the orientation array/Eigen vector
{
    return_ori[0] = ori[0];
    return_ori[1] = ori[1];
    return_ori[2] = ori[2];
}
Vector3d HET_PIC2D::get_ori()
{
    return ori;
}
//-----------------------------------//

void HET_PIC2D::initialize_HET_sim()
{
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

void HET_PIC2D::run_step_HET_sim(double mass_flow, double discharge_volt) 
{
    // Electron fluid updates (not affecting Ez in this test)
    electrons.updateElectronTemperature(dt);
    electrons.updateElectronVelocity(field.Ex, field.Ez, field.Bz);

    // --- Debug: Comment out Boltzmann overwrite of phi ---
    // field.computePotentialFromBoltzmann(electrons.Te, electrons.ne);

    // --- Apply fixed BC and electric field ---
    boundaries.applyToPhi(field.phi, discharge_volt);
    // phi: 300 at left, 0 at right
    field.computeElectricField(domain.dx, domain.dz);

    // --- Optional: override Ez with fixed 1kV/m in z-direction ---
    int Nx = domain.Nx;
    int Nz = domain.Nz;
    //field.Ez.resize(Nx, std::vector<double>(Nz, 1000.0)); // 1e3 V/m

    // Neutral dynamics
    neutrals.injectNeutrals(mass_flow, 300.0, domain);
    neutrals.moveNeutrals(dt);

    // Ionization (should still be active)
    ionizer.performIonization(electrons.Te, electrons.ne, neutrals, ions, domain, dt);

    // Push ions using manually set Ez
    ions.pushParticles(field.Ez, domain, dt);
    ions.applyDomainBounds(domain);

    // Apply boundary conditions again to Te
    boundaries.applyToTe(electrons.Te);

    // --- Compute thrust and log ion count ---
    thrust = 0.0;
    int count = 0;
    thrustCalc.computeThrust(ions, thrust, count);
}

void HET_PIC2D::get_force(double available_mass, double F[3], double F_pos[3])
{
    if ((!R_matrix_set || !pos_set || !ori_set) && WARN) {
        cerr << ORANGE << "Warning: " << RESET << "Not all values was not updated in this cycle. The function (update_force) requires the most recent rotation matrix, position, and orientation values for proper use. Check to see if update_R_matrix, update_pos, update_ori, or update_pos_ori have been run to remoce this warning.\n \n";
    }

    F_ref.insert(thrust * ref_ori[0], thrust * ref_ori[1], thrust * ref_ori[2]);
    Force = R_matrix.inverse() * F_ref;

    if (available_mass <= 0 || !state_on){

        for (int i = 0; i < 3; i++){
        F[i] = 0;
        F_pos[i] = 0;
        }
    }
    else
    {
        for (int i = 0; i < 3; i++){
            F[i] = Force[i];
            F_pos[i] = pos[i];
        }
    }

    pos_set = false;
    ori_set = false;
    R_matrix_set = false;
}
void HET_PIC2D::get_force(double available_mass, Vector3d& F, Vector3d& F_pos)
{
    if ((!R_matrix_set || !pos_set || !ori_set) && WARN) {
        cerr << ORANGE << "Warning: " << RESET << "Not all values was not updated in this cycle. The function (update_force) requires the most recent rotation matrix, position, and orientation values for proper use. Check to see if update_R_matrix, update_pos, update_ori, or update_pos_ori have been run to remoce this warning.\n \n";
    }
    Vector3d F_ref;
    F_ref.insert(thrust * ref_ori[0], thrust * ref_ori[1], thrust * ref_ori[2]);
    Force = R_matrix.inverse() * F_ref;  

    if (available_mass <= 0 || !state_on){

        F.insert(0, 0, 0);
        F_pos.insert(0, 0, 0);
    }
    else
    {
        F = Force;
        F_pos = pos;
    }

    pos_set = false;
    ori_set = false;
    R_matrix_set = false;    
}

void HET_PIC2D::switch_stateon()
    //Description:      switches thruster on
    //Preconditions:    None
    //Postconditions:   thruster is on. running get_force() function will generate an output
{
    state_on = true;
}

void HET_PIC2D::switch_stateoff()
    //Description:      switches thruster off
    //Preconditions:    None
    //Postconditions:   thruster is on. running get_force() function will return zero
{
    state_on = false;
}

bool HET_PIC2D::is_state_on()
    //Description:      checks if thruster is on.
    //Preconditions:    None
    //Postconditions:   None
{
    return state_on;
}