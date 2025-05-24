/********************************* TRICK HEADER *******************************
PURPOSE: (Simulate A Motor)
LIBRARY DEPENDENCY:
    ((motor.o)
     (control_wheels.o))
*******************************************************************************/

#include <iostream>
#include "../include/motor.hh"
#include "../include/control_wheels.hh"
//#include "../../../Lib/eigen-3.4.0/Eigen/Dense"

#define ORANGE "\033[38;5;208m" //orange color
#define RESET "\033[0m"

//using namespace Eigen;
using namespace std;

motor::motor()
    //Description:      Default constructor setting all values to their default state.
    //Preconditions:    None
    //Postconditions:   Motor with several settings
{
    R_a = 1.0;
    L_a = 0.5;
    k_b = 0.01;
    k_1 = 0.01;
    B = 1.0;

    b = 0.1;

    //Initial values
    V_a = 0.0;
    J = 0.1;
    I_a = 0.0;
    w = 0.0;
    T_m = 0.0;
    wheel.initialize(12.0, 8.0, 6.0);
    wheel.calc_I_0();

    ref_pos_set = true;
    ref_ori_set = false;
    R_matrix_set = false;
    pos_set = false;
    ori_set = false;

}

motor::motor(double resistance , double inductance, double k_back, double k1_const, double field, double damping)
    //Description:      Method sets up all values(electrical components) within the DC motor circut
    //Preconditions:    None
    //Postconditions:   All electical values (resistance, inductance, k constant, magnetic field and damping are set)
    //                  In addition incomming voltage and current also are set to 0 representing motor is off
{
    R_a = resistance;
    L_a = inductance;
    k_b = k_back;
    k_1 = k1_const;
    B = field;

    b = damping;

    //Initial values
    V_a = 0.0;
    I_a = 0.0;
    w = 0.0;
    wheel.initialize(12.0, 8.0, 6.0);
    wheel.calc_I_0();

    ref_pos_set = true;
    ref_ori_set = false;
    R_matrix_set = false;
    pos_set = false;
    ori_set = false;

}

void motor::initialize(double resistance , double k_back, double inductance, double k1_const, double field, double damping)
    //Description:      Method sets up all values(electrical components) within the DC motor circut
    //Preconditions:    Motor object must be created along with values for inputs
    //Postconditions:   All electical values (resistance, inductance, k constant, magnetic field and damping are set)
    //                  In addition incomming voltage and current also are set to 0 representing motor is off
{
    R_a = resistance;
    L_a = inductance;
    k_b = k_back;
    k_1 = k1_const;
    B = field;

    b = damping;
}

void motor::set_wheel_values(double mass, double length, double hight)
    //Description:      Sets the dimensions and mass of wheel based on a cylander
    //Preconditions:    Motor object must be created
    //Postconditions:   Wheel values set will effect future calculations
{
    wheel.initialize(mass, length, hight);
    wheel.calc_I_0();
}

void motor::update_Voltage(double voltage)
    //Description:      Sets source voltage
    //Preconditions:    Motor object must be created
    //Postconditions:   Incomming voltage is set will effect future calculations
{
    V_a = voltage;
}

void motor::update_inertia(double inertia)
    //Description:      NOTE: FUNCTION IS NOTE INTENDED FOR USE ANYMORE
    //                  Function directly updates inertia
    //Preconditions:    Motor object must be created
    //Postconditions:   Updated inertia
{
    J = inertia;
}

void motor::update_inertia()
    //Description:      Updates Inertia based on orientation of motor and the wheels
    //Preconditions:    Wheel function must have run setting the initial values for wheels
    //Postconditions:   Updates inertia
{
    double x = ori[0];
    double y = ori[1];
    double z = ori[2];
    J = wheel.get_I_0_double(x, y, z);
    cout << "Inertial is: " << J;
}

double motor::state_deriv_get_dI()
    //Description:      Current must be known to find torque. (Electrical Part of DC motor equation) 
    //                  Functions returns rate of change of current (returned value is intended to be put though an integrator to find current)
    //Preconditions:    Values of Voltage, Resistance, K constent, inductance, and magnetic field are set up.
    //Postconditions:   returns the rate of change of current at that point in time
{
    return (V_a - R_a*I_a - k_1*B*w)/(L_a);
}

double motor::state_dirv_getAlpha()
    //Description:      Mechanical part of DC motor Equation.
    //Preconditions:    The current must be set (preferably after integration)
    //                  In addition the k constant, magnetic field, and dampaning, and wheel must be setup
    //Postconditions:   return the angular acceleration (rate of change of angular velocity)
{
    return (k_1*B*I_a - b*w)/(J);
}

void motor::update_I(double current)
    //Description:      Updates current
    //Preconditions:    None
    //Postconditions:   Updated I for future calculations
{
    I_a = current;
}

void motor::update_omega(double omega)
    //Description:      Updates omega (intended to be updated after angular velocity is taken though an integrator)
    //Preconditions:    None
    //Postconditions:   Updated omega for future calculations
{
    w = omega;
}

//[[Set Refrence Position Function]]//
void motor::set_refrence_pos(double r_pos[3])
    //Description:      Initializes position in refrence to satellite position as if it were attached
    //Preconditions:    Array[3]/Eigen Vector3 variable with x, y, z corrdinates relative to satellite center
    //Postconditions:   new refrence position
{
    ref_pos.insert(r_pos[0], r_pos[1], r_pos[2]);
    pos.insert(0, 0, 0);
    pos = pos + ref_pos;
    ref_pos_set = true;
}
void motor::set_refrence_pos(double x, double y, double z)
{
    ref_pos.insert(x, y, z);
    pos.insert(0, 0, 0);
    pos = pos + ref_pos;
    ref_pos_set = true;
}
void motor::set_refrence_pos(Vector3d r_pos)
{
    ref_pos = r_pos;
    pos.insert(0, 0, 0);
    pos = pos + ref_pos;
    ref_pos_set = true;
}
//-----------------------------------//

//[[Set Refrence Orientation Function]]//
void motor::set_refrence_ori(double r_ori[3])
    //Description:      Initializes orientation (direction) in refrence to satellite orientation as if it were attached
    //Preconditions:    Array[3]/Eigen Vector3 with the x, y, z corrdinates representing direction relative to satellite direction
    //Postconditions:   new refrence orientation
{
    ref_ori.insert(r_ori[0], r_ori[1], r_ori[2]);
    ori.insert(0, 0, 0);
    ori = ori + ref_ori;
    ref_ori_set = true;
}
void motor::set_refrence_ori(double x, double y, double z)
{
    ref_ori.insert(x, y, z);
    ori.insert(0, 0, 0);
    ori = ori + ref_ori;
    ref_ori_set = true;  
}
void motor::set_refrence_ori(Vector3d r_ori)
{
    ref_ori = r_ori;
    ori.insert(0, 0, 0);
    ori = ori + ref_ori;
    ref_ori_set = true;
}
//-----------------------------------//

//[[Update Rotation Matrix Funnction]]//
void motor::update_R_matrix(double R[3][3])
    //Description:      updates rotation matrix. intended to get rotation matrix from satellite body
    //Preconditions:    either 2d 3x3 array or an Eigen matrix representing rotation matrix
    //Postconditions:   updated rotation matrix
{
    R_matrix.insert(R[0][0], R[0][1], R[0][2],
                R[1][0], R[1][1], R[1][2],
                R[2][0], R[2][1], R[2][2]);
    R_matrix_set = true;
}

void motor::update_R_matrix(const Matrix3d& R){
    R_matrix = R;
    R_matrix_set = true;
}
//-----------------------------------//

//[[Update Position Funnction]]//
void motor::update_pos(double ref[3], double R[3][3])
    //Description:      updates global position of thruster based on satellite updated center position
    //Preconditions:    if update_R_matrix is used then preconditions are an array/Eigen vector of the satellite center
    //                  if update_R_matrix is NOT used then requires array/Eigen vector representing satellite center and 3x3array/Eigen matrix for rotation matrix
    //Postconditions:   global position updated
{
    if(R_matrix_set){
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

void motor::update_pos(Vector3d ref, const Matrix3d& R){
    if(R_matrix_set){
        cerr << ORANGE << "Warning: " << RESET << "Rotation matrix was already set during this cycle. Ensure the values are not being set twice.\n \n";
    }
    R_matrix = R;
    pos = ref + R_matrix*ref_pos;
    pos_set = true;
}

void motor::update_pos(double ref[3]){
    if (!R_matrix_set) {
        cerr << ORANGE << "Warning: " << RESET << "Rotation matrix was not updated in this cycle. The function (update_pos) requires the most recent rotation matrix values for proper use.\n \n";
    }
    Vector3d refrence;
    refrence.insert(ref[0], ref[1], ref[2]);
    pos = refrence + R_matrix*ref_pos;
    pos_set = true;
}

void motor::update_pos(Vector3d ref){
    if (!R_matrix_set) {
        cerr << ORANGE << "Warning: " << RESET << "Rotation matrix was not updated in this cycle. The function (update_pos) requires the most recent rotation matrix values for proper use.\n \n";
    }
    pos = ref + R_matrix*ref_pos;
    pos_set = true;
}
//-----------------------------------//

//[[Update Orientation Funnction]]//
void motor::update_ori(double R[3][3])
    //Description:      updates global orienttation of thruster based on satellite updated orientation
    //Preconditions:    if update_R_matrix is NOT used then requires 3x3array/Eigen matrix for rotation matrix. Otherwise None
    //Postconditions:   global orientation updated
{
    if(R_matrix_set){
        cerr << ORANGE << "Warning: " << RESET << "Rotation matrix was already set during this cycle. Ensure the values are not being set twice.\n \n";
    }
    R_matrix.insert(R[0][0], R[0][1], R[0][2],
                R[1][0], R[1][1], R[1][2],
                R[2][0], R[2][1], R[2][2]);
    ori = R_matrix.inverse() * ref_ori;
    ori_set = true;
}

void motor::update_ori(const Matrix3d& R){
    if(R_matrix_set){
        cerr << ORANGE << "Warning: " << RESET << "Rotation matrix was already set during this cycle. Ensure the values are not being set twice.\n \n";
    }
    R_matrix = R;
    ori = R_matrix.inverse() * ref_ori;
    ori_set = true;
}

void motor::update_ori(){
    if (!R_matrix_set) {
        cerr << ORANGE << "Warning: " << RESET << "Rotation matrix was not updated in this cycle. The function (update_ori) requires the most recent rotation matrix values for proper use."<< '\n' << '\n';
    }
    R_matrix.inverse() * ref_ori;
    ori_set = true;
}
//-----------------------------------//

//[[Update Position Orientation Funnction]]//
void motor::update_pos_ori(double ref[3], double R[3][3]){
    if(R_matrix_set){
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

void motor::update_pos_ori(Vector3d ref, const Matrix3d& R){
    if(R_matrix_set){
        cerr << ORANGE << "Warning: " << RESET << "Rotation matrix was already set during this cycle. Ensure the values are not being set twice.\n \n";
    }
    R_matrix = R;

    pos = ref + R_matrix*ref_pos;
    ori = R_matrix.inverse() * ref_ori;
    R_matrix_set = true;
    pos_set = true;
    ori_set = true;
}

void motor::update_pos_ori(double ref[3]){
    if (!R_matrix_set) {
        cerr << ORANGE << "Warning: " << RESET << "Rotation matrix was not updated in this cycle. The function (update_ori) requires the most recent rotation matrix values for proper use."<< '\n' << '\n';
    }
    Vector3d refrence;
    refrence.insert(ref[0], ref[1], ref[2]);

    pos = refrence + R_matrix*ref_pos;
    ori = R_matrix.inverse() * ref_ori;
    pos_set = true;
    ori_set = true;
}

void motor::update_pos_ori(Vector3d ref){
    if (!R_matrix_set) {
        cerr << ORANGE << "Warning: " << RESET << "Rotation matrix was not updated in this cycle. The function (update_ori) requires the most recent rotation matrix values for proper use." << '\n' << '\n';
    }
    pos = ref + R_matrix*ref_pos;
    ori = R_matrix.inverse() * ref_ori;
    pos_set = true;
    ori_set = true;
}
//-----------------------------------//

//[[Get Position Function]]//
void motor::get_pos(double return_pos[3])
    //Description:      gets position of satellite
    //Preconditions:    None
    //Postconditions:   reciver gets the position array/Eigen vector
{
    return_pos[0] = pos[0];
    return_pos[1] = pos[1];
    return_pos[2] = pos[2];
}

Vector3d motor::get_pos(){
    return pos;
}
//-----------------------------------//

//[[Get Orientation Function]]//
void motor::get_ori(double return_ori[3])
    //Description:      gets orientation of satellite
    //Preconditions:    None
    //Postconditions:   reciver gets the orientation array/Eigen vector
{
    return_ori[0] = ori[0];
    return_ori[1] = ori[1];
    return_ori[2] = ori[2];
}

Vector3d motor::get_ori(){
    return ori;
}
//-----------------------------------//

double motor::get_torque()
    //Description:      Gets the torque caused by the motor
    //Preconditions:    Both the current and angular velocity values should been updated
    //Postconditions:   Recives the calculated torque for those said current and angular velocity values
{
    T_m = k_1*B*I_a;
    T_r = b*w;
    return T_m - T_r;
}

void motor::get_torque_vec(double torque[3])
    //Description:      Gets the torque as a vector represented as an array caused by the motor
    //Preconditions:    Both the current and angular velocity values should been updated
    //Postconditions:   Recives the calculated torque as a vector for those said current and angular velocity values
{
    if (!ori_set) {
        cerr << ORANGE << "Warning: " << RESET << "orientation was not updated in this cycle. The function (get_torc_vec) requires the most recent rotation matrix values for proper use." << '\n' << '\n';
    }
    T_m = k_1*B*I_a;
    T_r = b*w;
    double T_net = T_m - T_r;
    Vector3d unitOri = ori.normalized();
    Vector3d torque_vec = T_net * unitOri;
    torque[0] = torque_vec[0];
    torque[1] = torque_vec[1];
    torque[2] = torque_vec[2];
}

Vector3d motor::get_torque_vec()
    //Description:      Gets the torque as a vector represented as an array caused by the motor
    //Preconditions:    Both the current and angular velocity values should been updated
    //Postconditions:   Recives the calculated torque as a vector for those said current and angular velocity values
{
    if (!ori_set) {
        cerr << ORANGE << "Warning: " << RESET << "orientation was not updated in this cycle. The function (get_torc_vec) requires the most recent rotation matrix values for proper use." << '\n' << '\n';
    }
    T_m = k_1*B*I_a;
    T_r = b*w;
    double T_net = T_m - T_r;
    Vector3d unitOri = ori.normalized();
    Vector3d torque_vec = T_net * unitOri;
    return torque_vec;
}

void motor::reset_flags(){
    R_matrix_set = false;
    pos_set = false;
    ori_set = false;
}