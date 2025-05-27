/********************************* TRICK HEADER *******************************
PURPOSE: (Attitude Control System)
LIBRARY DEPENDENCY:
    ((Attitude_Control_System.o)
     (motor.o)
     (h-bridge.o)
     (control_wheels.o))
*******************************************************************************/


#include "../include/Attitude_Control_System.hh"

Attitude_Control_System::Attitude_Control_System()
    //Description:      Initializes attitude control system variables with difaut values
    //Precondition:     None
    //Postconditions:   All motors are initialized with zero and power is initialized with 10
{
    for (int i = 0; i < 3; i++){
        motor_on[i] = false;
        motor_Pow[i] = 10.0;
        V_polarity[i] = 1;
        motors[i].initialize(1.0, 0.05, 0.1, 0.1, 1.0, 0.1);
        motors[i].set_wheel_values(120, 8, 4);
    }

    motors[0].set_refrence_pos(0, 0, 0);
    motors[1].set_refrence_pos(0, 0, 0);
    motors[2].set_refrence_pos(0, 0, 0);

    motors[0].set_refrence_ori(1, 0, 0); //along x
    motors[1].set_refrence_ori(0, 1, 0); //along y
    motors[2].set_refrence_ori(0, 0, 1); //along z
}

Attitude_Control_System::Attitude_Control_System(double power)
    //Description:      Initializes attitude control system with variables. Additinally power of thruster is selected by user
    //Precondition:     Power
    //Postconditions:   All motors are initialized with zero and power is initialized with inputed value
{
    for (int i = 0; i < 3; i++){
        motor_I[i] = 0.0;
        V[i] = 0.0;
        motor_on[i] = false;
        motor_Pow[i] = power;
        V_polarity[i] = 1;
    }

    motors[0].set_refrence_pos(0, 0, 0);
    motors[1].set_refrence_pos(0, 0, 0);
    motors[2].set_refrence_pos(0, 0, 0);

    motors[0].set_refrence_ori(1, 0, 0); //along x
    motors[1].set_refrence_ori(0, 1, 0); //along y
    motors[2].set_refrence_ori(0, 0, 1); //along z
}

void Attitude_Control_System::Initialize_Power(double power)
{
    for (int i = 0; i < 3; i++){
        motor_Pow[i] = power;
    }

}

void Attitude_Control_System::update_Voltage(int index, double voltage)
    //Description:      Updates voltage for the specified motor using the index value
    //Precondition:     index and voltage
    //Postconditions:   voltage though a motor is updated
{
    if (index >= 0 && index < 3) {
        motors[index].update_Voltage(V_polarity[index]*voltage);
    }
}

void Attitude_Control_System::update_I(int index, double current)
    //Description:      Updates current for the specified motor using the index value
    //Precondition:     index and current
    //Postconditions:   current though a motor is updated
{
    if (index >= 0 && index < 3) {
        motors[index].update_I(V_polarity[index]*current);
    }
}

void Attitude_Control_System::motor_off(int index)
    //Description:      turns motor off by setting voltage and current to 0
    //Precondition:     index
    //Postconditions:   current and voltage though specific motor is zero meaning no torque should flow though
{
    if (index >= 0 && index < 3) {
        motor_on[index] = false;
        motors[index].update_Voltage(0.0);
    }
}

void Attitude_Control_System::state_deriv_getALL_dI(double dI[3])
    //Description:      Electrical component part of DC motor Equation returning dI
    //Preconditions:    The current must be set (preferably after integration)
    //                  In addition the k constant, magnetic field, and dampaning, and wheel must be setup
    //Postconditions:   return the angular acceleration (rate of change of angular velocity)
{
    dI[0] = motors[0].state_deriv_get_dI();
    dI[1] = motors[1].state_deriv_get_dI();
    dI[2] = motors[2].state_deriv_get_dI();
}

void Attitude_Control_System::state_deriv_getALL_Alpha(double alpha[3])
    //Description:      Mechanical part of DC motor Equation returning derivative of w
    //Preconditions:    The current must be set (preferably after integration)
    //                  In addition the k constant, magnetic field, and dampaning, and wheel must be setup
    //Postconditions:   return the angular acceleration (rate of change of angular velocity)
{
    alpha[0] = motors[0].state_dirv_getAlpha();
    alpha[1] = motors[1].state_dirv_getAlpha();
    alpha[2] = motors[2].state_dirv_getAlpha();
}

void Attitude_Control_System::motor_clock(int index, double voltage)

{
    V_polarity[index] = 1;
    motor_on[index] = true;
    motors[index].update_Voltage(V_polarity[index]*voltage);
}

void Attitude_Control_System::motor_contClock(int index, double voltage)

{
    V_polarity[index] = -1;
    motor_on[index] = true;
    motors[index].update_Voltage(V_polarity[index]*voltage);
}

void Attitude_Control_System::update_all_ori(double rotation_matrix[3][3])
    //Description:   Updates the orientation of all motors based on the spacecraft's orientation matrix.
    //Preconditions: Satellite rotation matrix must be provided.
    //Postconditions: motors orientations are updated.
{
    for (int i = 0; i < 3; i++) {
        motors[i].update_ori(rotation_matrix);
    }
}

void Attitude_Control_System::update_all_ori(const Matrix3d& R)
    //Description:   Updates the orientation of all motors based on the spacecraft's orientation matrix.
    //Preconditions: Satellite rotation matrix must be provided.
    //Postconditions: motors orientations are updated.
{
    for (int i = 0; i < 3; i++) {
        motors[i].update_ori(R);
        motors[i].update_inertia();
    }
}

void Attitude_Control_System::updateAll_I(double c1, double c2, double c3){
    motors[0].update_I(c1);
    motors[1].update_I(c2);
    motors[2].update_I(c3);
}
void Attitude_Control_System::updateAll_I(const double currents[3]){
    for (int i = 0; i < 3; i++) {
        motors[i].update_I(currents[i]);
    }
}

void Attitude_Control_System::updateAll_omega(double w1, double w2, double w3){
    motors[0].update_I(w1);
    motors[1].update_I(w2);
    motors[2].update_I(w3);
}
void Attitude_Control_System::updateAll_omega(const double omegas[3]){
    for (int i = 0; i < 3; i++) {
        motors[i].update_omega(omegas[i]);
    }
}

//Description: Reterns torque prduced by all motors to user
void Attitude_Control_System::get_total_torque(double torque[3]){
    Vector3d sumTorque;
    sumTorque.insert(0, 0, 0);
    for (int i =0; i < 3; i++){
        Vector3d addedTorque;
        addedTorque = motors[i].get_torque_vec();
        sumTorque = sumTorque + addedTorque;
    }
    torque[0] = sumTorque[0];
    torque[1] = sumTorque[1];
    torque[2] = sumTorque[2];
}

Vector3d Attitude_Control_System::get_total_torque(){
    Vector3d sumTorque;
    sumTorque.insert(0, 0, 0);
    for (int i =0; i < 3; i++){
        Vector3d addedTorque;
        addedTorque = motors[i].get_torque_vec();
        sumTorque = sumTorque + addedTorque;
    }
    return sumTorque;
}

double Attitude_Control_System::get_drawn_I(){
    double drawnI = 0;
    for (int i = 0; i < 3; i++){
        if (motor_on[i]) {
            drawnI = drawnI + motor_Pow[i]/V[i];
        }
    }
    return drawnI;
}
//ADD on that gets drawn I of a particular motor not all motors.


bool Attitude_Control_System::is_motor_on(int index){
    return motor_on[index];
}