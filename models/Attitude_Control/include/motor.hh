/*
PURPOSE:    This module provides a modle for the motor. Though the motor could be used
            in any situation it was designed to model the rotation caused by the control
            wheels.

NOTE:       This file gives a summary of each group of functions. See hall_thruster.cpp for more
            detailed description of implementation.
TERMS USED:
    -> (k1) k1 constant
    -> resistance equates to electrical resistance within motor DC circut
*/

#include "control_wheels.hh"
#include "../../Recources/src/Linear_Algebra.cpp"
//#include "../../../Lib/eigen-3.4.0/Eigen/Dense"
//#include "../../../Lib/eigen-3.4.0/Eigen/Geometry"

#ifndef MOTOR_HH
#define MOTOR_HH

#ifdef __cplusplus
    extern "C"
    {
#endif

class motor {
	public:
    	//Description: Creates a motor with no initial values
    	motor();

    	//Description: Creates a motor with all initial values
    	motor(double/*resistance*/, double/*inductance*/, double /*back k*/, double/*k1_const*/, double/*magnetic_field*/, double/*damping*/);

    	//Description: Initializes all values required to run the motor
    	void initialize(double/*resistance*/,  double /*back k*/, double/*inductance*/, double/*k1_const*/, double/*magnetic_field*/, double/*damping*/);
    	void set_wheel_values(double /*mass*/, double /*length*/, double /*hight*/);
    	//Description: Updates the voltage whitch can control if motor is running, reverses, or is off
    	void update_Voltage(double);

    	//Description: Updates the inertia whitch can be used for calculating the the torque.
    	void update_inertia(double);
    	void update_inertia();

    	//[[RATE OR CHANGES]]//
    	//Description: Gets the rate of change of current. Returned value should be implemented with and integrator function
    	double state_deriv_get_dI();

    	//Description: Gets the rate of change of angular velocity. Returned value should be implemented with an integrator function
    	double state_dirv_getAlpha();
    	//------------------//

    	//[[INTEGRATED VALUES]]//
    	//Description: Updates the current after the rate of change of current is integrated
    	void update_I(double);
    	//Description: Updates the angular velocity after the angular acceleration is integrated
    	void update_omega(double);
    	//--------------------//

    	//[[Functions That Control Position]]//
    	//Description: Sets position in refrence to satellite
    	void set_refrence_pos(double[3]);
    	void set_refrence_pos(double /*x*/, double /*y*/, double/*z*/);
    	void set_refrence_pos(Vector3d);

    	//Description: Sets orientation in refrence to satellite
    	void set_refrence_ori(double[3]);
    	void set_refrence_ori(double /*x*/, double /*y*/, double/*z*/);
    	void set_refrence_ori(Vector3d);

    	//Description: Sets rotation matrix up for calculating position and orientation
    	void update_R_matrix(double[3][3]);
    	void update_R_matrix(const Matrix3d&);

    	//Description: Updates global position of motor
    	void update_pos(double[3], double[3][3]);
    	void update_pos(Vector3d, const Matrix3d&);
    	void update_pos(double[3]);
    	void update_pos(Vector3d);

    	//Description: Updates global orientation of thruster
    	void update_ori(double[3][3]);
    	void update_ori(const Matrix3d&);
    	void update_ori();

    	//Description: Updates both global position and orientation (more efficient then just doing former functions separatly)
    	void update_pos_ori(double[3], double[3][3]);
    	void update_pos_ori(Vector3d, const Matrix3d&);
    	void update_pos_ori(double[3]);
    	void update_pos_ori(Vector3d);

    	//Description: Returns the global position of the thruster
    	void get_pos(double[3]);
    	Vector3d get_pos();
   	 
    	//Description: Returns the global orientation of the thruster
    	void get_ori(double[3]);
    	Vector3d get_ori();
    	//----------------------//

    	//Description: Gets the torque magnitude.
    	double get_torque();

    	//Description: Gets the torque vector
    	void get_torque_vec(double[3]);
    	Vector3d get_torque_vec();

    	//Description: Resets warning flags used in the module
    	void reset_flags();
	private:
    	//circut model
    	double V_a; //voltage supplied though wires
    	double R_a; //resistance in circut
    	double L_a; //inductance
    	double k_1; //constent 2
    	double B; //magnetic field

    	double k_b; //constent 1

        	//Find this value
    	double I_a; //current into circut


    	//mechanical model
    	control_wheels wheel;
    	double b; //dampaning coefficient
    	double J; //moment of inertia

        	//Find
    	double w; //omega
    	double T_m; //torque caused by motor
    	double T_r; //resistance torque of load

    	//motor body
    	Vector3d ref_pos; //position in refrence to another spot
    	Vector3d ref_ori; //orientation in refrence to another spot
    	Vector3d pos; //position of thruster
    	Vector3d ori; //orientation of thruster
    	Matrix3d R_matrix;

    	//flags
    	bool ref_pos_set;
    	bool ref_ori_set;
    	bool R_matrix_set;
    	bool pos_set;
    	bool ori_set;
};



#ifdef __cplusplus
    }
#endif

#endif