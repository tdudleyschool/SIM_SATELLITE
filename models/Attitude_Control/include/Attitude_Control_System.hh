/********************************* TRICK HEADER *******************************
PURPOSE: (Attitude Control System)
*******************************************************************************/

/*
PURPOSE:    This module provides a modle for the motor. Though the motor could be used
            in any situation it was designed to model the rotation caused by the control
            wheels.

NOTE:       This file gives a summary of each group of functions. See hall_thruster.cpp for more
            detailed description of implementation.
TERMS USED:
    -> (k1) k1 constant
*/

#include "motor.hh"
#include "h-bridge.hh"
#include "../../Recources/src/Linear_Algebra.cpp"

//#include "../../../Lib/eigen-3.4.0/Eigen/Dense"
//#include "../../../Lib/eigen-3.4.0/Eigen/Geometry"

//using namespace Eigen;

#ifndef ATTITUDE_CONTROL_SYSTEM_HH
#define ATTITUDE_CONTROL_SYSTEM_HH

#ifdef __cplusplus
    extern "C"
    {
#endif

class Attitude_Control_System {
    public:
        //Description: Creates a motor with no initial values
        Attitude_Control_System();
        Attitude_Control_System(double /*power*/);

        void Initialize_Power(double /*power*/);

        //Description: Updates voltage into motor
        void update_Voltage(int /*index of motor*/, double /*Voltage*/);

        void state_deriv_getALL_dI(double[3]);

        void state_deriv_getALL_Alpha(double[3]);

        //Description: Updates incomming current into motor
        void update_I(int /*index of motor*/, double /*current*/);

        //Description: Turns motor off by setting values of voltage and current to 0
        void motor_off(int /*Index of motor*/);

        //Description: Automatically turns counter on rotating clockwise
        void motor_clock(int /*Index of motor*/, double /*current*/, double /*Voltage*/);
        
        //Description: Automatically turns moter on counter clockwise
        void motor_contClock(int /*Index of motor*/, double /*current*/, double /*Voltage*/);        

        //Description: Updates motor orientation
        void update_all_ori(double[3][3] /*rotation matrix*/);
        void update_all_ori(const Matrix3d& /*rotation matrix*/);

        void updateAll_I(double /*current1*/, double /*current2*/, double /*current3*/);
        void updateAll_I(const double[3] /*currents for diffrent motors*/);

        void updateAll_omega(double /*w1*/, double /*w2*/, double /*w3*/);
        void updateAll_omega(const double[3] /*angular velocity for diffrent motors*/);

        //Description: Reterns torque prduced by all motors to user
        void get_total_torque(double[3] /*returned torque*/);
        Vector3d get_total_torque();

        bool is_motor_on(int);

        double get_drawn_I();
    private:
        motor motors[3];        // Motors
        double motor_I[3];
        double V[3];
        double motor_Pow[3];    // Power of motors
        float motor_on[3];      // Flag noting when motor is on
        double V_polarity[3];      // Polarity Variable either 1 or -1
};

#ifdef __cplusplus
    }
#endif

#endif