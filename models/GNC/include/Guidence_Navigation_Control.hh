/*
PURPOSE:    This module provides a modle for the motor. Though the motor could be used
            in any situation it was designed to model the rotation caused by the control
            wheels.

NOTE:       This file gives a summary of each group of functions. See hall_thruster.cpp for more
            detailed description of implementation.
TERMS USED:
    -> (k1) k1 constant
*/

#include "gyroscope.hh"
#include "accelerometer.hh"
//#include "../../../Lib/eigen-3.4.0/Eigen/Dense"
//#include "../../../Lib/eigen-3.4.0/Eigen/Geometry"

#ifndef GNC_HH
#define GNC_HH

#ifdef __cplusplus
    extern "C"
    {
#endif

class GNC {
    public:
        //Description: Creates a motor with no initial values
        GNC();
        //Simulation of the spring system
        void state_deriv_getAccelerometerSpringAccel(double&, double&, double&);

        void state_deriv_getGyroAccel_drive(double&, double&, double&);
        void state_deriv_getGyroAccel_sense(double&, double&, double&);

        void update_Accelerometer_position(double, double, double);
        void update_Accelerometer_velocity(double, double, double);

        void update_Gyro_position_drive(double, double, double);
        void update_Gyro_position_sense(double, double, double);
        void update_Gyro_velocity_drive(double, double, double);
        void update_Gyro_velocity_sense(double, double, double);

        //Reciving values from the accelerometer and gyroscope
        void state_deriv_getAccel(double&, double&, double&);
        void state_deriv_get_omega(double&, double&, double&);

        void update_calc_pos(double, double, double);
        void update_calc_vel(double, double, double);
        void update_calc_ori_by_theta(double);

        void get_calc_pos(double[3]);
        void get_calc_vel(double[3]);
        void get_calc_ori(double[3]);

    private:
        accelerometer Accel[3];
        gyroscope Gyro[3];
};

#ifdef __cplusplus
    }
#endif

#endif