/*
PURPOSE:    This is a simplified representation of the control wheels that are attached to a dc motor.
            Wheel primary goal is to calculate the inertial values of a wheel based on it's cylyndrical
            shape within 3d space when given vector to rotate around.
*/

//#include "../../../Lib/eigen-3.4.0/Eigen/Dense"
#include <iostream>

#include "../../structs.hh"
#include "../../Recources/src/Linear_Algebra.cpp"

#ifndef CONTROL_WHEELS_HH
#define CONTROL_WHEELS_HH

#ifdef __cplusplus
    extern "C"
    {
#endif


//using namespace Eigen;

class control_wheels {
    public:
        //Description: Creates a wheel with default values.
        control_wheels();
        
        //Description: Creates a wheel with user defined values for mass, cylander length, and cylander hight
        control_wheels(double/*mass*/, double/*length*/, double/*hight*/);

        //Description: Resets essential wheel values for mass, cylander, and hight
        void initialize(double, double, double);

        //Description: Calculates the inertial matrix based on the mass, hight, and length values
        void calc_I_0();

        //Description: Returns the inertial matrix as a 3x3 array
        void get_I_0(double[3][3] /*inertial matrix*/);

        //Description: Returns the inverse inertial matrix as a 3x3 array
        void get_inv_I_0(double[3][3] /*inertial matrix*/);

        //Description: Returns the inertial scalar value based on the direction of rotation of wheels
        double get_I_0_double(double /*x*/, double /*y*/, double /*z*/);

        //Description: Returns the inverse inertial scalar value based on the direction of rotation of wheels
        double get_inv_I_0_double(double /*x*/, double /*y*/, double/*z*/);

        //Description: Returns the mass of wheel
        double getmass();

    private:
        //Linear Motion
        double m; //mass
        double V; //volume
        double p; //density
        double r; //length
        double h; //hight

        Matrix3d I_0; //inertial matrix
        Matrix3d inv_I_0; //inverse inertial matrix
        Matrix3d R_z; //rotation matrix to orient in z
};

#ifdef __cplusplus
    }
#endif

#endif