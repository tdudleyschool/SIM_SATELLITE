#include "../../../Lib/eigen-3.4.0/Eigen/Dense"
#include "../include/control_wheels.hh"
#include <iostream>

control_wheels::control_wheels()
    //Description:      Default constructor setting rotation up pointing in the z direction
    //Preconditions:    None
    //Postconditions:   Wheel object created
{
    R_z << 0, 0, 1,
           0, 1, 0,
           1, 0, 0;
}

control_wheels::control_wheels(double mass, double radius, double hight)
    //Description:      Constructor up the rotation matrix, mass, radius, hight, volume, and density
    //Preconditions:    None
    //Postconditions:   Wheel with is setup with mass, radius, and hight
{
    m = mass;
    r = radius;
    h = hight;

    V = (3.142)*r*r*h;
    p = m/V;

    R_z << 0, 0, 1,
           0, 1, 0,
           1, 0, 0;
}

void control_wheels::initialize(double mass, double radius, double hight)
    //Description:      Method changes the mass, radius, and hight (intended for initialization)
    //Preconditions:    Wheel object must be connected
    //Postconditions:   Wheel with is setup with mass, radius, and hight
{
    m = mass;
    r = radius;
    h = hight;

    V = (3.142)*r*r*h;
    p = m/V;
}

void control_wheels::calc_I_0()
    //Description:      Metod caculates the inertial matrix based on the mass, radius and hight
    //Preconditions:    Wheel object must be initialized with mass, radius and hight
    //Postconditions:   Creates and updates it's own inertial matrix
{
    I_0 << ((m*h*h) / 12)+((m*r*r) / 4), 0, 0,
           0,((m*h*h) / 12)+((m*r*r) / 4), 0,
           0, 0, ((m*r*r) / 2);
    inv_I_0 = I_0.inverse();
}

void control_wheels::get_I_0(double inert[3][3])
    //Description:      Method is used to retrive inertial matrix as a 3x3 array
    //Preconditions:    inertial matrix must be updated
    //Postconditions:   returns current inertial matrix
{
    for(int i = 0; i < 3; i++){
        inert[i][0] = I_0(i, 0);
        inert[i][1] = I_0(i, 1);
        inert[i][2] = I_0(i, 2);
    }
}

void control_wheels::get_inv_I_0(double inv_Inert[3][3])
    //Description:      Method is used to retrive inverse inertial matrix as a 3x3 array
    //Preconditions:    inertial matrix must be updated
    //Postconditions:   returns current inverse inertial matrix
{
        for(int i = 0; i < 3; i++){
        inv_Inert[i][0] = inv_I_0(i, 0);
        inv_Inert[i][1] = inv_I_0(i, 1);
        inv_Inert[i][2] = inv_I_0(i, 2);
    }
}

double control_wheels::get_I_0_double(double x, double y, double z)
    //Description:      Method is used to retrive inertial value as a scalar based on direction it is pointing to
    //Preconditions:    inertial matrix must be updated
    //Postconditions:   returns current inertial value as scalar
{
    Vector3d dir_wheel;
    dir_wheel << x, y, z;
    dir_wheel = R_z * dir_wheel;
    return dir_wheel[0]*I_0(0,0) + dir_wheel[1]*I_0(1,1) + dir_wheel[2]*I_0(2,2);
}

double control_wheels::get_inv_I_0_double(double x, double y, double z)
    //Description:      Method is used to retrive inverse inertial value as a scalar based on direction it is pointing to
    //Preconditions:    inertial matrix must be updated
    //Postconditions:   returns current inverse inertial value as scalar
{
    return x * inv_I_0(0,0) + y * inv_I_0(1,1) + z*inv_I_0(2,2);
}

double control_wheels::getmass()
    //Description:      Method is used to retrive mass of wheel
    //Preconditions:    wheel object must be created
    //Postconditions:   returns mass
{
    return m;
}