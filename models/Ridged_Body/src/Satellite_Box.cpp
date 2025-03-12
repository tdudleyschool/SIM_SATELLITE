/********************************* TRICK HEADER *******************************
PURPOSE: (Simulate A Motor)
LIBRARY DEPENDENCY:
    ((Satellite_Box.o))
*******************************************************************************/
/*
PURPOSE:    This is the implementation of file 'Satellite_Box.hh'

NOTE:       Models a satellite as a cubic box with mass properties, point locations, and inertia tensor calculations.

TERMS USED:
    -> m - mass of the satellite
    -> l - side length of the satellite's cubic shape
    -> V - volume of the satellite box
    -> p - density (mass/volume)
    -> X - coordinates of the satellite's corner points
    -> I_0 - inertia tensor matrix at rest
    -> inv_I_0 - inverse of the inertia tensor matrix at rest

EQUATIONS TO MODEL:
    -> V = l^3
    -> p = m/V
    -> I_0 = (1/12) * m * (l^2 + l^2) * Identity matrix (since cube)
*/

#include "../include/Satellite_Box.hh"
//#include "../../../Lib/eigen-3.4.0/Eigen/Dense"
#include <cmath>
#include <iostream>
//using namespace Eigen;
#include <initializer_list>

//index => 0->x, 1->y, 2->z
satellite_box::satellite_box()
    //Description:    Default constructor initializing mass, length, volume, and density to zero.
    //Preconditions:  None
    //Postconditions: All parameters set to zero.
{
    m = 400;
    l = 0.0;
    V = 0.0;
    p = 0.0;
}

satellite_box::satellite_box(double mass, double side_size)
    //Description:    Constructor initializing mass, side length, volume, and density based on provided values.
    //Preconditions:  mass and side_size must be greater than zero.
    //Postconditions: Volume and density are calculated.
{
    m = mass;
    l = side_size;
    V = l * l * l;
    p = m / V;
}

void satellite_box::initialize(double mass, double side_size)
    //Description:    Initializes the satellite's mass, side length, volume, and density.
    //Preconditions:  mass and side_size must be greater than zero.
    //Postconditions: Volume and density are recalculated.
{
    m = mass;
    l = side_size;
    V = l * l * l;
    p = m / V;
}

void satellite_box::initialize_points(double x, double y, double z)
    //Description:    Initializes the corner points of the satellite's cubic structure based on the center position.
    //Preconditions:  The mass and side length must be initialized.
    //Postconditions: Corner points are set with respect to the center.
{
    double half = l / 2.0;
    Vector3d cent_pos;
    cent_pos.insert(x, y, z);
    
    // Initialize all points
    for (int i = 0; i < 8; i++) {
        X[i][0] = cent_pos[0] + pow(-1, ceil(i / 2.0)) * half;
        X[i][1] = cent_pos[1] + pow(-1, floor(i / 2.0)) * half;
        X[i][2] = cent_pos[2] + pow(-1, ceil((i + 1) / 4.0)) * half;
    }
}

void satellite_box::state_update(const double c[3], const double c_ori[3][3])
    //Description:    Updates the satellite's corner points based on the central position and orientation matrix.
    //Preconditions:  The central position and orientation must be provided.
    //Postconditions: Corner points are updated.
{
    Vector3d cent_pos;
    cent_pos.insert(c[0], c[1], c[2]);

    Matrix3d  cent_orientation;
    cent_orientation.insert(c_ori[0][0], c_ori[0][1], c_ori[0][2],
                        c_ori[1][0], c_ori[1][1], c_ori[1][2],
                        c_ori[2][0], c_ori[2][1], c_ori[2][2]);

    for (int i = 0; i < 8; i++) {
        Vector3d pos_local = cent_pos - X[i];
        X[i] = cent_pos + cent_orientation * pos_local;
    }
}

void satellite_box::calc_I_0()
    //Description:    Calculates the inertia tensor for the satellite box at rest.
    //Preconditions:  The mass and side length must be initialized.
    //Postconditions: Inertia tensor (I_0) and its inverse (inv_I_0) are calculated.
{
    I_0.insert(1.0 / 12.0 * m * ((l * l) + (l * l)), 0, 0,
           0, 1.0 / 12.0 * m * ((l * l) + (l * l)), 0,
           0, 0, 1.0 / 12.0 * m * ((l * l) + (l * l)));

    inv_I_0 = I_0.inverse();
}

void satellite_box::get_I_0(double inert[3][3])
    //Description:    Returns the inertia tensor I_0 to the user in a 2D array format.
    //Preconditions:  Inertia tensor must be calculated.
    //Postconditions: I_0 is provided in a 3x3 array.
{
    for (int i = 0; i < 3; i++) {
        inert[i][0] = I_0(i, 0);
        inert[i][1] = I_0(i, 1);
        inert[i][2] = I_0(i, 2);
    }
}

void satellite_box::get_inv_I_0(double inv_Inert[3][3])
    //Description:    Returns the inverse inertia tensor inv_I_0 to the user in a 2D array format.
    //Preconditions:  Inverse inertia tensor must be calculated.
    //Postconditions: inv_I_0 is provided in a 3x3 array.
{
    for (int i = 0; i < 3; i++) {
        inv_Inert[i][0] = inv_I_0(i, 0);
        inv_Inert[i][1] = inv_I_0(i, 1);
        inv_Inert[i][2] = inv_I_0(i, 2);
    }
}

double satellite_box::getmass()
    //Description:    Returns the mass of the satellite.
    //Preconditions:  The mass must be initialized.
    //Postconditions: The mass is returned.
{
    return m;
}

void satellite_box::get_points(point points[8])
    //Description:    Returns the corner points of the satellite box.
    //Preconditions:  Points must be initialized.
    //Postconditions: Points are provided in an array of structures.
{
    for (int i = 0; i < 8; i++) {
        points[i].x = X[i][0];
        points[i].y = X[i][1];
        points[i].z = X[i][2];
    }
}
