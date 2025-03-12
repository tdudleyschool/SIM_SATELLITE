/********************************* TRICK HEADER *******************************
PURPOSE: (Simulate A Motor)
LIBRARY DEPENDENCY:
    ((cilestial_body.o))
*******************************************************************************/

/*
PURPOSE:    This is the implementation of file 'cilestial_body.hh'

NOTE:       Represents a celestial body with properties such as mass, radius, and position in 3D space.

TERMS USED:
    -> mass - mass of the celestial body
    -> radius - radius of the celestial body
    -> x - position vector of the celestial body in 3D space
    -> v - velocity vector of the celestial body
    -> a - acceleration vector of the celestial body

EQUATIONS TO MODEL:
    -> a = F / m
*/

#include <iostream>
//#include "../../../Lib/eigen-3.4.0/Eigen/Dense"
#include "../include/cilestial_body.hh"

cilestial_body::cilestial_body()
    //Description:    Default constructor initializing the position of the celestial body to the origin.
    //Preconditions:  None
    //Postconditions: Position (x) is set to (0, 0, 0).
{
    x.insert(0, 0, 0);
    mass = 1;
}

cilestial_body::cilestial_body(double m, double r)
    //Description:    Constructor initializing the mass and radius of the celestial body based on provided values.
    //Preconditions:  mass (m) and radius (r) must be greater than zero.
    //Postconditions: Mass and radius are set.
{
    mass = m;
    radius = r;
}

void cilestial_body::initialize(double m, double r)
    //Description:    Initializes the mass and radius of the celestial body.
    //Preconditions:  mass (m) and radius (r) must be greater than zero.
    //Postconditions: Mass and radius are updated.
{
    mass = m;
    radius = r;
}

void cilestial_body::set_position(double pos[3])
    //Description:    Sets the position of the celestial body based on the provided coordinates.
    //Preconditions:  Position array (pos) must contain three valid values.
    //Postconditions: Position (x) is updated.
{
    x.insert(pos[0], pos[1], pos[2]);
}

void cilestial_body::set_relative_pos(double reference[3], double pos[3])
    //Description:    Sets the position of the celestial body relative to a reference position.
    //Preconditions:  Reference array (reference) and position array (pos) must contain three valid values.
    //Postconditions: Position (x) is updated based on the relative position.
{
    x.insert(reference[0] + pos[0], reference[1] + pos[1], reference[2] + pos[2]);
}

void cilestial_body::state_deriv_get_accel(const double F[3], double acel[3])
    //Description:    Calculates the acceleration of the celestial body based on the applied force.
    //                The returned acceleration is intended to be used in an integration function.
    //Preconditions:  Force array (F) must contain three valid values.
    //Postconditions: Acceleration (a) is computed and stored in the provided array (acel).
{
    Vector3d force;
    force.insert(F[0], F[1], F[2]);
    a = force / mass; // Calculate acceleration using Newton's second law
    acel[0] = a[0];
    acel[1] = a[1];
    acel[2] = a[2];
}

void cilestial_body::update_v(double velocity[3])
    //Description:    Updates the velocity of the celestial body based on the provided velocity vector.
    //Preconditions:  Velocity array (velocity) must contain three valid values.
    //Postconditions: Velocity (v) is updated.
{
    v.insert(velocity[0], velocity[1], velocity[2]);
}

void cilestial_body::update_pos(double pos[3])
    //Description:    Updates the position of the celestial body based on the provided position vector.
    //Preconditions:  Position array (pos) must contain three valid values.
    //Postconditions: Position (x) is updated.
{
    x.insert(pos[0], pos[1], pos[2]);
}

// Description: Retrieves the mass of the celestial body
double cilestial_body::get_mass() {
    return mass;
}

// Description: Retrieves the radius of the celestial body
double cilestial_body::get_radius() {
    return radius;
}

// Description: Retrieves the current position of the celestial body
void cilestial_body::get_pos(double pos[3]) {
    pos[0] = x(0);
    pos[1] = x(1);
    pos[2] = x(2);
}
