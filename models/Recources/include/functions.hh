/*
PURPOSE:    This header file defines a collection of mathematical functions 
            used for physics simulations involving gravitational forces, 
            directional calculations, and collision detection between 
            rays and spheres. It utilizes the Eigen library for vector 
            and matrix operations.
*/

#ifndef FUNCTIONS_HH
#define FUNCTIONS_HH

#include "../src/Linear_Algebra.cpp"

//#include "../../../Lib/eigen-3.4.0/Eigen/Dense"

//using namespace Eigen;

// Description: Calculates the gravitational force magnitude between two masses 
//              given their masses and positions in 3D space.
double gravForceMagnitude(double /*mass1*/, double /*mass2*/, const double[3] /*position1*/, const double[3] /*position2*/);

// Description: Computes the unit direction vector from position 1 to position 2.
void getUnitDir(const double[3] /*position 1*/, const double[3] /*position 2*/, double[3] /*resultant unit vector*/);

// Description: Determines whether a ray intersects with a sphere using 
//              the sphere's radius, center, ray origin, and direction.
bool collisionRaySphere(double /*radius*/, Vector3d /*sphere center*/, Vector3d /*ray origin*/, Vector3d /*ray direction*/);

// Description: Calculates the unit direction vector between two given vectors.
Vector3d getUnitDir(Vector3d /*vector 1*/, Vector3d /*vector 2*/);

// Description: Calculates the gravitational force between 2 masses and returns force vetors at each location for reach object
void get_Grav_Force(const double /*mass1*/, const double[3] /*pos. 1*/ ,const double /*mass2*/, const double[3] /*pos2*/, double[3] /*Force m1*/, double[3] /*Force m2*/);

double get_Grav_Force_Mag(const double /*mass1*/, const double[3] /*pos. 1*/ ,const double /*mass2*/, const double[3] /*pos2*/);

#endif
