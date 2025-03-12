/*
PURPOSE:    Simulates a satellite box model representing its physical properties
            including mass, volume, and inertial characteristics.

NOTE:       This class allows for the initialization and updating of the box's
            position and orientation in 3D space based on its mass and side length.

TERMS USED:
    -> m - mass of the satellite box
    -> V - volume of the satellite box
    -> p - density of the satellite box
    -> l - length of the side of the satellite box
*/

#include "../../Recources/src/Linear_Algebra.cpp"

//#include "../../../Lib/eigen-3.4.0/Eigen/Dense"
#include "../../structs.hh"

#ifndef SATELLITE_BOX_HH
#define SATELLITE_BOX_HH

#ifdef __cplusplus
	extern "C"
	{
#endif

//using namespace Eigen;

class satellite_box {
	public:
    	// Description: Default constructor creating a satellite box object
    	satellite_box();
    	
    	// Description: Constructor initializing a satellite box with given mass and side size
    	satellite_box(double mass, double side_size);
    	
    	// Description: Initializes the satellite box's mass and side length
    	void initialize(double /*mass*/, double /*side_size*/);
    	
    	// Description: Initializes the corner points of the satellite box in 3D space
    	void initialize_points(double /*x*/, double /*y*/, double /*z*/);
    	
    	// Description: Updates the state of the satellite box based on center position and orientation
    	void state_update(const double[3] /*center position*/, const double[3][3] /*center orientation*/);
    	
    	// Description: Calculates the inertial matrix of the satellite box
    	void calc_I_0();
    	
    	// Description: Retrieves the inertial matrix of the satellite box
    	void get_I_0(double[3][3] /*inertial matrix*/);
    	
    	// Description: Retrieves the inverse inertial matrix of the satellite box
    	void get_inv_I_0(double[3][3] /*inverse inertial matrix*/);
    	
    	// Description: Returns the mass of the satellite box
    	double getmass();
    	
    	// Description: Fills an array with the corner points of the satellite box
    	void get_points(point[8] /*points*/);

    	// Alternate functions for outside program to directly integrate with Eigen
	private:
    	// Linear Motion
    	Vector3d X[8]; // position, 8 points can model a box about the center
    	double m; // mass of the satellite box
    	double V; // volume of the satellite box
    	double p; // density of the satellite box
    	double l; // length of the side of the satellite box

    	Matrix3d I_0; // inertial matrix of the satellite box
    	Matrix3d  inv_I_0; // inverse inertial matrix of the satellite box
};

#ifdef __cplusplus
	}
#endif

#endif
