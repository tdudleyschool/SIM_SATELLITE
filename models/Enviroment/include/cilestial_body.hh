/*
PURPOSE:    Simulates a celestial body in a three-dimensional space,
            allowing for the management of its mass, radius, position, 
            and velocity. The class provides methods to initialize the 
            body, set its position, calculate accelerations based on 
            applied forces, and update its state over time.

NOTE:       The celestial body can interact with external forces and 
            maintain its position and motion within a simulated 
            environment. The position and velocity are represented 
            using 3D vectors, leveraging the Eigen library for 
            mathematical operations.
*/

#ifndef CILESTIAL_BODY_HH
#define CILESTIAL_BODY_HH

#include "../../Recources/src/Linear_Algebra.cpp"
//#include "../../../Lib/eigen-3.4.0/Eigen/Dense"

//using namespace Eigen;

#ifdef __cplusplus
	extern "C"
	{
#endif

class cilestial_body {
	public:
    	// Description: Default constructor creating a celestial body with default parameters
    	cilestial_body();

    	// Description: Constructor to initialize the celestial body with given mass and radius
    	cilestial_body(double /*mass*/, double /*radius*/);

    	// Description: Initializes the celestial body with specified mass and radius
    	void initialize(double /*mass*/, double /*radius*/);

    	// Description: Sets the position of the celestial body in 3D space
    	void set_position(double[3] /*position*/);

    	// Description: Sets the relative position of the celestial body based on a reference position
    	void set_relative_pos(double[3] /*reference position*/, double[3] /*body position*/);

    	// Description: Calculates the acceleration of the body based on applied force
    	void state_deriv_get_accel(const double[3] /*Force*/, double[3] /*Acceleration*/);

    	// Description: Updates the velocity of the celestial body
    	void update_v(double[3] /*velocity*/);

    	// Description: Updates the position of the celestial body
    	void update_pos(double[3] /*position*/);

    	// Description: Retrieves the mass of the celestial body
    	double get_mass();

    	// Description: Retrieves the radius of the celestial body
    	double get_radius();

    	// Description: Retrieves the current position of the celestial body
    	void get_pos(double[3] /*position*/);

	private:
    	double mass; // Mass of the celestial body
    	double radius; // Radius of the celestial body

    	Vector3d x; // Position vector of the celestial body
    	Vector3d v; // Velocity vector of the celestial body
    	Vector3d a; // Acceleration vector of the celestial body
};

#ifdef __cplusplus
	}
#endif

#endif
