/*
PURPOSE:    Simulates a force and torque tracker that accumulates
            the net forces and torques acting on a rigid body.
            This class allows for the addition of forces and torques 
            applied at specified positions and computes the resulting 
            net force and torque. It also provides a method to reset 
            the accumulated values for reinitialization.

TERMS USED:
    -> NetForce - the total force acting on the body
    -> NetTorque - the total torque acting on the body
*/

#ifndef FORCE_TORQUE_TRACKER_HH
#define FORCE_TORQUE_TRACKER_HH

#include "../src/Linear_Algebra.cpp"

//#include "../../../Lib/eigen-3.4.0/Eigen/Dense"

//using namespace Eigen;

#ifdef __cplusplus
	extern "C"
	{
#endif

class force_torque_tracker {
	public:
    	// Description: Default constructor initializes net force and torque to zero
    	force_torque_tracker();
    	
    	// Description: Adds a force vector applied at a specific position
    	void addForce(double /*Force_x*/, double /*Force_y*/, double /*Force_z*/, 
                    double /*pos_x*/, double /*pos_y*/, double /*pos_z*/);
    	
		void addForce(const double[3], const double[3]);

    	// Description: Adds a torque vector to the system
    	void addTorque(double /*Torque_x*/, double /*Torque_y*/, double /*Torque_z*/);
    	
		void addTorque(const double[3]);
		
    	// Description: Retrieves the net force acting on the body
    	void getNetForce(double[3] /*Force_net*/);
    	
    	// Description: Retrieves the net torque acting on the body
    	void getNetTorque(double[3] /*Torque_net*/);
    	
    	// Description: Resets the accumulated net force and torque to zero
    	void resetValues();
    	
	private:
    	Vector3d NetForce; // Accumulated net force acting on the body
    	Vector3d NetTorque; // Accumulated net torque acting on the body
};

#ifdef __cplusplus
	}
#endif

#endif
