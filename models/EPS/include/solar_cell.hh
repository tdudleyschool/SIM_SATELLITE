/*
PURPOSE:    This class simulates a solar cell that calculates 
            the short circuit current based on the light 
            incident on its surface. It manages the orientation 
            and position of the solar cell in a three-dimensional 
            space and provides methods to lock its axes and update 
            its reference position and orientation.

NOTE:       The solar cell can be oriented in space and will 
            return the short circuit current based on the 
            incoming light ray vector.
*/

#ifndef SOLAR_CELL_HH
#define SOLAR_CELL_HH

//#include "../../../Lib/eigen-3.4.0/Eigen/Dense"
#include "../../Recources/src/Linear_Algebra.cpp"
#define PI 3.14159

//using namespace Eigen;

#ifdef __cplusplus
	extern "C"
	{
#endif

class solar_cell {
	public:
    	// Description: Default constructor initializing the solar cell
    	solar_cell();
    	
    	// Description: Constructor initializing the solar cell with max current
    	solar_cell(double /*max current*/);

    	// Description: Locks the solar cell's orientation along the X-axis
    	void lock_axis_x();

    	// Description: Locks the solar cell's orientation along the Y-axis
    	void lock_axis_y();

    	// Description: Locks the solar cell's orientation along the Z-axis
    	void lock_axis_z();

    	// Description: Sets the reference position using an array
    	void set_refrence_pos(double[3] /*reference position*/);
    	void set_refrence_pos(Vector3d /*reference position*/);
    	
    	// Description: Sets the reference orientation using an array
    	void set_refrence_ori(double[2] /*reference orientation*/);
    	void set_refrence_ori(Vector2d /*reference orientation*/);

    	// Description: Updates the rotation matrix using an array
    	void update_R_matrix(double[3][3] /*Rotation Matrix*/);
    	void update_R_matrix(const Matrix3d& /*Rotation Matrix*/);

    	// Description: Updates position and orientation using reference position and rotation matrix
    	void update_pos_ori(double[3] /*reference position*/, double[3][3] /*Rotation Matrix*/);
    	void update_pos_ori(Vector3d /*reference position*/, const Matrix3d& /*Rotation Matrix*/);
    	void update_pos_ori(double[3] /*reference position*/);
    	void update_pos_ori(Vector3d /*reference position*/);

    	// Description: Rotates the solar cell direction clockwise by theta degrees
    	void rotate_dir_clock(double /*theta*/);
    	
    	// Description: Rotates the solar cell direction counter-clockwise by theta degrees
    	void rotate_dir_contclock(double /*theta*/);
    	
    	// Description: Calculates the short circuit current based on the incoming light ray vector
    	double get_I_sc(double[3] /*light ray vector*/);

	private:
    	// Maximum short circuit current the solar cell can handle
    	double max_I_sc;
    	// Calculated short circuit current based on light conditions
    	double I_sc;
    	// Lock status of the solar cell's axes
    	int lock_axis;

    	// Reference position and orientation vectors
    	Vector3d ref_pos;
    	Vector3d ref_normal_vec;
    	Vector3d pos;
    	Vector3d ori; // Normal to the satellite
    	Matrix3d R_matrix; // Rotation matrix
    	Vector3d normal_vec; // Normal vector of the solar cell plane
};

#ifdef __cplusplus
	}
#endif

#endif
