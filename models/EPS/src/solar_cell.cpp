/*
PURPOSE:    This is the implementation of file 'solar_cell.hh'

NOTE:       Models a solar cell with capabilities for positioning, orientation, and current calculations based on light incidence.

TERMS USED:
    -> max_I_sc - maximum short-circuit current of the solar cell
    -> ref_normal_vec - reference normal vector representing the orientation of the solar cell
    -> lock_axis - axis used to lock the orientation of the normal vector
    -> ref_pos - reference position of the solar cell
    -> R_matrix - rotation matrix used for positioning and orientation transformations
    -> pos - current position of the solar cell
    -> normal_vec - current normal vector of the solar cell
    -> I_sc - short-circuit current based on light incidence

EQUATIONS TO MODEL:
    -> I_sc = max_I_sc * (l_ray · unit_normal_vec) where l_ray is the incident light vector and unit_normal_vec is the normalized reference normal vector
*/

#include "../include/solar_cell.hh"
//#include "../../../Lib/eigen-3.4.0/Eigen/Dense"

using namespace std;
//using namespace Eigen;

solar_cell::solar_cell()
    //Description:    Default constructor initializing the reference normal vector and locking the axis to z.
    //Preconditions:  None
    //Postconditions: Reference normal vector set to (1, 0, 0) and lock_axis set to 2 (z-axis).
{
    ref_normal_vec.insert(1, 0, 0);
    lock_axis = 2;
}

solar_cell::solar_cell(double max_current)
    //Description:    Constructor initializing the maximum short-circuit current and the reference normal vector.
    //Preconditions:  max_current must be a non-negative value.
    //Postconditions: max_I_sc is set, reference normal vector initialized to (1, 0, 0), and lock_axis set to 2 (z-axis).
{
    max_I_sc = max_current;
    ref_normal_vec.insert(1, 0, 0);
    lock_axis = 2;
}

void solar_cell::initialize(double max_current)
    //Description:    Constructor initializing the maximum short-circuit current and the reference normal vector.
    //Preconditions:  max_current must be a non-negative value.
    //Postconditions: max_I_sc is set, reference normal vector initialized to (1, 0, 0), and lock_axis set to 2 (z-axis).
{
    max_I_sc = max_current;
    ref_normal_vec.insert(1, 0, 0);
    lock_axis = 2;
}

void solar_cell::lock_axis_x()
    //Description:    Locks the orientation of the solar cell normal vector to the x-axis.
    //Preconditions:  None
    //Postconditions: lock_axis is set to 1.
{
    lock_axis = 1;
}

void solar_cell::lock_axis_y()
    //Description:    Locks the orientation of the solar cell normal vector to the y-axis.
    //Preconditions:  None
    //Postconditions: lock_axis is set to 2.
{
    lock_axis = 2;
}

void solar_cell::lock_axis_z()
    //Description:    Locks the orientation of the solar cell normal vector to the z-axis.
    //Preconditions:  None
    //Postconditions: lock_axis is set to 3.
{
    lock_axis = 3;
}

void solar_cell::set_refrence_pos(double ref_point[3])
    //Description:    Sets the reference position of the solar cell using an array.
    //Preconditions:  ref_point must contain three valid double values.
    //Postconditions: ref_pos is updated to the new reference position.
{
    ref_pos.insert(ref_point[0], ref_point[1], ref_point[2]);
}

void solar_cell::set_refrence_pos(Vector3d r_pos)
    //Description:    Sets the reference position of the solar cell using a Vector3d object.
    //Preconditions:  r_pos must be a valid Vector3d object.
    //Postconditions: ref_pos is updated to the new reference position.
{
    ref_pos = r_pos;
}

void solar_cell::set_refrence_ori(double ref_o[2])
    //Description:    Sets the reference normal vector based on a 2D orientation while respecting the locked axis.
    //Preconditions:  The lock_axis must be set before calling this function.
    //Postconditions: ref_normal_vec is updated based on the provided orientation.
{
    if(lock_axis == 1)
        ref_normal_vec.insert(0, ref_o[0], ref_o[1]);
    else if (lock_axis == 2)
        ref_normal_vec.insert(ref_o[0], 0, ref_o[1]);
    else
        ref_normal_vec.insert(ref_o[0], ref_o[1], 0);
}

void solar_cell::set_refrence_ori(Vector2d ref_o)
    //Description:    Sets the reference normal vector based on a 2D orientation while respecting the locked axis.
    //Preconditions:  The lock_axis must be set before calling this function.
    //Postconditions: ref_normal_vec is updated based on the provided orientation.
{
    if(lock_axis == 1)
        ref_normal_vec.insert(0, ref_o[0], ref_o[1]);
    else if (lock_axis == 2)
        ref_normal_vec.insert(ref_o[0], 0, ref_o[1]);
    else
        ref_normal_vec.insert(ref_o[0], ref_o[1], 0);
}

void solar_cell::update_R_matrix(double R[3][3])
    //Description:    Updates the rotation matrix of the solar cell from a 2D array.
    //Preconditions:  R must be a valid 3x3 matrix.
    //Postconditions: R_matrix is updated with the new rotation values.
{
    R_matrix.insert(R[0][0], R[0][1], R[0][2],
                R[1][0], R[1][1], R[1][2],
                R[2][0], R[2][1], R[2][2]);
}

void solar_cell::update_R_matrix(const Matrix3d& R)
    //Description:    Updates the rotation matrix of the solar cell from a Matrix object.
    //Preconditions:  R must be a valid 3x3 matrix.
    //Postconditions: R_matrix is updated with the new rotation values.
{
    R_matrix = R;
}

void solar_cell::update_pos_ori(double ref[3], double R[3][3])
    //Description:    Updates the position and orientation of the solar cell based on reference position and rotation matrix.
    //Preconditions:  ref must contain three valid double values and R must be a valid 3x3 matrix.
    //Postconditions: pos and normal_vec are updated based on the new reference and rotation.
{
    Vector3d refrence;

    refrence.insert(ref[0], ref[1], ref[2]);
    R_matrix.insert(R[0][0], R[0][1], R[0][2],
                R[1][0], R[1][1], R[1][2],
                R[2][0], R[2][1], R[2][2]);
    
    pos = refrence + R_matrix * ref_pos;
    normal_vec = R_matrix.inverse() * ref_normal_vec;
}

void solar_cell::update_pos_ori(Vector3d ref, const Matrix3d& R)
    //Description:    Updates the position and orientation of the solar cell based on reference position and rotation matrix.
    //Preconditions:  ref must be a valid Vector3d object and R must be a valid 3x3 matrix.
    //Postconditions: pos and normal_vec are updated based on the new reference and rotation.
{
    R_matrix = R;

    pos = ref + R_matrix * ref_pos;
    normal_vec = R_matrix.inverse() * ref_normal_vec;
}

void solar_cell::update_pos_ori(double ref[3])
    //Description:    Updates the position of the solar cell based on the reference position using the current rotation matrix.
    //Preconditions:  ref must contain three valid double values.
    //Postconditions: pos and normal_vec are updated based on the new reference.
{
    Vector3d refrence;
    refrence.insert(ref[0], ref[1], ref[2]);

    pos = refrence + R_matrix * ref_pos;
    normal_vec = R_matrix.inverse() * ref_normal_vec;
}

void solar_cell::update_pos_ori(Vector3d ref)
    //Description:    Updates the position of the solar cell based on the reference position using the current rotation matrix.
    //Preconditions:  ref must be a valid Vector3d object.
    //Postconditions: pos and normal_vec are updated based on the new reference.
{
    pos = ref + R_matrix * ref_pos;
    normal_vec = R_matrix.inverse() * ref_normal_vec;
}

void solar_cell::rotate_dir_clock(double theta)
    //Description:    Rotates the reference normal vector clockwise by an angle theta.
    //Preconditions:  theta must be a valid angle in degrees.
    //Postconditions: ref_normal_vec is updated according to the clockwise rotation.
{
    double rad = theta * PI / 180.0; // Corrected from PI * 180 to PI / 180
    Vector2d normal_vec_comp;
    Matrix2d R_Matrix2d;

    R_Matrix2d.insert(cos(rad), sin(rad),
                   -sin(rad), cos(rad));
    normal_vec_comp.insert(ref_normal_vec[0], ref_normal_vec[2]);
    normal_vec_comp = R_Matrix2d * normal_vec_comp;

    if(lock_axis == 1)
        ref_normal_vec.insert(0, normal_vec_comp[0], normal_vec_comp[1]);
    else if (lock_axis == 2)
        ref_normal_vec.insert(normal_vec_comp[0], 0, normal_vec_comp[1]);
    else
        ref_normal_vec.insert(normal_vec_comp[0], normal_vec_comp[1], 0);
}

void solar_cell::rotate_dir_contclock(double theta)
    //Description:    Rotates the reference normal vector counterclockwise by an angle theta.
    //Preconditions:  theta must be a valid angle in degrees.
    //Postconditions: ref_normal_vec is updated according to the counterclockwise rotation.
{
    double rad = -theta * PI / 180.0; // Corrected from -PI * 180 to -PI / 180
    Vector2d normal_vec_comp;
    Matrix2d R_Matrix2d;

    R_Matrix2d.insert(cos(rad), sin(rad),
                   -sin(rad), cos(rad));
    normal_vec_comp.insert(ref_normal_vec[0], ref_normal_vec[2]);
    normal_vec_comp = R_Matrix2d * normal_vec_comp;
    
    if(lock_axis == 1)
        ref_normal_vec.insert(0, normal_vec_comp[0], normal_vec_comp[1]);
    else if (lock_axis == 2)
        ref_normal_vec.insert(normal_vec_comp[0], 0, normal_vec_comp[1]);
    else
        ref_normal_vec.insert(normal_vec_comp[0], normal_vec_comp[1], 0);
}

double solar_cell::get_I_sc(double light_ray[3])
    //Description:    Calculates the short-circuit current based on the incident light ray.
    //Preconditions:  light_ray must contain three valid double values representing the direction of the light.
    //Postconditions: Returns the calculated short-circuit current (I_sc).
{
    Vector3d l_ray;
    Vector3d unit_normal_vec;
    unit_normal_vec = ref_normal_vec;
    l_ray.insert(light_ray[0], light_ray[1], light_ray[2]);
    l_ray.normalize();
    unit_normal_vec.normalize();
    double amountHit = l_ray.dot(unit_normal_vec);
    I_sc = max_I_sc * amountHit;
    return I_sc;
}



