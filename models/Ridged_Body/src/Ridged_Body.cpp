/********************************* TRICK HEADER *******************************
PURPOSE: (Simulate A Motor)
LIBRARY DEPENDENCY:
    ((Ridged_Body.o)
     (Satellite_Box.o))
*******************************************************************************/

/*
PURPOSE: This is the implementation of file 'ridged_body.hh'

NOTE: The ridged body model is used to simulate the motion of a rigid body in space, including its rotation and linear motion.

TERMS USED:
    -> F - Force acting on the rigid body
    -> T - Torque acting on the rigid body
    -> L - Angular momentum of the rigid body
    -> w - Angular velocity of the rigid body
    -> R - Rotation matrix representing the orientation of the body
    -> v - Linear velocity of the rigid body
    -> X - Position of the rigid body
    -> Qori - Quaternion orientation of the rigid body

EQUATIONS TO MODEL:
    -> Newton's second law for translation: F = m * a
    -> Euler's equations for rotation: I * alpha = T - w × (I * w)
*/

//#include "../../../Lib/eigen-3.4.0/Eigen/Dense"
//#include "../../../Lib/eigen-3.4.0/Eigen/Geometry"

#include <iostream>
#include <cmath>

#include "../include/Ridged_Body.hh"
#include "../include/Satellite_Box.hh"

//using namespace Eigen;
using namespace std;

ridged_body::ridged_body()
	//Description:  	Default constructor setting all variables for linear and angular motion to their default state.
	//Preconditions:	None
	//Postconditions:   Initialized ridged body object with linear and angular properties set to zero.
{
	// Linear Motion
	X.insert(0, 0, 0); // position
	v.insert(0, 0, 0); // velocity
	a.insert(0, 0, 0);// acceleration

	F.insert(0, 0, 0); // force
   	 
	// Angular Motion
	Qori.w(1.0);
	Qori.x(0);
	Qori.y(0);
	Qori.z(0);
	R = Qori.toRotationMatrix(); // quaternion to rotation matrix
	w.insert(0, 0, 0); // angular velocity (omega)
	L.insert(0, 0, 0); // angular momentum
	alpha.insert(0, 0, 0); // angular acceleration
    
	T.insert(0, 0, 0); // torque

	// Satellite body
	B.initialize(400, 15);
	B.initialize_points(X[0], X[1], X[2]);
	B.calc_I_0();
    
    double I_0[3][3];
	double inv_I_0[3][3];
    B.get_I_0(I_0);
	B.get_inv_I_0(inv_I_0);
    I0.insert(I_0[0][0], I_0[0][1], I_0[0][2],
        I_0[1][0], I_0[1][1], I_0[1][2],
        I_0[2][0], I_0[2][1], I_0[2][2]); //inertial tensor
	inv_I0.insert(inv_I_0[0][0], inv_I_0[0][1], inv_I_0[0][2],
          	inv_I_0[1][0], inv_I_0[1][1], inv_I_0[1][2],
          	inv_I_0[2][0], inv_I_0[2][1], inv_I_0[2][2]); // inverse inertial tensor
}

ridged_body::ridged_body(double pos[3], double vel[3], double acc[3])
	//Description:  	Constructor setting position, velocity, and acceleration based on input values.
	//Preconditions:	Position, velocity, and acceleration arrays must be provided.
	//Postconditions:   Initialized ridged body object with provided values for position, velocity, and acceleration.
{
	// Linear Motion
	X.insert(pos[0], pos[1], pos[2]);
	v.insert(vel[0], vel[1], vel[2]);
	a.insert(acc[0], acc[1], acc[2]);

	F.insert(0, 0, 0); // force
   	 
	// Angular Motion
	Qori.w(1.0);
	Qori.x(0);
	Qori.y(0);
	Qori.z(0);
	R = Qori.toRotationMatrix(); // quaternion to rotation matrix
	w.insert(0, 0, 0); // angular velocity (omega)
	L.insert(0, 0, 0); // angular momentum
	alpha.insert(0, 0, 0); // angular acceleration
    
	T.insert(0, 0, 0); // torque

	// Satellite body
	B.initialize(400, 15);
	B.initialize_points(X[0], X[1], X[2]);
	B.calc_I_0();
    
    double I_0[3][3];
	double inv_I_0[3][3];
    B.get_I_0(I_0);
	B.get_inv_I_0(inv_I_0);
    I0.insert(I_0[0][0], I_0[0][1], I_0[0][2],
        I_0[1][0], I_0[1][1], I_0[1][2],
        I_0[2][0], I_0[2][1], I_0[2][2]); //inertial tensor
	inv_I0.insert(inv_I_0[0][0], inv_I_0[0][1], inv_I_0[0][2],
          	inv_I_0[1][0], inv_I_0[1][1], inv_I_0[1][2],
          	inv_I_0[2][0], inv_I_0[2][1], inv_I_0[2][2]); // inverse inertial tensor
}

void ridged_body::initialize_body(double mass, double length){
    // Satellite body
    m = mass;
	B.initialize(mass, length);
	B.initialize_points(X[0], X[1], X[2]);
	B.calc_I_0();
    
    double I_0[3][3];
	double inv_I_0[3][3];
    B.get_I_0(I_0);
	B.get_inv_I_0(inv_I_0);
    I0.insert(I_0[0][0], I_0[0][1], I_0[0][2],
        I_0[1][0], I_0[1][1], I_0[1][2],
        I_0[2][0], I_0[2][1], I_0[2][2]); //inertial tensor
	inv_I0.insert(inv_I_0[0][0], inv_I_0[0][1], inv_I_0[0][2],
          	inv_I_0[1][0], inv_I_0[1][1], inv_I_0[1][2],
          	inv_I_0[2][0], inv_I_0[2][1], inv_I_0[2][2]); // inverse inertial tensor
}

void ridged_body::initialize_motion(double pos[3], double vel[3], double acc[3]){
    // Linear Motion
	X.insert(pos[0], pos[1], pos[2]);
	v.insert(vel[0], vel[1], vel[2]);
	a.insert(acc[0], acc[1], acc[2]);
   
}

// Update forces acting on the body
void ridged_body::update_force(double x, double y, double z)
    //Description:   Updates the force vector acting on the body.
    //Preconditions: None
    //Postconditions: Force vector F updated.
{
    F.insert(x, y, z);
}

// Update torques acting on the body
void ridged_body::update_torque(double x, double y, double z)
    //Description:   Updates the torque vector acting on the body.
    //Preconditions: None
    //Postconditions: Torque vector T updated.
{
    T.insert(x, y, z);
}

// Update angular momentum
void ridged_body::update_L(double x, double y, double z)
    //Description:   Updates the angular momentum vector. This is used to compute angular velocity.
    //Preconditions: Inertia matrix (inv_I0) and rotation matrix (R) must be set.
    //Postconditions: Angular momentum L and angular velocity w updated.
{
    L.insert(x, y, z);
    w = R * inv_I0 * R.transpose() * L;
}

// Update angular velocity
void ridged_body::update_w(double x, double y, double z)
    //Description:   Updates the angular velocity vector.
    //Preconditions: None
    //Postconditions: Angular velocity vector w updated.
{
    w.insert(x, y, z);
}

// Update linear velocity
void ridged_body::update_v(double x, double y, double z)
    //Description:   Updates the linear velocity vector.
    //Preconditions: None
    //Postconditions: Linear velocity vector v updated.
{
    v.insert(x, y, z);
}

// Update position of the body
void ridged_body::update_pos(double x, double y, double z)
    //Description:   Updates the position vector of the body.
    //Preconditions: None
    //Postconditions: Position vector X updated.
{
    X.insert(x, y, z);
}

// Update rotation matrix
void ridged_body::update_R(const double rot_matrix[3][3])
    //Description:   Updates the rotation matrix R using an input 3x3 matrix.
    //Preconditions: A valid rotation matrix must be provided.
    //Postconditions: Rotation matrix R updated.
{
    Matrix3d input;
    input.insert(rot_matrix[0][0], rot_matrix[0][1], rot_matrix[0][2],
            rot_matrix[1][0], rot_matrix[1][1], rot_matrix[1][2],
            rot_matrix[2][0], rot_matrix[2][1], rot_matrix[2][2]);
    R = R + input;
}

// Update quaternion orientation
void ridged_body::update_Qori(double quat[4])
    //Description:   Updates the quaternion orientation and corresponding rotation matrix.
    //Preconditions: A valid quaternion must be provided.
    //Postconditions: Quaternion Qori and rotation matrix R updated.
{
    Qori.w(quat[0]);
    Qori.x(quat[1]);
    Qori.y(quat[2]);
    Qori.z(quat[3]);
    Qori.normalize();
    R = Qori.toRotationMatrix();
}

// Update quaternion by angle
void ridged_body::update_QoriByAngle(double angl)
    //Description:   Updates the quaternion orientation by a given rotation angle around the axis defined by angular momentum.
    //Preconditions: Angular momentum L must be updated.
    //Postconditions: Quaternion Qori and rotation matrix R updated.
{
    Vector3d dir;
    dir << L;
    dir.normalize();
    Quaterniond r(cos(angl / 2),
                  sin(angl / 2) * dir.x,
                  sin(angl / 2) * dir.y,
                  sin(angl / 2) * dir.z);

    Quaterniond r_inv(cos(angl / 2),
                      sin(-angl / 2) * dir.x,
                      sin(-angl / 2) * dir.y,
                      sin(-angl / 2) * dir.z);

    Vector3d R_x = R.col(0);
    Vector3d R_y = R.col(1);
    Vector3d R_z = R.col(2);
    Quaterniond QR_x(0, R_x[0], R_x[1], R_x[2]);
    Quaterniond QR_y(0, R_y[0], R_y[1], R_y[2]);
    Quaterniond QR_z(0, R_z[0], R_z[1], R_z[2]);

    QR_x = r * QR_x * r_inv;
    QR_y = r * QR_y * r_inv;
    QR_z = r * QR_z * r_inv;

    R.insert(QR_x.x(), QR_y.x(), QR_z.x(),
        QR_x.y(), QR_y.y(), QR_z.y(),
        QR_x.z(), QR_y.z(), QR_z.z());
}

// Get the state derivative of acceleration
void ridged_body::state_deriv_getAccel(double acceleration[3])
    //Description:   Returns the state derivative of linear acceleration, to be integrated later for position.
    //Preconditions: Force vector F must be updated.
    //Postconditions: Returns the linear acceleration that can be integrated to get position.
{
    a = F / B.getmass();

    acceleration[0] = a[0];
    acceleration[1] = a[1];
    acceleration[2] = a[2];
}

// Get the state derivative of angular momentum
void ridged_body::state_deriv_get_dL(double d_angularMomentum[3])
    //Description:   Returns the state derivative of angular momentum, to be integrated later for angular momentum update.
    //Preconditions: Torque vector T must be updated.
    //Postconditions: Returns the angular momentum derivative to be integrated for angular momentum.
{
    dL = T;

    d_angularMomentum[0] = dL[0];
    d_angularMomentum[1] = dL[1];
    d_angularMomentum[2] = dL[2];
}

// Get the state derivative of angular acceleration
void ridged_body::state_deriv_get_alpha(double a[3])
    //Description:   Returns the state derivative of angular acceleration, to be integrated later for angular velocity.
    //Preconditions: Torque T and angular velocity w must be updated.
    //Postconditions: Returns the angular acceleration to be integrated for angular velocity.
{
    alpha = inv_I0 * (T - w.cross(I0 * w));

    a[0] = alpha[0];
    a[1] = alpha[1];
    a[2] = alpha[2];
}

// Get the state derivative of cross-product matrix
void ridged_body::state_deriv_getCross(double cross_matrix[3][3])
    //Description:   Returns the state derivative of the cross-product matrix, used for rotational kinematics.
    //Preconditions: Angular velocity w must be updated.
    //Postconditions: Returns the cross-product matrix for further kinematic calculations.
{
    Vector3d Rx = R.col(0);
    Vector3d Ry = R.col(1);
    Vector3d Rz = R.col(2);

    Vector3d w_crossX = w.cross(Rx);
    Vector3d w_crossY = w.cross(Ry);
    Vector3d w_crossZ = w.cross(Rz);

    for (int i = 0; i < 3; i++){
        cross_matrix[i][0] = w_crossX[i];
        cross_matrix[i][1] = w_crossY[i];
        cross_matrix[i][2] = w_crossZ[i];
    }
}

// Get the state derivative of quaternion orientation
void ridged_body::state_deriv_getQori(double d_quat[4])
    //Description:   Returns the state derivative of quaternion orientation, to be integrated for updating quaternion.
    //Preconditions: Quaternion Qori and angular velocity w must be updated.
    //Postconditions: Returns the quaternion derivative to be integrated for orientation.
{
    Quaterniond quat_w(0, w(0), w(1), w(2));
    Quaterniond dq = Qori * quat_w;

    d_quat[0] = dq.w() * 0.5;
    d_quat[1] = dq.x() * 0.5;
    d_quat[2] = dq.y() * 0.5;
    d_quat[3] = dq.z() * 0.5;
}

void ridged_body::get_L(double l[3])
    //Description:   Returns the angular momentum vector.
    //Preconditions: Angular momentum L must be updated.
    //Postconditions: Provides the angular momentum vector to the user.
{
    l[0] = L(0);
    l[1] = L(1);
    l[2] = L(2);
}

void ridged_body::get_v(double V[3])
    //Description:   Returns the linear velocity vector.
    //Preconditions: Linear velocity v must be updated.
    //Postconditions: Provides the linear velocity vector to the user.
{
    V[0] = v(0);
    V[1] = v(1);
    V[2] = v(2);
}

void ridged_body::get_pos(double pos[3])
    //Description:   Returns the position vector of the body.
    //Preconditions: Position vector X must be updated.
    //Postconditions: Provides the position vector to the user.
{
    pos[0] = X(0);
    pos[1] = X(1);
    pos[2] = X(2);
}

void ridged_body::get_w(double omega[3])
    //Description:   Returns the angular velocity vector.
    //Preconditions: Angular velocity w must be updated.
    //Postconditions: Provides the angular velocity vector to the user.
{
    omega[0] = w(0);
    omega[1] = w(1);
    omega[2] = w(2);
}

double ridged_body::get_w_mag()
    //Description:   Returns the magnitude of the angular velocity vector.
    //Preconditions: Angular velocity w must be updated.
    //Postconditions: Provides the magnitude of the angular velocity vector.
{
    return w.norm();
}

void ridged_body::get_R(double R_matrix[3][3])
    //Description:   Returns the rotation matrix representing the body's orientation.
    //Preconditions: Rotation matrix R must be updated.
    //Postconditions: Provides the 3x3 rotation matrix to the user.
{
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            R_matrix[i][j] = R(i, j);
        }
    }
}

void ridged_body::get_Qori(double quat[4])
    //Description:   Returns the quaternion representing the body's orientation.
    //Preconditions: Quaternion Qori must be updated.
    //Postconditions: Provides the quaternion to the user.
{
    quat[0] = Qori.w();
    quat[1] = Qori.x();
    quat[2] = Qori.y();
    quat[3] = Qori.z();
}

double ridged_body::get_mass(){
    return m;
}

void ridged_body::get_ref_i_vec(double vec[3])
{
    Vector3d Rx = R.col(0);
    vec[0] = Rx(0);
    vec[1] = Rx(1);
    vec[2] = Rx(2);
}

void ridged_body::get_ref_j_vec(double vec[3])
{
    Vector3d Ry = R.col(1);
    vec[0] = Ry(0);
    vec[1] = Ry(1);
    vec[2] = Ry(2);
}

void ridged_body::get_ref_k_vec(double vec[3])
{
    Vector3d Rz = R.col(2);
    vec[0] = Rz(0);
    vec[1] = Rz(1);
    vec[2] = Rz(2);
}

void ridged_body::convert_to_ref_frame(double vec[3])
{
    Vector3d v_local;
    Vector3d v_global;
    v_global[0] = vec[0];
    v_global[1] = vec[1];
    v_global[2] = vec[2];

    v_local = R * v_global;
    
    vec[0] = v_local[0];
    vec[1] = v_local[1];
    vec[2] = v_local[2];

}