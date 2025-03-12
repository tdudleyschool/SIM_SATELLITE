/*
PURPOSE:    Simulate rigid body dynamics including linear
            and angular motion using Eigen vectors and matrices
*/

#ifndef RIDGED_BODY_HH
#define RIDGED_BODY_HH

#include "../../Recources/src/Linear_Algebra.cpp"
//#include "../../../Lib/eigen-3.4.0/Eigen/Dense"
//#include "../../../Lib/eigen-3.4.0/Eigen/Geometry"
#include "Satellite_Box.hh"

//using namespace Eigen;

#ifdef __cplusplus
    extern "C"
    {
#endif

class ridged_body{
    public:
        // Description: Constructor initializing ridged body to default values
        ridged_body();

        // Description: Constructor initializing ridged body with position, velocity, and acceleration
        ridged_body(double pos[3] /*position*/, double vel[3] /*velocity*/, double acc[3] /*acceleration*/);

        void initialize_body(double /*mass*/, double /*side length*/);

        void initialize_motion(double pos[3] /*position*/, double vel[3] /*velocity*/, double acc[3] /*acceleration*/);
        
        // Description: Update the applied force vector
        void update_force(double x /*force_x*/, double y /*force_y*/, double z /*force_z*/);

        // Description: Update the applied torque vector
        void update_torque(double x /*torque_x*/, double y /*torque_y*/, double z /*torque_z*/);

        // Description: Update the angular momentum
        void update_L(double x /*L_x*/, double y /*L_y*/, double z /*L_z*/);

        // Description: Update the angular velocity
        void update_w(double x /*w_x*/, double y /*w_y*/, double z /*w_z*/);

        // Description: Update the linear velocity
        void update_v(double x /*v_x*/, double y /*v_y*/, double z /*v_z*/);

        // Description: Update the position vector
        void update_pos(double x /*pos_x*/, double y /*pos_y*/, double z /*pos_z*/);

        // Description: Update the rotation matrix
        void update_R(const double rot_matrix[3][3] /*rotation matrix*/);

        // Description: Update orientation using a quaternion
        void update_Qori(double quat[4] /*quaternion*/);

        // Description: Update orientation by an angle along the angular momentum axis
        void update_QoriByAngle(double angl /*angle*/);

        // Description: Calculate the acceleration from the applied force
        void state_deriv_getAccel(double accel[3] /*acceleration*/);

        // Description: Calculate the derivative of angular momentum (dL)
        void state_deriv_get_dL(double dL[3] /*angular momentum*/);

        // Description: Calculate angular acceleration (alpha)
        void state_deriv_get_alpha(double alpha[3] /*angular acceleration*/);

        // Description: Calculate the cross product matrix from angular velocity
        void state_deriv_getCross(double cross_matrix[3][3] /*cross product matrix*/);

        // Description: Get the rate of change of quaternion (Qori)
        void state_deriv_getQori(double d_quat[4] /*quaternion*/);

        // Description: Get the current angular momentum
        void get_L(double L[3] /*angular momentum*/);

        // Description: Get the current velocity
        void get_v(double V[3] /*velocity*/);

        // Description: Get the current position
        void get_pos(double pos[3] /*position*/);

        // Description: Get the current angular velocity
        void get_w(double omega[3] /*angular velocity*/);

        // Description: Get the magnitude of the angular velocity
        double get_w_mag();

        // Description: Get the current rotation matrix
        void get_R(double R_matrix[3][3] /*rotation matrix*/);

        // Description: Get the current quaternion orientation
        void get_Qori(double quat[4] /*quaternion*/);

        double get_mass();

    private:
        // Linear Motion
        Vector3d X; // position
        Vector3d v; // velocity
        Vector3d a; // acceleration

        Vector3d F; // force

        // Angular Motion
        Matrix3d R; // rotation matrix
        Quaterniond Qori; // orientation (quaternion)
        Vector3d w; // angular velocity (omega)
        Vector3d L; // angular momentum
        Vector3d alpha; // angular acceleration

        Vector3d dL; // derivative of angular momentum

        Vector3d T; // torque

        // Satellite body
        satellite_box B;
        Matrix3d inv_I0; // inverse of the inertia tensor (I_0)

        double m;
};

#ifdef __cplusplus
    }
#endif

#endif
