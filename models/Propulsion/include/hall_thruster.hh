/*
PURPOSE:    This module provides a simplified model for the popular Hall-Effect Thruster.
            In short the main goal to calculate the output force vector based on changing
            position and orientation.

NOTE:       This file gives a summary of each group of functions. See hall_thruster.cpp for more
            detailed description of implementation.
*/

#ifndef HALL_THRUSTER_HH
#define HALL_THRUSTER_HH

#include "../../Recources/src/Linear_Algebra.cpp"
//#include "../../../Lib/eigen-3.4.0/Eigen/Dense" //include Eigen Library directory here

//using namespace Eigen;

#ifdef __cplusplus
    extern "C"
    {
#endif

class hall_thruster {
    public:
        //Description: Create satellite thruster with not initial values
        hall_thruster();

        //Description: Create satellite thruster with initial values
        hall_thruster(double /*power*/, double /*discharge voltage*/, double /*efficiency*/);
        void initialize_state(double /*power*/, double /*discharge voltage*/, double /*efficiency*/);

        //Description: sets position in refrence to satellite
        void set_refrence_pos(double[3] /*position in refrence to satellite*/);
        void set_refrence_pos(double /*x*/, double/*y*/, double/*z*/);
        void set_refrence_pos(Vector3d /*position in refrence to satellite*/);

        //Description: sets orientation in refrence to satellite
        void set_refrence_ori(double[3] /*orientation in refrence to satellite*/);
        void set_refrence_ori(double/*x*/, double/*y*/, double/*z*/);
        void set_refrence_ori(Vector3d /*orientation in refrence to satellite*/);

        //Description: sets rotation matrix up for calculating position and orientation
        void update_R_matrix(double[3][3] /*rotation matrix*/);
        void update_R_matrix(const Matrix3d& /*rotation matrix*/);

        //Description: updates global position of thruster
        void update_pos(double[3] /*satellite center*/, double[3][3] /*rotation matrix*/);
        void update_pos(Vector3d /*satellite center*/, const Matrix3d& /*rotation matrix*/);
        void update_pos(double[3] /*satellite center*/);
        void update_pos(Vector3d /*satellite center*/);

        //Description: updates global orientation of thruster
        void update_ori(double[3][3] /*rotation matrix*/);
        void update_ori(const Matrix3d& /*rotation matrix*/);
        void update_ori();

        //Description: updates both global position and orientation (more efficient then just doing former functions separatly)
        void update_pos_ori(double[3] /*satellite center*/, double[3][3] /*rotation matrix*/);
        void update_pos_ori(Vector3d /*satellite center*/, const Matrix3d& /*rotation matrix*/);
        void update_pos_ori(double[3] /*satellite center*/);
        void update_pos_ori(Vector3d /*satellite center*/);

        //Description: returns the global position of the thruster
        void get_pos(double[3] /*position*/);
        Vector3d get_pos();

        //Description: returns the global orientation of thruster
        void get_ori(double[3] /*orientation*/);
        Vector3d get_ori();

        //Description: returns both the force and position at whitch the force happens
        void get_force(double /*available mass*/, double[3] /*force*/, double[3] /*position*/);
        void get_force(double /*available mass*/, Vector3d& /*force*/, Vector3d& /*position*/);

        //Description: group of functions controls the on and off behavior of thruster
        void switch_stateon();
        void switch_stateoff();
        bool is_state_on();

        //Description: returns mass flow rate to operator
        double get_massflow();

    private:
        //initialization variables
        double q;           //charge of ion
        double V_d;         //discharge voltage
        double m_i;          //mass of ion
        double n;           //efficiency
        double P_i;         //input power
        double P_out;       //output power

        //calculated variables
        double dm;          //mass flow rate
        double v_i;         //velocity of ion
        double T;           //trust

        //on and off behavior
        bool state_on;

        //thruster "body"
        Vector3d ref_pos;   //position in refrence to satellite
        Vector3d ref_ori;   //orientation in refrence to satellite
        Vector3d pos;       //position of thruster
        Vector3d ori;       //orientation of thruster
        Matrix3d R_matrix;  //rotation matrix from satellite

        Vector3d F_ref;     //force in refrence to satellite pos and orientation
        Vector3d Force;     //global force

        //flags, used to check if functions are being executed in optimal order
        bool ref_pos_set;
        bool ref_ori_set;
        bool R_matrix_set;
        bool pos_set;
        bool ori_set;
};

#ifdef __cplusplus
    }
#endif

#endif