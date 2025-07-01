#ifndef HET_THRUSTER
#define HET_THRUSTER

#include "../../Recources/src/Linear_Algebra.cpp"
#include "HET_simulation_2D_PIC.hh"

class HET_PIC2D {
    public:
        HET_PIC2D();

        // ===============================================
        // Keep Track Of Position and Dynamics Like Before
        // ===============================================

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

        // ======================
        // Hall Effect Simulation
        // ======================
        void initialize_HET_sim();
        void run_step_HET_sim(double /*mass flow*/, double /*Discharge Voltage*/);
        void get_force(double available_mass, double F[3], double F_pos[3]);
        void get_force(double available_mass, Vector3d& F, Vector3d& F_pos);

        //Description: group of functions controls the on and off behavior of thruster
        void switch_stateon();
        void switch_stateoff();
        bool is_state_on();

    private:
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

        //on and off behavior
        bool state_on;

        //HET Sim Variables
        SimulationDomain domain;
        ElectronFluid electrons;
        ElectricField field;
        IonPIC ions;
        NeutralPIC neutrals;
        Ionization ionizer;
        BoundaryConditions boundaries;
        ThrustCalculator thrustCalc;
        double dt = 1e-8;
        double thrust = 0.0;
};

#endif