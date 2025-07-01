/*
PURPOSE:    This module is a framework that provides comminication between
            the xenon tanks and thrusters. This will package the amount of
            thrusters that are in the PPE specifically. (This will need to
            be changed if it is used to model another satellite with a
            diffrent amount of thrusters at diffrent locations).

NOTE:       For description into the modules used here look at that codes 
            specification and implementation files.

            The thrusters from the PPE are assumed to be symmetric.

            THRUSTER ORGANIZATION:
                PPE has 7 thrusters. All are stored into an array and the index
                represents diffrent ones.
                indexs: 0->center
                        1,2-> outer large thrusters 
                        3,4-> left small thrusters
                        5,6-> right small thrusters

TERMS USED:
    -> (D1): Distance smaller inside thruster is from edge of PPE
    -> (D2): Distance smaller outside thruster is from edge of PPE
*/

#ifndef PROPULSION_HH
#define PROPULSION_HH

#include "hall_thruster_PIC2D.hh"
#include "xenon_tank.hh"
#include "../../Recources/src/Linear_Algebra.cpp"
//#include "../../../Lib/eigen-3.4.0/Eigen/Dense" //include Eigen Library directory here

//using namespace Eigen;

#ifdef __cplusplus
    extern "C"
    {
#endif

class Propulsion_System_PIC2D {
    public:
        //Desctiption: Constructor that [DOSE NOT] set up thruster initial positions, power, or efficiency
        Propulsion_System_PIC2D();

        //Description: Constructor that [DOSE] set up thruster initial position
        Propulsion_System_PIC2D(double /*length*/, double /*width*/, double /*hight*/, double /*D1*/, double /*D2*/);

        //Description: function that dose set up initial thruster position and orientation
        void set_all_thruster_ref_pos(double /*length*/, double /*width*/, double /*hight*/, double /*D1*/, double /*D2*/);
        void set_all_thruster_ref_ori(double[3] /*orientation*/);
        void set_all_thruster_ref_ori(double /*x*/, double /*y*/, double /*z*/);
        void set_all_thruster_ref_ori(Vector3d /*orientation*/);
        
        //Description: updates all the thruster positions
        void update_all_pos_ori(double[3] /*satellite position*/, double[3][3] /*rotation matrix*/);
        void update_all_pos_ori(Vector3d /*satellite position*/, Matrix3d /*rotation matrix*/);
        
        void initialize_HET_sim(int /*thruster number*/);
        void initialize_all_HET_sim();

        void run_step_HET_sim(int /*thruster index*/, double /*mass flow*/, double /*Discharge Voltage*/);

        //Description: gets force created by one thruster
        void get_all_force(double[3] /*net thruster force*/, double[3] /*position of force*/);
        void get_all_force(Vector3d& /*net thruster force*/, Vector3d& /*position of force*/);

        void get_thruster_force(int /*thruster number*/, double[3] /*thruster force*/, double[3] /*position of force*/);
        void get_thruster_force(int /*thruster number*/, Vector3d /*thruster force*/, Vector3d /*position of force*/);

        //Description: controls on and off capabilities
        void turn_thruster_on(int /*thruster number*/);
        void turn_all_on();
        void turn_thruster_off(int /*thruster number*/);
        void turn_all_off();

        //Description: calculates total mass flow rate
        double get_total_mass_flow();
        double get_current_tankmass();

        void update_tankmass();

    private:
        double total_massflow;
        double available_mass;
        xenon_tank tanks[3];
        HET_PIC2D thrusters[7];
        double tank_mass_flow[7];

};

#ifdef __cplusplus
    }
#endif

#endif