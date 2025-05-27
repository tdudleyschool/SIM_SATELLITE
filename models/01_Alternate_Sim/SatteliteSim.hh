#ifndef SATTELITE_SIM_HH
#define SATTELITE_SIM_HH

#include "sim_object.hh"

#include "../Ridged_Body/include/Ridged_Body.hh"

#include "../Attitude_Control/include/Attitude_Control_System.hh"
#include "../Enviroment/include/cilestial_body.hh"

#include "../EPS/include/Battery.hh"
#include "../EPS/include/bus.hh"
#include "../EPS/include/Solar_Power_System.hh"
#include "../EPS/include/solar_cell.hh"

#include "../GNC/include/accelerometer.hh"
#include "../GNC/include/gyroscope.hh"

#include "../Propulsion/include/Propulsion_System.hh"
#include "../Recources/include/force_torque_tracker.hh"
#include "../Recources/include/functions.hh"

class ACS_Sim : public Sim_Object {
    private:
        Attitude_Control_System ACS;
        double I[3];
        double w[3];
        double I_prev[3];
        double w_prev[3];
        double R_matrix_ori[3][3];

        double dI[3];
        double dw[3];
        
    protected:
        void initialize() override;
        void update(double dt) override;
    public:
        ACS_Sim(double timestep, double max_time = -1.0);
        void set_current_voltage(const double);
        void get_drawn_current(double[3]);

};

/*
        //====ENVIROMENT====//
        double Earth_x[3];
        double Earth_v[3];
        double Earth_a[3]; //state
        double Earth_rad;
        double Earth_m;

        double Moon_x[3];
        double Moon_v[3];
        double Moon_a[3]; //state
        double Moon_rad;
        double Moon_m;

        double Sun_x[3];
        double Sun_v[3];
        double Sun_a[3]; //state
        double Sun_rad;
        double Sun_m;
        

        //SATTELITE BODY
        ridged_body Sattelite_Body;
        double Sat_x[3];
        double Sat_v[3];
        double Sat_a[3]; //state

        double Sat_L[3]; //state
        double Sat_w[3]; //integ
        double Sat_theta[3];
        double R_matrix[3][3];

        //ELECTRICAL POWER SYSTEM */


#endif