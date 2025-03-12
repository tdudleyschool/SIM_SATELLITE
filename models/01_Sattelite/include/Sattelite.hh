/********************************* TRICK HEADER *******************************
PURPOSE: ( Simulate a sattelite)
*******************************************************************************/
#ifndef SATTELITE_HH
#define SATTELITE_HH

#include "../../Ridged_Body/include/Ridged_Body.hh"

#include "../../Attitude_Control/include/Attitude_Control_System.hh"
#include "../../Enviroment/include/cilestial_body.hh"
#include "../../Enviroment/include/gravitational_force.hh"

#include "../../EPS/include/Battery.hh"
#include "../../EPS/include/bus.hh"
#include "../../EPS/include/Solar_Power_System.hh"
#include "../../EPS/include/solar_cell.hh"

#include "../../GNC/include/accelerometer.hh"
#include "../../GNC/include/gyroscope.hh"

#include "../../Propulsion/include/Propulsion_System.hh"
#include "../../Recources/include/force_torque_tracker.hh"

class satellite {
public:
    //--EPS--//
    battery EPS_bat;
    bus High_bus;
    bus Low_bus;
    solar_array sol_arrays[2];
    solar_cell sol_cell[2];

    //--Attitude Control System--//
    Attitude_Control_System ACS;

    //--Propulsion
    Propulsion_System PropSys;

    //--GNC
    accelerometer accel[3];
    gyroscope gyro[3];

    //--body
    ridged_body satellite_body;

    //--Enviroment
    force_torque_tracker Sattelite_Forces;
    force_torque_tracker Moon_Forces;
    force_torque_tracker Earth_Forces;

    cilestial_body Earth;
    cilestial_body Moon;

    gravitational_force grav_Earth_Moon;
    gravitational_force grav_sat_Earth;
    gravitational_force grav_sat_Moon;

    double sat_m;                   /* kg satellite mass */
    double Earth_m;                 /* kg Earth's mass */
    double Moon_m;                  /* kg Moon's mass */

    //Derivative Variables
    //Enviroment and sattelite
    double sat_a[3];                /* m/s2 satellite's true acceleration */
    double sat_dL[3];               /* N*m satellite's true torque */

    double Earth_a[3];              /* m/s2 Earth's acceleration */
    double Moon_a[3];               /* m/s2 Moon's acceleration */

    //Attitude Control
    double dI[3];                   /* A/s incomming changing current */
    double alpha[3];                /* rad/s2 angular acceleration of wheels */

    //propulsion
    double total_massflow;          /* kg/s mass flow */
    double current_propellant_mass; /* kg propallant mass */    
    
    //Guidence Navigation and Control
    //-DIRV:
    double accel_proof_mass_a[3];   /* m/s2 proof of mass acceleration */
    double gyro_a_drive[3];         /* m/s2 gyroscope proof of mass acceleration due to motion */
    double gyro_a_sense[3];         /* m/s2 gyroscope proof of mass acceleration due to coriolis force */

    double GNC_cal_a[3];            /* m/s2  */
    double GNC_cal_w[3];            /*  */
    //-INTEG
    double accel_proof_mass_x[3];
    double accel_proof_mass_v[3];
    double gyro_x_drive[3];
    double gyro_x_sense[3];
    double gyro_v_drive[3];
    double gyro_v_sense[3];

    double GNC_cal_v[3];
    double GNC_cal_x[3];
    //calculate orientation from omega

    //Integrated Variables
    //Attitude
    double attitude_I;
    double attitude_w;

    double R_matrix[3][3];

    double sat_x[3];
    double Earth_x[3];
    double Moon_x[3];

    double sat_v[3];
    double sat_L[3];
    double sat_w[3];
    double sat_w_mag;
    double sat_theta_mag;

    double Moon_v[3];
    double Earth_v[3];

    //Enviroment and Sattelite

    //--Missilanious
    double sim_time;
    double dt;

    double sun_pos[3];
    double High_thruster_pow = 12.5e3;
    double Low_thruster_pow = 6.5e3;
    double High_bus_node_num = 7;
    double Low_bus_node_num = 4;

    int default_data();
    int state_init();
    int state_deriv();
    int state_integ();
    int shutdown();

};
#endif