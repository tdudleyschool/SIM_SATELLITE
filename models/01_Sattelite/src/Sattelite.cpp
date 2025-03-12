/********************************* TRICK HEADER *******************************
PURPOSE: ( Simulate a sattelite)
LIBRARY DEPENDENCY:
    ((Sattelite.o)
     (Ridged_Body/src/Ridged_Body.o)
     (Attitude_Control/src/Attitude_Control_System.o)
     (Enviroment/src/cilestial_body.o)
     (Enviroment/src/gravitational_force.o)
     (EPS/src/Battery.o)
     (EPS/src/bus.o)
     (EPS/src/Solar_Power_System.o)
     (EPS/src/solar_cell.o)
     (GNC/src/accelerometer.o)
     (GNC/src/gyroscope.o)
     (Propulsion/src/Propulsion_System.o)
     (Recources/src/force_torque_tracker.o))
*******************************************************************************/

#include "../include/Sattelite.hh"
#include <math.h>
#include <iostream>

using namespace std;

void integrate2(double& val, double integ_val, double dt){
    double interval = integ_val*dt;
    val = val + interval;
}

int satellite::default_data(){
    sim_time = 0.0;
    dt = 0.01;

    //================//
    //ENVIROMENT SETUP//
    //================//

    Earth_m = 5.97e24;
    double Earth_Radius = 6378137;
    double EarthInitPos[3] = {0, 0, 0};
    //Maybe Velocity
    Earth.initialize(Earth_m, Earth_Radius);
    Earth.set_position(EarthInitPos);

    Moon_m = 7.3e22;
    double Moon_Radius = 1737500;
    double MoonInitPos[3] = {3.8444e8, 0, 0};
    //Mayber Velocity
    Moon.initialize(Moon_m, Moon_Radius);
    Moon.set_relative_pos(EarthInitPos, MoonInitPos);

    //Gravitational Force Setup
    grav_Earth_Moon.update_mass(Earth_m, Moon_m);
    grav_Earth_Moon.update_pos(EarthInitPos, MoonInitPos);

    sun_pos[0] = -1.5e11;
    sun_pos[1] = 0.0;
    sun_pos[2] = 0.0;

    //====================//
    //SATTELITE BODY SETUP//
    //====================//
    sat_m = 400;
    double Sattelite_Length = 15;
    double SatteliteInitPos[3] = {6778137.0, 0, 0};
    double SatteliteInitVel[3] = {0, 0, 0};
    double SatteliteInitAccel[3] = {0, 0, 0};

    satellite_body.initialize_body(sat_m, Sattelite_Length);
    satellite_body.initialize_motion(SatteliteInitPos, SatteliteInitVel, SatteliteInitAccel);

    grav_sat_Earth.update_mass(sat_m, Earth_m);
    grav_sat_Earth.update_pos(SatteliteInitPos, EarthInitPos);
    grav_sat_Moon.update_mass(sat_m, Moon_m);
    grav_sat_Moon.update_pos(SatteliteInitPos, MoonInitPos);

    //=============================//
    //ELECTRICAL POWER SYSTEM SETUP//
    //=============================//
    //Bus Setup//
    High_bus.initialize(120, 7);
    Low_bus.initialize(120, 4);
    for (int i = 0; i < High_bus_node_num; i++){
        if (i < 3) {
            High_bus.update_node_drawn_P(i, High_thruster_pow);
        }
        else {
            High_bus.update_node_drawn_P(i, Low_thruster_pow);
        }
    }
    for (int i = 0; i < Low_bus_node_num; i++){
        Low_bus.update_node_drawn_P(i, 400);
    }

    //Solar Power System//
    sol_arrays[0].initialize(130/*OC V*/, 265/*I_CO*/, 140/*Max V*/, 265/*Max I*/, 0.2586 /*terminalV*/);
    sol_arrays[1].initialize(130/*OC V*/, 265/*I_CO*/, 140/*Max V*/, 265/*Max I*/, 0.2586 /*terminalV*/);

    //Find Battery Metrics and fill in

    //=======================//
    //PROPULSION SYSTEM SETUP//
    //=======================//
    PropSys.set_all_thruster_ref_pos(Sattelite_Length, Sattelite_Length, Sattelite_Length, 5, 4);
    PropSys.set_all_thruster_ref_ori(0, -1, 0);
    for (int i = 0; i < 7; i++){
        if (i < 3){
            PropSys.set_thruster_specs(i, High_thruster_pow, 700, 0.57);
        }
        else {
            PropSys.set_thruster_specs(i, Low_thruster_pow, 700, 0.57);
        }
    }

    //======================//
    //ATTITUDE CONTROL SETUP//
    //======================//
    ACS.Initialize_Power(400);

    //=====================================//
    //GUIDENCE NAVIGATION AND CONTROL SETUP//
    //=====================================//
    for (int i = 0; i < 3; i++){
        accel[i].initialize(10.0, 10.0, 3.0, 100, 0.0, 0.0, 0.0, 0.0);
        gyro[i].initialize(10, 1, 5, 10);
    }

    return (0);
}

int satellite::state_init(){
    //Getting Metrics From Sattelite Body
    R_matrix[3][3];
    satellite_body.get_R(R_matrix);
    satellite_body.get_pos(sat_x);
    sat_v[0] = 0;
    sat_v[1] = 0;
    sat_v[2] = 0;

    //Setup Moons Initial Velocity For Orbit
    Moon_v[0] = 0;
    Moon_v[1] = 1022.0;
    Moon_v[2] = 0;
    Moon.update_v(Moon_v);

    //Sattelite Body Initial Variables
    for(int i = 0; i < 3; i++){
        sat_L[i] = 0;
        sat_w[i] = 0;
    }
    sat_w_mag = 0;
    sat_theta_mag = 0;

    //Attitude Control Initial Variables
    ACS.update_all_ori(R_matrix);
    ACS.update_I(0, 0);
    ACS.update_I(1, 0);
    ACS.update_I(2, 0);

    //GNC Initial Variables
    for (int i = 0; i < 3; i++){
        accel_proof_mass_x[i] = 0;
        accel_proof_mass_v[i] = 0;

        gyro_x_drive[i] = 0;
        gyro_x_sense[i] = 0;
        gyro_v_drive[i] = 0;
        gyro_v_sense[i] = 0;

        gyro[i].update_position_sense(0);
        gyro[i].update_velocity_drive(0);
        gyro[i].update_velocity_sense(0);

        GNC_cal_v[i] = 0;
        GNC_cal_x[i] = 0;

        GNC_cal_w[i] = 0;
    }
    
    PropSys.turn_all_on();

    return (0);
}

int satellite::state_deriv(){
    //Getting Metrics From Sattelite Body
    satellite_body.get_R(R_matrix);
    satellite_body.get_pos(sat_x);
    satellite_body.get_v(sat_v);

    Earth.get_pos(Earth_x);
    Moon.get_pos(Moon_x);

    //Calculating Total Forces & Torques
    double sat_F[3] = {0, 0, 0};
    double sat_F_pos[3] = {0, 0, 0};
    double sat_T[3] = {0, 0, 0};

    double Earth_F[3] = {0, 0, 0};
    double Earth_F_pos[3] = {0, 0, 0};

    double Moon_F[3] = {0, 0, 0};
    double Moon_F_pos[3] = {0, 0, 0};
    
    //Gravitation
    grav_Earth_Moon.update_pos(Earth_x, Moon_x);
    grav_sat_Earth.update_pos(sat_x, Earth_x);
    grav_sat_Moon.update_pos(sat_x, Moon_x);

    grav_Earth_Moon.calculate_force();
    grav_Earth_Moon.get_grav_force_at_mass1(Earth_F, Earth_F_pos);
    grav_Earth_Moon.get_grav_force_at_mass2(Moon_F, Moon_F_pos);
    Moon_Forces.addForce(Moon_F, Moon_F_pos);
    Earth_Forces.addForce(Earth_F, Earth_F_pos);


    grav_sat_Earth.calculate_force();
    grav_sat_Moon.calculate_force();
    grav_sat_Earth.get_grav_force_at_mass1(sat_F, sat_F_pos);
    Sattelite_Forces.addForce(sat_F, sat_F_pos);
    grav_sat_Moon.get_grav_force_at_mass1(sat_F, sat_F_pos);
    Sattelite_Forces.addForce(sat_F, sat_F_pos);


    //Sattelite Forces/Torques
    PropSys.get_all_force(sat_F, sat_F_pos);
    ACS.get_total_torque(sat_T);

    Sattelite_Forces.addForce(sat_F[0], sat_F[1], sat_F[2], sat_F_pos[0], sat_F_pos[1], sat_F_pos[2]);
    Sattelite_Forces.addTorque(sat_T[0], sat_T[1], sat_T[2]);

    double sat_F_net[3];
    double sat_T_net[3];
    Sattelite_Forces.getNetForce(sat_F_net);
    Sattelite_Forces.getNetTorque(sat_T_net);

    Sattelite_Forces.resetValues();
    
    double V = 120;

    //RIDGID BODY HANDLING
    //update L once set
    satellite_body.update_L(sat_L[0], sat_L[1], sat_L[2]);
    //look into reversing since magnitude cannot be negative but is is in L direction
    satellite_body.update_QoriByAngle(sat_theta_mag);
    sat_w_mag = satellite_body.get_w_mag();
    satellite_body.get_w(sat_w);
    

    satellite_body.update_force(sat_F_net[0], sat_F_net[1], sat_F_net[2]);
    satellite_body.update_torque(sat_T_net[0], sat_T_net[1], sat_T_net[2]);

    satellite_body.state_deriv_getAccel(sat_a);
    satellite_body.state_deriv_get_dL(sat_dL);

    satellite_body.get_R(R_matrix);


    //================================
    //ELECTRICAL POWER SYSTEM HANDLING
    //================================
    sol_arrays[0].update_V(V);
    sol_arrays[1].update_V(V);
    sol_arrays[0].update_I();
    sol_arrays[1].update_I();
    double Solar_I = sol_arrays[0].get_I() + sol_arrays[1].get_I();
    double Bus_drawn_I = High_bus.get_drawn_I() + Low_bus.get_drawn_I();

    double I_in;

    if (Solar_I > Bus_drawn_I) {
        double Excess_I = Solar_I - Bus_drawn_I;
        EPS_bat.update_I(-1*Excess_I);
        EPS_bat.update_soc(dt);
        I_in = Bus_drawn_I;
    }
    else if (Solar_I = Bus_drawn_I){
        I_in = Bus_drawn_I;
    }
    else {
        double Needed_I = Bus_drawn_I - Solar_I;
        EPS_bat.update_I(Bus_drawn_I);
        EPS_bat.update_soc(dt);
        if(EPS_bat.get_soc() <= 0.2) {
            I_in = Solar_I;
        }
        else {
            I_in = Bus_drawn_I;
        }
    }

    //Current Into Busses
    double Low_Voltage_I = 0;
    double High_Voltage_I = I_in;
    if(High_Voltage_I - High_bus.get_drawn_I() < 0) {
        Low_Voltage_I = 0;
    }
    else
    {
        Low_Voltage_I = High_Voltage_I - High_bus.get_drawn_I();
    }

    //=========================
    //ATTITUDE CONTROL HANDLING
    //=========================
    ACS.update_all_ori(R_matrix);
    ACS.update_I(0, Low_Voltage_I);
    ACS.update_I(1, Low_Voltage_I);
    ACS.update_I(2, Low_Voltage_I);
    ACS.state_deriv_getALL_dI(dI);
    ACS.state_deriv_getALL_Alpha(alpha);

    //==========================
    //PROPULSION SYSTEM HANDLING
    //==========================
    PropSys.update_all_pos_ori(sat_x, R_matrix);

    //========================================
    //GUIDENCE NAVIGATION AND CONTROL HANDLING
    //========================================

    for (int i = 0; i < 3; i++){
        accel_proof_mass_a[i] = accel[i].state_deriv_getAccel(sat_F_net[i], accel_proof_mass_v[i], accel_proof_mass_x[i]);
        GNC_cal_a[i] = accel[i].get_acceleration(sat_m);

        gyro_a_drive[i] = gyro[i].state_deriv_getAccel_drive();
        gyro_a_sense[i] = gyro[i].state_deriv_getAccel_sense(sat_w[i]);
    }

    //===
    //Temparary integration
    //===

    for (int i = 0; i < 3; i++){
        integrate2(sat_v[i], sat_a[i], dt);
        integrate2(sat_x[i], sat_a[i], dt);
        GNC_cal_w[i] = GNC_cal_w[i] + 0.1592341*dt;
    }
    satellite_body.update_pos(sat_x[0], sat_x[1], sat_x[2]);
    satellite_body.update_v(sat_v[0], sat_v[1], sat_v[2]);


    cout << "======================SATTELITE OUTPUT=====================" << endl;
    cout << "CURRENT IN BUS 1: " << High_Voltage_I << endl;
    cout << "CURRENT IN BUS 2: " << Low_Voltage_I << endl;
    cout << "SATTELITE ACCELERATION: " << GNC_cal_a[0] << ", " << GNC_cal_a[1] << ", " << GNC_cal_a[2];
    cout << "SATTELITE VELOCITY: " << sat_v[0] << ", " << sat_v[1] << ", " << sat_v[2];
    cout << "SATTELITE POSITION: " << sat_x[0] << ", " << sat_x[1] << ", " << sat_x[2];
    cout << "SATTELITE ANGULAR VELOCITY: " << GNC_cal_w[0] << ", " << GNC_cal_w[1] << ", " << GNC_cal_w[2];
    cout << endl;

    return (0);
}

#include "sim_services/Integrator/include/integrator_c_intf.h"

int satellite::state_integ(){
    int integration_step;

    load_state (&sim_time, 
               &attitude_I, &attitude_w, 
               &sat_x[0], &sat_x[1], &sat_x[2],
               &sat_v[0], &sat_v[1], &sat_v[2], 
               &Earth_x[0], &Earth_x[1], &Earth_x[2],
               &Earth_v[0], &Earth_v[1], &Earth_v[2],
               &Moon_x[0], &Moon_x[1], &Moon_x[2], 
               &Moon_v[0], &Moon_v[1], &Moon_v[2], 
               &sat_L[0], &sat_L[1], &sat_L[2],
               &sat_theta_mag, 
               &accel_proof_mass_x[0], &accel_proof_mass_x[1], &accel_proof_mass_x[2],
               &accel_proof_mass_v[0], &accel_proof_mass_v[1], &accel_proof_mass_v[2],
               &gyro_x_drive[0], &gyro_x_drive[1], &gyro_x_drive[2], &gyro_x_sense[0], &gyro_x_sense[1], &gyro_a_sense[2],
               &gyro_v_drive[1], &gyro_v_drive[2], &gyro_v_drive[2], &gyro_v_sense[0], &gyro_v_sense[1], &gyro_a_sense[2],
               &GNC_cal_x[0], &GNC_cal_x[1], &GNC_cal_x[2],
               &GNC_cal_v[0], &GNC_cal_v[1], &GNC_cal_v[2],
               (double*)0);

    load_deriv (&dt, 
               &dI, &alpha, 
               &sat_v[0], &sat_v[1], &sat_v[2],
               &sat_a[0], &sat_a[1], &sat_a[2], 
               &Earth_v[0], &Earth_v[1], &Earth_v[2],
               &Earth_a[0], &Earth_a[1], &Earth_a[2],
               &Moon_v[0], &Moon_v[1], &Moon_v[2], 
               &Moon_a[0], &Moon_a[1], &Moon_a[2], 
               &sat_dL[0], &sat_dL[1], &sat_dL[2],
               &sat_w_mag, 
               &accel_proof_mass_v[0], &accel_proof_mass_v[1], &accel_proof_mass_v[2],
               &accel_proof_mass_a[0], &accel_proof_mass_a[1], &accel_proof_mass_a[2],
               &gyro_v_drive[0], &gyro_v_drive[1], &gyro_v_drive[2], &gyro_v_sense[0], &gyro_v_sense[1], &gyro_v_sense[2],
               &gyro_a_drive[1], &gyro_a_drive[2], &gyro_a_drive[2], &gyro_a_sense[0], &gyro_a_sense[1], &gyro_a_sense[2],
               &GNC_cal_v[0], &GNC_cal_v[1], &GNC_cal_v[2],
               &GNC_cal_a[0], &GNC_cal_a[1], &GNC_cal_a[2],
               (double*)0);
    
    integration_step = integrate();

    unload_state (&sim_time, 
            &attitude_I, &attitude_w, 
            &sat_x[0], &sat_x[1], &sat_x[2],
            &sat_v[0], &sat_v[1], &sat_v[2], 
            &Earth_x[0], &Earth_x[1], &Earth_x[2],
            &Earth_v[0], &Earth_v[1], &Earth_v[2],
            &Moon_x[0], &Moon_x[1], &Moon_x[2], 
            &Moon_v[0], &Moon_v[1], &Moon_v[2], 
            &sat_L[0], &sat_L[1], &sat_L[2],
            &sat_theta_mag, 
            &accel_proof_mass_x[0], &accel_proof_mass_x[1], &accel_proof_mass_x[2],
            &accel_proof_mass_v[0], &accel_proof_mass_v[1], &accel_proof_mass_v[2],
            &gyro_x_drive[0], &gyro_x_drive[1], &gyro_x_drive[2], &gyro_x_sense[0], &gyro_x_sense[1], &gyro_a_sense[2],
            &gyro_v_drive[1], &gyro_v_drive[2], &gyro_v_drive[2], &gyro_v_sense[0], &gyro_v_sense[1], &gyro_a_sense[2],
            &GNC_cal_x[0], &GNC_cal_x[1], &GNC_cal_x[2],
            &GNC_cal_v[0], &GNC_cal_v[1], &GNC_cal_v[2],
            (double*)0);

/*
    for (int i = 0; i < 3; i++){
        gyro[i].update_position_drive(gyro_x_drive[0]);
        gyro[i].update_position_sense(gyro_x_sense[0]);
        gyro[i].update_velocity_drive(gyro_v_drive[0]);
        gyro[i].update_velocity_sense(gyro_v_sense[0]);
        GNC_cal_w[i] = gyro[i].get_angularVelocity();
    }
*/
    return (integration_step);
}

int satellite::shutdown(){
    double High_Voltage_I = High_bus.get_drawn_I();
    double Low_Voltage_I = Low_bus.get_drawn_I();
    cout << "=============================================================" << endl;
    cout << "======================SATTELITE SHUTDOWN=====================" << endl;
    cout << "=============================================================" << endl;
    cout << "CURRENT IN BUS 1: " << High_Voltage_I << endl;
    cout << "CURRENT IN BUS 2: " << Low_Voltage_I << endl;
    cout << "SATTELITE ACCELERATION: " << GNC_cal_a[0] << ", " << GNC_cal_a[1] << ", " << GNC_cal_a[2] << endl;
    cout << "SATTELITE VELOCITY: " << sat_v[0] << ", " << sat_v[1] << ", " << sat_v[2] << endl;
    cout << "SATTELITE POSITION: " << sat_x[0] << ", " << sat_x[1] << ", " << sat_x[2] << endl;
    cout << "SATTELITE ANGULAR VELOCITY: " << GNC_cal_w[0] << ", " << GNC_cal_w[1] << ", " << GNC_cal_w[2] << endl;
    cout << "CLOSE TIME: " << sim_time << endl;
    cout << endl;

    return(0);
}