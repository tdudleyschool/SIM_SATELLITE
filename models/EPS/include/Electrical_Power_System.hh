/*
PURPOSE:    This module provides a modle for the motor. Though the motor could be used
            in any situation it was designed to model the rotation caused by the control
            wheels.

NOTE:       This file gives a summary of each group of functions. See hall_thruster.cpp for more
            detailed description of implementation.
TERMS USED:
    -> (k1) k1 constant
*/

#include "solar_cell.hh"
#include "Solar_Power_System.hh"
#include "battery.hh"
#include "distributer.hh"


#ifndef ELECTRICAL_POWER_SYSTEM_HH
#define ELECTRICAL_POWER_SYSTEM_HH

#ifdef __cplusplus
    extern "C"
    {
#endif

class EPS {
    public:
        //Description: Creates a motor with no initial values
        EPS();
        EPS(double /*sol max V*/, double /*sol max I*/);

        double sol_update_I(double /*V*/);
        //void set_solarpannel_size();
        //void set_solarpannal_ref_pos();
        //void set_solarpannal_ref_ori();

        void initialize_battery();

        void initialize_bus(int/*num of loads*/);
        void set_bus_voltage(int /*bus index*/, double /*voltage*/);
        void set_load_voltage(int /*bus*/, int/*load*/, double /*voltage*/);

        void calc_solar_current(double[3]/*light comming in*/);

        double get_bus_current(int /*bus*/);
        double get_load_current(int /*bus*/, int /*load*/)

    private:
        double sol_I;
        double bus_drawn_I;
        double given_I;
        solar_cell solar_body[2];
        solar_array solar_power[2];
        battery ion_batt;
        bus busses[2];
};

#ifdef __cplusplus
    }
#endif

#endif