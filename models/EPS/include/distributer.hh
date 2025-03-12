/*
PURPOSE: 
*/

#include "bus.hh"

#ifndef DISTRIBUTER_HH
#define DISTRIBUTER_HH

#ifdef __cplusplus
    extern "C"
    {
#endif

class regulator {
    public:
        regulator();
        void chargeBattery(double);
        double dischargeBattery();
    private:
        double V_bat;
        double I_bat;
}

#ifndef __cplusplus
    }
#endif