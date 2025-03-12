/*
PURPOSE:    This module keeps track of the mass of propellant (Xenon)
            inside the tank.

GUIDE:      Since the module that is intended to be used with this (TICK)
            has it's own integration step the user is intended to recive
            the mass with getmass(), and update it with intagration and 
            a change in mass before inputing the new mass back into module
            with update_mass()
            
*/

#ifndef XENON_TANK_HH
#define XENON_TANK_HH

#ifdef __cplusplus
    extern "C"
    {
#endif

class xenon_tank {
    public:
        //Description: creates tank object
        xenon_tank();

        //Description: updates mass inside tank.
        //             intended be used after integration step.
        void update_mass(double);

        //Description: returns current mass
        double getmass();
    private:
        double mass; //self explainetory
};

#ifdef __cplusplus
    }
#endif

#endif