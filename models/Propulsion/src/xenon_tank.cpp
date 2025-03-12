/*
PURPOSE:    This is the implementation of the xenon_tank module. Everyting
            Here is mostly explained in the xenon_tank.hh file.

GUIDE:      Since the module that is intended to be used with this (TICK)
            has it's own integration step the user is intended to recive
            the mass with getmass(), and update it with intagration and 
            a change in mass before inputing the new mass back into module
            with update_mass()
*/

#include "../include/xenon_tank.hh"

xenon_tank::xenon_tank()
    //Description:      Default constructor setting initial tank amount to 1500 kg.
    //Preconditions     NONE.
    //Postconditions:   xenon tank object
{
    mass = 1500.00;
}

void xenon_tank::update_mass(double m)
    //Description:      Function that updates mass within tank object. Intended to be used
    //                  after integration step.
    //Preconditions     Technically none. If used after integration then function would need
    //                  to be implemented alongside a rate of change of mass. (massflow)
    //Postconditions:   Updated mass
{
    mass = m;
}

double xenon_tank::getmass()
    //Description:      Returns mass to user. If used with integrator then it would be used
    //                  to input mass into integrator.
    //Preconditions     NONE.
    //Postconditions:   Reterned mass.
{
    return mass;
}