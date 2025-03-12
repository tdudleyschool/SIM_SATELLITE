/*
PURPOSE:    This is the implementation of file 'gyroscope.hh'

NOTE:       MEMS gyroscope is based on the 
            Corilos effect requireing them 
            to not need any rotating parts

TERMS USED:
    -> k - spring constant
    -> c - dampaning
    -> drive force - initial force setting vibration

EQUATIONS TO MODEL:
        -> m*d2x + c*dx + k*x = F_d
        -> m*d2y + c*dy + k*y = -2m(Omega)dx
*/

#include "../include/gyroscope.hh"

gyroscope::gyroscope()
    //Description:      Default constructor setting all values to their default state.
    //Preconditions:    None
    //Postconditions:   gyroscope with several settings
{
    m = 0.01; //kg
    k = 100; //N/m
    c = 0.005;
    F_d = 1.0;

    x = 0;
    v_x = 0;
    y = 0;
    v_y = 0;
}

gyroscope::gyroscope(double mass, double spr_const, double damp_const, double F_drive)
    //Description:      Default constructor setting all values to their default state.
    //Preconditions:    None
    //Postconditions:   gyroscope with several settings
{
    m = mass;
    k = spr_const;
    c = damp_const;
    F_d = F_drive;

    x = 0;
    v_x = 0;
    y = 0;
    v_y = 0;
}

void gyroscope::initialize(double mass, double spr_const, double damp_const, double F_drive){
    //Description:      function that intializes values needed to simuate the proof of masses change in position.
    //Preconditions:    None
    //Postconditions:   gyroscope with several settings
    m = mass;
    k = spr_const;
    c = damp_const;
    F_d = F_drive;

    x = 0;
    v_x = 0;
    y = 0;
    v_y = 0;
}

double gyroscope::state_deriv_getAccel_drive()
    //Description:      function that gets the acceleration if drive direction back as a state derivative
    //Preconditions:    initialized function must have been run
    //Postconditions:   user gets acceleration in the drive direction
{
    a_x = (F_d - c*v_x - k*x)/m;
    return a_x;
}

double gyroscope::state_deriv_getAccel_sense(double omega)
    //Description:      function that gets the acceleration if sense direction back as a state derivative
    //Preconditions:    initialized function must have been run
    //Postconditions:   user gets acceleration in the sense direction
{
    a_y = (2*m*omega*v_x - c*v_y - k*y)/m;
    return a_y;
}

void gyroscope::update_position_drive(double pos)
    //Description:      Function updates position in drive axis. Position should be attained by taking the state derivative in drive and running an integration loop
    //Preconditions:    initialized function must have been run
    //Postconditions:   user gets acceleration in the drive direction
{
    x = pos;
}

void gyroscope::update_position_sense(double pos)
    //Description:      Function updates position in sense axis. Position should be attained by taking the state derivative in drive and running an integration loop
    //Preconditions:    initialized function must have been run
    //Postconditions:   user gets acceleration in the drive direction
{
    y = pos;
}

void gyroscope::update_velocity_drive(double vel)
    //Description:      Function updates velocity in drive axis. Position should be attained by taking the state derivative in drive and running an integration loop
    //Preconditions:    initialized function must have been run
    //Postconditions:   user gets acceleration in the drive direction
{
    v_x = vel;
}

void gyroscope::update_velocity_sense(double vel)
    //Description:      Function updates velocity in sense axis. Position should be attained by taking the state derivative in drive and running an integration loop
    //Preconditions:    initialized function must have been run
    //Postconditions:   user gets acceleration in the drive direction
{
    v_y = vel;
}

double gyroscope::get_angularVelocity()
    //Description:      function calculates the agular velocity with the values of position and velecity being set
    //Preconditions:    position drive, and sense, and velocity drive and sense being set after outside integration loop
    //Postconditions:   returns the angular velocity simulating the gyroscope in one axis
{
    return (m*a_y + c*v_y + k*y)/(2*m*v_x);
}

