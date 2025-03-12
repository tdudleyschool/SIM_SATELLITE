#include "../include/h-bridge.hh"

h_bridge::h_bridge()
    //Description:      Constructor creating H-bridge circut object with circut being off
    //Preconditions:    None
    //Postconditions:   H-bridge circut
{
    s[0] = 0;
    s[1] = 0;
}

void h_bridge::forward()
    //Description:      Method turns 1 switch on allowing voltage polarity to be positive
    //Preconditions:    None
    //Postconditions:   Positive voltage polarity
{
    s[0] = 1;
    s[1] = 0;
}

void h_bridge::backwards()
    //Description:      Method turns 1 switch on allowing voltage polarity to be negative
    //Preconditions:    None
    //Postconditions:   Negative voltage polarity
{
    s[0] = 0;
    s[1] = 1;
}

void h_bridge::off()
    //Description:      Method turns both switches off. No voltage
    //Preconditions:    None
    //Postconditions:   H-Bridge circut is off
{
    s[0] = 0;
    s[1] = 0;
}

void h_bridge::update_bridge_Voltage(double V_input)
    //Description:      Method updates the voltage within the H-bridge circut
    //Preconditions:    None
    //Postconditions:   H-Bridge circut internal voltage
{
    V_l = s[0] * V_input;
    V_r = s[1] * V_input;
    V_bridge = V_l - V_r;
}

double h_bridge::get_V_bridge()
    //Description:      Method returns the internal voltage within the H-bridge circut
    //Preconditions:    None
    //Postconditions:   returned voltage
{
    return V_bridge;
}