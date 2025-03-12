/*
PURPOSE:    This module models the H-Bridge circut. 
            the model perpose is to flip the polarity
            of the voltage and thus the direction of
            the motor
*/

#ifndef H_BRIDGE_HH
#define H_BRIDGE_HH

#ifdef __cplusplus
    extern "C"
    {
#endif

class h_bridge {
    public:
        //Description: Constructor initializing circut with no incomming current
        h_bridge();

        //Description: function that flips switch such that current flows forward (Positive voltage)
        void forward();

        //Description: function reverses polarity of voltage by allowing flow of current in other direction
        void backwards();

        //Description: turns the circut off
        void off();

        //Description: updates the incomming voltage based on current direction caused by the switches
        void update_bridge_Voltage(double);

        //Description: gets the voltage across the H-bridge circut
        double get_V_bridge();
    private:
        double s[2]; //switches
        double V_l; //left voltage
        double V_r; //right side voltage

        //calculate
        double V_bridge;
};

#ifdef __cplusplus
    }
#endif

#endif