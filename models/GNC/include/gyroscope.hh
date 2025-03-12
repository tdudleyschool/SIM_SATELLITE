/*
PURPOSE:    This class will simulate a gyroscope

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

#ifndef GYROSCOPE_HH
#define GYROSCOPE_HH

#ifdef __cplusplus
    extern "C"
    {
#endif

class gyroscope {
    public:
        //Description: Constructor creating gyroscope object
        gyroscope();

        //Description: Constructor creating gyroscope object with initial values
        gyroscope(double /*mass*/, double /*k*/, double /*c*/, double /*Drive Force*/);
        
        //Description: Sets or resets values used to calculate masses position within the gyroscope
        void initialize(double /*mass*/, double /*k*/, double /*c*/, double /*Drive Force*/);

        //Description: Returns the drive acceleration whitch is partly used to find position of mass
        double state_deriv_getAccel_drive();

        //Description: Returns the sense acceleration witch is the other part used to find spring mass
        double state_deriv_getAccel_sense(double /*Omega*/);
        
        //Description: updates mass velocity and position. Intended input the retulst of the acceleration values being put through an integrator
        void update_position_drive(double);
        void update_position_sense(double);
        void update_velocity_drive(double);
        void update_velocity_sense(double);
        
        //Description: uses the calculated values to get the angular velocity like a real gyroscope would
        double get_angularVelocity();
    private:
        double m; //mass
        double c; //drag constant
        double k; //spring constant
        double F_d; //initial force along one axis
        
        //Calculate
        double x; //position in the drive
        double v_x; //velocity in drive
        double y; //position in sense
        double v_y; //y is the sense

        double a_x; //acceleration drive
        double a_y; //acceleration sense
        };

#ifdef __cplusplus
    }
#endif
#endif
