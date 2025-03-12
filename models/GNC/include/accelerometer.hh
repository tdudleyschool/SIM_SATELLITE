/*
PURPOSE:	Simulates An Accelerometer That Is Restricted To One Axis
        	The simple accelerometer could be simulated by a second-
        	order spring mass damper system.

NOTE:   	MEMS accelerometer is based on the
        	a system modeling a second order simple
        	harmonic oscillator

TERMS USED:
	-> k - spring constant
	-> c - dampaning
	-> drive force - initial force setting vibration

EQUATIONS TO MODEL:
    	-> m*d2x_r + c*dx_r + k*x_r = -m*d2x
*/

#ifndef ACCELEROMETER_HH
#define ACCELEROMETER_HH

#ifdef __cplusplus
	extern "C"
	{
#endif

class accelerometer {
	public:
    	// Description: Default constructor creating an accelerometer object
    	accelerometer();
    	
    	// Description: Constructor to initialize accelerometer object with given parameters
    	accelerometer(double /*mass*/, double /*k*/, double /*k_a*/, double /*c*/, double /*d*/, double /*initial x*/, double /*initial v*/, double /*initial a*/);
    	
    	// Description: Sets or resets values for accelerometer simulation
    	void initialize(double /*mass*/, double /*k*/, double /*k_a*/, double /*c*/, double /*d*/, double /*initial x*/, double /*initial v*/, double /*initial a*/);

    	// Description: Calculates acceleration based on external force and current state
    	double state_deriv_getAccel(double /*External Force*/, double, double);

    	// Description: Updates the position of the mass after integration step
    	void update_position(double);
    	
    	// Description: Updates the velocity of the mass after integration step
    	void update_velocity(double);
   	 
    	// Description: Returns the current acceleration of the system
    	double get_acceleration(double);
    	
    	// Description: Returns the output signal (Voltage) representing the accelerometer's measurement
    	double get_signal();

	private:
    	double m; // mass of the sample mass

    	double k_a; // gain or sensitivity of the accelerometer
    	double k; // spring constant of the system
    	double c; // system damping constant for first equation
    	double l; // damping ratio for second equation
    	double w; // resonant frequency of the system
    	double d; // 0g offset, representing the output when no acceleration is applied

    	// Calculation variables
    	double x_r; // position of the sample mass
    	double v_r; // velocity of the sample mass
    	double a_r; // acceleration of the sample mass

    	double V; // output signal, typically represented as voltage
};

#ifdef __cplusplus
	}
#endif

#endif
