#include <cmath>
#include "../include/accelerometer.hh"

accelerometer::accelerometer()
	//Description:  	Default constructor initializing default values for the accelerometer model.
	//Preconditions:	None
	//Postconditions:   Initializes mass, spring constant, damping coefficient, sensitivity, and other variables.
{
	m = 0.01; //kg
	k = 100; //N/m
	c = 0.005;
	a_r = 9.81;
	x_r = 0;
	v_r = 0;
	d = 0;
	k_a = 10; //sensitivity

	w = sqrt(k/m);
	l = c / (2 * sqrt(k*m));
}

accelerometer::accelerometer(
	double mass,
	double spr_const,
	double sensitivity,
	double damp_const,
	double offset,
	double init_x,
	double init_v,
	double init_a)
	//Description:  	Parameterized constructor for accelerometer model with user-defined values.
	//Preconditions:	Mass (kg), spring constant (N/m), sensitivity, damping constant, offset, initial position, initial velocity, and initial acceleration.
	//Postconditions:   Initializes the accelerometer model based on provided input values.
{
	m = mass;
	k = spr_const;
	k_a = sensitivity;
	c = damp_const;
	d = offset;
	a_r = init_a;

	x_r = init_x;
	v_r = init_v;

	w = sqrt(k/m);
	l = c / (2 * sqrt(k*m));
}

void accelerometer::initialize(
	double mass,
	double spr_const,
	double sensitivity,
	double damp_const,
	double offset,
	double init_x,
	double init_v,
	double init_a)
	//Description:  	Initializes the accelerometer with user-defined values.
	//Preconditions:	Mass (kg), spring constant (N/m), sensitivity, damping constant, offset, initial position, and initial velocity.
	//Postconditions:   Updates the internal state variables with the provided values.
{
	m = mass;
	k = spr_const;
	k_a = sensitivity;
	c = damp_const;
	d = offset;

	a_r = init_a;
	x_r = init_x;
	v_r = init_v;

	w = sqrt(k/m);
	l = c / (2 * sqrt(k*m));
}

double accelerometer::state_deriv_getAccel(double F_ext, double v_r, double x_r)
	//Description:  	Calculates the current acceleration of the system based on external force, velocity, and position.
	//Preconditions:	External force (N), velocity (v_r), and position (x_r).
	//Postconditions:   Returns the calculated acceleration (a_r).
{
	a_r = (F_ext - c*v_r - k*x_r) / m;
	return a_r;
}

void accelerometer::update_position(double pos)
	//Description:  	Updates the position of the accelerometer model.
	//Preconditions:	New position (pos) of the accelerometer.
	//Postconditions:   Internal position (x_r) is updated with the new value.
{
	x_r = pos;
}

void accelerometer::update_velocity(double velocity)
	//Description:  	Updates the velocity of the accelerometer model.
	//Preconditions:	New velocity (velocity) of the accelerometer.
	//Postconditions:   Internal velocity (v_r) is updated with the new value.
{
	v_r = velocity;
}

double accelerometer::get_acceleration(double mass)
	//Description:  	Calculates and returns the acceleration based on the current state.
	//Preconditions:	Mass (kg) to be used in the calculation.
	//Postconditions:   Returns the calculated acceleration.
{
	return (a_r*m + c*v_r + k*x_r) / mass;
}

double accelerometer::get_signal()
	//Description:  	Returns the accelerometer signal, which is a combination of sensitivity, acceleration, frequency, and offset.
	//Preconditions:	None
	//Postconditions:   Returns the calculated signal (k_a * a_r + w + d).
{
	return k_a * a_r + w + d;
	//try k_a * actual acceleration + w + d
}
