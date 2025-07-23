/*
PURPOSE:    This class models a battery system using an equivalent circuit model. It simulates battery behavior based on 
            internal resistances, capacitors, and state of charge (SOC) and computes voltages, currents, and SOC changes 
            over time. 

NOTE:       This code assumes a simple battery model, where SOC is bounded between 0.2 and 0.9, and current flow (I) is
            zero when the SOC is at these limits. The `findOCV` function approximates the Open Circuit Voltage (OCV) based
            on SOC.

TERMS USED:
    -> SOC - State of charge of the battery, between 0.2 and 0.9.
    -> OCV - Open Circuit Voltage, derived from SOC.
    -> I - Current flowing through the battery (input/output).
    -> Vt - Terminal voltage of the battery.
    -> V1, V2 - Voltages across internal components of the equivalent circuit.
    -> R0, R1, R2 - Resistances in the battery circuit.
    -> C1, C2 - Capacitances in the battery circuit.
*/

#include "../include/Battery.hh"
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <iostream>

//Prototypes
double findOCV(double SOC);

battery::battery()
    //Description:      Default constructor that initializes battery parameters with predefined values.
    //Preconditions:    None
    //Postconditions:   Battery parameters (capacity, resistances, capacitances, SOC) are initialized with default values.
{
	Q = 36000;
	R_0 = 0.001;
	R1 = 0.02;
	R2 = 0.01;
	C1 = 1000;
	C2 = 2000;
	soc = 0.8;

	V1 = 0;
	V2 = 0;
}

battery::battery(double capacity, double res0, double res1, double res2, double cap1, double cap2, double state_charge)
    //Description:      Overloaded constructor to initialize the battery with custom values.
    //Preconditions:    Valid values for capacity, resistances, capacitances, and SOC must be provided.
    //Postconditions:   Battery parameters are initialized with custom values.
{
	Q = capacity;
	R_0 = res0;
	R1 = res1;
	R2 = res2;
	C1 = cap1;
	C2 = cap2;
	soc = state_charge;

	V1 = 0;
	V2 = 0;
}

void battery::initialize(double capacity, double res0, double res1, double res2, double cap1, double cap2, double state_charge)
    //Description:      Initializes the battery parameters.
    //Preconditions:    Valid values for capacity, resistances, and capacitances must be provided.
    //Postconditions:   Battery parameters are initialized, and internal voltages are reset.
{
	Q = capacity;
	R_0 = res0;
	R1 = res1;
	R2 = res2;
	C1 = cap1;
	C2 = cap2;
    soc = state_charge;

	V1 = 0;
	V2 = 0;
}

void battery::state_deriv_getVolteges(double& dV1, double& dV2)
    //Description:      Calculates the rate of change of voltages across the internal components (V1, V2). Intended to be used in integration loop.
    //Preconditions:    The current (I) must be set, and valid capacitance and resistance values must be initialized.
    //Postconditions:   Returns the derivatives of V1 and V2, which can be integrated externally to update these voltages.
{
	dV1 = (I / C1) - (V1 / (R1 * C1));
	dV2 = (I / C2) - (V2 / (R2 * C2));
}

double battery::state_deriv_getI()
    //Description:      Returns the current flowing through the battery. Intented to be used in integration loop.
    //Preconditions:    None
    //Postconditions:   The current (I) is returned.
{
	return I;
}

void battery::update_I(double current)
    //Description:      Updates the current flowing through the battery.
    //Preconditions:    Valid current value must be provided.
    //Postconditions:   The current (I) is updated with the provided value.
{
	I = current;
}

void battery::update_soc(double dt)
    //Description:      Updates the state of charge (SOC) based on current (I) and time step (dt).
    //Preconditions:    A valid time step (dt) must be provided. The current (I) must be updated before calling this function.
    //Postconditions:   The SOC is updated, and if SOC is outside the 0.2 - 0.9 range, it is clamped and the current (I) is set to 0.
{
	soc = soc - I * dt / Q;
	if (soc < 0.2) {
    	soc = 0.2;
    	I = 0;
	}
	else if (soc > 0.9) {
    	soc = 0.9;
    	I = 0;
	}
	ocv = findOCV(soc);
}

void battery::update_V1(double Voltage)
    //Description:      Updates the voltage across the first internal component (V1).
    //Preconditions:    A valid voltage value must be provided.
    //Postconditions:   V1 is updated.
{
	V1 = Voltage;
}

void battery::update_V2(double Voltage)
    //Description:      Updates the voltage across the second internal component (V2).
    //Preconditions:    A valid voltage value must be provided.
    //Postconditions:   V2 is updated.
{
	V2 = Voltage;
}

void battery::update_Vt()
    //Description:      Updates the terminal voltage (Vt) of the battery based on the current, OCV, and internal voltages.
    //Preconditions:    The current (I), OCV, and internal voltages (V1, V2) must be updated before calling this function.
    //Postconditions:   The terminal voltage (Vt) is calculated and updated.
{
	Vt = ocv - I * R_0 - V1 - V2;
}

double battery::get_Vt()
    //Description:      Returns the terminal voltage (Vt) of the battery.
    //Preconditions:    The terminal voltage (Vt) must be updated before calling this function.
    //Postconditions:   The terminal voltage (Vt) is returned.
{
	return Vt;
}

double battery::get_soc()
    //Description:      Returns the state of charge (SOC) of the battery.
    //Preconditions:    SOC must be updated before calling this function.
    //Postconditions:   The SOC is returned.
{
	return soc;
}

double battery::get_V1()
    //Description:      Returns the voltage across the first internal component (V1).
    //Preconditions:    V1 must be updated before calling this function.
    //Postconditions:   The voltage (V1) is returned.
{
	return V1;
}

double battery::get_V2()
    //Description:      Returns the voltage across the second internal component (V2).
    //Preconditions:    V2 must be updated before calling this function.
    //Postconditions:   The voltage (V2) is returned.
{
	return V2;
}

double findOCV(double SOC)
    //Description:      Calculates the Open Circuit Voltage (OCV) based on the state of charge (SOC).
    //Preconditions:    A valid SOC value between 0 and 1 must be provided.
    //Postconditions:   The OCV is calculated and returned based on a linear relationship with SOC.
{
	return 10.7 + 0.4 * SOC;
}
