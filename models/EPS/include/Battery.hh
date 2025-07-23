/*
PURPOSE:    This class simulates a battery's behavior, modeling its 
            state of charge, voltages, and current through a simple 
            electrical circuit representation.

NOTE:       The battery model takes into account various parameters 
            such as resistances, capacitances, and open-circuit voltage 
            to calculate the state of charge and the resulting voltages.

TERMS USED:
    -> R_0 - initial resistance
    -> Q - capacity of the battery
    -> V1, V2 - voltages across different components
    -> R1, R2 - resistances in the circuit
    -> C1, C2 - capacitances in the circuit
    -> soc - state of charge
    -> I - current flowing through the battery

EQUATIONS TO MODEL:
    - The specific equations governing the behavior of the battery
      will depend on the circuit configuration and battery chemistry.
*/

#ifndef BATTERY_HH
#define BATTERY_HH

#ifdef __cplusplus
	extern "C"
	{
#endif

class battery {
	private:
    	double R_0; // Initial resistance of the battery
    	double Q;   // Capacity of the battery

    	double V1;  // Voltage across the first component
    	double V2;  // Voltage across the second component
    	double R1;  // Resistance of the first component
    	double R2;  // Resistance of the second component
    	double C1;  // Capacitance of the first component
    	double C2;  // Capacitance of the second component
    	double ocv; // Open-circuit voltage
    	double soc; // State of charge
    	double I;   // Current flowing through the battery

    	double Vt;  // Total voltage output
    	double V_min; // Minimum voltage level

	public:
    	// Description: Default constructor creating a battery object
    	battery();
    	
    	// Description: Constructor to initialize battery object with given parameters
    	battery(double /*capacity*/, double /*resistor0*/, double /*resistor1*/, 
               double /*resistor2*/, double /*capacitor1*/, double /*capacitor2*/, 
               double /*state_of_charge*/);
               
    	// Description: Sets or resets values used for battery simulation
    	void initialize(double /*capacity*/, double /*resistor0*/, 
                     double /*resistor1*/, double /*resistor2*/, 
                     double /*capacitor1*/, double /*capacitor2*/, double /*state of charge*/);
                     
    	// Description: Calculates the voltages based on the current state
    	void state_deriv_getVolteges(double& /*dVoltage1*/, double&/*dVoltage2*/);
    	
    	// Description: Calculates the current flowing through the battery
    	double state_deriv_getI();
    	
    	// Description: Updates the current flowing through the battery
    	void update_I(double /*current*/);
    	
    	// Description: Updates the state of charge of the battery
    	void update_soc(double /*delta time*/);
    	
    	// Description: Updates the voltage across the first component
    	void update_V1(double /*Voltage 1*/);
    	
    	// Description: Updates the voltage across the second component
    	void update_V2(double /*Voltage 2*/);
    	
    	// Description: Updates the total voltage output
    	void update_Vt();
    	
    	// Description: Returns the voltage across the first component
    	double get_V1();
    	
    	// Description: Returns the voltage across the second component
    	double get_V2();
    	
    	// Description: Returns the total voltage output
    	double get_Vt();
    	
    	// Description: Returns the state of charge of the battery
    	double get_soc();
};

#ifdef __cplusplus
	}
#endif

#endif
