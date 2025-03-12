/*
PURPOSE:    This class simulates a bus system that manages nodes 
            by controlling power supply and consumption. It tracks 
            voltage, current, and power for each node in the system.

NOTE:       The bus can handle a limited number of nodes, defined 
            by the MAX_SIZE constant. It supports turning nodes on 
            and off, updating their power requirements, and monitoring 
            the overall power and current drawn from the bus.

TERMS USED:
    -> V - voltage of the bus
    -> I_draw - total current drawn from the bus
    -> P_draw - total power drawn by the nodes
    -> num_of_nodes - number of active nodes in the bus system
*/

#include "../../structs.hh"
//#include <iostream>
#define MAX_SIZE 10

#ifndef BUS_HH
#define BUS_HH

#ifdef __cplusplus
	extern "C"
	{
#endif

class bus {
	public:
		//Description: Constructor with default in itializations
		bus();

    	// Description: Constructor to initialize the bus with voltage and number of nodes
    	bus(double /*Voltage*/, double /*num_of_nodes*/);

		// Description: Initialize both voltage and number of nodes in bus
		void initialize(double /*voltage*/, double /*num _of_nodes*/);

    	// Description: Turns a specific node on with the required power
    	void turn_node_on(int /*node index*/, double /*required power*/);

    	// Description: Turns a specific node off
    	void turn_node_off(int /*node index*/);

    	// Description: Updates the state of the bus with incoming current
    	void state_update(double /*incoming current*/);

    	// Description: Updates the drawn power for a specific node
    	void update_node_drawn_P(int /*node index*/, double /*power*/);

    	// Description: Updates the bus voltage
    	void update_V(double /*voltage*/);
    	
    	// Description: Retrieves the voltage for a specific node
    	double get_node_V(int /*node index*/);
    	
    	// Description: Retrieves the power consumption for a specific node
    	double get_node_P(int /*node index*/);
    	
    	// Description: Retrieves the current for a specific node
    	double get_node_I(int /*node index*/);

    	// Description: Retrieves the total drawn power from the bus
    	double get_drawn_P();

    	// Description: Retrieves the total drawn current from the bus
    	double get_drawn_I();

	private:
    	double V; // Voltage of the bus
    	double I_draw; // Total current drawn from the bus
    	double I; // Current being supplied to the nodes
    	double P_draw; // Total power drawn by the nodes
    	double P; // Power being supplied to the nodes
    	double Extra_I; // Additional current for overhead

    	int head; // Index for tracking the head node
    	int size; // Current size of the bus (number of active nodes)
    	int num_of_nodes; // Maximum number of nodes in the bus

    	Node nodes[MAX_SIZE]; // Array to hold the nodes in the bus

};

#ifdef __cplusplus
	}
#endif

#endif
