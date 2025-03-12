/*
PURPOSE:    This is the implementation of file 'bus.hh'

NOTE:       Models an electrical bus with nodes, where each node draws power and current based on its operational state.

TERMS USED:
    -> V - bus voltage
    -> I - total current on the bus
    -> P - total power on the bus
    -> I_draw - current drawn by active nodes
    -> P_draw - power drawn by active nodes
    -> R - resistance at each node, calculated based on power draw
    -> prev_node - previous active node in the sequence of nodes
    -> nodes - array of bus nodes, each having individual voltage, current, and power

EQUATIONS TO MODEL:
    -> P = I * V (Power relation with current and voltage)
    -> I_draw = P_draw / V (Current draw based on power draw and voltage)
    -> R = V^2 / P_draw (Resistance calculated for each node)
*/

#include "../include/bus.hh"
#include <iostream>
#include <cmath>

double minimum_val(double value, double min)
    //Description:    Function that returns the larger of two values: the input value or the specified minimum.
    //Preconditions:  None
    //Postconditions: Returns the maximum of value and min.
{
    if (value < min)
        return min;
    else
        return value;
}

bus::bus()

{
    I_draw = 0.0;
    P_draw = 0.0;
}

bus::bus(double voltage, double node_num)
    //Description:    Constructor that initializes the bus with specified voltage and number of nodes, and sets the power and current to default values.
    //Preconditions:  node_num must be greater than zero.
    //Postconditions: Voltage is set, and each node's voltage, current, and power are initialized.
{
    V = voltage;
    I = 0.0;
    num_of_nodes = node_num;
    I_draw = 0.0;
    P_draw = 0.0;

    for (int i = 0; i < num_of_nodes; i++) {
        nodes[i].V = V;
        nodes[i].I = 0.0;
        nodes[i].P = 0.0;
    }
}

void bus::initialize(double voltage, double num_node)

{
    V = voltage;
    I = 0.0;
    num_of_nodes = num_node;
    I_draw = 0.0;
    P_draw = 0.0;

    for (int i = 0; i < num_of_nodes; i++) {
        nodes[i].V = V;
        nodes[i].I = 0.0;
        nodes[i].P = 0.0;
    }
}

void bus::turn_node_on(int n, double required_pow)
    //Description:    Turns on the specified node and assigns the required power draw. Updates node resistance and current draw.
    //Preconditions:  The node index must be valid, and required_pow must be positive.
    //Postconditions: Node is set to "on", and the power and current draw for the node are calculated.
{
    nodes[n].on = true;
    nodes[n].P_draw = required_pow;
    nodes[n].I_draw = nodes[n].P_draw / V;
    nodes[n].R = (V * V) / nodes[n].P_draw;

    int node_before = -1;
    for (int i = 0; i < n; i++) {
        if (nodes[i].on) {
            node_before = i;
        }
    }

    nodes[n].prev_node = node_before;

    for (int i = n + 1; i < num_of_nodes; i++) {
        if (nodes[i].on) {
            nodes[i].prev_node = n;
            break;
        }
    }

    I_draw = 0.0;
    P_draw = 0.0;

    for (int i = 0; i < num_of_nodes; i++) {
        if (nodes[i].on) {
            I_draw += nodes[i].I_draw;
            P_draw += nodes[i].P_draw;
        }
    }
}

void bus::turn_node_off(int n)
    //Description:    Turns off the specified node, setting its power and current draw to zero. Updates the previous node's reference.
    //Preconditions:  The node index must be valid.
    //Postconditions: Node is set to "off", and its power and current draw are reset to zero.
{
    nodes[n].on = false;
    nodes[n].P_draw = 0.0;
    nodes[n].I_draw = 0.0;
    nodes[n].I = 0;
    nodes[n].P = 0;

    for (int i = n + 1; i < num_of_nodes; i++) {
        if (nodes[i].on) {
            nodes[i].prev_node = nodes[n].prev_node;
            break;
        }
    }

    I_draw = 0.0;
    P_draw = 0.0;

    for (int i = 0; i < num_of_nodes; i++) {
        if (nodes[i].on) {
            I_draw += nodes[i].I_draw;
            P_draw += nodes[i].P_draw;
        }
    }
}



void bus::state_update(double current)
    //Description:    Updates the state of the bus, including current and power, based on the current flowing through the bus and the nodes' draw. Returned values are expected to be passed through an integration function.
    //Preconditions:  The current must be a positive value.
    //Postconditions: Total current and power are updated, and each active node's current and power draw are recalculated.
{
    I = current;
    P = I * V;

    if (I >= I_draw) {
        for (int i = 0; i < num_of_nodes; i++) {
            if (nodes[i].on) {
                if (nodes[i].prev_node == -1) {
                    nodes[i].I = nodes[i].P_draw / V;
                    nodes[i].I_out = I_draw - nodes[i].I;
                } else {
                    nodes[i].I = nodes[i].P_draw / V;
                    nodes[i].I_out = nodes[nodes[i].prev_node].I_out - nodes[i].I;
                }
                nodes[i].P = nodes[i].I * V;
            }
        }
    } else {
        for (int i = 0; i < num_of_nodes; i++) {
            if (nodes[i].on) {
                if (nodes[i].prev_node == -1) {
                    if (V / nodes[i].R <= I)
                        nodes[i].I = nodes[i].P_draw / V;
                    else
                        nodes[i].I = I;
                    nodes[i].I_out = minimum_val(I - nodes[i].I, 0.0);
                } else {
                    if (V / nodes[i].R <= nodes[nodes[i].prev_node].I_out)
                        nodes[i].I = nodes[i].P_draw / V;
                    else
                        nodes[i].I = nodes[nodes[i].prev_node].I_out;
                    nodes[i].I_out = minimum_val(nodes[nodes[i].prev_node].I_out - nodes[i].I, 0.0);
                }
                nodes[i].P = nodes[i].I * V;

                std::cout << "prev node: " << nodes[i].prev_node << " \n";
            }
        }
    }
}

void bus::update_node_drawn_P(int n, double pow)
    //Description:    Updates the power drawn by a specified node.
    //Preconditions:  The node index must be valid, and the power must be positive.
    //Postconditions: Node's power draw is updated.
{
    nodes[n].P_draw = pow;
}

void bus::update_V(double voltage)
    //Description:    Updates the voltage of the bus.
    //Preconditions:  Voltage must be a positive value.
    //Postconditions: Bus voltage is updated.
{
    V = voltage;
}

double bus::get_node_V(int n)
    //Description:    Returns the voltage at a specified node.
    //Preconditions:  The node index must be valid.
    //Postconditions: Node's voltage is returned.
{
    return nodes[n].V;
}

double bus::get_node_P(int n)
    //Description:    Returns the power drawn by a specified node.
    //Preconditions:  The node index must be valid.
    //Postconditions: Node's power is returned.
{
    return nodes[n].P;
}

double bus::get_node_I(int n)
    //Description:    Returns the current at a specified node.
    //Preconditions:  The node index must be valid.
    //Postconditions: Node's current is returned.
{
    return nodes[n].I;
}

double bus::get_drawn_P()
    //Description:    Returns the total power drawn by all active nodes.
    //Preconditions:  At least one node must be on.
    //Postconditions: Total drawn power is returned.
{
    return P_draw;
}

double bus::get_drawn_I()
    //Description:    Returns the total current drawn by all active nodes.
    //Preconditions:  At least one node must be on.
    //Postconditions: Total drawn current is returned.
{
    return I_draw;
}
