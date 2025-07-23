#include "../include/bus_modified.hh"
#include <iostream>
#include <cmath>

bus::bus() {}

bus::bus(double source_voltage, int num_nodes, double resistance, SourceType type) {
    initialize(source_voltage, num_nodes, resistance, type);
}

void bus::initialize(double source_voltage, int num_nodes, double resistance, SourceType type) {
    start_V = source_voltage;
    num_of_nodes = num_nodes;
    R_bus = resistance;
    source_type = type;

    I_total = 0.0;
    P_total = 0.0;
    V = start_V;

    for (int i = 0; i < num_of_nodes; ++i) {
        nodes[i] = Node{};
        nodes[i].on = false;
        nodes[i].active = false;
    }
}

void bus::turn_node_on(int n, double required_power, double input_voltage, double max_voltage) {
    nodes[n].on = true;
    nodes[n].P_draw = required_power;
    nodes[n].V_req = max_voltage;
    nodes[n].R = (max_voltage * max_voltage) / required_power;
    nodes[n].I_draw = input_voltage / nodes[n].R;
}

void bus::turn_node_on(int n, double required_power, double max_voltage) {
    nodes[n].on = true;
    nodes[n].P_draw = required_power;
    nodes[n].V_req = max_voltage;
    nodes[n].R = (max_voltage * max_voltage) / required_power;
    nodes[n].I_draw = max_voltage / nodes[n].R;
}

void bus::turn_node_off(int n) {
    nodes[n].on = false;
    nodes[n].active = false;
    nodes[n].I = 0.0;
    nodes[n].P = 0.0;
    nodes[n].V_at_node = 0.0;
}

void bus::update_start_voltage(double voltage) { start_V = voltage; }

void bus::update_voltage(double I_estimate) {
    if (source_type == IDEAL_VOLTAGE) {
        V = start_V;
    } else {
        V = start_V - I_estimate * R_bus;
        if (V < 0) V = 0;
    }
}

void bus::state_update() {
    // For IDEAL_VOLTAGE or INTERNAL_RESISTANCE — assume ideal current needed
    double I_estimate = 0;
    for (int i = 0; i < num_of_nodes; ++i)
        if (nodes[i].on)
            I_estimate += nodes[i].I_draw;

    update_voltage(I_estimate);
    
    //needed so that we can save the actual state withough constently resetting to zero. usful for parallel so current is always a value
    double I_tot = 0.0; 
    double P_tot = 0.0;

    for (int i = 0; i < num_of_nodes; ++i) {
        Node& n = nodes[i];
        if (!n.on) continue;

        n.V_at_node = V;
        if (V >= n.V_req) {
            n.I = n.I_draw;
            n.P = n.I * V;
            n.active = true;

            I_tot += n.I;
            P_tot += n.P;

            I_total = I_tot;
            P_total = P_tot;
        } else {
            I_total = 0.0;
            P_total = 0.0;
            n.active = false;
        }
    }
}

void bus::state_update(double input_current) {
    // CURRENT_LIMITED
    update_voltage(input_current);

    //needed so that we can save the actual state withough constently resetting to zero. usful for parallel so current is always a value
    double I_tot = 0.0; 
    double P_tot = 0.0;

    double I_available = input_current;

    for (int i = 0; i < num_of_nodes; ++i) {
        Node& n = nodes[i];
        if (!n.on) continue;

        n.V_at_node = V;
        if (V < n.V_req || I_available <= 0) {
            n.I = n.P = 0.0;
            n.active = false;
            continue;
        }

        double I_take = std::min(n.I_draw, I_available);
        n.I = I_take;
        n.P = I_take * V;
        n.active = I_take > 0;

        I_available -= I_take;
        I_tot += n.I;
        P_tot += n.P;
    }

    I_total += I_tot;
    P_total += P_tot;
}

void bus::state_update_power(double input_power) {
    // POWER_LIMITED
    double I_estimate = 0;
    for (int i = 0; i < num_of_nodes; ++i)
        if (nodes[i].on)
            I_estimate += nodes[i].I_draw;

    update_voltage(I_estimate);

    //needed so that we can save the actual state withough constently resetting to zero. usful for parallel so current is always a value
    double I_tot = 0.0; 
    double P_tot = 0.0;

    double P_available = input_power;

    for (int i = 0; i < num_of_nodes; ++i) {
        Node& n = nodes[i];
        if (!n.on) continue;

        n.V_at_node = V;
        if (V < n.V_req || P_available <= 0) {
            n.I = n.P = 0.0;
            n.active = false;
            continue;
        }

        double P_take = std::min(n.P_draw, P_available);
        double I_take = P_take / V;

        n.I = I_take;
        n.P = P_take;
        n.active = P_take > 0;

        P_available -= P_take;
        I_tot += n.I;
        P_tot += n.P;
    }

    I_total = I_tot;
    P_total = P_tot;
}

double bus::get_node_V(int n) const { return nodes[n].V_at_node; }
double bus::get_node_P(int n) const { return nodes[n].P; }
double bus::get_node_I(int n) const { return nodes[n].I; }
double bus::get_node_R(int n) const { return nodes[n].R; }
double bus::get_bus_resistance() { return R_bus; }
double bus::get_total_I() { return I_total; }
double bus::get_total_P() { return P_total; }

double bus::get_drawn_I() const {
    double sum = 0;
    for (int i = 0; i < num_of_nodes; ++i)
        if (nodes[i].on)
            sum += nodes[i].I_draw;
    return sum;
}

double bus::get_drawn_P() const {
    double sum = 0;
    for (int i = 0; i < num_of_nodes; ++i)
        if (nodes[i].on)
            sum += nodes[i].P_draw;
    return sum;
}