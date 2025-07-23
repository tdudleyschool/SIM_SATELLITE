#ifndef BUS_HH
#define BUS_HH

#define MAX_SIZE 10

enum SourceType {
    IDEAL_VOLTAGE,
    INTERNAL_RESISTANCE,
    CURRENT_LIMITED,
    POWER_LIMITED
};

struct Node {
    double P_draw;  // desired power
    double V_req;   // required voltage to turn on

    double I_draw;  // ideal current needed if on
    double R;       // resistance of node

    double I;       // actual current supplied
    double P;       // actual power supplied
    double V_at_node;

    bool on;
    bool active;
};

class bus {
public:
    bus();
    bus(double source_voltage, int num_nodes, double resistance, SourceType type);

    void initialize(double source_voltage, int num_nodes, double resistance, SourceType type);

    void turn_node_on(int node_index, double required_power, double required_voltage, double max_voltage);
    void turn_node_on(int node_index, double required_power, double max_voltage);
    void turn_node_off(int node_index);

    void update_start_voltage(double voltage);
    void state_update();                        // Ideal voltage or internal resistance
    void state_update(double input_current);    // Current source
    void state_update_power(double input_power); // Power source

    double get_node_V(int n) const;
    double get_node_P(int n) const;
    double get_node_I(int n) const;
    double get_node_R(int n) const;
    double get_bus_resistance();
    double get_total_I();
    double get_total_P();

    double get_drawn_I() const;
    double get_drawn_P() const;

private:
    double start_V;
    double V;
    double R_bus;
    SourceType source_type;

    double I_total;
    double P_total;

    int num_of_nodes;
    Node nodes[MAX_SIZE];

    void update_voltage(double I_estimate);
    void update_draw_totals();
};

#endif