//g++ src/bus_modified.cpp test/bus_modified_test.cpp -o bus_modified_program
#include <iostream>
#include "../include/bus_modified.hh"

void print_bus(bus& b, int num_nodes, const char* label) {
    std::cout << "\n--- " << label << " ---\n";
    for (int i = 0; i < num_nodes; ++i) {
        std::cout << "Node " << i
                  << " | V: " << b.get_node_V(i) << " V"
                  << " | I: " << b.get_node_I(i) << " A"
                  << " | P: " << b.get_node_P(i) << " W"
                  << " | Active: " << (b.get_node_P(i) > 0 ? "Yes" : "No") << "\n";
    }

    std::cout << "Drawn Total: I = " << b.get_drawn_I()
              << " A | P = " << b.get_drawn_P() << " W\n";
    std::cout << "Internal Totals: I_total = " << b.get_total_I()
              << " A | P_total = " << b.get_total_P() << " W\n";
    std::cout << "Bus Resistance: R_bus = " << b.get_bus_resistance() << " Ohms\n";
}

int main() {
    const int N = 5;

    // --- 1. IDEAL VOLTAGE TEST ---
    bus ideal_bus(28.0, N, 0.0, IDEAL_VOLTAGE);
    for (int i = 0; i < N; ++i)
        ideal_bus.turn_node_on(i, 50.0, 22.0);  // 50W @ 22V ~ 2.27A
    ideal_bus.state_update();
    print_bus(ideal_bus, N, "IDEAL VOLTAGE SOURCE");

    // --- 2. INTERNAL RESISTANCE TEST ---
    bus resistive_bus(28.0, N, 0.8, INTERNAL_RESISTANCE);
    resistive_bus.turn_node_on(0, 50.0, 20.0);
    resistive_bus.turn_node_on(1, 60.0, 22.0);
    resistive_bus.turn_node_on(2, 40.0, 24.0);
    resistive_bus.state_update();
    print_bus(resistive_bus, N, "INTERNAL RESISTANCE SOURCE");

    // --- 3. CURRENT-LIMITED TEST (e.g., 7A total) ---
    bus current_bus(28.0, N, 0.3, CURRENT_LIMITED);
    for (int i = 0; i < N; ++i)
        current_bus.turn_node_on(i, 60.0, 22.0);  // ~2.73A each
    current_bus.state_update(7.0); // Limit is 7A
    print_bus(current_bus, N, "CURRENT-LIMITED SOURCE");

    // --- 4. POWER-LIMITED TEST (e.g., 120W total) ---
    bus power_bus(28.0, N, 0.2, POWER_LIMITED);
    for (int i = 0; i < N; ++i)
        power_bus.turn_node_on(i, 60.0, 26.0);
    power_bus.state_update_power(120.0);
    print_bus(power_bus, N, "POWER-LIMITED SOURCE");

    return 0;
}
