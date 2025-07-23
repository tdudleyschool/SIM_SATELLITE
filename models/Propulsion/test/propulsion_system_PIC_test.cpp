//g++ -Iinclude src/Propulsion_System_PIC2D.cpp src/hall_thruster_PIC2D.cpp test/propulsion_system_PIC_test.cpp -o propulsion_test_PIC -std=c++17 -O2


#include "../include/Propulsion_System_PIC2D.hh"
#include <iostream>

int main() {
    Propulsion_System_PIC2D propulsion(2.0, 2.0, 2.0, 0.5, 1.0);

    // Orientation and pose
    double orientation[3] = {0, 0, -1};
    propulsion.set_all_thruster_ref_ori(orientation);

    double pos[3] = {0.0, 0.0, 0.0};
    double R[3][3] = {
        {  0.7071, 0, 0.7071 },
        {  0,      1, 0      },
        { -0.7071, 0, 0.7071 }
    };


    propulsion.update_all_pos_ori(pos, R);


    propulsion.initialize_all_HET_sim();
    propulsion.turn_all_on();

    for (int step = 0; step < 10; step++) {
        propulsion.update_all_pos_ori(pos, R);
        for (int i = 0; i < 7; ++i) {
            propulsion.run_step_HET_sim(i, 100 /*mass flow*/, 300.0 /*thrust*/);
        }

        double net_force[3];
        double net_pos[3];
        propulsion.get_all_force(net_force, net_pos);

        std::cout << "Step " << step
                  << " | Net Force: [" << net_force[0] << ", " << net_force[1] << ", " << net_force[2] << "]"
                  << " | Force Pos: [" << net_pos[0] << ", " << net_pos[1] << ", " << net_pos[2] << "]\n";

        //can add this part once we get conversion from massflow to amount of ions correct.
        //double total_mass_flow = propulsion.get_total_mass_flow();
        propulsion.update_tankmass();
    }

    std::cout << "Remaining tank mass: " << propulsion.get_current_tankmass() << " kg\n";
    return 0;
}