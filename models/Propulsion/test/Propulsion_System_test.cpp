/*
PURPOSE: (This testing script for the celestial body system
          just want to see if it is being created)
COMMANDS:
    USE this G++ command until the make file is created
    : g++ src/hall_thruster.cpp src/xenon_tank.cpp src/Propulsion_System.cpp test/Propulsion_System_test.cpp -o Propulsion_System_program
*/

#include <iostream>
#include "../include/Propulsion_System.hh"

using namespace std;

int main(){
    Propulsion_System PPE_Propulsion_System;
    //Required to run
    PPE_Propulsion_System.set_all_thruster_ref_pos(10, 10, 20, 5, 4);
    PPE_Propulsion_System.set_all_thruster_ref_ori(0, 0, -1);
    PPE_Propulsion_System.set_all_thruster_specs(6000, 700, 0.57);
    
    double R[3][3] = {{0.707, -0.707, 0},
                      {0.707, 0.707, 0},
                      {0, 0, 1}};
    double cent_pos[3] = {0, 0, 0};

    PPE_Propulsion_System.update_all_pos_ori(cent_pos, R);
    PPE_Propulsion_System.turn_all_on();

    double net_force[3];
    double net_pos[3];

    double force_one[3];
    double pos_one[3];
    int index = 3;

    PPE_Propulsion_System.get_all_force(net_force, net_pos);
    PPE_Propulsion_System.turn_thruster_off(4);
    PPE_Propulsion_System.turn_thruster_off(5);
    PPE_Propulsion_System.turn_thruster_off(6);
    PPE_Propulsion_System.get_all_force(force_one, pos_one);

    cout << "Net Force: " << net_force[0] << ", " << net_force[1] << ", " << net_force[2] << endl;
    cout << "Net Position: " << net_pos[0] << ", " << net_pos[1] << ", " << net_pos[2] << endl << endl;

    cout << "Force at " << index << ": " << force_one[0] << ", " << force_one[1] << ", " << force_one[2] << endl;
    cout << "Position at " << index << ": " << pos_one[0] << ", " << pos_one[1] << ", " << pos_one[2] << endl;

    return 0;
}