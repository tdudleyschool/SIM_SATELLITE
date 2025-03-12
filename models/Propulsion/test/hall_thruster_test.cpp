/*
PURPOSE: (This testing script for the celestial body system
          just want to see if it is being created)
COMMANDS:
    USE this G++ command until the make file is created
    : g++ src/hall_thruster.cpp test/hall_thruster_test.cpp -o thruster_program
*/

#include <iostream>
#include "../include/hall_thruster.hh"

using namespace std;

int main(){
    double pos[3];
    double ori[3];
    double ref_pos[3] = {2, 0, 0};
    double ref_ori[3] = {1, 0, 0};
    double cent_pos[3] = {0, 0, 0};

    double force[3];
    double f_pos[3];

    hall_thruster thruster(6000, 600, 0.59);
    thruster.switch_stateon();
    thruster.set_refrence_pos(ref_pos);
    thruster.set_refrence_ori(ref_ori);
    thruster.get_pos(pos);
    
    cout << "Position: " << pos[0] << ", " << pos[1] << ", " << pos[2] << endl;
    
    double R[3][3] = {{0.707, -0.707, 0},
                      {0.707, 0.707, 0},
                      {0, 0, 1}};

    thruster.update_pos_ori(cent_pos, R);

    thruster.get_pos(pos);
    thruster.get_ori(ori);

    cout << "Updated Position: " << pos[0] << ", " << pos[1] << ", " << pos[2] << endl;
    cout << "Updated Orientation: " << ori[0] << ", " << ori[1] << ", " << ori[2] << endl;

    thruster.get_force(100, force, f_pos);

    cout << "Force: " << force[0] << ", " << force[1] << ", " << force[2] << '\n';
    cout << "Force Location: " << f_pos[0] << ", " << f_pos[1] << ", " <<f_pos[2] << endl;


}