/*
PURPOSE: (Testing bus capabilities
          )
COMMANDS:
    USE this G++ command until the make file is created
    : g++ src/force_torque_tracker.cpp src/forces_test.cpp src/functions.cpp -o forces_program
*/

#include <iostream>
#include <fstream>
#include <cmath>
#include "../include/force_torque_tracker.hh"
#include "../include/functions.hh"

using namespace std;

int main(){
    double force[3];
    double torque[3];

    force_torque_tracker tracker;
    tracker.getNetForce(force);

    cout << force[0] << ", " << force[1] << ", " << force[2] << endl;

    tracker.addForce(2, 1, 0, 0, 2, 0);
    tracker.addForce(2, 4, 0, 0, 1, 1);

    tracker.getNetForce(force);
    tracker.getNetTorque(torque);

    cout << force[0] << ", " << force[1] << ", " << force[2] << endl;
    cout << torque[0] << ", " << torque[1] << ", " << torque[2] << endl;

    tracker.resetValues();

    tracker.getNetForce(force);

    cout << force[0] << ", " << force[1] << ", " << force[2] << endl;

    //Function test

    double loc1[3] = {2, 1, 3};
    double loc2[3] = {-1, 7, -5};
    cout << "Gravitation Force Test: " << gravForceMagnitude(9876975, 345678, loc1, loc2) << endl;

    double unit_dir[3];
    
    getUnitDir(loc1, loc2, unit_dir);

    cout << "Unit Direction: " << unit_dir[0] << ", " << unit_dir[1] << ", " << unit_dir[2] << endl;

}