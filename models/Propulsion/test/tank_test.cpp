/*
PURPOSE: (This testing script for the celestial body system
          just want to see if it is being created)
COMMANDS:
    USE this G++ command until the make file is created
    : g++ src/xenon_tank.cpp test/tank_test.cpp -o xenon_tank_program
*/

#include <iostream>
#include "../include/xenon_tank.hh"

using namespace std;

int main(){
    xenon_tank tank;
    tank.update_mass(100);
    double mass;
    cout << "tank mass is: " << tank.getmass();
}