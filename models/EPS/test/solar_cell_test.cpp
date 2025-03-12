/*
PURPOSE: (This testing script checks if an
          output is made form the accelerometer.
          This is an unofficial test script since
          the one I tried to get running was running
          into issues.
          )
COMMANDS:
    USE this G++ command until the make file is created
    : g++ src/solar_cell.cpp test/solar_cell_test.cpp -o solar_cell_program
*/

#include <iostream>
#include <fstream>
#include <cmath>
#include "../include/solar_cell.hh"

using namespace std;

int main(){
    double ref_pos[3] = {0, 0, 3};
    double ref_ori[2] = {1, 0};
    double light_vec[3] = {2, 1, 1};

    solar_cell cell(100);
    cell.set_refrence_pos(ref_pos);
    cell.lock_axis_y();
    cell.set_refrence_ori(ref_ori);

    double current = cell.get_I_sc(light_vec);
    cout << "I = " << current << endl;


}