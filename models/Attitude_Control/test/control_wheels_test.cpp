/*
PURPOSE: (This testing script checks if an
          output is made form the sattelite_box.
          This is an unofficial test script since
          the one I tried to get running was running
          into issues.
          )
COMMANDS:
    USE this G++ command until the make file is created
    : g++ src/control_wheels.cpp test/control_wheels_test.cpp -o wheeltest_program
*/

#include <iostream>
#include "../include/control_wheels.hh"

using namespace std;

int main(){
    control_wheels c_wheel(12.0, 8.0, 6.0);

    //Test The Value Of The Inertial Matrix
    c_wheel.calc_I_0();
    double inertial_matrix[3][3];
    double inverse_inertial_matrix[3][3];
    c_wheel.get_I_0(inertial_matrix);
    c_wheel.get_inv_I_0(inverse_inertial_matrix);

    cout << "INERTIAL MATRIX: " << "\n";
    for(int i = 0; i < 3; i++){
        cout << inertial_matrix[i][0] << " " << inertial_matrix[i][1] << " " << inertial_matrix[i][2] << "\n";
    };

    cout << "INVERSE INERTIAL MATRIX: " << "\n";
    for(int i = 0; i < 3; i++){
        cout << inverse_inertial_matrix[i][0] << " " << inverse_inertial_matrix[i][1] << " " << inverse_inertial_matrix[i][2] << "\n";
    };

    //Test How Points Adapt To Center Position
}