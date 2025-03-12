#include <iostream>
#include <fstream>
#include <cmath>
#include <gtest/gtest.h>

#include "../include/Ridged_Body.hh"

using namespace std;

/*
PURPOSE: (This testing script checks if an
          output is made form the sattelite_box.
          This is an unofficial test script since
          the one I tried to get running was running
          into issues.
g++ src/Ridged_Body.cpp src/Satellite_Box.cpp test/Ridged_Body_test.cpp -o test_ridged          )
COMMANDS:
    USE this G++ command until the make file is created
    : 
*/

int main(){
    ofstream outData("rotation1_output.txt");
    ofstream outData2("rotation2_output.txt");
    ofstream outData3("rotation3_output.txt");

    ofstream outOmega("omega_output.txt");



    ridged_body R1;

    //Rotation Test With Angular Momentum
    double q[4];
    double R_M[3][3];
    double omega[3];

    R1.update_L(9000, 1000, -6000);
    R1.get_w(omega);
    R1.get_Qori(q);

    cout << "Omaga: x = " << omega[0] << " y = " << omega[1] << " z = " << omega[2] << endl;
    cout << "quat:" << q[0] << " " << q[1]  << " " << q[2] << " " << q[3] << endl;
    outOmega << omega[0] << ", " << omega[1] << ", " << omega[2];
    //Test Output Of Rotation Matrix
    R1.get_R(R_M);
    cout << "Original Matrix" << endl;
    for(int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            cout << R_M[i][j] << ", ";
        }
        cout << endl;
    }
    cout << endl;

    double dtheta = R1.get_w_mag();
    double theta = dtheta;
    cout << "Theta: " << theta << endl;
    R1.update_QoriByAngle(theta);
    /*double dquat[4];
    R1.state_deriv_getQori(dquat);
    for(int i = 0; i < 4; i++){
        q[i] = q[i] + dquat[i] * (1.9);
    }
    R1.update_Qori(q);
   
    */
    R1.get_R(R_M);
    
    cout << "Matrix After Update" << endl;
    for(int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            cout << R_M[j][i] << ", ";
            outData <<R_M[j][i];
            if (j < 2)
                outData << ", ";
        }
        cout << endl;
        if (i < 2)
            outData << endl;
    }
}