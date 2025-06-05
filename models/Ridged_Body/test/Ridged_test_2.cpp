#include <iostream>
#include <fstream>
#include <cmath>

#include "../include/Ridged_Body.hh"

using namespace std;

/*
PURPOSE: (This testing script checks if an
          output is made form the sattelite_box.
          This is an unofficial test script since
          the one I tried to get running was running
          into issues.
g++ src/Ridged_Body.cpp src/Satellite_Box.cpp test/Ridged_test_2.cpp -o ridged_test_2         )
COMMANDS:
    USE this G++ command until the make file is created
    : 
*/



int main(){
    ridged_body R1;
    R1.initialize_body(4.0, 3.0);
    double R_M[3][3];
    R1.get_R(R_M);

    //printing Rotation first
    cout << "Original Matrix" << endl;
    for(int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            cout << R_M[i][j] << ", ";
        }
        cout << endl;
    }

    double w[3];
    double q_ori[4];
    R1.update_torque(0, 2, 0);
    R1.get_w(w);
    R1.get_Qori(q_ori);

    double t = 0.0;
    double dt = 0.01;
    cout  << "Omega initial (w): " << w[0]  << ", " << w[1] << ", " << w[2] << "\n";
    cout << "Quaterniod: " << q_ori[0] << ", " << q_ori[1] << ", " << q_ori[2] << ", " << q_ori[3] << "\n";
    while (t <= 4.34) {
        double dw[3];
        R1.state_deriv_get_alpha(dw);
        w[0] = w[0] + dt * dw[0];
        w[1] = w[1] + dt * dw[1];
        w[2] = w[2] + dt * dw[2];

        R1.update_w(w[0], w[1], w[2]);
        cout  << "Omega initial (w): " << w[0]  << ", " << w[1] << ", " << w[2] << "\n";

        double dq[4];
        R1.state_deriv_getQori(dq);
        //for some reason i still need to add the 0.5. idk why.
        q_ori[0] = q_ori[0] + dt * dq[0]*0.5;
        q_ori[1] = q_ori[1] + dt * dq[1]*0.5;
        q_ori[2] = q_ori[2] + dt * dq[2]*0.5;
        q_ori[3] = q_ori[3] + dt * dq[3]*0.5;

        R1.update_Qori(q_ori);
        
        t = t + dt;
    }

    R1.get_R(R_M);
    cout << "Final Matrix" << endl;
    for(int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            cout << R_M[i][j] << ", ";
        }
        cout << endl;
    }
}