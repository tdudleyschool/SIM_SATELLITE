/*
PURPOSE: (This testing script checks if an
          output is made form the accelerometer.
          This is an unofficial test script since
          the one I tried to get running was running
          into issues.
          )
COMMANDS:
    USE this G++ command until the make file is created
    : g++ src/Battery.cpp test/battery_test.cpp -o batttest_program
*/

#include <iostream>
#include <fstream>
#include <cmath>
#include "../include/Battery.hh"

using namespace std;

void integrate(double &, double, double, double);

int main(){
    ofstream outData("battery_output.txt");

    battery bat;
    double dV1 = 0.0;
    double dV1_prev;
    double dV2 = 0.0;
    double dV2_prev;

    double I = 50;
    double I_integ = 0.0;
    bat.update_I(I);

    double V1 = bat.get_V1();
    double V2 = bat.get_V2();


    double t = 0.0;
    double dt = 0.1;

    outData << "t, V_t, SOC" << endl;

    for(t; t<120; t = t+dt){
        if(t > 20.0){
            I = -20;
            bat.update_I(I);
        }
        if (t > 40.0){
            I = -50;
            bat.update_I(I);
        }
        if (t > 90.0) {
            I = 200;
            bat.update_I(I);
        }
        dV1_prev = dV1;
        dV2_prev = dV2;

        bat.update_soc(dt);
        double soc = bat.get_soc();
        bat.state_deriv_getVolteges(dV1, dV2);
        integrate(V1, dV1_prev, dV1, dt);
        integrate(V2, dV2_prev, dV2, dt);
        bat.update_V1(V1);
        bat.update_V2(V2);
        bat.update_Vt();
        double Vt = bat.get_Vt();


        outData << t << " , " << Vt << ", "<< soc << endl; 
    }

}

void integrate(double& val, double before, double after, double dt){
    double interval = 0.5*(before+after)*dt;
    val = val + interval;
}