/*
PURPOSE: (This testing script checks if an
          output is made form the accelerometer.
          This is an unofficial test script since
          the one I tried to get running was running
          into issues.
          )
COMMANDS:
    USE this G++ command until the make file is created
    : g++ src/Solar_Power_System.cpp test/solar_power_test.cpp -o solar_test_program
*/

#include <iostream>
#include <fstream>
#include <cmath>
#include "../include/Solar_Power_System.hh"

using namespace std;

void integrate(double &, double, double, double);

int main(){
    ofstream outData("solar_output.txt");

    solar_array sol(120/*OC V*/, 20/*I_CO*/, 120/*Max V*/, 20/*Max I*/, 0.2586 /*terminal V*/);
    double voltage = 0.0;
    double I;
    double I2;
    double V = 0;

    outData << "V, I \n";
    for(V = 0; V < 123; V = V + 0.1){
        sol.update_V(V);
        sol.update_I();
        I = sol.get_I();
        //I2 = sol.get_I2();

        outData << V << ", " << I << endl;
    }
}

void integrate(double& val, double before, double after, double dt){
    double interval = 0.5*(before+after)*dt;
    val = val + interval;
}