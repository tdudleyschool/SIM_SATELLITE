/*
PURPOSE: (This testing script checks if an
          output is made form the sattelite_box.
          This is an unofficial test script since
          the one I tried to get running was running
          into issues.
          )
COMMANDS:
    USE this G++ command until the make file is created
    : g++ src/Attitude_Control_System.cpp src/control_wheels.cpp src/motor.cpp test/ATT_test.cpp -o ATT_program
*/

#include <iostream>
#include "../include/Attitude_Control_System.hh"

using namespace std;

void integrate(double &, double, double, double);

int main(){
    Attitude_Control_System att;
}

void integrate(double& val, double before, double after, double dt){
    double interval = 0.5*(before+after)*dt;
    val = val + interval;
}