/*
PURPOSE: (This testing script checks if an
          output is made form the accelerometer.
          This is an unofficial test script since
          the one I tried to get running was running
          into issues.
          )
COMMANDS:
    USE this G++ command until the make file is created
    : g++ src/gyroscope.cpp test/gyroscope_test.cpp -o gyrotest_program
*/

#include <iostream>
#include <fstream>
#include <cmath>
#include "../include/gyroscope.hh"

using namespace std;

void integrate(double &, double, double, double);

int main(){
    ofstream outData("gyroscope_output.txt");
    gyroscope gyro(10, 1, 5, 10);
    double v_s = 0; //sense
    double v_d = 0; //drive
    double x_s = 10;
    double x_d = 0;

    double a_s;
    double a_d;

    double prev_a_s;
    double prev_a_d;
    double prev_v_s;
    double prev_v_d;

    double t = 0.0;
    double dt = 0.01;

    gyro.update_position_drive(x_d);
    gyro.update_position_sense(x_s);
    gyro.update_velocity_drive(v_d);
    gyro.update_velocity_sense(v_s);

    double omega = 134;

    outData << "t, w, w_g" << endl;

    for(t; t<10; t = t+dt){
        if (t > 5 && t<7)
            omega = -234;
        if (t > 7)
            omega = omega + 0.8;

        prev_v_s = v_s;
        prev_v_d = v_d;
        prev_a_s = a_s;
        prev_a_d = a_d;

        a_d = gyro.state_deriv_getAccel_drive();
        a_s = gyro.state_deriv_getAccel_sense(omega);

        integrate(v_d, prev_a_d, a_d, dt);
        integrate(v_s, prev_a_s, a_s, dt);
        integrate(x_d, prev_v_d, v_d, dt);
        integrate(x_s, prev_v_s, v_s, dt);

        gyro.update_position_drive(x_d);
        gyro.update_position_sense(x_s);
        gyro.update_velocity_drive(v_d);
        gyro.update_velocity_sense(v_s);

        cout << "t=" << t << " a_d=" << a_d << " a_s="<< a_s << " v_d=" << v_d << " v_s" << v_s << " x_d=" << x_d << " x_s=" << x_s << endl; 
        cout << "omega ->" << gyro.get_angularVelocity() << endl;
        double w_g = gyro.get_angularVelocity();
        outData << t << ", " << omega << ", " << w_g << endl;
    }

}

void integrate(double& val, double before, double after, double dt){
    double interval = 0.5*(before+after)*dt;
    val = val + interval;
}