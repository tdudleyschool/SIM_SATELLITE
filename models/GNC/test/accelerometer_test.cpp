/*
PURPOSE: (This testing script checks if an
          output is made form the accelerometer.
          This is an unofficial test script since
          the one I tried to get running was running
          into issues.
          )
COMMANDS:
    USE this G++ command until the make file is created
    : g++ src/accelerometer.cpp test/accelerometer_test.cpp -o acceltest_program
*/

#include <iostream>
#include <fstream>
#include <cmath>
#include "../include/accelerometer.hh"

using namespace std;

void integrate(double &, double, double, double);
void integrateRungKutta(double&, double&, accelerometer*, double);

int main(){
    //output file to graph
    ofstream outData("accelerometer_output.txt");


    accelerometer Div_accel(10.0, 10.0, 3.0, 100, 0.0, 0.0, 0.0, 0.0);
    accelerometer* accel_point;
    accel_point = &Div_accel;
    double v_prev;
    double v = 0;
    double accel_prev;
    double spring_accel = 0;

    double x = 0.0;
    double t = 0.0;
    double dt = 0.01;

    Div_accel.update_velocity(v);
    Div_accel.update_position(x);

    double force = 12;
    double mass = 4.34;

    //actual values
    double a = 0;
    double vel = 0;
    double pos = 0;
    double a_prev;
    double vel_prev;
    //gyro values
    double a_gyro = 0;
    double vel_gyro = 0;
    double x_gyro;
    double a_gyro_prev;
    double vel_gyro_prev;

    outData << "t, a, v, x, a_g, v_g, x_g" << endl;
    for (t; t < 10; t = t + dt){
        if(t > 4){
            force = 0;
        }
        if (t > 6){
            force = -9;
        }
        v_prev = v; 
        accel_prev= spring_accel;
        spring_accel = Div_accel.state_deriv_getAccel(force, v, x);
    
        integrate(v, accel_prev, spring_accel, dt);
        integrate(x, v_prev, v, dt);
        Div_accel.update_velocity(v);
        Div_accel.update_position(x);
        //outData << "t=" << t << " a=" << spring_accel << " v=" << v << " x=" << x << endl; 
        cout << "t=" << t << " a=" << spring_accel << " v=" << v << " x=" << x << endl; 
        cout << "acceleration ->" << Div_accel.get_acceleration(1) << endl;
        //graph test
        a_gyro_prev = a_gyro;
        vel_gyro_prev = vel_gyro;

        a_gyro = Div_accel.get_acceleration(mass);

        a_prev = a;
        vel_prev = vel;

        a = force/mass;
        integrate(vel, a_prev, a, dt);
        integrate(pos, vel_prev, vel, dt);
        integrate(vel_gyro, a_gyro_prev, a_gyro, dt);
        integrate(x_gyro, vel_gyro_prev, vel_gyro, dt);

        outData << t << ", " << a << ", " << vel << ", " << pos << ", " << a_gyro << ", " << vel_gyro << ", " << x_gyro << endl;
    }
}

void integrate(double& val, double before, double after, double dt){
    double interval = 0.5*(before+after)*dt;
    val = val + interval;
}

void integrateRungKutta(double & x, double & v, accelerometer* ac, double dt){
    double k1x, k1v, k2x, k2v, k3x, k3v, k4x, k4v;
    k1x = dt * v;
    k1v = dt * ac->state_deriv_getAccel(0, x, v);
    k2x = dt * (v + 0.5 * k1v);
    k2v = dt * ac->state_deriv_getAccel(0, x+0.5*k1x, v+0.5*k1v);
    k3x = dt * (v + 0.5*k2v);
    k3v = dt * ac->state_deriv_getAccel(0, x +0.5*k2x, v+0.5*k2v);
    k4x = dt*(v+k3v);
    k4v = dt * ac->state_deriv_getAccel(0, x+k3x, v+k3v);
    cout << k1v << " " <<  k2v << " " <<  k3v << " " <<  k4v << endl;
    x = x + (k1x + 2*k2x + 2*k3x + k4x)/6.0;
    v = v + (k1v + 2*k2v + 2*k3v + k4v)/6.0;

}