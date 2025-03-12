/*
PURPOSE: (This testing script checks if an
          output is made form the sattelite_box.
          This is an unofficial test script since
          the one I tried to get running was running
          into issues.
          )
COMMANDS:
    USE this G++ command until the make file is created
    : g++ src/motor.cpp src/control_wheels.cpp test/motor_test.cpp -o motortest_program
*/

#include <iostream>
#include "../include/motor.hh"
#include "../include/control_wheels.hh"

using namespace std;

void integrate(double &, double, double, double);

int main(){
    double I = 0;
    double dI_prev;
    double alpha = 0;
    double alpha_prev;
    double c_pos[] = {0, 0, 0};
    double R[3][3] = {{1, 0, 0},{0, 1, 0},{0, 0, 1}};

    double ref_pos[] = {0, 0, 0};
    double ref_ori[] = {1, 0, 0};

    motor cont_wheel(1.0, 0.01, 1.0, 1.0, 1.0, 2.0);
    cont_wheel.update_Voltage(-200);
    cont_wheel.set_refrence_pos(ref_pos);
    cont_wheel.set_refrence_ori(ref_ori);
    cont_wheel.update_pos_ori(c_pos, R);
    cont_wheel.update_inertia();


    double dt = 0.1;
    double t = 0.0;

    I = 0;
    cont_wheel.update_I(I);
    double dI = 0;
    
    for (t; t < 100; t = t + dt){
        I = -400;
        if(t>50){
            I = 0;
            cont_wheel.update_Voltage(0);

        }
        cout << I << endl;

        dI_prev = dI;
        alpha_prev = alpha;

        dI = cont_wheel.state_deriv_get_dI();
        integrate(I, dI_prev, dI, dt);
        cont_wheel.update_I(I);

        alpha = cont_wheel.state_dirv_getAlpha(0);
        double w;
        integrate(w, alpha_prev, alpha, dt);
        cont_wheel.update_omega(w);

        //outData << "t=" << t << " a=" << spring_accel << " v=" << v << " x=" << x << endl; 
        cout << "t: " << t << ":    I=" << I << " dI=" << dI << " a=" << alpha << " w=" << w << endl; 
    }
}

void integrate(double& val, double before, double after, double dt){
    double interval = 0.5*(before+after)*dt;
    val = val + interval;
}