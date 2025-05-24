#include "sim_object.hh"
#include <iostream>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <ctime>

using namespace std;

Sim_Object::Sim_Object(double timestep, double max_time) 
//Preparing comment
{
    dt = timestep;
    maxTime = max_time;
    time = 0.0;
}

void Sim_Object::run()

{
    initialize();

    cout << "Starting Sim";

    if (maxTime > 0)
        cout << " for " << maxTime << "seconds \n";
    else
        cout << " indefinently \n ";

    //convert to miliseconds
    int sleep_ms = static_cast<int>(dt * 1000);

    // Print the current time
    
    while (maxTime < 0 || time < maxTime) {
        update(dt);

        time = time += dt;
        cout << "Time: " << time << endl;
        this_thread::sleep_for(chrono::milliseconds(sleep_ms));

    }

    cout << "Simulation complete. \n";
}

void Sim_Object::initialize() {
    //default version
}

void Sim_Object::update(double) {
    //default
}