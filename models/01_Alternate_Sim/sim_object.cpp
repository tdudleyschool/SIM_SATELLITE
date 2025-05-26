#include "sim_object.hh"
#include <iostream>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <ctime>
#include <time.h>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#endif

using namespace std;

#ifdef __linux__
void set_realtime_priority() {
    sched_param sch_params;
    sch_params.sched_priority = 80;
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &sch_params) != 0) {
        perror("Failed to set real-time priority");
    }
}
#else
void set_realtime_priority() {
    // No-op on non-Linux platforms
}
#endif

Sim_Object::Sim_Object(double timestep, double max_time) 
//Preparing comment
{
    dt = timestep;
    maxTime = max_time;
    time = 0.0;
}

void Sim_Object::run()

{
    set_realtime_priority();
    initialize();

    cout << "Starting Sim";

    if (maxTime > 0)
        cout << " for " << maxTime << "seconds \n";
    else
        cout << " indefinently \n ";

    //convert to miliseconds
    int sleep_ms = static_cast<int>(dt * 1000);

    // Print the current time
    auto next_tick = chrono::steady_clock::now() + chrono::duration_cast<chrono::steady_clock::duration>(chrono::duration<double>(dt));
    
    while (maxTime < 0 || time < maxTime) {
        // Start measuring time

        auto start = chrono::steady_clock::now();
        update(dt);

        //time = time += dt;

        auto now = chrono::steady_clock::now();

        double lateness = chrono::duration<double>(now - next_tick).count();

        //if (lateness > 0.01) { // More than 2ms late
            //cout << "Missed deadline by " << lateness * 1000 << " ms\n";
        ///}
        
        next_tick = next_tick + chrono::duration_cast<chrono::steady_clock::duration>(chrono::duration<double>(dt));
        auto sleep_duration = next_tick - chrono::steady_clock::now();
        cout << "sleep duration " << chrono::duration<double>(sleep_duration).count() << "\n";
        if (sleep_duration > chrono::steady_clock::duration::zero()) {
            this_thread::sleep_for(sleep_duration);
        }
        else
        {
            cout << "skip \n";
        }

        
        
        // Stop measuring time and calculate the elapsed time
        auto end = chrono::steady_clock::now();
        double elapsed = chrono::duration<double>(end - start).count();

        cout << "Elepst time: " << elapsed << endl;
    }

    cout << "Simulation complete. \n";
}

void Sim_Object::initialize() {
    //default version
}

void Sim_Object::update(double) {
    //default
}