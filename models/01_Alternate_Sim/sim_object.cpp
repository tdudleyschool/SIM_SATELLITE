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

Sim_Object::Sim_Object(int step, double max_time) 
//Preparing comment
{
    timestep = step;
    dt = 1 / timestep;
    maxTime = max_time;
    time = 0.0;
}

void Sim_Object::run()

{
    set_realtime_priority();
    initialize();
    using clock = chrono::steady_clock;

    cout << "Starting Sim";

    if (maxTime > 0)
        cout << " for " << maxTime << "seconds \n";
    else
        cout << " indefinently \n ";

    //convert to miliseconds
    int sleep_ms = static_cast<int>(dt * 1000);

    //setting up steps
    auto step = chrono::duration<int, ratio<1, 40>>(1);
    while (maxTime < 0 || time < maxTime) {
        // Start measuring time
        auto start = clock::now();
        update(dt);
        auto finish = clock::now();

        //busy wait seemed to keep the sim running at 25 hz the most consistantly
        while (clock::now() < start + step){
            this_thread::yield();
        }

        //comparison of durations directly
        //if (finish - start > step) {
        //    this_thread::sleep_until(start+step);
        //}

        //calculate the delay
        //auto delay = step - (finish - start);
        //using namespace std::chrono_literals;
        //if (delay > 0s) {
        //    this_thread::sleep_for(delay);
        //}
        
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