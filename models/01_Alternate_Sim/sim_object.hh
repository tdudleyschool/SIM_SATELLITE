#ifndef SIMOBJECT_HH
#define SIMOBJECT_HH

class Sim_Object {
    protected:
        double dt;
        double maxTime;
        double time;

        virtual void initialize();

        virtual void update(double dt);
    public:
        Sim_Object(double timestep, double max_time = -1.0);
        virtual ~Sim_Object() = default;

        void run();
};
#endif