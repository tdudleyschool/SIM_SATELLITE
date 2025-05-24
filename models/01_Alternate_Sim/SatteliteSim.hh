#ifndef SATTELITE_SIM_HH
#define SATTELITE_SIM_HH

#include "sim_object.hh"

class SatteliteSim : public Sim_Object {
    private:
        double value;
    protected:
        void initialize() override;
        void update(double dt) override;
    public:
        SatteliteSim(double timestep, double max_time = -1.0);
};

#endif