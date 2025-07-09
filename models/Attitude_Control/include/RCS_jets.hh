#ifndef REACTION_CONTROL_THRUSTERS_HH
#define REACTION_CONTROL_THRUSTERS_HH

#include <vector>
#include <array>

// Represents a single cold gas RCS jet
struct Thruster {
    std::array<double, 3> position;   // (x, y, z) in meters
    std::array<double, 3> direction;  // unit thrust vector
    double maxForce;                 // Newtons
    bool isFiring = false;          // Commanded on/off
};

class ReactionControlThrusters {
public:
    ReactionControlThrusters(double width, double height, double depth, double maxThrust);

    // Command a desired torque vector (Nm)
    void commandTorque(const std::array<double, 3>& desiredTorque);

    // Get which thrusters are firing
    std::vector<bool> getFiringStates() const;

    // Get total torque currently applied
    std::array<double, 3> getNetTorque() const;

    const std::vector<Thruster>& getThrusters() const;

private:
    std::vector<Thruster> thrusters;
};

#endif
