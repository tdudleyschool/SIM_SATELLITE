#include "../include/RCS_jets.hh"
#include <cmath>

ReactionControlThrusters::ReactionControlThrusters(double width, double height, double depth, double maxThrust) {
    double x = width / 2.0;
    double y = height / 2.0;
    double z = depth / 2.0;

    // Roll control (X-axis torque) - jets on +Y/-Y faces, thrust in ±Z
    thrusters.push_back({{ 0.0,  y,  0.0}, { 0.0, 0.0,  1.0}, maxThrust}); // +Z thrust at +Y face
    thrusters.push_back({{ 0.0, -y, 0.0}, { 0.0, 0.0, -1.0}, maxThrust}); // -Z thrust at -Y face
    thrusters.push_back({{ 0.0,  y,  0.0}, { 0.0, 0.0, -1.0}, maxThrust}); // -Z thrust at +Y
    thrusters.push_back({{ 0.0, -y, 0.0}, { 0.0, 0.0,  1.0}, maxThrust}); // +Z thrust at -Y

    // Pitch control (Y-axis torque) - jets on ±Z faces, thrust in ±X
    thrusters.push_back({{ 0.0, 0.0,  z}, { 1.0, 0.0, 0.0}, maxThrust});
    thrusters.push_back({{ 0.0, 0.0, -z}, {-1.0, 0.0, 0.0}, maxThrust});
    thrusters.push_back({{ 0.0, 0.0,  z}, {-1.0, 0.0, 0.0}, maxThrust});
    thrusters.push_back({{ 0.0, 0.0, -z}, { 1.0, 0.0, 0.0}, maxThrust});

    // Yaw control (Z-axis torque) - jets on ±X faces, thrust in ±Y
    thrusters.push_back({{ x, 0.0, 0.0}, { 0.0, 1.0, 0.0}, maxThrust});
    thrusters.push_back({{-x, 0.0, 0.0}, { 0.0, -1.0, 0.0}, maxThrust});
    thrusters.push_back({{ x, 0.0, 0.0}, { 0.0, -1.0, 0.0}, maxThrust});
    thrusters.push_back({{-x, 0.0, 0.0}, { 0.0, 1.0, 0.0}, maxThrust});
}

void ReactionControlThrusters::commandTorque(const std::array<double, 3>& desiredTorque) {
    for (auto& thruster : thrusters) {
        thruster.isFiring = false;

        // Compute r × F for this thruster
        const auto& r = thruster.position;
        const auto& F = thruster.direction;
        std::array<double, 3> torque = {
            r[1]*F[2] - r[2]*F[1],
            r[2]*F[0] - r[0]*F[2],
            r[0]*F[1] - r[1]*F[0]
        };

        // Align torque direction with desired torque
        double alignment = torque[0]*desiredTorque[0] +
                           torque[1]*desiredTorque[1] +
                           torque[2]*desiredTorque[2];

        if (alignment > 0.0) {
            thruster.isFiring = true;
        }
    }
}

std::vector<bool> ReactionControlThrusters::getFiringStates() const {
    std::vector<bool> states;
    for (const auto& thruster : thrusters)
        states.push_back(thruster.isFiring);
    return states;
}

std::array<double, 3> ReactionControlThrusters::getNetTorque() const {
    std::array<double, 3> net = {0.0, 0.0, 0.0};

    for (const auto& thruster : thrusters) {
        if (!thruster.isFiring) continue;

        std::array<double, 3> F = {
            thruster.direction[0] * thruster.maxForce,
            thruster.direction[1] * thruster.maxForce,
            thruster.direction[2] * thruster.maxForce
        };

        const auto& r = thruster.position;
        std::array<double, 3> torque = {
            r[1]*F[2] - r[2]*F[1],
            r[2]*F[0] - r[0]*F[2],
            r[0]*F[1] - r[1]*F[0]
        };

        net[0] += torque[0];
        net[1] += torque[1];
        net[2] += torque[2];
    }

    return net;
}

const std::vector<Thruster>& ReactionControlThrusters::getThrusters() const {
    return thrusters;
}
