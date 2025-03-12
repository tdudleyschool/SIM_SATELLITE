//Test file for gravitational force.cpp
//cammand is
//:  g++ src/gravitational_force.cpp test/grav_force_test.cpp -o grav_force_program

#include <iostream>
#include <cassert>
#include "../include/gravitational_force.hh"

int main() {
    // Test 1: Default constructor and initialization
    gravitational_force gf1;
    std::cout << "Test 1: Default constructor and initialization\n";
    std::cout << "Mass 1: " << gf1.get_grav_force_magnitude() << std::endl;
    std::cout << "Mass 2: " << gf1.get_grav_force_magnitude() << std::endl;

    // Test 2: Constructor with mass inputs
    gravitational_force gf2(5.0, 10.0);
    std::cout << "\nTest 2: Constructor with mass inputs\n";
    std::cout << "Mass 1: " << gf2.get_grav_force_magnitude() << std::endl;
    std::cout << "Mass 2: " << gf2.get_grav_force_magnitude() << std::endl;

    // Test 3: Update positions and calculate force
    double pos1[3] = {1.0, 1.0, 1.0};
    double pos2[3] = {2.0, 2.0, 2.0};
    gf2.update_pos(pos1, pos2);
    gf2.calculate_force();

    std::cout << "\nTest 3: Update positions and calculate force\n";
    std::cout << "Force Magnitude: " << gf2.get_grav_force_magnitude() << std::endl;

    // Get gravitational force at mass 1
    double force1[3], force_pos1[3];
    gf2.get_grav_force_at_mass1(force1, force_pos1);
    std::cout << "Force at Mass 1: ";
    for (int i = 0; i < 3; i++) {
        std::cout << force1[i] << " ";
    }
    std::cout << "\nPosition at Mass 1: ";
    for (int i = 0; i < 3; i++) {
        std::cout << force_pos1[i] << " ";
    }
    std::cout << std::endl;

    // Test 4: Gravitational force at mass 2
    double force2[3], force_pos2[3];
    gf2.get_grav_force_at_mass2(force2, force_pos2);
    std::cout << "\nTest 4: Gravitational force at Mass 2\n";
    std::cout << "Force at Mass 2: ";
    for (int i = 0; i < 3; i++) {
        std::cout << force2[i] << " ";
    }
    std::cout << "\nPosition at Mass 2: ";
    for (int i = 0; i < 3; i++) {
        std::cout << force_pos2[i] << " ";
    }
    std::cout << std::endl;


    // Test 6: Handling zero distance between masses
    gravitational_force gf3(5.0, 10.0);
    double same_pos1[3] = {0.0, 0.0, 0.0};
    double same_pos2[3] = {0.0, 0.0, 0.0};
    gf3.update_pos(same_pos1, same_pos2);
    gf3.calculate_force();

    std::cout << "\nTest 6: Handling zero distance between masses\n";
    std::cout << "Force Magnitude (should be zero): " << gf3.get_grav_force_magnitude() << std::endl;
    double force3[3], force_pos3[3];
    gf3.get_grav_force_at_mass1(force3, force_pos3);
    std::cout << "Force at Mass 1 (should be zero): ";
    for (int i = 0; i < 3; i++) {
        std::cout << force3[i] << " ";
    }
    std::cout << std::endl;

    return 0;
}


