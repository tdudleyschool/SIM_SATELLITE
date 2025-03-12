#include "../include/gravitational_force.hh"
#include <cmath>

void get_unit_vector(const double[3], double[3]);

gravitational_force::gravitational_force(){
    m1 = 0.0;
    m2 = 0.0;
    for (int i = 0; i < 3; i++) {  // Loop through array elements
        m1_pos[i] = 0.0;
        m2_pos[i] = 0.0;
        f_m1[i] = 0.0;
        f_m2[i] = 0.0;
    }
    f_mag = 0.0;
}

gravitational_force::gravitational_force(double mass1, double mass2){
    m1 = mass1;
    m2 = mass2;
    for (int i = 0; i < 3; i++) {  // Loop through array elements
        m1_pos[i] = 0.0;
        m2_pos[i] = 0.0;
        f_m1[i] = 0.0;
        f_m2[i] = 0.0;
    }
    f_mag = 0.0;
}

void gravitational_force::update_pos(double pos1[3], double pos2[3]){
    for (int i = 0; i < 3; i++){
        m1_pos[i] = pos1[i];
        m2_pos[i] = pos2[i];
    }
}

void gravitational_force::update_mass(double mass1, double mass2){
    m1 = mass1;
    m2 = mass2;
}

void gravitational_force::calculate_force(){
    double dist_squared = pow(m1_pos[0] - m2_pos[0], 2) + pow(m1_pos[1] - m2_pos[1], 2) + pow(m1_pos[2] - m2_pos[2], 2);

    if(dist_squared == 0){
        f_mag = 0.0;
        for (int i = 0; i < 3; i++) {  // Loop through array elements
            f_m1[i] = 0.0;
            f_m2[i] = 0.0;
        }
    }
    else{
        f_mag = G * (m1 * m2) / dist_squared;

        double vec_12[3] = {m2_pos[0] - m1_pos[0], m2_pos[1] - m1_pos[1], m2_pos[2] - m1_pos[2]};
        double vec_21[3] = {m1_pos[0] - m2_pos[0], m1_pos[1] - m2_pos[1], m1_pos[2] - m2_pos[2]};
        double unit_dir_12[3];
        double unit_dir_21[3];
        get_unit_vector(vec_12, unit_dir_12);
        get_unit_vector(vec_21, unit_dir_21);

        for(int i = 0; i < 3; i++){
            f_m1[i] = f_mag * unit_dir_12[i];  // Corrected to assign the proper unit vector component
            f_m2[i] = f_mag * unit_dir_21[i];  // Corrected to assign the proper unit vector component
        }
    }
}

double gravitational_force::get_grav_force_magnitude(){
    return f_mag;
}

void gravitational_force::get_grav_force_at_mass1(double force[3], double force_pos[3]){
    for (int i = 0; i < 3; i++) {
        force[i] = f_m1[i];
        force_pos[i] = m1_pos[i];
    }
}

void gravitational_force::get_grav_force_at_mass2(double force[3], double force_pos[3]){
    for (int i = 0; i < 3; i++) {
        force[i] = f_m2[i];
        force_pos[i] = m2_pos[i];
    }
}

void get_unit_vector(const double vec[3], double unit_vec[3]) {
    double magnitude = sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);

    if (magnitude == 0) {
        for (int i = 0; i < 3; i++) {  // Loop through array elements
            unit_vec[i] = 0.0;  // Correctly setting each element of unit_vec to 0 if magnitude is zero
        }
    } else {
        unit_vec[0] = vec[0] / magnitude;
        unit_vec[1] = vec[1] / magnitude;
        unit_vec[2] = vec[2] / magnitude;
    }
}

