#include <cmath>
#include <iostream>
#include "../include/functions.hh"
//#include "../../../Lib/eigen-3.4.0/Eigen/Dense"

using namespace std;

double gravForceMagnitude(double m1, double m2, const double x1[3], const double x2[3]) {
    const double G = 6.67430e-11;
    double dist = sqrt(pow(x2[0] - x1[0], 2) + pow(x2[1] - x1[1], 2) + pow(x2[2] - x1[2], 2));

    if (dist == 0.0){
        return 0;
    }

    return G * (m1 * m2) / (dist * dist);
}

void getUnitDir(const double p1[3], const double p2[3], double unit_vec[3]){
    Vector3d vec;
    vec.insert((p2[0] - p1[0]), (p2[1]-p1[1]), (p2[2]-p1[1]));
    vec.normalize();

    //return
    unit_vec[0] = vec[0];
    unit_vec[1] = vec[1];
    unit_vec[2] = vec[2];
}

bool collisionRaySphere(double r, Vector3d sphere_cent, Vector3d ray_origin, Vector3d ray_dir){
    Vector3d L;
    L = ray_origin - sphere_cent;
    double A = ray_dir.dot(ray_dir);
    double B = 2 * L.dot(ray_dir);
    double C = L.dot(L) - r;

    double descriminant = (B*B) - 4*A*C;

    bool collide = false;

    if(descriminant >= 0.0)
        collide = true;
    else
        collide = false;

    return collide;
}

Vector3d getUnitDir(Vector3d vec1, Vector3d vec2){
    Vector3d vec3;
    vec3 = vec2 - vec1;
    vec3.normalize();

    return vec3;
}