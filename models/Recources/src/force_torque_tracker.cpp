#include <iostream>
#include "../../structs.hh"
#include "../include/force_torque_tracker.hh"
//#include "../../../Lib/eigen-3.4.0/Eigen/Dense"

//using namespace Eigen;

force_torque_tracker::force_torque_tracker(){
    NetForce.insert(0.0, 0.0, 0.0);
    NetTorque.insert(0.0, 0.0, 0.0);
}


void force_torque_tracker::addForce(double vec_x, double vec_y, double vec_z, double x, double y, double z){
    Vector3d rel_pos;
    Vector3d addedForce;
    Vector3d addedTorque;


    rel_pos.insert(x, y, z);
    addedForce.insert(vec_x, vec_y, vec_z);
    addedTorque = addedForce.cross(rel_pos);

    NetForce = NetForce + addedForce;
    NetTorque = NetTorque + addedTorque;

}

void force_torque_tracker::addForce(const double F_vec[3], const double pos[3]){
    Vector3d rel_pos;
    Vector3d addedForce;
    Vector3d addedTorque;


    rel_pos.insert(pos[0], pos[1], pos[2]);
    addedForce.insert(F_vec[0], F_vec[1], F_vec[2]);
    addedTorque = addedForce.cross(rel_pos);

    NetForce = NetForce + addedForce;
    NetTorque = NetTorque + addedTorque;

}

void force_torque_tracker::addTorque(double vec_x, double vec_y, double vec_z){
    Vector3d addedTorque;
    addedTorque.insert(vec_x, vec_y, vec_z);

    NetTorque = NetTorque + addedTorque;
}

void force_torque_tracker::addTorque(const double T_vec[3]){
    Vector3d addedTorque;
    addedTorque.insert(T_vec[0], T_vec[1], T_vec[2]);

    NetTorque = NetTorque + addedTorque;
}

void force_torque_tracker::getNetForce(double netF[3]){
    netF[0] = NetForce[0];
    netF[1] = NetForce[1];
    netF[2] = NetForce[2];
}

void force_torque_tracker::getNetTorque(double netT[3]){
    netT[0] = NetTorque[0];
    netT[1] = NetTorque[1];
    netT[2] = NetTorque[2];
}

void force_torque_tracker::resetValues(){
    NetForce.insert(0.0, 0.0, 0.0);
    NetTorque.insert(0.0, 0.0, 0.0);
}