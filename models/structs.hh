#ifndef STRUCTS_HH
#define STRUCTS_HH

#ifdef __cplusplus
    extern "C"
    {
#endif

struct point {
    double x;
    double y;
    double z;
};

struct Force_Node {
    int lable;
    double value[3];
    Force_Node* next;
    Force_Node* prev;
};

struct Torque_Node {
    int lable;
    double value[3];
    Torque_Node* next;
    Torque_Node* prev;
};

struct Node {
    double P_draw; //power needed
    double V;

    double I_draw; //current needed
    double R; //load risistance

    //actual I and P
    double I;
    double I_out;
    double P;

    bool on;
    
    int prev_node = -1;
};

#ifdef __cplusplus
    }
#endif

#endif