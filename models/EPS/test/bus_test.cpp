/*
PURPOSE: (Testing bus capabilities
          )
COMMANDS:
    USE this G++ command until the make file is created
    : g++ src/bus.cpp test/bus_test.cpp -o bus_program
*/

#include <iostream>
#include <fstream>
#include <cmath>
#include "../include/bus.hh"

using namespace std;

int main(){
    bus goodbus(50, 5);

    cout << "total drawn current " << goodbus.get_drawn_I() << " total drawn power " << goodbus.get_drawn_P() << endl;

    goodbus.turn_node_on(0, 600.0);
    goodbus.state_update(10);

    cout << "total current " << goodbus.get_drawn_I() << " total power " << goodbus.get_drawn_P() << endl;
    cout << "current at 0 " << goodbus.get_node_I(0) << " power " << goodbus.get_node_P(0) << endl;
    cout << "current at 2 " << goodbus.get_node_I(2) << " power " << goodbus.get_node_P(2) << endl;

    goodbus.turn_node_on(2, 600.0);
    goodbus.turn_node_on(3, 600.0);
    goodbus.state_update(30);


    cout << "total current " << goodbus.get_drawn_I() << " total power " << goodbus.get_drawn_P() << endl;
    cout << "current at 0 " << goodbus.get_node_I(0) << " power " << goodbus.get_node_P(0) << endl;
    cout << "current at 1 " << goodbus.get_node_I(1) << " power " << goodbus.get_node_P(1) << endl;
    cout << "current at 2 " << goodbus.get_node_I(2) << " power " << goodbus.get_node_P(2) << endl;
    cout << "current at 3 " << goodbus.get_node_I(3) << " power " << goodbus.get_node_P(3) << endl;

    goodbus.turn_node_off(2);
    goodbus.state_update(50);

    cout << "current at 3 " << goodbus.get_node_I(3) << " power " << goodbus.get_node_P(3) << endl;

}