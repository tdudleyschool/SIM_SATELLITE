/*
PURPOSE: (This testing script for the celestial body system
          just want to see if it is being created)
COMMANDS:
    USE this G++ command until the make file is created
    : g++ src/cilestial_body.cpp test/celestial_test.cpp -o celestial_program
*/

#include "../include/cilestial_body.hh"
#include <iostream>

using namespace std;

int main(){
    cilestial_body body(1000000, 50000);
}