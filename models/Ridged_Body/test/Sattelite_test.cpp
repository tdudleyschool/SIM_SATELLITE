#include <iostream>
#include <cmath>
#include <gtest/gtest.h>
#include <initializer_list>

#include "../include/Satellite_Box.hh"

using namespace std;

/*
PURPOSE: (This testing script checks if an
          output is made form the sattelite_box.
          This is an unofficial test script since
          the one I tried to get running was running
          into issues.
          )
COMMANDS:
    USE this G++ command until the make file is created
    : g++ src/Satellite_Box.cpp test/Sattelite_test.cpp -o test_program
*/

int main(){
    satellite_box j(100.0, 4.0);
    j.initialize_points(0, 0, 0);

    //Test Output Of Each Point
    point p[8];
    j.get_points(p);
    for(int i = 0; i < 8; i++){
        cout << "POINT: " << i << "\n";
        cout << p[i].x << " " << p[i].y << " " << p[i].z << "\n" << "\n";
    };

    //Test The Value Of The Inertial Matrix
    j.calc_I_0();
    double inertial_matrix[3][3];
    double inverse_inertial_matrix[3][3];
    j.get_I_0(inertial_matrix);
    j.get_inv_I_0(inverse_inertial_matrix);

    cout << "INERTIAL MATRIX: " << "\n";
    for(int i = 0; i < 3; i++){
        cout << inertial_matrix[i][0] << " " << inertial_matrix[i][1] << " " << inertial_matrix[i][2] << "\n";
    };

    cout << "INVERSE INERTIAL MATRIX: " << "\n";
    for(int i = 0; i < 3; i++){
        cout << inverse_inertial_matrix[i][0] << " " << inverse_inertial_matrix[i][1] << " " << inverse_inertial_matrix[i][2] << "\n";
    };

    //Test How Points Adapt To Center Position

}


//======================================================================================================
//Old test script
/*
Test Fixture
*/

/*class SatelliteBodyTest : public ::testing::TEST {
    protected:
        int x;
        satellite_box s(100.0, 4.0);

        SatelliteBodyTest(){

        }

        ~SatelliteBodyTest(){

        }
        
        void SetUp() {}
        void TurnDown() {}      
};

TEST(SatelliteBodyTest, PointTest){
    satellite_box j(100.0, 4.0);
    point p[8];
    j.get_points(p);
    for(int i = 0; i < 8; i++){
        EXPECT_EQ(fabs(p[i].x), 2);
    };
}*/

