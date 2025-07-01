////g++ src/hall_thruster_PIC2D.cpp test/hall_thruster_3Test.cpp -o het_implementation_sim
#include "../include/hall_thruster_PIC2D.hh"

using namespace std;

int main(){
    double pos[3];
    double ori[3];
    double ref_pos[3] = {2, 0, 0};
    double ref_ori[3] = {1, 0, 0};
    double cent_pos[3] = {0, 0, 0};

    double force[3];
    double f_pos[3];

    HET_PIC2D thruster;
    thruster.switch_stateon();
    thruster.initialize_HET_sim();
    
    for (int i = 0; i < 100; i++) {
        thruster.set_refrence_pos(ref_pos);
        thruster.set_refrence_ori(ref_ori);
        thruster.get_pos(pos);
        
        cout << "Position: " << pos[0] << ", " << pos[1] << ", " << pos[2] << endl;
        
        double R[3][3] = {{0.707, -0.707, 0},
                        {0.707, 0.707, 0},
                        {0, 0, 1}};

        thruster.update_pos_ori(cent_pos, R);

        thruster.get_pos(pos);
        thruster.get_ori(ori);

        cout << "Updated Position: " << pos[0] << ", " << pos[1] << ", " << pos[2] << endl;
        cout << "Updated Orientation: " << ori[0] << ", " << ori[1] << ", " << ori[2] << endl;

        thruster.run_step_HET_sim(0.0, 300.0);
        thruster.get_force(100, force, f_pos);

        cout << "Force: " << force[0] << ", " << force[1] << ", " << force[2] << '\n';
        cout << "Force Location: " << f_pos[0] << ", " << f_pos[1] << ", " <<f_pos[2] << endl;
    }

    

}