#ifndef GRAVITATIONAL_FORCE
#define GRAVITATIONAL_FORCE

#ifdef __cplusplus
	extern "C"
	{
#endif

const double G = 6.67430e-11;

class gravitational_force {
    public:
        gravitational_force();
        gravitational_force(double, double);
        void update_pos(double[3], double[3]);
        void update_mass(double, double);
        void calculate_force();
        double get_grav_force_magnitude();
        void get_grav_force_at_mass1(double[3], double[3]);
        void get_grav_force_at_mass2(double[3], double[3]);
    private:
        double m1;
        double m2;
        double m1_pos[3];
        double m2_pos[3];

        double f_mag;
        double f_m1[3];
        double f_m2[3];

};

#ifdef __cplusplus
	}
#endif

#endif