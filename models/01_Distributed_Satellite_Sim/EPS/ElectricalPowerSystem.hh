#ifndef EPS_HH
#define EPS_HH

#include "../Actor/Actor.hh"
#include <string>
#include <vector>
#include <unordered_map>

#include "../../EPS/include/Battery.hh"
#include "../../EPS/include/bus_modified.hh"
#include "../../EPS/include/solar_cell.hh"
#include "../../EPS/include/Solar_Power_System.hh"

struct load_specs {
    double max_pow;
    double req_voltage;
};

class EPS_Module : public Actor {
public:
    EPS_Module(const std::string& tk_ip, int tk_port,
               const std::string& mp_ip, int mp_port,
               int listen_port);

    ~EPS_Module();

protected:
    void initializeNetwork() override;
    void run() override;

private:
    std::string tk_ip;
    int tk_port;

    std::string mp_ip;
    int mp_port;

    int listen_port; // For inbound connections, 0 means no listen

    socket_t server_fd;  // <-- Add this declaration here

    // Internal state variables for EPS
    battery EPS_bat[3];
    double dV1[3];
    double dV1_prev[3];
    double dV2[3];
    double dV2_prev[3];
    double Vt[3];

    bus High_bus;
    bus Low_bus;

    load_specs high_thruster_specs;
    load_specs low_thruster_specs;
    load_specs motor_specs;
    load_specs ACS_specs;

    load_specs GNC_specs;

    load_specs Communication_specs;
    load_specs Cammand_Data_Handler_specs;

    solar_array sol_pow_sys[2];
    solar_cell sol_cell[2];
    double I_sol_out[2];

    double source_voltage = 120;
    double bus_resistance = 0.009; //Estimate Resistance

    // Outbound connection list (name, ip, port)
    std::vector<std::tuple<std::string, std::string, int>> outboundConnections;

    void configureBehaviors();
};

#endif
