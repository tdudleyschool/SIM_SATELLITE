#include "ElectricalPowerSystem.hh"
#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>
#include <algorithm>

#ifdef _WIN32
  #include <ws2tcpip.h>
#else
  #include <arpa/inet.h>
#endif

#include "../../EPS/src/Battery.cpp"
#include "../../EPS/src/bus_modified.cpp"
#include "../../EPS/src/solar_cell.cpp"
#include "../../EPS/src/Solar_Power_System.cpp"

void integrate(double& val, double before, double after, double dt){
    double interval = 0.5*(before+after)*dt;
    val = val + interval;
}

EPS_Module::EPS_Module(const std::string& tk_ip_, int tk_port_,
                      const std::string& mp_ip_, int mp_port_,
                      int listen_port_)
    : Actor("eps_module"),
      tk_ip(tk_ip_),
      tk_port(tk_port_),
      mp_ip(mp_ip_),
      mp_port(mp_port_),
      listen_port(listen_port_),
      server_fd(INVALID_SOCKET) // We'll add this socket for listening
{
    // Add MissionProcessor connection as outbound
    outboundConnections.emplace_back("MissionProcessor", mp_ip, mp_port);

    //=============================//
//ELECTRICAL POWER SYSTEM SETUP//
//=============================//
    for (int i = 0; i < 3; ++i) {
    dV1[i] = 0.0;
    dV1_prev[i] = 0.0;
    dV2[i] = 0.0;
    dV2_prev[i] = 0.0;
    }

    //Initialization Conditions 
    //Bus Initial Conditions
    double source_voltage = 120;
    double high_bus_loads = 7;
    double low_bus_loads = 4;
    double bus_resistance = 0.009; //Estimate Resistance
    double high_thruster_pow = 12.5e3;
    double low_thrust_pow = 6.5e3;
    double req_voltage = 600;
    double wheel_voltage = 10.0;
    double wheel_power = 20.0;//watts

    //Solar Array Initial Conditions
    double OC_V_sol = 130; //a bit higher than source voltage
    double MAX_V_sol = 140; //a bit higher than OC_Voltage
    double I_SC = 265; //should be the short I, same as max I
    double term_V = 0.2568; //terminal voltage

    //Batteries Initial Conditions
    double Q_Bat = 1; //capacity
    double R_0_Bat = 1; //Resistors
    double R_1_Bat = 1;
    double R_2_Bat = 1;
    double C_1_Bat = 1; //capacitors for modeling battery
    double C_2_Bat = 1; 
    double SOC_bat = 0.8; //State of change

    //Power Bus Initialization
    High_bus.initialize(source_voltage, high_bus_loads, bus_resistance, CURRENT_LIMITED);
    Low_bus.initialize(source_voltage, high_bus_loads, bus_resistance, CURRENT_LIMITED);

    high_thruster_specs.max_pow = high_thruster_pow;
    high_thruster_specs.req_voltage = req_voltage;
    low_thruster_specs.max_pow = low_thrust_pow;
    low_thruster_specs.req_voltage = req_voltage;

    motor_specs.max_pow = wheel_power;
    motor_specs.req_voltage = wheel_voltage;
    ACS_specs.max_pow = 52.0; //represents all other modules

    GNC_specs.max_pow = 65.0; //Watts
    GNC_specs.req_voltage = 100.0; //Volts

    Communication_specs.max_pow = 150.0; //Watts
    Communication_specs.req_voltage = 100.0; //Volts
    Cammand_Data_Handler_specs.max_pow = 50.0; //Watts
    Cammand_Data_Handler_specs.req_voltage = 15.0; //Volts

    //Solar Power System Initialization
    sol_pow_sys[0].initialize(OC_V_sol, I_SC /*I_SC*/, MAX_V_sol, I_SC /*I_MAX*/, term_V);
    sol_pow_sys[1].initialize(OC_V_sol, I_SC /*I_SC*/, MAX_V_sol, I_SC /*I_MAX*/, term_V);
    sol_cell[0].initialize(I_SC);

    //length is 15 so 7.5 + extra length
    //ppe solar cells are around 75 meters long meaning taht center will be as shown
    //35.5 + 7.5 = 45
    double ref_pos_sol1[3] = {0.0, 45.0, 0.0};
    double ref_pos_sol2[3] = {0.0, -45.0, 0.0};
    double ref_ori_sol[2] = {1, 0};
    //input light vector to adapt.
    //will need a controller variables to controll solar cells.
    sol_cell[0].set_refrence_pos(ref_pos_sol1);
    sol_cell[1].set_refrence_pos(ref_pos_sol2);
    //reference orientation will just point up like stated ealier this will need
    //to have a motor and controller controlling it. alot simpler since we dont have 3 motors
    sol_cell[0].lock_axis_y();
    sol_cell[1].lock_axis_y();
    sol_cell[0].set_refrence_ori(ref_ori_sol);
    sol_cell[1].set_refrence_ori(ref_ori_sol);

    //Battery Initialization
    for(int i = 0; i < 3; i++) {
        EPS_bat[i].initialize(Q_Bat, R_0_Bat, R_1_Bat, R_2_Bat, C_1_Bat, C_2_Bat, SOC_bat);
    }

    //turning on default systems
    Low_bus.turn_node_on(0, GNC_specs.max_pow, GNC_specs.req_voltage);
    Low_bus.turn_node_on(1, Cammand_Data_Handler_specs.max_pow, Cammand_Data_Handler_specs.req_voltage);
    Low_bus.turn_node_on(3, Communication_specs.max_pow, Communication_specs.req_voltage);
}

EPS_Module::~EPS_Module() {
    stop();
}

void EPS_Module::initializeNetwork() {
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::cerr << "[EPS_Module] WSAStartup failed.\n";
        exit(1);
    }
#endif

    // --- Connect to Timekeeper ---
    {
        socket_t tkSock = socket(AF_INET, SOCK_STREAM, 0);
        if (tkSock == INVALID_SOCKET) {
            std::cerr << "[EPS_Module] Failed to create socket for Timekeeper.\n";
            exit(1);
        }

        sockaddr_in tk_server{};
        tk_server.sin_family = AF_INET;
        tk_server.sin_port = htons(tk_port);

#ifdef _WIN32
        if (InetPton(AF_INET, tk_ip.c_str(), &tk_server.sin_addr) != 1) {
            std::cerr << "[EPS_Module] Invalid Timekeeper IP.\n";
            closesocket(tkSock);
            exit(1);
        }
#else
        if (inet_pton(AF_INET, tk_ip.c_str(), &tk_server.sin_addr) <= 0) {
            std::cerr << "[EPS_Module] Invalid Timekeeper IP.\n";
            closesocket(tkSock);
            exit(1);
        }
#endif

        // Retry until connected
        while (connect(tkSock, (sockaddr*)&tk_server, sizeof(tk_server)) == SOCKET_ERROR) {
            std::cerr << "[EPS_Module] Waiting to connect to Timekeeper...\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        sockets.push_back(tkSock);
        std::cout << "[EPS_Module] Connected to Timekeeper.\n";

        // Send ready message on Timekeeper connection
        Message readyMsg;
        readyMsg.sender = getName();
        readyMsg.type = "ready";
        sendMessage(readyMsg, tkSock);
        std::cout << "[EPS_Module] Sent ready to Timekeeper\n";
    }

    // --- Connect to MissionProcessor ---
    {
        socket_t mpSock = socket(AF_INET, SOCK_STREAM, 0);
        if (mpSock == INVALID_SOCKET) {
            std::cerr << "[EPS_Module] Failed to create socket for MissionProcessor.\n";
            exit(1);
        }

        sockaddr_in mp_server{};
        mp_server.sin_family = AF_INET;
        mp_server.sin_port = htons(mp_port);

#ifdef _WIN32
        if (InetPton(AF_INET, mp_ip.c_str(), &mp_server.sin_addr) != 1) {
            std::cerr << "[EPS_Module] Invalid MissionProcessor IP.\n";
            closesocket(mpSock);
            exit(1);
        }
#else
        if (inet_pton(AF_INET, mp_ip.c_str(), &mp_server.sin_addr) <= 0) {
            std::cerr << "[EPS_Module] Invalid MissionProcessor IP.\n";
            closesocket(mpSock);
            exit(1);
        }
#endif

        // Retry until connected
        while (connect(mpSock, (sockaddr*)&mp_server, sizeof(mp_server)) == SOCKET_ERROR) {
            std::cerr << "[EPS_Module] Waiting to connect to MissionProcessor...\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        sockets.push_back(mpSock);
        std::cout << "[EPS_Module] Connected to MissionProcessor.\n";

        // Send ready message on MissionProcessor connection
        Message readyMsg;
        readyMsg.sender = getName();
        readyMsg.type = "ready";
        sendMessage(readyMsg, mpSock);
        std::cout << "[EPS_Module] Sent ready to MissionProcessor\n";
    }

    // --- (Optional) Listen for downstream connections if needed ---
    if (listen_port > 0) {
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd == INVALID_SOCKET) {
            std::cerr << "[EPS_Module] Failed to create listen socket.\n";
            exit(1);
        }

        sockaddr_in listen_addr{};
        listen_addr.sin_family = AF_INET;
        listen_addr.sin_port = htons(listen_port);
        listen_addr.sin_addr.s_addr = INADDR_ANY;

        if (bind(server_fd, (sockaddr*)&listen_addr, sizeof(listen_addr)) == SOCKET_ERROR) {
            std::cerr << "[EPS_Module] Bind failed on listen port.\n";
            exit(1);
        }

        if (listen(server_fd, 5) == SOCKET_ERROR) {
            std::cerr << "[EPS_Module] Listen failed on listen port.\n";
            exit(1);
        }

        std::cout << "[EPS_Module] Listening for downstream actor connections on port " << listen_port << "\n";

        std::thread acceptThread([this]() {
            while (running) {
                sockaddr_in client_addr{};
                socklen_t addr_len = sizeof(client_addr);
                socket_t new_sock = accept(server_fd, (sockaddr*)&client_addr, &addr_len);
                if (new_sock == INVALID_SOCKET) {
                    std::cerr << "[EPS_Module] Accept failed or stopped.\n";
                    break;
                }

                std::string buffer;
                char ch;
                while (true) {
                    int ret = recv(new_sock, &ch, 1, 0);
                    if (ret <= 0) {
                        closesocket(new_sock);
                        new_sock = INVALID_SOCKET;
                        break;
                    }
                    if (ch == '\n') break;
                    buffer.push_back(ch);
                }

                if (buffer.empty() || new_sock == INVALID_SOCKET) continue;

                Message msg = deserializeMessage(buffer);

                if (msg.type == "ready") {
                    std::string actorName = msg.sender;
                    std::transform(actorName.begin(), actorName.end(), actorName.begin(), ::tolower);

                    // Optionally track downstream connections here
 
                    sockets.push_back(new_sock);
                    std::cout << "[EPS_Module] Connected downstream actor '" << actorName << "'\n";
                } else {
                    std::string actorName = msg.sender;
                    std::cout << "[EPS_Module] Unexpected message from downstream actor '" << actorName << "'.\n";
                    //closesocket(new_sock);
                    continue;
                }
            }
        });
        acceptThread.detach();
    }

    configureBehaviors();
}


// Your existing configureBehaviors() goes here without change — 
// since it mostly handles messages and internal state updates.
void EPS_Module::configureBehaviors() {

    setBehavior([this](const Message& msg) {
        try {
            std::cout << "[EPS_Module] Handling message: '" << msg.type << "'\n";

            
        // Updating EPS ON EVERY TICK
        if (msg.type == "tick") {
            
        //GETTING DT:
            double dt = stod(msg.fields.at("dt"));

        //SOLAR POWER INITIAL HANDLING
            double total_drawn_I = Low_bus.get_drawn_I() + High_bus.get_drawn_I();

            //getting current I

            //ADD: Controller For Directing I based on cammands
            //     maybe not here but at anothe behavior

            //use this later
            //double I_SC_0 = sol_cell[0].get_I_sc(/*light vector*/);
            //double I_SC_1 = sol_cell[1].get_I_sc(/*light vector*/); 
            //for now
            double I_SC_0 = 455.0; //a bit lower than given I_SC;
            double I_SC_1 = 455.0; //a bit lower than given I_SC;
            
            sol_pow_sys[0].set_I_sc(I_SC_0);
            sol_pow_sys[1].set_I_sc(I_SC_1);

            //Node Voltage
            double total_incomming_I = High_bus.get_total_I() + Low_bus.get_total_I();
            //calculating the voltage drop for node voltage. V_s - I*R. here we use resistance in parallel formula
            double Node_Voltage = source_voltage - total_incomming_I * ((bus_resistance * bus_resistance)/(bus_resistance + bus_resistance));

            sol_pow_sys[0].update_V(Node_Voltage);
            sol_pow_sys[1].update_V(Node_Voltage);

            //total current from solar power system
            double total_I_sol = sol_pow_sys[0].get_I() + sol_pow_sys[1].get_I();
            

        //LOAD CALCULATIONS CALCULATING AMOUNT OF CURRENT NEEDED
            //use total_drawn_I from above

            High_bus.update_start_voltage(Node_Voltage);
            Low_bus.update_start_voltage(Node_Voltage);

            //for battery negative value is charge and positive value is take
            double battery_I = total_drawn_I - total_I_sol; //meaning that is total drawn I is less then charge
            int bat_i = -1; //index changed takes so we just need to look at the change in one battery
            double input_I;

            input_I = total_drawn_I; //initially set input to drawn I in most cases this will be the I

            if (battery_I <= 0) //meaning there is enough current to supply
            {
                //Here input I is full
                if (EPS_bat[2].get_soc() >= 0.89) {
                    EPS_bat[2].update_I(battery_I);
                    bat_i = 2;
                }
                else if (EPS_bat[1].get_soc() >= 0.89) {
                    EPS_bat[1].update_I(battery_I);
                    bat_i = 1;
                }
                else if (EPS_bat[0].get_soc() >= 0.89) {
                    EPS_bat[0].update_I(battery_I);
                    bat_i = 0;
                }
                else {
                    bat_i = -1;
                    cout << "[EPS Module: ALL BATTERIES ARE CHARGED] \n";
                }
            } 
            else if (battery_I > 0 && EPS_bat[0].get_soc() <= 0.21) {
                //Here input I is full
                EPS_bat[0].update_I(battery_I);
                bat_i = 0;
            }
            else if (battery_I > 0 && EPS_bat[1].get_soc() <= 0.21) {
                //Here input I is full
                EPS_bat[1].update_I(battery_I);
                bat_i = 1;
            }
            else if (battery_I > 0 && EPS_bat[2].get_soc() <= 0.21) {
                //Here input I is full
                EPS_bat[2].update_I(battery_I);
                bat_i = 2;
            }
            else {
                //Here input I is not enough for required I.
                //Prioritize low bus.
                input_I = total_I_sol;
                bat_i = -1;
                cout << "[EPS Module] ALL BATTERIES ARE EMPTY NOT ENOUGH CURRENT \n";
            }
            double low_bus_I = min(input_I, Low_bus.get_drawn_I());
            
            Low_bus.state_update(low_bus_I);
            High_bus.state_update(input_I - Low_bus.get_drawn_I());
            
        //BATTERY INTEGRATION INTEGRATING ALL BATTERIES
            if (bat_i != -1) {
                dV1_prev[bat_i] = dV1[bat_i];
                dV2_prev[bat_i] = dV2[bat_i];

                EPS_bat[bat_i].update_soc(dt);
                EPS_bat[bat_i].state_deriv_getVolteges(dV1[bat_i], dV2[bat_i]);
                double V1 = EPS_bat[bat_i].get_V1();
                double V2 = EPS_bat[bat_i].get_V2();
                integrate(V1, dV1_prev[bat_i], dV1[bat_i], dt);
                integrate(V2, dV2_prev[bat_i], dV2[bat_i], dt);
                
                EPS_bat[bat_i].update_V1(V1);
                EPS_bat[bat_i].update_V2(V2);
                EPS_bat[bat_i].update_Vt();

                Vt[bat_i] = EPS_bat[bat_i].get_Vt();
                cout << "[EPS Module] Battery(" << bat_i << ") SOC: " << EPS_bat[bat_i].get_soc() << "Terminal Voltage: " << Vt[bat_i] << "\n";
            }
            else {
                cout << "No updates batteries are either full or empty \n";
            }
            //else no updates to batteries

        }

        //!!!! NEED TO ADD UPDATE THE STATE OF LOAD AT THE PART WHERE I TURN ON/TURN OFF LOADS!!!!
    // Turn Load On Handling
        if (msg.type == "turn_load_on") {
            string load_type = msg.fields.at("load_type");
            //double input_voltage = stod(msg.fields.at("input_voltage"));
            int num_of_loads = stod(msg.fields.at("num_of_loads"));
            int loads_on = 0;
            double input_voltage[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            double input_mass_flow[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            int active_nodes[7] = {0, 0, 0, 0, 0, 0, 0}; //here 1 = on, 0 = off

            //message load_nums can't be more than 7 as 7 is max load size for our thing
            
            if (msg.fields.find("input_voltage") != msg.fields.end()) 
            {
                string arrayStr = msg.fields.at("input_voltage");
                stringstream ss(arrayStr);
                string val;
                int i = 0;
                while (getline(ss, val, ',') && i < num_of_loads) {
                    input_voltage[i] = stod(val);
                    i++;
                }
            }

            string mass_flow = msg.fields.at("input_mass_flow");
            /*
            //Mass flow when through no processing so we just pass it on as a string to 
            the thruster
            if (msg.fields.contains("input_mass_flow") && stod(msg.fields.at("input_mass_flow")) != '-') 
            {
                string arrayStr = msg.fields.at("input_mass_flow");
                stringstream ss(arrayStr);
                string val;
                int i = 0;
                while (getline(ss, val, ',') && i < num_of_loads) {
                    input_mass_flow[i] = stod(val);
                    i++;
                }
            }
            */
            if (msg.fields.find("active_nodes") != msg.fields.end()) {
                string arrayStr = msg.fields.at("active_nodes");
                stringstream ss(arrayStr);
                string val;
                int i = 0;
                while (getline(ss, val, ',') && i < num_of_loads) {
                    active_nodes[i] = stod(val);
                    i++;
                }
            }

            //positional inputs for passing off from mission processor
            string r_matrix = msg.fields.at("rotation_matrix");
            string sat_pos = msg.fields.at("sat_position");
            string sat_v = msg.fields.at("sat_velocity");
            string sat_a = msg.fields.at("sat_acceleration");
            string sat_w = msg.fields.at("sat_angularvelocity");

            //send message to various divices depending of the load type

            if (load_type == "LOW_BUS") {
                if(active_nodes[0] == 1) {
                    Low_bus.turn_node_on(0, GNC_specs.max_pow, GNC_specs.req_voltage);
                }
                if(active_nodes[1] == 1) {
                    Low_bus.turn_node_on(1, Cammand_Data_Handler_specs.max_pow, Cammand_Data_Handler_specs.req_voltage);
                }
                if(active_nodes[2] == 1) {
                    Low_bus.turn_node_on(3, Communication_specs.max_pow, Communication_specs.req_voltage);
                }
            }
            else if (load_type == "ACS") {
                double required_voltage = ACS_specs.req_voltage;
                double max_pow = ACS_specs.max_pow;

                for (int i = 0; i < num_of_loads; i++) {
                    if (active_nodes[i] == 1) {
                        max_pow += motor_specs.max_pow;
                    }
                }
                Low_bus.turn_node_on(3, max_pow, required_voltage);
                double result_I = Low_bus.get_node_I(3);
                
                //slightly unrealistic here we will just say it's on if we get a current
                //proper implementaton would be to calculat the voltage based on motor power and the resultant current
                //  V_input = Power / result_I

                Message ACS_message;
                ACS_message.sender = name;
                ACS_message.type = "ACS:input";

                if (result_I >= 0) {
                    //prepare voltage string

                    // Serialize to comma-separated string aka converting array into csv format
                    string volt_msg;
                    for (int i = 0; i < 3; ++i) {
                        volt_msg += to_string(input_voltage[i]);
                        if (i < 2) volt_msg += ","; // Don't add comma after last
                    }

                    //Sending voltage as a field
                    std::cout << "Voltage=" << volt_msg << "\n";
                    ACS_message.fields["input_voltage"] = volt_msg;
                }
                else {//Not enough current for load to be on
                    ACS_message.fields["input_voltage"] = "0.0,0.0,0.0";
                }
            }
            else if (load_type == "THRUSTER") {
                double all_input_voltages[7]; //all voltages as a result from inputs
                for (int i = 0; i < num_of_loads; i++) {
                    if (i < 3) { //representing large thrusters
                        if (active_nodes[i] == 1) {
                            High_bus.turn_node_on(i, high_thruster_specs.max_pow, input_voltage[i], high_thruster_specs.req_voltage);
                        }
                    } else {
                        if (active_nodes[i] == 1) {
                            High_bus.turn_node_on(i, low_thruster_specs.max_pow, input_voltage[i], low_thruster_specs.req_voltage);
                        }
                    }
                    //stor the voltage in the list. basically this compairs the actual current going in the load. if not enoguh then power will be weaker
                    //V = IR
                    all_input_voltages[i] = High_bus.get_node_I(i) * High_bus.get_node_R(i);
                    //only the lower of the two will be used for final input.
                    all_input_voltages[i] = min(all_input_voltages[i], input_voltage[i]);
                }
            // Message to thruster with input voltage and massflow
                //preparing message
                string volt_msg;
                for (int i = 0; i < 7; ++i) {
                    volt_msg += to_string(all_input_voltages[i]);
                    if (i < 6) volt_msg += ","; // Don't add comma after last
                }

                Message THRUSTER_message;
                THRUSTER_message.sender = name;
                THRUSTER_message.type = "THRUSTER:input";
                THRUSTER_message.fields["input_voltage"] = volt_msg;
                THRUSTER_message.fields["input_mass_flow"] = mass_flow;
            }
        }

    //Turn Load Off Handling
        if (msg.type == "turn_load_off") {
            string load_type = msg.fields.at("load_type");
            int load_index = stod(msg.fields.at("load_index"));
            
            if (load_type == "LOW_BUS") {
                if(load_index == 0) {
                    Low_bus.turn_node_off(0);
                }
                if(load_index == 1) {
                    Low_bus.turn_node_off(1);
                }
                if(load_index == 2) {
                    Low_bus.turn_node_off(2);
                }
            }
            else if (load_type == "ACS") {
                Low_bus.turn_node_off(3);

                Message ACS_message;
                ACS_message.sender = name;
                ACS_message.type = "ACS:input";
                ACS_message.fields["input_voltage"] = "0.0,0.0,0.0";

                
            }
            else if (load_type == "THRUSTER") {
                High_bus.turn_node_off(load_index);
                
                Message THRUSTER_message;
                THRUSTER_message.sender = name;
                THRUSTER_message.type = "THRUSTER:input";
                THRUSTER_message.fields["input_voltage"] = "0.0";
                THRUSTER_message.fields["input_mass_flow"] = "0.0";
            }
        }

        } catch (const std::exception& e) {
            std::cerr << "[EPS_Module] Exception in behavior: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[EPS_Module] Unknown exception in behavior.\n";
        }
    });

}
// The run() function stays mostly the same, just keep it as is.

void EPS_Module::run() {
    initializeNetwork();

    running = true;
    while (running) {
        Message incoming;
        // Keep reading as long as messages are available
        do {
            incoming = readIncomingMessage();
            if (incoming.type.empty()) continue;

            std::cout << "[EPS_Module] Received message: " << incoming.type << std::endl;

            {
                std::lock_guard<std::mutex> lock(mailboxMutex);
                mailbox.push(incoming);
            }
            mailboxCV.notify_one();

            Message msg = receiveMessage();
            if (!msg.type.empty()) {
                handleMessage(msg);
                std::cout << "[EPS_Module] Handled message: " << msg.type << std::endl;
            }
        } while (!incoming.type.empty());

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "[EPS_Module] Exiting run loop.\n";
    closeAllSockets();

#ifdef _WIN32
    WSACleanup();
#endif
}


