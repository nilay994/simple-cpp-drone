#pragma once

#include <vector>
#include <string>
#include <thread>

/** gate type **/
enum waypoint_type_t {
	WAYPOINT  = 0,
	GATE      = 1,
	FINISH    = 2,
	START     = 4,
	SKIP_GATE = 8,
};

/** each gate must populate this struct **/
struct waypoint_t {
    float x;              /** x co-ordinate setpoint in world frame **/
    float y;              /** y co-ordinate setpoint in world frame **/
    float z;              /** z co-ordinate setpoint in world frame **/
    float gate_psi;       /** gate yaw in world frame, face of the gate towards you, easier for snakegate/gateprior **/
    float drone_psi;      /** approach angle through the waypoint, for transverse pass psi = gatepsi ± 180 **/
    float v_sp;           /** velocity setpoint at the gate **/
    waypoint_type_t type;
};

class FlightPlan {

    public:
        int num_wp;
        std::vector<waypoint_t> wp; // holds all the waypoints, populated on constructor invoke
        int wp_selector = 0;
        bool trigger_wp_change = false;
        bool flightplan_running = false;

        bool close_to_gate = false;
		float dist_to_target = 1000.0; // init to safe value

        FlightPlan();
        ~FlightPlan();
        void flightplan_thread();
        
    private:
        std::thread flightplan_thread_;
        
        bool flightplan_run();
        void add_wp(float x, float y, float z, float gate_psi, float drone_psi, float v_sp, waypoint_type_t type);
        float distance_to_wp(int wp_ID);
        void print_flightplan();
        std::vector<std::vector<float>> parse_csv_file(std::string inputFileName);
};