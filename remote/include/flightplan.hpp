#pragma once

#include <vector>

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
    float x;        /** x co-ordinate setpoint in world frame **/
    float y;        /** y co-ordinate setpoint in world frame **/
    float z;        /** z co-ordinate setpoint in world frame **/
    float gatepsi;  /** gate yaw in world frame, face of the gate towards you, easier for snakegate/gateprior **/
    float psi;      /** approach angle through the waypoint, for transverse pass psi = gatepsi ± 180 **/
    float v_sp;     /** velocity setpoint **/
    waypoint_type_t type;
};

class FlightPlan {

    public:
        int num_wp;
        std::vector<waypoint_t> wp; // holds all the waypoints, populated on constructor invoke
        int wp_selector = 0;
        bool close_to_gate = false;
		float dist_to_target = 1000.0; // init to safe value

        FlightPlan();
        ~FlightPlan();
    private:
        bool flightplan_run();
        void add_wp(float x, float y, float z, float gatepsi, float psi, float v_sp, waypoint_type_t type);
        float distance_to_wp(int wp_ID);
};