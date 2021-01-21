#include "flightplan.hpp"
#include "utils.h"
#include "settings.h"
#include "user_ai.hpp"

FILE *flightplan_f;

FlightPlan::FlightPlan() {

    this->add_wp(-2.0, -2.0, -1.5, D2R * -90,  D2R * -90, 1.5, START);
    this->add_wp(-2.0, +2.0, -1.5, D2R * 0,    D2R * 0,   1.5, GATE);
    this->num_wp = (int)this->wp.size();

    /** write which waypoints were populated **/
    flightplan_f = fopen("waypoints.csv", "w+");
	for (int i = 0; i < this->num_wp; i++) {
		fprintf(flightplan_f, "%f, %f, %f, %f, %f, %f, %d\n", this->wp[i].x, this->wp[i].y, this->wp[i].z,
                    this->wp[i].gatepsi, this->wp[i].psi, this->wp[i].v_sp, this->wp[i].type);
	}

    fclose(flightplan_f);
	
    /** log current progress of flight through the track **/
    flightplan_f = fopen("flightplan.csv", "w+");
}

FlightPlan::~FlightPlan() {
    fflush(flightplan_f);
}

// add waypoint to the fligbtplan
void FlightPlan::add_wp(float x, float y, float z, float gatepsi, float psi, float v_sp, waypoint_type_t type) {
    waypoint_t wp_tmp = {x, y, z, gatepsi, psi, v_sp, type};
    this->wp.push_back(wp_tmp);
}

// change outer loop commands based on drone's state
bool FlightPlan::flightplan_run() {
	if (ai->curr_time <= START_FLIGHTPLAN) {
        return false;
	}

	if (this->wp[this->wp_selector].type == START) {
		this->wp_selector++;	
	}

	// position and velocity commands
	controller->setpoint.pos.x = this->wp[this->wp_selector].x;
	controller->setpoint.pos.y = this->wp[this->wp_selector].y;
	controller->setpoint.pos.z = this->wp[this->wp_selector].z;
	controller->setpoint.vel.x = this->wp[this->wp_selector].v_sp; // * cos(this->wp[this->wp_selector].psi);
	controller->setpoint.vel.y = this->wp[this->wp_selector].v_sp * sin(this->wp[this->wp_selector].psi);

	this->dist_to_target = this->distance_to_wp(this->wp_selector);

	if ((this->wp[this->wp_selector].type == GATE      && this->dist_to_target < GATE_THRESHOLD) ||
		(this->wp[this->wp_selector].type == SKIP_GATE && this->dist_to_target < SKIP_GATE_THRESHOLD) || 
		(this->wp[this->wp_selector].type == FINISH    && this->dist_to_target < FINISH_THRESHOLD) ||
	    (this->wp[this->wp_selector].type == WAYPOINT  && this->dist_to_target < WAYPOINT_THRESHOLD)) {
		
        // print waypoint
        // this->print_waypoint();

		// reset ransac buffer on waypoint change

		// update waypoint counter
		if (this->wp[this->wp_selector].type != FINISH) {
			this->wp_selector++;
			if (this->wp_selector >= ((int)this->wp.size())) {
				this->wp_selector = 1;
				// this->lap_number ++;
			}
		}
	}

	// TODO: potential improvement in yaw command
	// modify the yaw based on distance to target
	this->dist_to_target = this->distance_to_wp(this->wp_selector);

	if (this->dist_to_target > CLOSE_TO_GATE_THRESHOLD) {
		this->close_to_gate = false;
	} else {
		this->close_to_gate = true;
	}

    #ifdef LOG
        int close = 0;
        if (this->close_to_gate) close = 1;
        fprintf(flightplan_f, "%f,%d,%d,%f\n", controller->curr_time, this->wp_selector, close, this->dist_to_target);
    #endif

	return true;
}

// distance to waypoint
float FlightPlan::distance_to_wp(int wp_ID) {

	float delta_x = this->wp[wp_ID].x - controller->robot.pos.x;
	float delta_y = this->wp[wp_ID].y - controller->robot.pos.y;

	float psi_gate = this->wp[this->wp_selector].psi;

	// Rotate delta_positon to gate coordinates, then X is the distance to the gate
	return  delta_x * cos(psi_gate) + delta_y * sin(psi_gate);

	// return norm2(delta_x, delta_y); // + pow(delta_z, 2));
}