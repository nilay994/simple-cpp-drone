#include "flightplan.hpp"
#include "utils.h"
#include "settings.h"
#include "user_ai.hpp"

#include <iostream>
#include <sstream>
#include <fstream>

FILE *flightplan_f;

void FlightPlan::flightplan_thread() {
    while(1) {
        if (st_mc->arm_status == ARM) {
            this->print_flightplan();
            this->flightplan_run();
        }
        // 5 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

FlightPlan::FlightPlan() {

    this->add_wp(-2.0, -2.0, -1.5, D2R * -90,  D2R * -90, 1.5, START);
    this->add_wp(-2.0, 0.0, -1.5, D2R * 0,    D2R * 0,   1.5, GATE);
    this->add_wp(0.0, 2.0, -1.5, D2R * 0,    D2R * 0,   1.5, GATE);
    this->add_wp(2.0, 0.0, -1.5, D2R * 90,    D2R * 90,   1.5, GATE);
    this->add_wp(0.0, -2.0, -1.5, D2R * 180,    D2R * 180,   1.5, FINISH);
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

    /** flightplan thread initialize **/
    flightplan_thread_ = std::thread(&FlightPlan::flightplan_thread, this);
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

    // close to gate bool for special actions to perform when camera fov sees less corners
	if (this->dist_to_target > CLOSE_TO_GATE_THRESHOLD) {
		this->close_to_gate = false;
	} else {
		this->close_to_gate = true;
	}

    // if ((this->wp[this->wp_selector].type == GATE      && this->dist_to_target < GATE_THRESHOLD) ||
    // (this->wp[this->wp_selector].type == SKIP_GATE && this->dist_to_target < SKIP_GATE_THRESHOLD) || 
    // (this->wp[this->wp_selector].type == FINISH    && this->dist_to_target < FINISH_THRESHOLD) ||
    // (this->wp[this->wp_selector].type == WAYPOINT  && this->dist_to_target < WAYPOINT_THRESHOLD)) {

    // switch to the next waypoint iff..
	if (this->trigger_wp_change == true) {
        // disable the trigger
        this->trigger_wp_change = false;

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



// print waypoint status
void FlightPlan::print_flightplan() {
	static int last_printed_wp = 99;
	if (last_printed_wp == this->wp_selector)
		return;
	last_printed_wp = this->wp_selector;

	printf("Gate or Waypoint passed!\n");
    printf("\033[1;32m");
    printf("Waypoint ");
    if (this->wp[this->wp_selector].type == START) {
        printf("START ");
    } else if (this->wp[this->wp_selector].type == GATE) {
        printf("GATE ");
    } else if (this->wp[this->wp_selector].type == WAYPOINT) {
        printf("WAYPOINT ");
    } else if (this->wp[this->wp_selector].type == FINISH) {
        printf("FINISH ");
    }
    printf("(ID: %i) reached.\n", this->wp_selector);
    printf("\033[0m");

    if (this->wp[this->wp_selector].type == FINISH) {
		printf("SCORE: %f\n", ai->curr_time);
		printf("\033[1;31m");
		float tp = ai->curr_time - START_FLIGHTPLAN;
		printf(" ******** SCORE : %f  **********", tp);
		printf("\033[0m");
        return;
    }

    printf("\033[1;31m");
    printf("Next waypoint: ");
    if (this->wp[this->wp_selector+1].type == START) {
        printf("START ");
    } else if (this->wp[this->wp_selector+1].type == GATE) {
        printf("GATE ");
    } else if (this->wp[this->wp_selector+1].type == WAYPOINT) {
        printf("WAYPOINT ");
    } else if (this->wp[this->wp_selector+1].type == FINISH) {
        printf("FINISH ");
    }
    printf("(ID: %i).\n", this->wp_selector+1);
    printf("X: %.2f m.\n", this->wp[this->wp_selector+1].x);
    printf("Y: %.2f m.\n", this->wp[this->wp_selector+1].y);
    printf("Z: %.2f m.\n", this->wp[this->wp_selector+1].z);
    printf("Yaw: %.2f deg.\n", this->wp[this->wp_selector+1].psi * R2D);
    printf("Speed: %.2f m/s.\n\n", this->wp[this->wp_selector+1].v_sp);
    printf("\033[0m");

}


/**
 * Reads csv file into table, returns a vector of vector of floats.
 * @param inputFileName input file name.
 */
std::vector<std::vector<float>> FlightPlan::parse_csv_file(std::string inputFileName) {

    std::vector<std::vector<float>> data;
    std::ifstream inputFile(inputFileName);
    int l = 0;
 
    while (inputFile) {
        l++;
        std::string s;
        if (!std::getline(inputFile, s)) break;
        if (s[0] != '#') {
            std::istringstream ss(s);
            std::vector<float> record;
 
            while (ss) {
                std::string line;
                if (!std::getline(ss, line, ','))
                    break;
                try {
                    record.push_back(stof(line));
                }
                catch (const std::invalid_argument e) {
                    std::cout << "NaN found in file " << inputFileName << " line " << l
                         << std::endl;
                    e.what();
                }
            }
 
            data.push_back(record);
        }
    }
 
    if (!inputFile.eof()) {
        std::cerr << "Could not read file " << inputFileName << "\n";
        std::__throw_invalid_argument("File not found.");
    }
 
    return data;
}