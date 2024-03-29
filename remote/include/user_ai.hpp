#pragma once

#include <chrono>
#include <math.h>

#include "state_machine.hpp"
#include "control.hpp"
#include "natnet.hpp"
#include "msp_node.hpp"
#include "flightplan.hpp"

extern state_mc *st_mc;
extern Controller *controller;
extern NatNet *gps;
extern msp_node *msp;
extern FlightPlan *flightplan;

// start timer
inline std::chrono::high_resolution_clock::time_point timer_start() {
	return std::chrono::high_resolution_clock::now();
}

// stop timer
inline float timer_check(std::chrono::high_resolution_clock::time_point start) {
	std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<float> time_span = std::chrono::duration_cast<std::chrono::duration<float>>(end - start);
	// std::cout << "Timer: " << time_span.count() * pow(10,3) << " ms\n";
	return (float)(time_span.count() * pow(10,3));
}

class user_ai {
    public:
        std::thread user_ai_thread_;
        float curr_time = 0;
        float dt = 0;
        user_ai();
        ~user_ai();
        void run_ai();
        void get_time();
};

// for accessing curr_time and dt..
extern user_ai *ai;