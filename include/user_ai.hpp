#pragma once

#include <chrono>
#include <math.h>

// instead include them in .cpp
#include "control.hpp"
#include "natnet.hpp"
extern Controller* controller;
extern NatNet* gps;

// start timer
inline std::chrono::high_resolution_clock::time_point timer_start() {
	return std::chrono::high_resolution_clock::now();
}

// stop timer
inline double timer_check(std::chrono::high_resolution_clock::time_point start) {
	std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
	// std::cout << "Timer: " << time_span.count() * pow(10,3) << " ms\n";
	return (double)(time_span.count() * pow(10,3));
}

extern bool kill_signal;
class user_ai {
    public:
        std::thread user_ai_thread_;
        // bool kill = false;
        float curr_time = 0;
        float dt = 0;
        user_ai();
        ~user_ai();
        void run_ai();
        void get_time();
};


// for accessing curr_time and dt..
extern user_ai *ai;