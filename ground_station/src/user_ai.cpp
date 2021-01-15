#include "user_ai.hpp"

// declare for starting the other threads from user_ai
user_ai *ai;
NatNet *gps;

std::chrono::high_resolution_clock::time_point time_obj;

// obtain elapsed time
void user_ai::get_time() {
    while(1) {
        static float current_time = 0;    // s
        static float previous_time = 0;   // s
        static float delta_t = 0.001;     // s (max = 1000Hz)
        
        static bool first_call = true;
        static float curr_time_i = 0; //s for curr_time correction

        if (first_call == true) {
            first_call = false;
            time_obj = timer_start();
            curr_time_i = timer_check(time_obj)*pow(10,-3);
        }

        current_time = (timer_check(time_obj) * pow(10,-3)) - curr_time_i;
        delta_t  = current_time - previous_time;
        if (delta_t < 0.001) { // based on 433MHz = 2.3ms
            delta_t = 0.001;
        }
        this->curr_time = current_time;
        this->dt = delta_t;
        previous_time = current_time;
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1000 Hz
    }
}

/* SIGINT Behaviour */
#include <signal.h>
#include <cstring>
#include <atomic>
std::atomic<bool> quit(false);    // signal flag
void got_signal(int) {
    quit.store(true);
}

/* always safely destruct on SIGINT */
void always_destruct() {
    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    sa.sa_handler = got_signal;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT,&sa,NULL);
}

/* main program constructor */

user_ai::user_ai() {
    user_ai_thread_ = std::thread(&user_ai::get_time, this);
}

user_ai::~user_ai() {

    printf("[AI] Killing and Disarming!\n");

    delete gps;

    // send final print!
    
    printf("[AI] thread killed!\n");

    user_ai_thread_.detach();

    // health
	// delete health;
}


int main () {

    printf("[AI] Spawning threads!\n");

    // send KILL signal to MSP and finish flush all files to memory
    // always_destruct();

    // order of starting thread matters because of externed pointer instantiation    
    ai = new user_ai();

    // natnet
    gps = new NatNet();

    printf("[AI] starting!\n");

    while(1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if(quit.load()) {
            printf("[AI] sigint, killing!\n");
            break;    // exit normally after SIGINT
        }
    }

    ai->~user_ai();

    return 0;
    
}