#include <signal.h>
#include <cstring>
#include <atomic>

#include "user_ai.hpp"
#include "control.hpp"
#include "natnet.hpp"
// #include "state_machine.hpp"

std::atomic<bool> quit(false);    // signal flag

// declare for starting the other threads from user_ai
Controller *controller;
user_ai *ai;
NatNet *gps;
// state_mc *st_mc;
// HealthMonitor *health;
bool kill_signal = false;
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
        std::this_thread::sleep_for(std::chrono::milliseconds(5)); // 200 Hz
    }
}

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

user_ai::user_ai() {

    user_ai_thread_ = std::thread(&user_ai::get_time, this);

    // health
	// health = new HealthMonitor();

    // start control thread
    controller = new Controller();

    // start state machine thread
    // st_mc = new state_mc();

    // natnet
    gps = new NatNet();
    // msp

}

user_ai::~user_ai() {
    
    // kill msp first

    // kill controller
    delete controller;

    // kill state machine
    // delete st_mc;

    delete gps;

    // send final print!
    printf("[AI] thread killed!\n");
}


int main () {

    printf("[AI] Spawning threads!\n");

    // send KILL signal to MSP and finish flush all files to memory
    always_destruct();
    
    ai = new user_ai();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    printf("[AI] starting!\n");
    while(1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if( quit.load()) {
            kill_signal = true;
            break;    // exit normally after SIGINT
        }
    }

    delete ai;

    return 0;
    
}