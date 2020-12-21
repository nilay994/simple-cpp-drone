#include <signal.h>
#include <cstring>
#include <atomic>

#include "user_ai.h"
#include "control.h"

std::atomic<bool> quit(false);    // signal flag

// declare for starting the other threads from user_ai
Controller *controller;
user_ai *ai;
// HealthMonitor *health;

std::chrono::high_resolution_clock::time_point time_obj;

// obtain elapsed time
void user_ai::get_time() {

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

    // health
	// health = new HealthMonitor();

    // start control thread
    controller = new Controller();

    // natnet

    // msp

}

user_ai::~user_ai() {
    
    // kill msp first

    // kill controller
    delete controller;

    // send final print!
    printf("[AI] thread killed!\n");
}

int main () {

    printf("[AI] Spawning threads!!\n");

    // send KILL signal to MSP and finish flush all files to memory
    always_destruct();
    
    ai = new user_ai();

    while(1) {
        ai->get_time();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if( quit.load()) break;    // exit normally after SIGINT
    }

    delete ai;

    return 0;
    
}