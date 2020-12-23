#include "state_machine.hpp"
// keyboard
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <chrono>

/*-------------------------------------------------
 initialize special non blocking non echoing Keyboard 
 ------------------------------------------------*/
struct termios orig_term, raw_term;

void init_keyboard(void)
{
  // Get terminal settings and save a copy for later
  tcgetattr(STDIN_FILENO, &orig_term);
  raw_term = orig_term;

  // Turn off echoing and canonical mode
  raw_term.c_lflag &= ~(ECHO | ICANON);

  // Set min character limit and timeout to 0 so read() returns immediately
  // whether there is a character available or not
  raw_term.c_cc[VMIN] = 0;
  raw_term.c_cc[VTIME] = 0;

  // Apply new terminal settings
  tcsetattr(STDIN_FILENO, TCSANOW, &raw_term);
}

void deinit_keyboard(void)
{
    char temp;
    /* Make sure no characters are left in the input stream as
	plenty of keys emit ESC sequences, otherwise they'll appear
	on the command-line after we exit.*/
	while(read(STDIN_FILENO, &temp, 1)==1);
	// Restore original terminal settings
	tcsetattr(STDIN_FILENO, TCSANOW, &orig_term);
}

void state_mc::set_current_state() {
    char key_in = 0;
    while(1) {
        int len = read(STDIN_FILENO, &key_in, 1);
        // todo: mutex this? and read state from MSP also..
        switch (key_in) {
            // arm
            case 'u': {
                this->arm_status = ARM;
                break;
            }
            // disarm
            case 'k': {
                this->arm_status = DISARM;
                // this->~state_mc();
                // std::terminate();
                break;
            }
            case 'd': {
                this->sigint_status = true;
                break;
            }
            default: {
                break;
            }
        }
        // 100 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

state_mc::state_mc() {
    init_keyboard();
    printf("[state-machine] thread spawned!\n");
    st_mc_thread_ = std::thread(&state_mc::set_current_state, this);
}


state_mc::~state_mc() {
    deinit_keyboard();
    st_mc_thread_.detach();
    printf("[state-machine] thread killed!\n");
}