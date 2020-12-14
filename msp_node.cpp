// Adopted from AscendNTNU: 
// https://github.com/AscendNTNU/msp_flightcontroller_interface

#include "msp.hpp"

#include <iostream>

#include <algorithm>
#include <cmath>
// threading
#include <chrono>
#include <thread>

// keyboard
#include <termios.h>
#include <unistd.h>

class MspInterface {
    MSP msp;
    std::vector<uint16_t> rcData;
    double max_roll_r  = 200;
    double max_pitch_r = 200;
    double max_yaw_r   = 200;
    double hover_thrust = 0.3;
    double mass = 1.0;
    bool new_rates = false;
    Payload serialize_rc_data() {
        Payload result;
        for (int i = 0; i < rcData.size(); i++) {
            result.put_u16(rcData[i]);
        }
        return result;
    }

public:
    MspInterface() : msp("/dev/ttyUSB0"), rcData(5, 1500)
    {
        // Set thurst to 0.5
        rcData[2] = 1500;

        // WARNING: always arm
        rcData[4] = 2000;

        msp.register_callback(MSP::RC, [this](Payload payload) {
            std::vector<uint16_t> droneRcData(payload.size() / 2);
            for (int i = 0; i < droneRcData.size(); i++) {
                droneRcData[i] = payload.get_u16();
            }
        });
    }
    
    void set_rates(char directn) {

        switch (directn) {
            case 'w':
                rcData[1] = 1600;
                break;
            case 'a':
                rcData[0] = 1400;
                break;
            case 'd':
                rcData[0] = 1600;
                break;
            case 's':
                rcData[1] = 1400;
                break;
            case 'q':
                rcData[3] = 1400;
                break;
            case 'e':
                rcData[3] = 1600;
                break;
            case 'r': {
                rcData[0] = 1500;
                rcData[1] = 1500;
                rcData[2] = 1000;
                rcData[3] = 1500;
                rcData[4] = 1000;
                break;}
            default: break;
        }
        
        // double thrust = rates.thrust.z / 9.81 / mass * hover_thrust;
        double thrust = 0.8;
        rcData[2] = (uint16_t) std::min(1000, std::max(0, (int) round(thrust * 1000))) + 1000;
    }
    void step_hf(char directn) {
        set_rates(directn);

        // Send rc data
        msp.send_msg(MSP::SET_RAW_RC, serialize_rc_data());

        // Recieve new msp messages
        msp.recv_msgs();
    }
    void step_lf() {
        // Request rc data
        msp.send_msg(MSP::RC, {});

        // Recieve new msp messages
        msp.recv_msgs();
    }
};

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
	// Restore original terminal settings
	tcsetattr(STDIN_FILENO, TCSANOW, &orig_term);
}


int main(int argc, char** argv) {
    MspInterface iface;
    init_keyboard();

    /* arm after 5 seconds */
    for (int i=5; i>0; i--) {
        std::cout << "Arming in " << i << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    int i = 0; char directn;
    // while key pressed!=kill
    while (directn!='k') {
        i++;
        if (i >= 10) {
            iface.step_lf();
            i = 0;
        }

        int len = read(STDIN_FILENO, &directn, 1); 
        iface.step_hf(directn);
        // 50 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    /* Make sure no characters are left in the input stream as
	plenty of keys emit ESC sequences, otherwise they'll appear
	on the command-line after we exit.*/
	while(read(STDIN_FILENO, &directn, 1)==1);
    deinit_keyboard();

    return 0;
}
