#pragma once

#include <thread>
#include <math.h>
#include "state.h"

/* remap thrust signals, 0 to 1 is mapped to 1000 to 2000
 * If this function is given val = 0.5, half of max thrust is sent to betaflight */
inline uint16_t remap_throttle_signals(float val, uint16_t min, uint16_t max) {
	uint16_t tmpval = round(min) + round(val * ((float)max - (float)min));
	if (tmpval > max) {
		tmpval = max;
	}
	if (tmpval < min) {
		tmpval = min;
	}
	return tmpval;
}


/* remap attitude signals, -1 to 1 is mapped to 1000 to 2000 
 * If this function returns value tmpval = 0, zero attitude command is sent to betaflight. */
inline uint16_t remap_attitude_signals(float val, uint16_t min, uint16_t max) {
	float avg = ((float) min + (float) max) * 0.5;
	uint16_t tmpval = round((float) avg + (val) * ((float)max - (float)min) * 0.5);
	if (tmpval > max) {
		tmpval = max;
	}
	if (tmpval < min) {
		tmpval = min;
	}
	return tmpval;
}

class Controller {
    private:
        std::thread control_job_;

    public:
		robot_t robot;

		/* signals_f for thrust must be between 0 to +1
		   signals_f for attitude must be between -1 to +1 */
        signals<float> signals_f;

		/* these signals are directly picked up by uart.
		   carefully bound these between 1000 to 2000 */
        signals<uint16_t> signals_i;

        Controller();
        ~Controller();
        
		void control_job();
        void altitude_control();
		float position_control();
		void velocity_control(float, float);
		void attitude_control();
        void toActuators();
};