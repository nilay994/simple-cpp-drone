#include "control.hpp"
#include "settings.h"
#include "utils.h"
#include "user_ai.hpp"

void Controller::control_job() {
    while(1) {
        if(st_mc->arm_status == ARM) {
            this->altitude_control();
            this->toActuators();
        }
        // 50 Hz loop
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

// constructor
Controller::Controller() {    
    try {
        control_job_ = std::thread(&Controller::control_job, this);
        printf("[ctrl] thread spawned!\n");
    } catch (...) {
        printf("[ctrl] can't start thread!\n");
    }
}

void Controller::altitude_control() {

    // printf("z: %f, vz: %f\n", this->robot.pos.z, this->robot.vel.z);

    printf("%f, %f\n", ai->curr_time, 1.0/ai->dt);

    static float prev_alttime = ai->curr_time;

	float alt_dt = ai->curr_time - prev_alttime;
	if (alt_dt < 0.001) {
		alt_dt = 0.001;
	}
    
    float error_z = SETPOINT_ALT - robot.pos.z;

    static float throttle_trim_integral = 0.0;
    // PD control: minus sign for NED, -1 * [KP * (position desired - position current) - KD * (zero velocity - velocity current)] + HOVER
	float throttle_cmd = - KP_ALT * error_z - throttle_trim_integral + KD_ALT * robot.vel.z + HOVERTHRUST; // / (cos(this->est_state.pitch)*cos(this->est_state.roll));

	if (throttle_cmd < 0.99) {
		throttle_trim_integral += (error_z) * alt_dt * KI_ALT;
	}

    // min safe throttle
    if (throttle_cmd < HOVERTHRUST - 0.15) {
        throttle_cmd = HOVERTHRUST - 0.15;
    }

    throttle_cmd = bound_f(throttle_cmd, RCMIN, RCMAX);
    this->signals_f.thr = throttle_cmd;
    prev_alttime = ai->curr_time;
}


// send control signals
void Controller::toActuators() {
	// auto signals = this_hal->get_nav()->get_signals();
	
    // float to uint16_t for sending over MSP UART
	signals_i.thr = remap_throttle_signals(this->signals_f.thr,  RCMIN, RCMAX);  // thrust
	signals_i.xb  = remap_attitude_signals(this->signals_f.xb,   RCMIN, RCMAX);  // roll
	signals_i.yb  = remap_attitude_signals(this->signals_f.yb,   RCMIN, RCMAX);  // pitch
	signals_i.zb  = remap_attitude_signals(this->signals_f.zb,   RCMIN, RCMAX);  // yaw

    // TODO: mutex control signals with MSP
	// this_hal->get_nav()->update_signals(signals);
	// this_hal->get_nav()->send_signals();

#ifdef LOG
	int t = signals.throttle;
	int x = signals.roll;
	int y = signals.pitch;
	int z = signals.yaw;
	fprintf(betaflight_f, "%f,%d,%d,%d,%d\n", this->curr_time, t, x, y, z);
#endif
}

// destructor
Controller::~Controller() {
    // fflush all files
    // if (control_job_.joinable()) {
        control_job_.detach();
    // }
    printf("[ctrl] thread killed!\n");
}