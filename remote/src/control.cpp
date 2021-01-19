#include "control.hpp"
#include "settings.h"
#include "utils.h"
#include "user_ai.hpp"


#define THRUST_RCMIN 1000
#define THRUST_RCMAX 2000

#define ATT_RCMIN 1300
#define ATT_RCMAX 1700

#define KP_ALT 0.35
#define KI_ALT 0.08
#define KD_ALT 0.1
#define HOVERTHRUST 0.34
#define SETPOINT_ALT (-1.5)

#define KP_POS      0.5
#define KP_VEL      0.5
#define MAX_BANK    0.5   // 26 deg max bank
#define K_FF        0.0
#define MAX_VEL     0.9

void Controller::control_job() {
    while(1) {
        if(st_mc->arm_status == ARM) {
            this->altitude_control();
            this->position_control();
            this->toActuators();
        }
        // 50 Hz loop
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        // lets send two of these samples to betaflight (MSP = 100 Hz)!
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

    printf("**** t: %f s****\n", ai->curr_time);

    printf("x: %.02f, y: %.02f, z: %.02f\n", this->robot.pos.x, this->robot.pos.y, this->robot.pos.z);
    printf("vx: %.03f, vy: %.03f, vz: %.03f\n", this->robot.vel.x, this->robot.vel.y, this->robot.vel.z);
    printf("r: %.02f, p: %.02f, y: %.02f\n", R2D * this->robot.att.roll, R2D * this->robot.att.pitch, R2D * this->robot.att.yaw);

    static float prev_alttime = ai->curr_time;

	float alt_dt = ai->curr_time - prev_alttime;
	if (alt_dt < 0.001) {
		alt_dt = 0.001;
	}
    
    float error_z = SETPOINT_ALT - robot.pos.z;

    static float throttle_trim_integral = 0.0;
    // PD control: minus sign for NED, -1 * [KP * (position desired - position current) - KD * (zero velocity - velocity current)] + HOVER
	float throttle_cmd = - KP_ALT * error_z - throttle_trim_integral + KD_ALT * robot.vel.z + HOVERTHRUST; // / (cos(this->est_state.pitch)*cos(this->est_state.roll));

    // anti-windup: trim can't be more than 20% of throttle
	if (fabsf(throttle_trim_integral) < 0.2) {
		throttle_trim_integral += (error_z) * alt_dt * KI_ALT;
	}

    // // min safe throttle
    if (throttle_cmd < 0.38) {
         throttle_cmd = 0.38;
    }

    this->signals_f.thr = throttle_cmd;
    prev_alttime = ai->curr_time;
}

void Controller::position_control() {

    // cmd pos = origin
    float curr_error_pos_w_x = (0.0 - robot.pos.x);
    float curr_error_pos_w_y = (0.0 - robot.pos.y);

    float curr_error_pos_x_velFrame =  cos(robot.att.yaw)*curr_error_pos_w_x - sin(robot.att.yaw)*curr_error_pos_w_y;
    float curr_error_pos_y_velFrame =  sin(robot.att.yaw)*curr_error_pos_w_x + cos(robot.att.yaw)*curr_error_pos_w_y;

    float vel_x_cmd_velFrame = curr_error_pos_x_velFrame * KP_POS; // FWD_VEL_CMD;
    float vel_y_cmd_velFrame = curr_error_pos_y_velFrame * KP_POS;

    vel_x_cmd_velFrame = bound_f(vel_x_cmd_velFrame, -MAX_VEL, MAX_VEL);
    vel_y_cmd_velFrame = bound_f(vel_y_cmd_velFrame, -MAX_VEL, MAX_VEL);

    this->velocity_control(vel_x_cmd_velFrame, vel_y_cmd_velFrame);
}

void Controller::velocity_control(float velcmdbody_x, float velcmdbody_y) {

    float vel_x_est_velFrame =  cos(robot.att.yaw) * robot.vel.x - sin(robot.att.yaw) * robot.vel.y;
    float vel_y_est_velFrame =  sin(robot.att.yaw) * robot.vel.x + cos(robot.att.yaw) * robot.vel.y;

    float curr_error_vel_x = velcmdbody_x - vel_x_est_velFrame;
    float curr_error_vel_y = velcmdbody_y - vel_y_est_velFrame;

    float accx_cmd_velFrame = curr_error_vel_x * KP_VEL + K_FF * velcmdbody_x;
    float accy_cmd_velFrame = curr_error_vel_y * KP_VEL + K_FF * velcmdbody_y;

    this->signals_f.yb = bound_f(accx_cmd_velFrame, -MAX_BANK, MAX_BANK);
    this->signals_f.xb = -1.0 * bound_f(accy_cmd_velFrame, -MAX_BANK, MAX_BANK);
    this->signals_f.zb = 0;            // TODO: yaw towards goal -> atan2(curr_error_pos_w_y, curr_error_pos_w_x);
}


// bound rate of change of these signals 
void Controller::rateBound(signals<float> *signal) {

    // printf("in: %.05f,%.05f,%.05f,%.05f\n", signal->thr, signal->xb, signal->yb, signal->zb);
    
    // 0.03 = 30 on radio
    #define MAXRATE 0.08

    // do static structs exist?
    static signals<float> prev_signal = *signal;

    float d_thr = bound_f(signal->thr - prev_signal.thr, -MAXRATE, MAXRATE);
    float d_xb  = bound_f(signal->xb - prev_signal.xb, -MAXRATE, MAXRATE);
    float d_yb  = bound_f(signal->yb - prev_signal.yb, -MAXRATE, MAXRATE);
    float d_zb  = bound_f(signal->zb - prev_signal.zb, -MAXRATE, MAXRATE);

    // new bounded signal output
    signal->thr = prev_signal.thr + d_thr;
    signal->xb = prev_signal.xb + d_xb;
    signal->yb = prev_signal.yb + d_yb;
    signal->zb = prev_signal.zb + d_zb;

    // prev_signal = *signal;
    prev_signal.thr = signal->thr;
    prev_signal.xb = signal->xb;
    prev_signal.yb = signal->yb;
    prev_signal.zb = signal->zb;

    // printf("out: %.05f,%.05f,%.05f,%.05f\n", signal->thr, signal->xb, signal->yb, signal->zb);

}

// send control signals
void Controller::toActuators() {
	// auto signals = this_hal->get_nav()->get_signals();
    this->rateBound(&this->signals_f);

    // float to uint16_t for sending over MSP UART
	this->signals_i.thr = remap_throttle_signals(this->signals_f.thr,  THRUST_RCMIN, THRUST_RCMAX);  // thrust
	this->signals_i.xb  = remap_attitude_signals(this->signals_f.xb,   ATT_RCMIN, ATT_RCMAX);  // roll
	this->signals_i.yb  = remap_attitude_signals(this->signals_f.yb,   ATT_RCMIN, ATT_RCMAX);  // pitch
	this->signals_i.zb  = remap_attitude_signals(this->signals_f.zb,   ATT_RCMIN, ATT_RCMAX);  // yaw

    // TODO: mutex control signals with MSP
	// this_hal->get_nav()->update_signals(signals);
	// this_hal->get_nav()->send_signals();
    printf("%.02f,%.02f,%.02f,%.02f\n", signals_f.thr, signals_f.xb, signals_f.yb, signals_f.zb);
    printf("%d,%d,%d,%d\n", signals_i.thr, signals_i.xb, signals_i.yb, signals_i.zb);
}

// destructor
Controller::~Controller() {
    // fflush all files
    // if (control_job_.joinable()) {
        control_job_.detach();
    // }
    printf("[ctrl] thread killed!\n");
}
