#include "control.hpp"
#include "settings.h"
#include "utils.h"
#include "user_ai.hpp"


#define THRUST_RCMIN 1000
#define THRUST_RCMAX 2000

#define ATT_RCMIN 1400
#define ATT_RCMAX 1600

#define KP_ALT 0.15
#define KI_ALT 0.01
#define KD_ALT 0.02
#define HOVERTHRUST 0.32
#define SETPOINT_ALT (-1.5)

#define FWD_VEL_CMD 0.2
#define KP_POS      0.2
#define KP_VEL      0.4
#define MAX_BANK    0.5   // 26 deg max bank
#define K_FF        0.0
#define MAX_VEL     0.5

void Controller::control_job() {
    while(1) {
        if(st_mc->arm_status == ARM) {
            this->altitude_control();
            this->position_control();
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

    // anti-windup
	if (fabsf(throttle_trim_integral) < 0.15) {
		throttle_trim_integral += (error_z) * alt_dt * KI_ALT;
	}

    // // min safe throttle
    // if (throttle_cmd < HOVERTHRUST - 0.15) {
    //     throttle_cmd = HOVERTHRUST - 0.15;
    // }

    // throttle_cmd = bound_f(throttle_cmd, RCMIN, RCMAX);
    this->signals_f.thr = throttle_cmd;
    prev_alttime = ai->curr_time;
}

void Controller::position_control() {

    // cmd pos = origin
    float curr_error_pos_w_x = (0.0 - robot.pos.x);
    float curr_error_pos_w_y = (0.0 - robot.pos.y);

    float curr_error_pos_x_velFrame =  cos(robot.att.yaw)*curr_error_pos_w_x + sin(robot.att.yaw)*curr_error_pos_w_y;
    float curr_error_pos_y_velFrame = -sin(robot.att.yaw)*curr_error_pos_w_x + cos(robot.att.yaw)*curr_error_pos_w_y;

    float vel_x_cmd_velFrame = curr_error_pos_x_velFrame * KP_POS; // FWD_VEL_CMD;
    float vel_y_cmd_velFrame = curr_error_pos_y_velFrame * KP_POS;

    vel_x_cmd_velFrame = bound_f(vel_x_cmd_velFrame, -MAX_VEL, MAX_VEL);
    vel_y_cmd_velFrame = bound_f(vel_y_cmd_velFrame, -MAX_VEL, MAX_VEL);

    this->velocity_control(vel_x_cmd_velFrame, vel_y_cmd_velFrame);
}

void Controller::velocity_control(float velcmdbody_x, float velcmdbody_y) {

    float vel_x_est_velFrame =  cos(robot.att.yaw) * robot.vel.x + sin(robot.att.yaw) * robot.vel.y;
    float vel_y_est_velFrame = -sin(robot.att.yaw) * robot.vel.x + cos(robot.att.yaw) * robot.vel.y;

    float curr_error_vel_x = velcmdbody_x - vel_x_est_velFrame;
    float curr_error_vel_y = velcmdbody_y - vel_y_est_velFrame;

    float accx_cmd_velFrame = curr_error_vel_x * KP_VEL + K_FF * velcmdbody_x;
    float accy_cmd_velFrame = curr_error_vel_y * KP_VEL + K_FF * velcmdbody_y;

    this->signals_f.yb = -1 * bound_f(accx_cmd_velFrame, -MAX_BANK, MAX_BANK);
    this->signals_f.xb =      bound_f(accy_cmd_velFrame, -MAX_BANK, MAX_BANK);
    this->signals_f.zb = 0;            // TODO: yaw towards goal -> atan2(curr_error_pos_w_y, curr_error_pos_w_x);
}


// send control signals
void Controller::toActuators() {
	// auto signals = this_hal->get_nav()->get_signals();
	
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