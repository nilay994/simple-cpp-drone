#include "control.hpp"
#include "settings.h"
#include "utils.h"
#include "user_ai.hpp"

// NOTE: ATT macros must be symmetric around 1500 due to mapping function
#define ATT_RCMIN 1200
#define ATT_RCMAX 1800

#define ATT_YAW_RCMIN 1300
#define ATT_YAW_RCMAX 1700

#define THRUST_FLOAT_MAX 0.8  // MAX THROTTLE 
#define THRUST_FLOAT_MIN 0.1  // MIN SAFE THROTTLE
#define KP_ALT 0.32
#define KI_ALT 0.0
#define KD_ALT 0.15
#define HOVERTHRUST 0.5
#define SETPOINT_ALT (-1.5)

#define KP_POS      1.0
#define KP_VEL      1.2
#define MAX_BANK    0.65   // 26 deg max bank
#define K_FF        0.0
#define MAX_VEL     2.5

#define MAX_YAW_RATE 0.5
#define KP_YAW       1.0  

void Controller::control_job() {

    while(1) {
        if(st_mc->arm_status == ARM) {
            this->altitude_control();
            // this->position_control();
            this->toActuators();

            /** change waypoint every twenty seconds **/
            #if 0
            static float prev_time = ai->curr_time;
            static int second_cnt = 0;
            // every second
            if (floor(ai->curr_time) - floor(prev_time) > 0) {
                second_cnt ++;
                printf("secnd_cnt: %d\n", second_cnt);
                // every 5 seconds
                if (second_cnt % 5 == 0) {
                    flightplan->trigger_wp_change = true;
                }
            }
            prev_time = ai->curr_time;
            #endif

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

    // printf("**** t: %f s****\n", ai->curr_time);

    // printf("x: %.02f, y: %.02f, z: %.02f\n", this->robot.pos.x, this->robot.pos.y, this->robot.pos.z);
    // printf("vx: %.03f, vy: %.03f, vz: %.03f\n", this->robot.vel.x, this->robot.vel.y, this->robot.vel.z);
    // printf("r: %.02f, p: %.02f, y: %.02f\n", R2D * this->robot.att.roll, R2D * this->robot.att.pitch, R2D * this->robot.att.yaw);

    static float prev_alttime = ai->curr_time;

	float alt_dt = ai->curr_time - prev_alttime;

    // cannot be lesser than 1ms
	if (alt_dt < 0.001) {
		alt_dt = 0.001;
	}
    
    float error_z = SETPOINT_ALT - robot.pos.z;

    static float throttle_trim_integral = 0.0;
    // PD control: minus sign for NED, -1 * [KP * (position desired - position current) - KD * (zero velocity - velocity current)] + HOVER
	float throttle_cmd = - KP_ALT * error_z - throttle_trim_integral + KD_ALT * robot.vel.z + HOVERTHRUST; // / (cos(this->est_state.pitch)*cos(this->est_state.roll));

    // anti-windup: trim can't be more than 8% of throttle
	if (fabsf(throttle_trim_integral) < 0.08) {
		throttle_trim_integral += (error_z) * alt_dt * KI_ALT;
	}

    throttle_cmd = bound_f(throttle_cmd, THRUST_FLOAT_MIN, THRUST_FLOAT_MAX);

    this->signals_f.thr = throttle_cmd;
    prev_alttime = ai->curr_time;
}

float Controller::position_control() {
    
    float setpoint_x = 0.0;
    float setpoint_y = 0.0;

    // cmd pos = origin
    float curr_error_pos_w_x = (setpoint_x - robot.pos.x);
    float curr_error_pos_w_y = (setpoint_y - robot.pos.y);

    float curr_error_pos_x_velFrame =  cos(robot.att.yaw)*curr_error_pos_w_x - sin(robot.att.yaw)*curr_error_pos_w_y;
    float curr_error_pos_y_velFrame =  sin(robot.att.yaw)*curr_error_pos_w_x + cos(robot.att.yaw)*curr_error_pos_w_y;

    float vel_x_cmd_velFrame = curr_error_pos_x_velFrame * KP_POS; // FWD_VEL_CMD;
    float vel_y_cmd_velFrame = curr_error_pos_y_velFrame * KP_POS;

    vel_x_cmd_velFrame = bound_f(vel_x_cmd_velFrame, -MAX_VEL, MAX_VEL);
    vel_y_cmd_velFrame = bound_f(vel_y_cmd_velFrame, -MAX_VEL, MAX_VEL);

    this->velocity_control(vel_x_cmd_velFrame, vel_y_cmd_velFrame);

    return sqrt(curr_error_pos_w_x*curr_error_pos_w_x + curr_error_pos_w_y*curr_error_pos_w_y);
}

void Controller::velocity_control(float velcmdbody_x, float velcmdbody_y) {

    float vel_x_est_velFrame =  cos(robot.att.yaw) * robot.vel.x - sin(robot.att.yaw) * robot.vel.y;
    float vel_y_est_velFrame =  sin(robot.att.yaw) * robot.vel.x + cos(robot.att.yaw) * robot.vel.y;

    float curr_error_vel_x = velcmdbody_x - vel_x_est_velFrame;
    float curr_error_vel_y = velcmdbody_y - vel_y_est_velFrame;

    float accx_cmd_velFrame = curr_error_vel_x * KP_VEL + K_FF * velcmdbody_x;
    float accy_cmd_velFrame = curr_error_vel_y * KP_VEL + K_FF * velcmdbody_y;

    float unused_yaw = 0.0;
    this->attitude_control(accx_cmd_velFrame, accy_cmd_velFrame, unused_yaw);


}

void::Controller::attitude_control(float a_x, float a_y, float w_z) {
    // 3 options for yaw:
    // 1. custom float yawcmd coming from w_z;
    // 2. yaw towards gate: yaw_cmd = atan2(curr_error_pos_w_y, curr_error_pos_w_x);
    // 3. yaw command from flightplan (drone yaw = gateyaw)

    float yawcmd = this->setpoint.att.yaw;
    float yawerror = yawcmd - this->robot.att.yaw;

    this->signals_f.xb = -1.0 * bound_f(a_y, -MAX_BANK, MAX_BANK);
    this->signals_f.yb = bound_f(a_x, -MAX_BANK, MAX_BANK);
    this->signals_f.zb = bound_f(KP_YAW * yawerror, -MAX_YAW_RATE, MAX_YAW_RATE);
}

// bound rate of change of these signals 
void rateBound(signals<float> *signal) {

    // printf("in: %.05f,%.05f,%.05f,%.05f\n", signal->thr, signal->xb, signal->yb, signal->zb);
    
    // 0.03 = 30 on radio
    #define MAXRATE 0.1

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
    rateBound(&this->signals_f);

    bool chk = !(isfinite(this->signals_f.thr));
    chk |= !(isfinite(this->signals_f.xb));
    chk |= !(isfinite(this->signals_f.yb));
    chk |= !(isfinite(this->signals_f.zb));
    chk |= isnan(this->signals_f.thr);
    chk |= isnan(this->signals_f.xb);
    chk |= isnan(this->signals_f.yb);
    chk |= isnan(this->signals_f.zb);
    
    if (chk) {
        // NaN or Inf in calculations before sending to betaflight
        printf(COLOR_FBLACK);
        printf(COLOR_BRED);
        printf("[control] NaN or Inf sent to betaflight\n");
        printf(COLOR_NONE);
        printf("\n");
        signals_f.thr = 0.0;
        signals_f.xb = 0.0;
        signals_f.yb = 0.0;
        signals_f.zb = 0.0;
        return;
    }

    // float to uint16_t for sending over MSP UART
	this->signals_i.thr = remap_throttle_signals(this->signals_f.thr,  THRUST_RCMIN, THRUST_RCMAX);  // thrust
	this->signals_i.xb  = remap_attitude_signals(this->signals_f.xb,   ATT_RCMIN, ATT_RCMAX);  // roll
	this->signals_i.yb  = remap_attitude_signals(this->signals_f.yb,   ATT_RCMIN, ATT_RCMAX);  // pitch
	this->signals_i.zb  = remap_attitude_signals(this->signals_f.zb,   ATT_YAW_RCMIN, ATT_YAW_RCMAX);  // yaw

    // TODO: mutex control signals with MSP
	// this_hal->get_nav()->update_signals(signals);
	// this_hal->get_nav()->send_signals();
    // printf("%.02f,%.02f,%.02f,%.02f\n", signals_f.thr, signals_f.xb, signals_f.yb, signals_f.zb);
    // printf("%d,%d,%d,%d\n", signals_i.thr, signals_i.xb, signals_i.yb, signals_i.zb);
}

// destructor
Controller::~Controller() {
    // fflush all files
    // if (control_job_.joinable()) {
        control_job_.detach();
    // }
    printf("[ctrl] thread killed!\n");
}
