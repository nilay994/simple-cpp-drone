#include "ctrl_optimizer.h"

#include <qpOASES.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

USING_NAMESPACE_QPOASES

#define MAX_N 160

float x_acc[MAX_N];
float y_acc[MAX_N];
unsigned int i = 0;
unsigned int invoke_cnt = 1;
unsigned int N = MAX_N;
volatile bool lock_optimal = 0;

// #define FB

FILE *states_f;

// constructor can be initialized by Hessian type, so it can stop checking for positive definiteness
Options optionsmpc;

Eigen::MatrixXf oldR(4, 2 * MAX_N);
Eigen::MatrixXf states(4, MAX_N);
Eigen::MatrixXf eye(2* MAX_N, 2 * MAX_N); 
Eigen::MatrixXf old_H(2 * MAX_N, 2 * MAX_N);
Eigen::MatrixXf R(4, 2 * MAX_N);
Eigen::MatrixXf f(1, 2 * MAX_N);

void* flush_job(void* data) {

	static bool opt_working = false;
	if (!opt_working) {
		printf("\n\n\n\nOPTIMAL: Job working\n\n\n\n\n\n");
		opt_working = true;
	}

	while(1) {
		if(lock_optimal && i < N) {
			// drone model discretized at h = 0.1 s, flush the inputs at the same rate
			std::this_thread::sleep_for(std::chrono::milliseconds(99));
			float tmp_yaw = controller->corr_state.yaw + OFFSET_YAW;

			#ifdef FB
				float curr_error_pos_w_x = states(1,i) - controller->corr_state.x;
				float curr_error_pos_w_y = states(3,i) - controller->corr_state.y;

				// Camera coordinates speed
				float curr_error_pos_x_camFrame = cos(tmp_yaw)*curr_error_pos_w_x + sin(tmp_yaw)*curr_error_pos_w_y;
				float curr_error_pos_y_camFrame =- sin(tmp_yaw)*curr_error_pos_w_x + cos(tmp_yaw)*curr_error_pos_w_y;

				// Create a speed command from the position errors
				float vel_x_fb_camFrame = curr_error_pos_x_camFrame * KP_POS_X;
				float vel_y_fb_camFrame = curr_error_pos_y_camFrame * KP_POS_Y; // * (1.0 - K_CENTERLINE) + gate_centerline_vy * K_CENTERLINE;

				// Actual speed in camera frame
				float vel_x_est_camFrame = cos(tmp_yaw) * controller->corr_state.vx + sin(tmp_yaw) * controller->corr_state.vy;
				float vel_y_est_camFrame = - sin(tmp_yaw) * controller->corr_state.vx + cos(tmp_yaw) * controller->corr_state.vy;

				float vel_x_ff_camFrame = cos(tmp_yaw) * states(0,i) + sin(tmp_yaw) * states(2,i);
				float vel_y_ff_camFrame = - sin(tmp_yaw) * states(0,i) + cos(tmp_yaw) * states(2,i);

				float curr_error_vel_x = vel_x_fb_camFrame - vel_x_est_camFrame;
				float curr_error_vel_y = vel_y_fb_camFrame - vel_y_est_camFrame;

				float accx_cmd_camFrame = curr_error_vel_x * KP_VEL_X + K_FF_THETA * vel_x_ff_camFrame;
				float accy_cmd_camFrame = curr_error_vel_y * KP_VEL_Y + K_FF_PHI   * vel_y_ff_camFrame;


				// in body frame
				// float accx_fb_bodyFrame = cos(-OFFSET_YAW) * accx_cmd_camFrame + sin(-OFFSET_YAW) * accy_cmd_camFrame;
				// float accy_fb_bodyFrame = -sin(-OFFSET_YAW) * accx_cmd_camFrame + cos(-OFFSET_YAW) * accy_cmd_camFrame;


				// Max roll rate
				float pitch_fb  = bound_f(-accx_cmd_camFrame, -0.2, 0);
				float roll_fb   = bound_f(accy_cmd_camFrame, -0.2, 0.2);

				float pitch_cmd_bodyframe_ff = cos(controller->corr_state.yaw) * x_acc[i] - sin(controller->corr_state.yaw) * y_acc[i];
				float roll_cmd_bodyframe_ff  = sin(controller->corr_state.yaw) * x_acc[i] + cos(controller->corr_state.yaw) * y_acc[i];

				printf("roll: fb: %f, ff: %f \t", roll_fb  * R2D, roll_cmd_bodyframe_ff  * R2D);
				printf("pitch fb: %f, ff: %f \n", pitch_fb * R2D, pitch_cmd_bodyframe_ff * R2D);
				// TODO: verify frame of ref
				controller->pitch_cmd = pitch_cmd_bodyframe_ff + pitch_fb;
				controller->roll_cmd  = roll_cmd_bodyframe_ff + roll_fb;
			#else 

				float pitch_cmd_bodyframe_ff = cos(controller->corr_state.yaw) * x_acc[i] - sin(controller->corr_state.yaw) * y_acc[i];
				float roll_cmd_bodyframe_ff  = sin(controller->corr_state.yaw) * x_acc[i] + cos(controller->corr_state.yaw) * y_acc[i];

				controller->pitch_cmd = pitch_cmd_bodyframe_ff;
				controller->roll_cmd  = roll_cmd_bodyframe_ff;
			#endif

			if (controller->pitch_cmd > -0.3) {
				controller->pitch_cmd = -0.3;
			} 
			
			printf("\033[1;35mwaypoint:%d, [%d/%d] \033[0m\n", flightplan->wp_selector, i, N);
			i++;

			// old_wp = flightplan->wp_selector;
		}
		if (i >= N) {
			controller->pitch_cmd = 0;
			controller->roll_cmd  = 0;
			printf("\033[1;31mFinised sequence at %f seconds\033[0m\n", controller->curr_time);
			lock_optimal = 0;
			i = 0;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	return NULL;
}

void* calc_job(void* data) {
	while(1) {
		static int old_wp = flightplan->wp_selector;
		int wp = flightplan->wp_selector;
		bool wp_changed = false;
		if (old_wp != wp) {
			wp_changed = true;
			printf("\n\n\n\nWP changed!!\n\n\n\n");
		}
		// take off delay
		if ((controller->curr_time > 0.5) && ((flightplan->dist_to_target > 6.5) || wp_changed)) { // 
			printf("dist to target: %f\n", flightplan->dist_to_target);
			lock_optimal = 0; 
			optimizer->optimize();
            i = 0;

			// prediction horizon of 2 seconds
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			wp_changed = false;
		}
		old_wp = wp;
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	
	return NULL;
}

Optimizer::Optimizer() {
	states_f = fopen("states.csv", "w+");
	//options.enableFlippingBounds = BT_FALSE;
	optionsmpc.printLevel = PL_NONE;  // DEBUG level high, but the prints are muted in the library :(
	optionsmpc.initialStatusBounds = ST_INACTIVE;
	optionsmpc.numRefinementSteps = 1;
	// optionsmpc.enableCholeskyRefactorisation = 1;
}


Optimizer::~Optimizer() {

}


void Optimizer::solveQP(void) {

	invoke_cnt = invoke_cnt + 1;
	Eigen::Matrix<float, 4, 4> A;

	// TODO: drone drag -0.3
	// sim drag -0.75
	A << 0.9277, 0, 0, 0,
		0.09634, 1, 0, 0,
		0, 0, 0.9277, 0,
		0, 0, 0.09634, 1;

	// sim drag -0.75
	Eigen::Matrix<float, 4, 2> B;
	B << -0.9451, 0,
		-0.04785, 0,
		0, 0.9451,
		0, 0.04785;

	// position reprimanded 5 times more
	Eigen::Matrix<float, 4, 4> P;
	P << 1,0,0,0,
		0,20,0,0,
		0,0,1,0,
		0,0,0,20;

	// position final is one of the gates in the arena
	float pos0[2] = {controller->robot.pos.x, controller->robot.pos.y};
    // flightplan->pos_cmd[0], flightplan->pos_cmd[1]
    // hardcoded gate2
    float gate2 = 2.0;
    float 
     
	float posf[2] = {};

	// go through the gate with 1.5m/s forward vel
	float fwd_speed = 1.5;
	float vel0[2] = {controller->robot.vel.x, controller->robot.vel.y};
	float velf[2] = {fwd_speed * cos(flightplan->wp[flightplan->wp_selector].psi), fwd_speed * sin(flightplan->wp[flightplan->wp_selector].psi)};

	// above state space matrices are discretized at 100 milliseconds/10 Hz
	float dt = 0.1;

	// break optimizer if more than 30% of banging
	float bangedtheta = 0; float bangedphi = 0;
	float iterate = 1;
	float calctimestart = ai->curr_time;

	// while(bangedphi < 30 && bangedtheta < 30) {
		// float T = sqrt(pow((pos0[0] - posf[0]),2) + pow((pos0[1] - posf[1]),2)) / iterate;
		
		// assumption: can reach anywhere in the arena if I have ten seconds 
		float T = 3.0; // 3 seconds 
		N = round(T/dt);

		// lowest samples for doing opt: two sample bang-bang
		// if removed: can lead to segfault
		if (N < 2) {
			N = 2;
		}
		// printf("Horizon: %d\n", N);  

		oldR.resize(4, 2*N);
		oldR.block(0, 2*N-2, 4, 2) =  B;
		Eigen::Matrix<float, 4, 4> AN = A;

		for(unsigned int i=1; i<N; i++) {
			oldR.block(0, 2*N-2*(i+1), 4, 2) =  A * oldR.block(0, 2*N-2*i, 4, 2);
			AN = A * AN; 
		}

		Eigen::MatrixXf R = oldR.block(0,0,4,2*N);
		Eigen::MatrixXf old_H = 2 * (R.transpose() * P * R);

		eye.resize(2*N, 2*N);
		eye.setIdentity();
		eye = 0.3 * eye; // TODO: verify after this change - augnment to keep hessian invertable says harvard
		// WARNING: augmenting on diagonals to improve invertability has an impact on the rate of change of input sequence
		Eigen::MatrixXf H = 0.5 * (old_H + old_H.transpose() + eye);

		Eigen::Matrix<float, 4, 1> x0; 
		Eigen::Matrix<float, 4, 1> Xf;
		x0 << vel0[0], pos0[0], vel0[1], pos0[1];
		Xf << velf[0], posf[0], velf[1], posf[1];
		
		Eigen::MatrixXf f;
		f = (2 * ((AN * x0)- Xf)).transpose() * P * R;

		int sizes = 2 * N;

		real_t ub[sizes]; real_t lb[sizes];
		
		for (int i=0; i<sizes; i++) {
			// if (i % 2 == 0) {
			// 	// don't pitch up
			// 	ub[i] =  0;
			// } else {
			// 	ub[i] = MAX_ANG_ROLL;
			// }
			ub[i] = MAX_ANG_TOTAL;
			lb[i] = -MAX_ANG_TOTAL;
		}
		// populate hessian and linear term
		real_t newH[sizes * sizes];	real_t newf[sizes];
		Eigen::Map<Eigen::MatrixXf>(newH, sizes, sizes) = H.transpose();
		Eigen::Map<Eigen::MatrixXf>(newf, 1, sizes) = f;

		// our class of problem, we don't have any constraints on position or velocity, just the inputs
		/* Setting up QProblemB object. */
		QProblemB mpctry(2*N);  
		mpctry.setOptions(optionsmpc);

		/* not sure: take 100 tries to solve the QP? */
		int nWSR = 100;
		
		/* Initialize QP. */
		real_t xOpt[sizes];
		if (mpctry.init(newH,newf, lb, ub, nWSR, 0) == SUCCESSFUL_RETURN) {
			float bangedtheta_acc = 0;
			float bangedphi_acc = 0;
			/* solve the QP */
			if (mpctry.getPrimalSolution(xOpt) == SUCCESSFUL_RETURN) {
				/* populate the command buffers */
				for (unsigned int i=0; i<N; i++) {
					x_acc[i] = (float) xOpt[2*i];
					y_acc[i] = (float) xOpt[2*i + 1];
					if ((fabs(fabs(x_acc[i]) - MAX_ANG_TOTAL) < 0.01) || (fabs(fabs(y_acc[i]) - MAX_ANG_TOTAL) < 0.01)) {
						bangedtheta_acc += fabs(x_acc[i]);
						bangedphi_acc   += fabs(y_acc[i]);
					}
				}
				bangedtheta = bangedtheta_acc / (N * MAX_ANG_TOTAL) * 100;
				bangedphi   = bangedphi_acc   / (N * MAX_ANG_TOTAL) * 100;
				// printf("fval = %e, bangedtheta: %f, bangedphi: %f\n", mpctry.getObjVal(), bangedtheta, bangedphi);
			}
			else {
				printf("QP couldn't be solved! \n");
			}
		}
		else {
			printf("QP couldn't be initialized! \n");
		}
		iterate = iterate * 1.3;
	// }

	// float calctimestop = controller->curr_time - calctimestart;
	// printf("calc time: %f\n", calctimestop);
	lock_optimal = 1;

	#ifdef FB
		states.resize(4, N);
		Eigen::Matrix<float,4,1> x0;
		x0 << vel0[0], pos0[0], vel0[1], pos0[1];
		states.col(0) = x0;
		Eigen::Matrix<float, 2, 1> inputs;
		float timeitisnow = controller->curr_time;
		fprintf(states_f, "%f,%f,%f\n", timeitisnow, states(1,0), states(3,0));
		
		for (int i=0; i<N-1; i++) {
			inputs(0) = x_acc[i];
			inputs(1) = y_acc[i];
			states.col(i+1) = A * states.col(i) + B * inputs; 
			fprintf(states_f, "%f,%f,%f\n", timeitisnow, states(1,i+1), states(3,i+1));
		}
	#endif
}