#include "ctrl_optimizer.hpp"

// #define __USE_SINGLE_PRECISION__

#include "qpOASES/include/qpOASES.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <thread>
#include <chrono>

#include "user_ai.hpp"
#include "utils.h"

USING_NAMESPACE_QPOASES

#define MAX_BANK (D2R * 30.0)

// constructor can be initialized by Hessian type, so it can stop checking for positive definiteness
Options options;

FILE *states_f;

Optimizer::Optimizer() {
	states_f = fopen("states.csv", "w+");
	this->optimizer_thread_ = std::thread(&Optimizer::solveQP, this);
	//options.enableFlippingBounds = BT_FALSE;
	options.printLevel = PL_NONE;  // DEBUG level high, but the prints are muted in the library :(
	options.initialStatusBounds = ST_INACTIVE;
	options.numRefinementSteps = 1;
}

Optimizer::~Optimizer() {
	fflush(states_f);
	fclose(states_f);
	this->optimizer_thread_.detach();
}


Eigen::MatrixXd oldR(4, 2 * MAX_N);
Eigen::MatrixXd states(4, MAX_N);
Eigen::MatrixXd eye(2* MAX_N, 2 * MAX_N); 
Eigen::MatrixXd old_H(2 * MAX_N, 2 * MAX_N);
Eigen::MatrixXd R(4, 2 * MAX_N);
Eigen::MatrixXd f(1, 2 * MAX_N);

void Optimizer::solveQP(void) {

	while(1) {
		
		if (st_mc->arm_status == ARM && flightplan->flightplan_running == true) {
			
			static int last_wp = 99;
			
			if (last_wp == flightplan->wp_selector) {
				continue;
			}
			last_wp = flightplan->wp_selector;

			printf("curr wp: %d\n", last_wp);
			Eigen::Matrix<double, 4, 4> A;

			// TODO: drone drag -0.3
			// sim drag -0.75
			A << 0.9277, 0, 0, 0,
				0.09634, 1, 0, 0,
				0, 0, 0.9277, 0,
				0, 0, 0.09634, 1;

			// sim drag -0.75
			Eigen::Matrix<double, 4, 2> B;
			B << -0.9451, 0,
				-0.04785, 0,
				0, 0.9451,
				0, 0.04785;

			// position reprimanded 5 times more
			Eigen::Matrix<double, 4, 4> P;
			P << 1,  0,  0,  0,
				0, 20,  0,  0,
				0,  0,  1,  0,
				0,  0,  0, 20;

			// position final is one of the gates in the arena
			double pos0[2] = {controller->robot.pos.x, controller->robot.pos.y};
			double posf[2] = {flightplan->wp[flightplan->wp_selector].x, flightplan->wp[flightplan->wp_selector].y};

			// go through the gate with 1.5m/s forward vel
			// double vel0[2] = {controller->robot.vel.x, controller->robot.vel.y};
			// double velf[2] = {fwd_speed * cos(flightplan->wp[flightplan->wp_selector].psi), fwd_speed * sin(flightplan->wp[flightplan->wp_selector].psi)};

			double vel0[2] = {controller->robot.vel.x, controller->robot.vel.y};
			double velf[2] = {flightplan->wp[flightplan->wp_selector].v_sp * cos(flightplan->wp[flightplan->wp_selector].drone_psi),
			 				  flightplan->wp[flightplan->wp_selector].v_sp * sin(flightplan->wp[flightplan->wp_selector].drone_psi)};

			// above state space matrices are discretized at 100 milliseconds/10 Hz
			double dt = 0.1;

			double calctimestart = ai->curr_time;
				
			// assumption: can reach anywhere in the arena if I have ten seconds 
			double T = 3.0; // 3 seconds 
			N = round(T/dt);

			/** lowest horizon for performing optimization **/
			if (N < 2) {
				N = 2;
			}

			oldR.resize(4, 2*N);
			oldR.block(0, 2*N-2, 4, 2) =  B;
			Eigen::Matrix<double, 4, 4> AN = A;


			for(unsigned int i=1; i<N; i++) {
				oldR.block(0, 2*N-2*(i+1), 4, 2) =  A * oldR.block(0, 2*N-2*i, 4, 2);
				AN = A * AN; 
			}

			Eigen::MatrixXd R     = oldR.block(0, 0, 4, 2*N);
			Eigen::MatrixXd old_H = 2 * (R.transpose() * P * R);

			eye.resize(2*N, 2*N);
			eye.setIdentity();
			eye = 0.3 * eye; // TODO: verify after this change - augnment to keep hessian invertable says harvard
			// WARNING: augmenting on diagonals to improve invertability has an impact on the rate of change of input sequence
			Eigen::MatrixXd H = 0.5 * (old_H + old_H.transpose() + eye);

			Eigen::Matrix<double, 4, 1> x0; 
			Eigen::Matrix<double, 4, 1> xd;
			x0 << vel0[0], pos0[0], vel0[1], pos0[1];
			xd << velf[0], posf[0], velf[1], posf[1];
			
			Eigen::MatrixXd f = (2 * ((AN * x0)- xd)).transpose() * P * R;

			int opt_size = 2 * N;
			real_t ub[opt_size];
			real_t lb[opt_size];
			
			/** populate the control signal constraints **/
			for (int i = 0; i < opt_size; i++) {
				ub[i] = MAX_BANK;        // upper bound
				lb[i] = -MAX_BANK;       // lower bound
			}
			
			/** populate hessian and linear term **/
			double newH[opt_size * opt_size];
			double newf[opt_size];
			Eigen::Map<Eigen::MatrixXd>(newH, opt_size, opt_size) = H.transpose();
			Eigen::Map<Eigen::MatrixXd>(newf, 1, opt_size) = f;

			/** our class of problem is QProblemB from qpOASES,
			since, we don't have any constraints on position or velocity but just on the inputs **/
			
			/* Setting up QProblemB object. */
			QProblemB qp_problem(2*N);

			// optionsmpc.enableCholeskyRefactorisation = 1;

			qp_problem.setOptions(options);

			/* not sure: take 100 tries to solve the QP? */
			int nWSR = 100;

			/* Initialize QP. */
			real_t xOpt[opt_size];
			if (qp_problem.init((real_t *)newH, (real_t *)newf, lb, ub, nWSR, 0) == SUCCESSFUL_RETURN) {
				/* solve the QP */
				if (qp_problem.getPrimalSolution(xOpt) == SUCCESSFUL_RETURN) {
					/* qp problem successfully solved, 
					we now have the trajectory and the control signals */

					for (unsigned int i=0; i<N; i++) {
						/* populate the control signal buffers */
						x_acc[i] = (double) xOpt[2*i];
						y_acc[i] = (double) xOpt[2*i + 1];
					}
				} else {
					printf("QP couldn't be solved! \n");
				}
			} else {
				printf("QP couldn't be initialized! \n");
			}

			double calctimestop = ai->curr_time - calctimestart;
			printf("calc time: %f\n", calctimestop);

			this->lock_optimal = 1;

			/** after solving for control inputs, propagate them in the future **/
			states.resize(4, N);
			states.col(0) = x0;  // x0 << vel0[0], pos0[0], vel0[1], pos0[1];
			Eigen::Matrix<double, 2, 1> inputs;
			double timeitisnow = ai->curr_time;
			fprintf(states_f, "%f,%f,%f\n", timeitisnow, states(1,0), states(3,0));
			
			for (unsigned int i = 0; i < (N-1); i++) {
				inputs(0) = x_acc[i];
				inputs(1) = y_acc[i];
				// x(k+1) = A x(k) + B u(k);
				states.col(i+1) = A * states.col(i) + B * inputs; 
				fprintf(states_f, "%f,%f,%f\n", timeitisnow, states(1,i+1), states(3,i+1));
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 100 Hz
	}
}