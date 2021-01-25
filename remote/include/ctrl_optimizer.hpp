#ifndef INCLUDE_CTRL_OPT_H_
#define INCLUDE_CTRL_OPT_H_

#include <thread>
#include <Eigen/Core>

/** control and prediction horizon are the same, 
 * MAX_N of 160 when dt = 0.01 equals 1.6 seconds  **/ 
#define MAX_N 160 

void* flush_job(void* data);
void* calc_job(void* data);

class Optimizer {
	private:
		std::thread optimizer_thread_;
		std::thread optimizer_thread2_;
			
	public:

		// iterator to run through the horizon
		unsigned int i = 0;

		// control and prediction horizon
		unsigned int N_hor;

		/** lock_optimal makes the robot follow the optimized sequence; it is not
		 *  allowed to re-optimize until it finishes executing the current sequence **/
		volatile bool lock_optimal = 0;

		Optimizer();
		~Optimizer();

		void solveQP(void);
		void execute_feedforward();
};


#endif /* INCLUDE_CTRL_OPT_H_ */
