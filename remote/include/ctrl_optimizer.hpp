#ifndef INCLUDE_CTRL_OPT_H_
#define INCLUDE_CTRL_OPT_H_

#include <thread>

/** control and prediction horizon are the same, 
 * MAX_N of 160 when dt = 0.01 equals 1.6 seconds  **/ 
#define MAX_N 160 

void* flush_job(void* data);
void* calc_job(void* data);

class Optimizer {
	private:
		std::thread optimizer_thread_;
	
	public:
		float x_acc[MAX_N];
		float y_acc[MAX_N];
		unsigned int i = 0;
		
		// initialize the matrices for holding entire horizon
		unsigned int N = MAX_N;

		/** lock_optimal makes the robot follow the optimized sequence; it is not
		 *  allowed to re-optimize until it finishes executing the current sequence **/
		volatile bool lock_optimal = 0;

		Optimizer();
		~Optimizer();

		void solveQP(void);
};


#endif /* INCLUDE_CTRL_OPT_H_ */
