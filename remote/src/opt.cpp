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
			float tmp_yaw = controller->corr_state.yaw;

			float pitch_cmd_bodyframe_ff = cos(controller->corr_state.yaw) * x_acc[i] - sin(controller->corr_state.yaw) * y_acc[i];
			float roll_cmd_bodyframe_ff  = sin(controller->corr_state.yaw) * x_acc[i] + cos(controller->corr_state.yaw) * y_acc[i];

			controller->pitch_cmd = pitch_cmd_bodyframe_ff;
			controller->roll_cmd  = roll_cmd_bodyframe_ff;
			

			if (controller->pitch_cmd > -0.3) {
				controller->pitch_cmd = -0.3;
			} 
			
			printf("\033[1;35mwaypoint:%d, [%d/%d] \033[0m\n", flightplan->wp_selector, i, N);
			i = i + 1;
			// old_wp = flightplan->wp_selector;
		} if (i >= N) {
			controller->pitch_cmd = 0;
			controller->roll_cmd  = 0;
			printf("\033[1;31mFinised sequence at %f seconds\033[0m\n", ai->curr_time);
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
