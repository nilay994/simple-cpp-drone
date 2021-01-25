#pragma once

#include "state.h"
#include <thread>

class state_mc {
    private:
        std::thread st_mc_thread_;
        
    public:
        arm_status_t arm_status = DISARM;
        bool sigint_status = false;
        
        state_mc();
        ~state_mc();
        void set_current_state();
};