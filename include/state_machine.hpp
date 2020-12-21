#pragma once

#include <thread>
#include "state.h"

class state_mc {
    private:
        std::thread st_mc_thread_;
        
    public:
        bool arm_status;
        state_mc();
        ~state_mc();
        void set_current_state();
};