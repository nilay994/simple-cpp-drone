#pragma once

#include <thread>

class NatNet {
    private:
        std::thread natnet_thread_;
        std::thread natnet_thread2_;
        
    public:    
        NatNet();
        ~NatNet();
        void sample_data();
        void velocity_thread();
};