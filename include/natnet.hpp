#pragma once

#include <thread>

class NatNet {
    private:
        std::thread natnet_thread_;
        
    public:    
        NatNet();
        ~NatNet();
        void sample_data();
};