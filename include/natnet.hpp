#pragma once

#include <thread>

class NatNet {
    private:
        

    public:
        std::thread natnet_thread_;
        NatNet();
        ~NatNet();
        void sample_data();
};