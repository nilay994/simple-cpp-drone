#pragma once

#include <thread>
#include <state.h>

class NatNet {
    private:
        std::thread natnet_thread_;
        std::thread natnet_thread2_;
        std::thread natnet_thread3_;
        
    public:
        robot_t robot;
        NatNet();
        ~NatNet();
        // rx from natnet and parse
        void natnet_rx();

        void calculate_states();
        void natnet_parse(unsigned char *in);

        // send parsed data to rpi
        void natnet_tx();
        void natnet_loopback();
};