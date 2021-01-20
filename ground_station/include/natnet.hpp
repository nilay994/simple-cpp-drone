#pragma once

#include <thread>
#include <state.h>

class NatNet {
    private:
        std::thread natnet_thread_;
        std::thread natnet_thread2_;
        #ifdef LOOPBACK
        std::thread natnet_thread3_;
        #endif
        std::thread natnet_thread4_;

    public:
        robot_t robot;
        bool gpslock = false;

        NatNet();
        ~NatNet();

        // rx from natnet
        void natnet_rx();

        // for calc attitude, pos, vel
        bool calculate_states();

        // for parsing from optitrack
        void natnet_parse(unsigned char *in);

        // send parsed data to rpi
        void natnet_tx();

        // for testing on local pc
        void natnet_loopback();

        // for checking the state of optitrack
        void connection_check();
};