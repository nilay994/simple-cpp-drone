#pragma once

#include <vector>
#include <thread>
#include <stdint.h>
#include "msp.hpp"

class MspInterface {
        MSP msp;
        std::vector<uint16_t> rcData{1500, 1500, 1500, 1500, 1500};
        Payload serialize_rc_data();
    public:
        MspInterface();
        void write_to_bf();
        void read_from_bf();
        void arm();
        void disarm();

};

class msp_node {
        MspInterface iface;
        std::thread msp_node_thread_;

    public:
        msp_node();
        ~msp_node();
        void msp_node_main();
};