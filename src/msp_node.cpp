// Adopted from AscendNTNU: 
// https://github.com/AscendNTNU/msp_flightcontroller_interface

#include "msp.hpp"

#include <iostream>
#include <iomanip>

#include <algorithm>
#include <cmath>
// threading
#include <chrono>
#include <thread>

#include "msp_node.hpp"
#include "user_ai.hpp"

Payload MspInterface::serialize_rc_data() {
    Payload result;
    for (int i = 0; i < this->rcData.size(); i++) {
        result.put_u16(this->rcData[i]);
    }
    return result;
}


MspInterface::MspInterface() {
    // msp("/dev/ttyUSB0"); 
    // Set thurst to 0
    this->rcData[2] = 1000;
    // DISARM
    this->rcData[4] = 1000;

    // https://stackoverflow.com/questions/42877001/how-do-i-read-gyro-information-from-cleanflight-using-msp
    msp.register_callback(MSP::ATTITUDE, [this](Payload payload) {
        std::vector<int16_t> attitudeData(payload.size() / 2);
        for (int i = 0; i < attitudeData.size(); i++) {
            // mapping an unsigned to a signed?!
            attitudeData[i] = payload.get_u16();
        }

        std::vector<float> att_f (3, 0);
        // weird 1/10 degree convention betaflight?
        att_f[0] = ((float) attitudeData[0]) / 10.0;
        att_f[1] = ((float) attitudeData[1]) / 10.0;
        att_f[2] = ((float) attitudeData[2]);
        std::cout << std::setprecision(3) << att_f[0] << ",\t" << att_f[1] << ",\t" << att_f[2] << "\n"; 
        });
} 

    
void MspInterface::write_to_bf() {

    // Send rc data
    msp.send_msg(MSP::SET_RAW_RC, serialize_rc_data());

    // Recieve new msp messages
    msp.recv_msgs();
}

void MspInterface::read_from_bf() {
    // Request telemetry
    msp.send_msg(MSP::ATTITUDE, {});

    // TODO: get also the arming signals for safety
    // msp.send_msg(MSP::RC, {});

    // Recieve new msp messages
    msp.recv_msgs();
}

void MspInterface::arm() {

    // arm sequence: disarm first, 
    for (int i = 0; i < 50; i++) {
        static uint16_t val = 800;
        val = val + 10;
        if (val < 1200) {
            rcData[0] = 1500;
            rcData[1] = 1500;
            rcData[2] = 890;
            rcData[3] = 1500;
            rcData[4] = 1000;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            st_mc->arm_status = DISARM;
        } 
        // disarmed, arm now
        if (val > 1200 && val < 1500) {
            rcData[0] = 1500;
            rcData[1] = 1500;
            rcData[2] = 890;
            rcData[3] = 1500;
            rcData[4] = 2000;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            st_mc->arm_status = ARM;
        }
    
        // Send rc data
        msp.send_msg(MSP::SET_RAW_RC, serialize_rc_data());

        // Recieve new msp messages
        msp.recv_msgs();
    }
}

void MspInterface::disarm() {
    // arm sequence: disarm first, 
    for (int i = 0; i < 90; i++) {
        rcData[0] = 1500;
        rcData[1] = 1500;
        rcData[2] = 890;
        rcData[3] = 1500;
        rcData[4] = 1000;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        st_mc->arm_status = DISARM;

        // Send rc data
        msp.send_msg(MSP::SET_RAW_RC, serialize_rc_data());

        // Recieve new msp messages
        msp.recv_msgs();
    }
}

msp_node::msp_node() {
    this->msp_node_thread_ = std::thread(&msp_node::msp_node_main, this);
    printf("[msp] thread spawned!\n");
    iface.arm();
}

void msp_node::msp_node_main() {
    while(1) {
        if (st_mc->arm_status == ARM) {
            // TODO: find a way to thread these, 
            // but for now they are resource constrainted 
            // over single uart bus over MSP, so this is okay
            iface.read_from_bf();
            iface.write_to_bf();
        }
        // 50 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

msp_node::~msp_node() {
    msp_node_thread_.detach();
    printf("[msp] thread killed!\n");
    printf("[msp] sending disarm signal!\n");
    iface.disarm();
}

