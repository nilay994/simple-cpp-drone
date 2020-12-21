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

class MspInterface {
    MSP msp;
    std::vector<uint16_t> rcData;
    Payload serialize_rc_data() {
        Payload result;
        for (int i = 0; i < rcData.size(); i++) {
            result.put_u16(rcData[i]);
        }
        return result;
    }

public:
    MspInterface() : msp("/dev/ttyUSB0"), rcData(5, 1500)
    {
        // Set thurst to 0
        rcData[2] = 1000;
        // DISARM
        rcData[4] = 1000;

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
    
    void set_rates(char directn) {

        switch (directn) {
            case 'w': {
                rcData[1] = 1600;
                // thrust
                rcData[2] = 1000;
                break;
            }
            case 'a': {
                rcData[0] = 1400;
                // thrust
                rcData[2] = 1000;
                break;
            }
            case 'd': {
                rcData[0] = 1600;
                // thrust
                rcData[2] = 1000;
                break;
            }
            case 's': {
                rcData[1] = 1400;
                // thrust
                rcData[2] = 1000;
                break;
            }
            case 'q': {
                rcData[3] = 1400;
                // thrust
                rcData[2] = 1000;
                break;
            }
            case 'e': {
                rcData[3] = 1600;
                // thrust
                rcData[2] = 1000;
                break;
            }
            case 'r': {
                rcData[0] = 1500;
                rcData[1] = 1500;
                rcData[3] = 1500;
                // thrust
                rcData[2] = 1000;
                // disarm
                rcData[4] = 1000;
                break;
            }
            default: break;
        }
        
        // double thrust = rates.thrust.z / 9.81 / mass * hover_thrust;
        // double thrust = 0.8;
        // rcData[2] = (uint16_t) std::min(1000, std::max(0, (int) round(thrust * 1000))) + 1000;
    }
    void step_hf(char directn) {
        set_rates(directn);

        // Send rc data
        msp.send_msg(MSP::SET_RAW_RC, serialize_rc_data());

        // Recieve new msp messages
        msp.recv_msgs();
    }
    void step_lf() {
        // Request telemetry
        msp.send_msg(MSP::ATTITUDE, {});

        // TODO: get also the arming signals for safety
        // msp.send_msg(MSP::RC, {});

        // Recieve new msp messages
        msp.recv_msgs();
    }
    void arm(bool arm_channel) {

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
            std::cout << "DISARMED" << std::endl;
         } 
         // disarmed, arm now
         if (val > 1200 && val < 1500) {
            rcData[0] = 1500;
            rcData[1] = 1500;
            rcData[2] = 890;
            rcData[3] = 1500;
            rcData[4] = 2000;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::cout << "ARMED" << std::endl;
        }
        
        // Send rc data
        msp.send_msg(MSP::SET_RAW_RC, serialize_rc_data());

        // Recieve new msp messages
        msp.recv_msgs();
        }
    }
    void disarm(bool arm_channel) {
        // arm sequence: disarm first, 
        for (int i = 0; i < 90; i++) {
            rcData[0] = 1500;
            rcData[1] = 1500;
            rcData[2] = 890;
            rcData[3] = 1500;
            rcData[4] = 1000;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            std::cout << "DISARMED" << std::endl;
        
            // Send rc data
            msp.send_msg(MSP::SET_RAW_RC, serialize_rc_data());

            // Recieve new msp messages
            msp.recv_msgs();
    }
    }
};



int main(int argc, char** argv) {
    
    MspInterface iface;

    iface.arm(true);

    int i = 0; char directn;

    // while key pressed!=kill
    while (directn!='k') {
        i++;
        if (i >= 5) {
            iface.step_lf();
            i = 0;
        }
        /* step_lf and step_hf are mutexed on uart bus, 
        so this method of threading is not so bad */
        iface.step_hf(directn);
        // 50 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // DISARM
    iface.disarm(false);

    return 0;
}
