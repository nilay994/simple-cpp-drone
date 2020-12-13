#include "msp.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>
#include <iostream>

class MspInterface {
    MSP msp;
    std::vector<uint16_t> rcData;
    double max_roll_r  = 200;
    double max_pitch_r = 200;
    double max_yaw_r   = 200;
    double hover_thrust = 0.3;
    double mass = 1.0;
    bool new_rates = false;
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
        // Set thurst to 0.5
        rcData[2] = 2000;

        // arm
        rcData[4] = 2000;

        msp.register_callback(MSP::RC, [this](Payload payload) {
            std::vector<uint16_t> droneRcData(payload.size() / 2);
            for (int i = 0; i < droneRcData.size(); i++) {
                droneRcData[i] = payload.get_u16();
            }
        });
    }
    
    void set_rates() {
        double roll_r    = 1500;
        double pitch_r   = 1400;
        double yaw_r     = 1300;
        rcData[0] = 1400;//(uint16_t) std::min(500,  std::max(-500, (int) round(roll_r  * 500))) + 1500;
        rcData[1] = 1300;//(uint16_t) std::min(500,  std::max(-500, (int) round(pitch_r * 500))) + 1500;
        rcData[3] = 1200;//(uint16_t) std::min(500,  std::max(-500, (int) round(yaw_r   * (-500)))) + 1500;
        
        // double thrust = rates.thrust.z / 9.81 / mass * hover_thrust;
        double thrust = 0.8;
        rcData[2] = (uint16_t) std::min(1000, std::max(0, (int) round(thrust * 1000))) + 1000;

    }
public:
    void step_hf() {
            // msp_fc_interface::RcData rc_msg;
            // for (int i = 0; i < std::min(6, (int) rcData.size()); i++) {
            //     rc_msg.channels[i] = rcData[i];
            // }

            // Send rc data
            msp.send_msg(MSP::SET_RAW_RC, serialize_rc_data());

            // Recieve new msp messages
            msp.recv_msgs();

        new_rates = false;
    }
public:
    void step_lf() {
        // Request rc data
        msp.send_msg(MSP::RC, {});

        // Recieve new msp messages
        msp.recv_msgs();
    }
};

int main(int argc, char** argv) {
    MspInterface iface;

    int i = 0;
    while (1) {
        i++;
        if (i >= 10) {
            iface.step_lf();
            i = 0;
        }
        iface.set_rates();
        iface.step_hf();
        // 50 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        std::cout << "w\n";
    }

    return 0;
}
