/** \file natnet2ivy.c
*  \brief NatNet (GPS) to RPI forwarder
*
* This receives aircraft position information through the Optitrack system
* NatNet UDP stream and forwards it to the Raspberry Pi. An aircraft with the gps
* subsystem "datalink" is then able to parse the GPS position and use it to
* navigate inside the Optitrack system.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "udp_socket.h"

#include "natnet.hpp"
#include "user_ai.hpp"

/** Debugging options */
uint8_t verbose = 0;
#define printf_natnet(...)   if(verbose > 1) fprintf (stderr, __VA_ARGS__)
#define printf_debug(...)    if(verbose > 0) fprintf (stderr, __VA_ARGS__)

/* rx udp */
char const *rx_addr = "127.0.0.1";
uint16_t rx_port = 5000;

FILE *optitrack_f;

/** Natnet socket connections */
struct UdpSocket rx_data;

void NatNet::natnet_rx() {
	// start loop before arming
	while(1) {
		if (st_mc->arm_status == ARM) {
			// TODO: circ buffer?
			static unsigned char buffer_data[50];
			static int bytes_data = 0;
			bytes_data += udp_socket_recv(&rx_data, buffer_data, 50);
			// Parse NatNet data
			if (bytes_data > sizeof(robot_t)) {
				if (buffer_data[0] == '$') {
					memcpy(&controller->robot, &buffer_data[1], sizeof(robot_t));
					// printf("%f, %.02f, %.02f, %.02f, %.02f, %.02f, %.02f, %.02f, %.02f, %.02f\n", 
					// ai->curr_time, robot.pos.x, robot.pos.y, robot.pos.z, robot.vel.x, robot.vel.y, robot.vel.z, robot.att.roll, robot.att.pitch, robot.att.yaw);
					
					// fprintf(optitrack_f, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", 
					// ai->curr_time, robot.pos.x, robot.pos.y, robot.pos.z, robot.vel.x, robot.vel.y, robot.vel.z, robot.att.roll, robot.att.pitch, robot.att.yaw);
				}
				bytes_data = 0;
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(2)); // 500 Hz listenin
	}
}

NatNet::NatNet() {

	// rx udp
	udp_socket_create(&rx_data, (char *) rx_addr, -1, rx_port, 0);
	udp_socket_set_recvbuf(&rx_data, 0x100000); // 1MB rx loopback buffer

	// start the natnet thread
	try {
		natnet_thread_ = std::thread(&NatNet::natnet_rx, this);
		optitrack_f = fopen("optitrack.csv", "w+");
		printf("[gps] thread spawned!\n");
	} catch (...) {
		printf("[gps] thread couldn't be spawned!\n");
	}

}

NatNet::~NatNet() {
	// if (natnet_thread_.joinable()) {
	fflush(optitrack_f);
	fclose(optitrack_f);
	natnet_thread_.detach();
	printf("[gps] thread killed!\n");
}
		
// sched_param sch;
// int policy; 
// pthread_getschedparam(natnet_thread_.native_handle(), &policy, &sch);
// sch.sched_priority = 20;
// if (pthread_setschedparam(natnet_thread_.native_handle(), SCHED_FIFO, &sch)) {
// printf("Failed to setschedparam: Natnet thread\n" );
// }

// Copy the quaternions and convert to euler angles for the heading
// orient.qi = rigidBodies[i].qw;
// orient.qx = rigidBodies[i].qx;
// orient.qy = rigidBodies[i].qy;
// orient.qz = rigidBodies[i].qz;
// float_eulers_of_quat(&orient_eulers, &orient);

// Calculate the heading by adding the Natnet offset angle and normalizing it
// float heading = -orient_eulers.psi + 90.0 / 57.6 -
//                  tracking_offset_angle; //the optitrack axes are 90 degrees rotated wrt ENU
// NormRadAngle(heading);
//   controller->robot.vel.x = cos(tracking_offset_angle) * rigidBodies[i].vel_x - sin(tracking_offset_angle) * rigidBodies[i].vel_y;
//   controller->robot.vel.y = sin(tracking_offset_angle) * rigidBodies[i].vel_x + cos(tracking_offset_angle) * rigidBodies[i].vel_y;
//   controller->robot.vel.z = - rigidBodies[i].vel_z;