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

/** NatNet defaults */
char const *natnet_addr           = "255.255.255.255";
char const *natnet_multicast_addr = "239.255.42.99";
uint16_t natnet_cmd_port          = 1510;
uint16_t natnet_data_port         = 1511;
uint8_t natnet_major              = 2;
uint8_t natnet_minor              = 9;

/* send to rpi */
// char *const tx_data_addr = "192.168.42.1";
char const *tx_data_addr = "127.0.0.1";
uint16_t tx_data_port = 5000;

/* loopback */
char const *rx_loopback_addr = "127.0.0.1";
uint16_t rx_loopback_port = 5000;

FILE *optitrack_f;

/** Connection timeout when not receiving **/
#define CONNECTION_TIMEOUT          .5

/** NatNet parsing defines */
#define MAX_PACKETSIZE    100000
#define MAX_NAMELENGTH    256
#define MAX_RIGIDBODIES   128
#define NAT_FRAMEOFDATA   7

/** Tracked rigid bodies */
struct RigidBody {
	int id;                           ///< Rigid body ID from the tracking system
	float x, y, z;                    ///< Rigid body x, y and z coordinates in meters (note y and z are swapped)
	float qx, qy, qz, qw;             ///< Rigid body qx, qy, qz and qw rotations (note qy and qz are swapped)
	int nMarkers;                     ///< Number of markers inside the rigid body (both visible and not visible)
	float error;                      ///< Error of the position in cm
	int nSamples;                     ///< Number of samples since last transmit
	bool posSampled;                  ///< If the position is sampled last sampling
};
struct RigidBody rigidBodies[MAX_RIGIDBODIES];    ///< All rigid bodies which are tracked

/** Natnet socket connections */
struct UdpSocket natnet_data, tx_data, rx_loopback_data;

/** Tracking location LTP and angle offset from north */
// struct LtpDef_d tracking_ltp;       ///< The tracking system LTP definition
float tracking_offset_angle;       ///< The offset from the tracking system to the North in degrees

/** Save the latency from natnet */
float natnet_latency;

/** Parse the packet from NatNet */
void NatNet::natnet_parse(unsigned char *in) {
	int i, j;

	// Create a pointer to go trough the packet
	char *ptr = (char *)in;
	printf_natnet("Begin Packet\n-------\n");

	// Message ID
	int MessageID = 0;
	memcpy(&MessageID, ptr, 2); ptr += 2;
	printf_natnet("Message ID : %d\n", MessageID);

	// Packet size
	int nBytes = 0;
	memcpy(&nBytes, ptr, 2); ptr += 2;
	printf_natnet("Byte count : %d\n", nBytes);

	if (MessageID == NAT_FRAMEOFDATA) {   // FRAME OF MOCAP DATA packet
		// Frame number
		int frameNumber = 0; memcpy(&frameNumber, ptr, 4); ptr += 4;
		printf_natnet("Frame # : %d\n", frameNumber);

		// ========== MARKERSETS ==========
		// Number of data sets (markersets, rigidbodies, etc)
		int nMarkerSets = 0; memcpy(&nMarkerSets, ptr, 4); ptr += 4;
		// printf_natnet("Marker Set Count : %d\n", nMarkerSets);

		for (i = 0; i < nMarkerSets; i++) {
			// Markerset name
			char szName[256];
			strcpy(szName, ptr);
			int nDataBytes = (int) strlen(szName) + 1;
			ptr += nDataBytes;
			// printf_natnet("Model Name: %s\n", szName);

			// marker data
			int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
			// printf_natnet("Marker Count : %d\n", nMarkers);

			for (j = 0; j < nMarkers; j++) {
				float x = 0; memcpy(&x, ptr, 4); ptr += 4;
				float y = 0; memcpy(&y, ptr, 4); ptr += 4;
				float z = 0; memcpy(&z, ptr, 4); ptr += 4;
				// printf_natnet("\tMarker %d : [x=%3.2f,y=%3.2f,z=%3.2f]\n", j, x, y, z);
			}
		}

		// Unidentified markers
		int nOtherMarkers = 0; memcpy(&nOtherMarkers, ptr, 4); ptr += 4;
		// printf_natnet("Unidentified Marker Count : %d\n", nOtherMarkers);
		for (j = 0; j < nOtherMarkers; j++) {
			float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
			float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
			float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
			// printf_natnet("\tMarker %d : pos = [%3.2f,%3.2f,%3.2f]\n", j, x, y, z);
		}

		// ========== RIGID BODIES ==========
		// Rigid bodies
		int nRigidBodies = 0;
		memcpy(&nRigidBodies, ptr, 4); ptr += 4;
		// printf_natnet("Rigid Body Count : %d\n", nRigidBodies);

		// Check if there ie enough space for the rigid bodies
		if (nRigidBodies > MAX_RIGIDBODIES) {
			fprintf(stderr, 
			"Could not sample all the rigid bodies because the amount of rigid bodies is bigger then %d (MAX_RIGIDBODIES).\r\n",
			MAX_RIGIDBODIES);
			exit(EXIT_FAILURE);
		}

		for (j = 0; j < nRigidBodies; j++) {
			memcpy(&rigidBodies[j].id, ptr, 4); ptr += 4;
			memcpy(&rigidBodies[j].y, ptr, 4); ptr += 4;   //x --> Y
			memcpy(&rigidBodies[j].z, ptr, 4); ptr += 4;   //y --> Z
			memcpy(&rigidBodies[j].x, ptr, 4); ptr += 4;   //z --> X
			memcpy(&rigidBodies[j].qx, ptr, 4); ptr += 4;  //qx --> QX
			memcpy(&rigidBodies[j].qz, ptr, 4); ptr += 4;  //qy --> QZ
			memcpy(&rigidBodies[j].qy, ptr, 4); ptr += 4;  //qz --> QY
			memcpy(&rigidBodies[j].qw, ptr, 4); ptr += 4;  //qw --> QW
			printf_natnet("ID (%d) : %d\n", j, rigidBodies[j].id);
			printf_natnet("pos: [%3.2f,%3.2f,%3.2f]\n", rigidBodies[j].x, rigidBodies[j].y, rigidBodies[j].z);
			printf_natnet("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", rigidBodies[j].qx, rigidBodies[j].qy, rigidBodies[j].qz, rigidBodies[j].qw);

			// TODO: Mutex around this
			this->robot.pos.x = rigidBodies[j].x;
			this->robot.pos.y = rigidBodies[j].y;
			this->robot.pos.z = -rigidBodies[j].z;
		}
	}
}

void NatNet::calculate_states() {

	/** velocity function **/
	static float prev_x = this->robot.pos.x;
	static float prev_y = this->robot.pos.y;
	static float prev_z = this->robot.pos.z;

	static float prev_vx = 0.0;
	static float prev_vy = 0.0;
	static float prev_vz = 0.0;

	float curr_x = this->robot.pos.x;
	float curr_y = this->robot.pos.y;
	float curr_z = this->robot.pos.z;

	static float prev_t = 0.0;
	float curr_t = ai->curr_time;
	float curr_vx = (curr_x - prev_x) / (curr_t - prev_t);
	float curr_vy = (curr_y - prev_y) / (curr_t - prev_t);
	float curr_vz = (curr_z - prev_z) / (curr_t - prev_t);

    // Exponential Moving Average: y += alpha * (x - y); y = out, x = in
	#define ALPHA 0.1
	this->robot.vel.x = ALPHA * curr_vx + (1 - ALPHA) * prev_vx;
	this->robot.vel.y = ALPHA * curr_vy + (1 - ALPHA) * prev_vy;
	this->robot.vel.z = ALPHA * curr_vz + (1 - ALPHA) * prev_vz;

	prev_vx = this->robot.vel.x;
	prev_vy = this->robot.vel.y;
	prev_vz = this->robot.vel.z;

	prev_x = curr_x;
	prev_y = curr_y;
	prev_z = curr_z;
	prev_t = curr_t;
}

/** The NatNet sampler periodic function */
void NatNet::natnet_rx() {

    // sched_param sch;
    // int policy; 
    // pthread_getschedparam(pthread_self(), &policy, &sch);
    // printf("[NatNet] thread is executing at priority: %d\n", sch.sched_priority);

	// flush UDP buffers
	unsigned char garb[MAX_PACKETSIZE];
	udp_socket_recv(&natnet_data, garb, MAX_PACKETSIZE);

	while(1) {
		if (1) {
			static unsigned char buffer_data[MAX_PACKETSIZE];
			static int bytes_data = 0;

			// Keep on reading until we have the whole packet
			bytes_data += udp_socket_recv(&natnet_data, buffer_data, MAX_PACKETSIZE);

			// Parse NatNet data
			if (bytes_data >= 2) {
				uint16_t packet_size = ((uint16_t)buffer_data[3])<<8 | (uint16_t)buffer_data[2];
				if( bytes_data - 4 >= packet_size) {  // 4 bytes for message id and packet size
					natnet_parse(buffer_data);
				}
				bytes_data = 0;
			}
		} else {
			// flush UDP buffer
			udp_socket_recv(&natnet_data, garb, MAX_PACKETSIZE);
		}
		// might be related to how fast the wifi chip is, else you are throwing away packets unecessarily..
		// std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
}

void NatNet::natnet_tx() {
	while(1) {
		if (1) {
			// populate pos and vel
			this->calculate_states();

			/* send to rpi */
			// find length of packet to send
			int len = sizeof(robot_t) + 2;
			
			// allocate
			unsigned char robot_str[len];
			
			// add header
			robot_str[0] = '$';
			
			// serialize packet from struct to uint8_t array
			memcpy(&robot_str[1], &this->robot, sizeof(robot_t));
			
			// add footer
			robot_str[len-1] = '*'; 	  

			// send over udp
			if (len != udp_socket_send(&tx_data, robot_str, len)) {
				printf_debug("[tx udp] losing packets! \n");
			}

			// fprintf(optitrack_f, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", 
			// 	ai->curr_time, robot.pos.x, robot.pos.y, robot.pos.z, robot.vel.x, robot.vel.y, robot.vel.z, robot.att.roll, robot.att.pitch, robot.att.yaw);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(5)); // 200 Hz
	}
}

void NatNet::natnet_loopback() {
	printf("started loopback!\n");
	while(1) {

		static unsigned char buffer_data[50];
		static int bytes_data = 0;
		bytes_data += udp_socket_recv(&rx_loopback_data, buffer_data, 50);
		// Parse NatNet data
		if (bytes_data > sizeof(robot_t)) {
			if (buffer_data[0] == '$') {
				robot_t robot;
				memcpy(&robot, &buffer_data[1], sizeof(robot_t));
				printf("%.02f, %.02f, %.02f, %.02f, %.02f, %.02f, %.02f, %.02f, %.02f\n", 
				robot.pos.x, robot.pos.y, robot.pos.z, robot.vel.x, robot.vel.y, robot.vel.z, robot.att.roll, robot.att.pitch, robot.att.yaw);
			}
			bytes_data = 0;
		}
		
		std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1000 Hz listenin
	}
}

NatNet::NatNet() {
	// Create the network connections
	printf_debug("Starting NatNet listening (multicast address: %s, data port: %d, version: %d.%d)\n",
				natnet_multicast_addr, natnet_data_port, natnet_major, natnet_minor);
	
	// 1511 natnet data port
	udp_socket_create(&natnet_data, (char*) "", -1, natnet_data_port, 0); // Only receiving
	udp_socket_subscribe_multicast(&natnet_data, natnet_multicast_addr);  // 239.255.42.99
	udp_socket_set_recvbuf(&natnet_data, 0x100000); // 1MB

	// tx to drone test
	udp_socket_create(&tx_data, (char *) tx_data_addr, tx_data_port, 0, 0);
	udp_socket_set_sendbuf(&tx_data, 0x100000); // 1MB tx buffer

	// rx loopback test
	udp_socket_create(&rx_loopback_data, (char *) rx_loopback_addr, -1, rx_loopback_port, 0);
	udp_socket_set_recvbuf(&rx_loopback_data, 0x100000); // 1MB rx loopback buffer

	printf("UDP sockets created! :D\n");

	// start the natnet thread
	try {
		natnet_thread_  = std::thread(&NatNet::natnet_rx, this);
		natnet_thread2_ = std::thread(&NatNet::natnet_tx, this);
		natnet_thread3_ = std::thread(&NatNet::natnet_loopback, this);
		
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
	natnet_thread2_.detach();
	natnet_thread3_.detach();
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