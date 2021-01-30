/**
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
#include "utils.h"

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
// TODO: Should instead broadcast on WiFi adapter's local network? 
// or will it overwhelm the networking part of the linux kernel? 
char const *tx_data_addr = "192.168.1.207";
// char const *tx_data_addr = "127.0.0.1";
uint16_t tx_data_port = 5000;

/* loopback */
#ifdef LOOPBACK
char const *rx_loopback_addr = "127.0.0.1";
uint16_t rx_loopback_port = 5000;
#endif

FILE *optitrack_f;

/** NatNet parsing defines */
#define MAX_PACKETSIZE    100000
#define MAX_NAMELENGTH    256
#define MAX_RIGIDBODIES   128
#define NAT_FRAMEOFDATA   7

/** GPS LOCK PARAMS **/
#define MAX_PACKETLOSS_ALLOWED 100
#define MIN_PACKETS_FOR_LOCK 20

/** Tracked rigid bodies */
struct RigidBody {
	int id;                           ///< Rigid body ID from the tracking system
	float x, y, z;                    ///< Rigid body x, y and z coordinates in meters (note y and z are swapped)
	float qx, qy, qz, qw;             ///< Rigid body qx, qy, qz and qw rotations (note qy and qz are swapped)
	int nMarkers;                     ///< Number of markers inside the rigid body (both visible and not visible)
	float error;                      ///< Error of the position in cm
};
struct RigidBody rigidBodies[MAX_RIGIDBODIES];    ///< All rigid bodies which are tracked

// local rigid body deep copied between natnet read and writes
robot_t probot;
struct FloatQuat orient;

// lock check
int frameNumber = 0;

/** Natnet socket connections */
struct UdpSocket natnet_data, tx_data, rx_loopback_data;

/* Parse the packet from NatNet */
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
		memcpy(&frameNumber, ptr, 4); ptr += 4;
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

			/** TODO: only ALLOW RIGID BODY WITH USER ID 1 **/
			if (rigidBodies[j].id == 1) {
				probot.pos.x = rigidBodies[j].x;
				probot.pos.y = rigidBodies[j].y;
				probot.pos.z = - rigidBodies[j].z;

				orient.qi = rigidBodies[j].qw;
				orient.qx = rigidBodies[j].qx;
				orient.qy = rigidBodies[j].qy;
				orient.qz = rigidBodies[j].qz;
			}
		}
	}
}

bool NatNet::calculate_states() {

	/** update position **/
	// TODO: Mutex around this
	this->robot.pos.x = probot.pos.x;
	this->robot.pos.y = probot.pos.y;
	this->robot.pos.z = probot.pos.z;

	/** update attitude **/
	// Copy the quaternions and convert to euler angles for the heading
	// TODO: verify if there is no need to wrap yaw.
	float_eulers_of_quat(&this->robot.att, &orient);

	/** update velocity **/
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
	float dt = curr_t - prev_t;

	// avoid NaNs; currently threading at 5ms
	if (dt < 0.001) {
		dt = 0.001;
	}

	float curr_vx = (curr_x - prev_x) / dt;
	float curr_vy = (curr_y - prev_y) / dt;
	float curr_vz = (curr_z - prev_z) / dt;

    // Exponential Moving Average: y += alpha * (x - y); y = out, x = in
	// LPF 0.1 of input at 200 Hz = 20 Hz of cutoff
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

	// TODO: copy probot to robot only when sure.
	// check if inf or NaN,
	bool chk = !(isfinite(robot.pos.x));
	chk |= !(isfinite(robot.pos.y));
	chk |= !(isfinite(robot.pos.z));
	chk |= isnan(robot.pos.x);
	chk |= isnan(robot.pos.y);
	chk |= isnan(robot.pos.z);

	chk |= !(isfinite(robot.vel.x));
	chk |= !(isfinite(robot.vel.y));
	chk |= !(isfinite(robot.vel.z));
	chk |= isnan(robot.vel.x);
	chk |= isnan(robot.vel.y);
	chk |= isnan(robot.vel.z);

	chk |= !(isfinite(robot.att.roll));
	chk |= !(isfinite(robot.att.pitch));
	chk |= !(isfinite(robot.att.yaw));
	chk |= isnan(robot.att.roll);
	chk |= isnan(robot.att.pitch);
	chk |= isnan(robot.att.yaw);

	return chk;

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
	}
}

void NatNet::natnet_tx() {
	while(1) {
		/** send packets to remote only if you have a GPS lock **/
		if (this->gpslock == true) {

			// populate pos, vel and att
			if (this->calculate_states() > 0) {
				// NaN or Inf
				printf(COLOR_FBLACK);
				printf(COLOR_BRED);
				printf("[natnet-err] NaN or Inf in sent states\n");
				printf(COLOR_NONE);
				printf("\n");
				this->gpslock = false;
				continue;
			}

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

			// LOG TO LOCAL PC FILE
			// fprintf(optitrack_f, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", 
			// 	ai->curr_time, robot.pos.x, robot.pos.y, robot.pos.z, robot.vel.x, robot.vel.y, robot.vel.z, robot.att.roll, robot.att.pitch, robot.att.yaw);

			// printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", 
			// 	ai->curr_time, robot.pos.x, robot.pos.y, robot.pos.z, robot.vel.x, robot.vel.y, robot.vel.z, R2D * robot.att.roll, R2D * robot.att.pitch, R2D * robot.att.yaw);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(5)); // 200 Hz
	}
}

// TODO: find a way to check if robot is taken out of the tracking area
/** function checks the state of the packets coming from optitrack and then decides to send them to remote **/
void NatNet::connection_check() {
	while (1) {
		/* rigid body lock - fuzzy logic */
		static int prevframeNumber = frameNumber;
		static int lock_p_ctr = 0;
		static int lock_n_ctr = 0;
		// printf("p: %d, n: %d\n", lock_p_ctr, lock_n_ctr);

		if (frameNumber != prevframeNumber) {
			// i.e. we keep getting new packets
			lock_p_ctr ++;
			lock_n_ctr = 0;
			
		} else {
			// old and new frames are the same, udp buffer flush reqd
			lock_n_ctr ++;
			lock_p_ctr = 0;
		}

		/** hysterisis-> take time to make a decision **/
		if (lock_p_ctr > MIN_PACKETS_FOR_LOCK) {
			this->gpslock = true;
		}
		
		if (lock_n_ctr > MAX_PACKETLOSS_ALLOWED) {
			this->gpslock = false;
		}

		/** only print on screen when the state changes **/
		static bool prev_gps_lock = this->gpslock;
		if (prev_gps_lock != this->gpslock) {
			if (this->gpslock) {
				printf(COLOR_FBLACK);
				printf(COLOR_BGREEN);
				printf("[natnet] LOCKED!");
				printf(COLOR_NONE);
				printf("\n");
			} else {
				printf(COLOR_FBLACK);
				printf(COLOR_BRED);
				printf("[natnet] LOST LOCK!");
				printf(COLOR_NONE);
				printf("\n");
			}
		}

		prev_gps_lock = this->gpslock;
		prevframeNumber = frameNumber;
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}



/* function not used unless loopback macro is enabled */
void NatNet::natnet_loopback() {
	while(1) {
		static unsigned char buffer_data[50];
		static int bytes_data = 0;
		bytes_data += udp_socket_recv(&rx_loopback_data, buffer_data, 50);
		// Parse NatNet data
		if (bytes_data > (int) sizeof(robot_t)) {
			if (buffer_data[0] == '$') {
				robot_t robot;
				memcpy(&robot, &buffer_data[1], sizeof(robot_t));
				// printf("%f, %.02f, %.02f, %.02f, %.02f, %.02f, %.02f, %.02f, %.02f, %.02f\n", 
				// ai->curr_time, robot.pos.x, robot.pos.y, robot.pos.z, robot.vel.x, robot.vel.y, robot.vel.z, robot.att.roll, robot.att.pitch, robot.att.yaw);
			}
			bytes_data = 0;
		}	
		std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1000 Hz listenin
	}
}

NatNet::NatNet() {
			
	// 1511 natnet data port
	udp_socket_create(&natnet_data, (char*) "", -1, natnet_data_port, 0); // Only receiving
	udp_socket_subscribe_multicast(&natnet_data, natnet_multicast_addr);  // 239.255.42.99
	udp_socket_set_recvbuf(&natnet_data, 0x100000); // 1MB

	// tx to drone test
	udp_socket_create(&tx_data, (char *) tx_data_addr, tx_data_port, 0, 0);
	udp_socket_set_sendbuf(&tx_data, 0x100000); // 1MB tx buffer

	// rx loopback test
	#ifdef LOOPBACK
	udp_socket_create(&rx_loopback_data, (char *) rx_loopback_addr, -1, rx_loopback_port, 0);
	udp_socket_set_recvbuf(&rx_loopback_data, 0x100000); // 1MB rx loopback buffer
	#endif
	// start the natnet thread
	try {
		natnet_thread_  = std::thread(&NatNet::natnet_rx, this);
		natnet_thread2_ = std::thread(&NatNet::natnet_tx, this);
		#ifdef LOOPBACK
		natnet_thread3_ = std::thread(&NatNet::natnet_loopback, this);
		#endif

		natnet_thread4_ = std::thread(&NatNet::connection_check, this);
		
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
	natnet_thread4_.detach();
	#ifdef LOOPBACK
	natnet_thread3_.detach();
	#endif
	printf("[gps] thread killed!\n");
}