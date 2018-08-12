/*
 * cansocket.h
 *
 *  Created on: February 20, 2018
 *      Author: Automodality Inc.
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_CANSOCKET_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_CANSOCKET_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <ros/ros.h>

#define CAN_TIMEOUT 1000

class CanSocket {

public:
	CanSocket(std::string ifname, int can_id);

	~CanSocket();

	int init(std::string ifname);
	int writeCAN(struct can_frame &frame);
	int readCAN(struct can_frame *frame);
	int writeCAN(struct canfd_frame &frame);
	int readCAN(struct canfd_frame *frame);
	bool getCANKey();
	void releaseCANKey();

protected:
	int can_id_;
	int s_; // cansocket file descriptor
	int can_key_;

private:
	std::string ifname_;
};


#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_CANSOCKET_H_ */
