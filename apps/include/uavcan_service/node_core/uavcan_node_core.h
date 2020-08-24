/****************************************************************************
 * app/include/uavcan_service/uavcan_node_core.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Peng wei <alvin.pengw@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __APPS_UAVCAN_NODE_CORE_H
#define __APPS_UAVCAN_NODE_CORE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* SYSTEM */
#include <nuttx/config.h>
#include <poll.h>

/* UAVCAN */
#include <uavcan_service/libcanard/canard.h>
#include <uavcan_service/libcanard/canard_nuttx.h>
#include <uavcan_service/dsdl/equipment/ahrs/FusionIMU.h>
#include <uavcan_service/dsdl/equipment/gnss/RawGPS.h>
#include <uavcan_service/dsdl/equipment/ahrs/SatelliteAndBeacon.h>
/* uORB */
#include <uORB/uorb/uORB.h>
#include <uORB/topic/imu_data_uorb.h>
#include <uORB/topic/gps_data_uorb.h>

#include <uORB/topic/beam_attitude_uorb.h>
#include <uORB/topic/beacon_info_uorb.h>
#include <beacon_receiver/beacon_types.h>
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define CANARD_MEMORY_SIZE  1024

enum {
	POLL_MSG_BEAM_ATTITUDE = 0,
	POLL_MSG_BEACON_POWER,
	POLL_MSG_MAX_CORE
};
/****************************************************************************
 * Public Types
 ****************************************************************************/

/*
 * uORB message structure
 */
struct uorb_pub_msg_attitude_body_t
{
	 orb_advert_t 				imu_fusion_pub;		///< Publish file describer.
	 struct imu_data_uorb_s   	imu_fusion_data;	///< IMU fusion data.
};

struct uorb_pub_msg_gps_data_t
{
	 orb_advert_t 				gps_data_pub;		///< Publish file describer.
	 struct gps_data_uorb_s   	gps_data;			///< gps data.
};

struct uorb_sub_msg_beam_attitude_t
{
	int 						fd;					///< Message file descriptor.
	struct beam_attitude_uorb_s data;				///< Beam attitude data.
};

struct uorb_sub_msg_beacon_info_t
{
	int 						fd;					///< Message file descriptor.
	struct beacon_info_uorb_s 	data;				///< Beacon info data.
};


/*
 * UAVCAN node structure
 */
struct uavcan_node_core_t
{
	/*
	 * Library instance.
	 * In simple applications it makes sense to make it static, but it is not necessary.
	 */
	 CanardInstance 			canard;                  			   ///< The library instance
	 CanardNuttXInstance 		out_ins;							   ///< CanPort instance
	 uint8_t 					canard_memory_pool[CANARD_MEMORY_SIZE];///< Arena for memory allocation, used by the library

	 /*
	  *  uORB publish message
	  */
	 struct uorb_pub_msg_attitude_body_t *pub_msg_attitude_body;	   ///< Publish message structure.
	 struct uorb_pub_msg_gps_data_t 	 *pub_msg_gps_data;	   		   ///< Publish message structure.

	 /*
	  *  uORB subscribe message
	  */
	 struct uorb_sub_msg_beam_attitude_t *sub_msg_beam_attitude;	   ///< Subscribe message structure.
	 struct uorb_sub_msg_beacon_info_t 	 *sub_msg_beacon_info;	   	   ///< Subscribe message structure.

	 /*
	  *  UAVCAN message
	  */
	 dsdl_uavcan_equipment_ahrs_SatelliteAndBeacon 	*infos;				///< Satellate and beacon structure

	 /*
	  * poll structure
	  */
	 struct pollfd fds[POLL_MSG_MAX_CORE];								///< array of poll fd
};



/****************************************************************************
 * Public Data
 ****************************************************************************/


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: uavcan_node_core_initialize
 *
 * Description:
 *   uavcan core node initialize.
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   the pointer of node.
 *
 ****************************************************************************/
struct uavcan_node_core_t* uavcan_node_core_initialize(void);

/****************************************************************************
 * Name: uavcan_node_core_run
 *
 * Description:
 *   uavcan service runtime for core node.
 *
 * Input Parameters:
 *   argc  - number of parameters.
 *   argv  - pointer of parameters.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int uavcan_node_core_run(int argc, FAR char *argv[]);


/****************************************************************************
 * Name: uavcan_node_core_transmit_run
 *
 * Description:
 *   uavcan node transmit runtime.
 *
 * Input Parameters:
 *   argc  - number of parameters.
 *   argv  - pointer of parameters.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
void uavcan_node_core_transmit_run(pthread_addr_t arg);

#endif /* __APPS_UAVCAN_NODE_CORE_H */
