/****************************************************************************
 * app/include/uavcan_service/uavcan_node_imu.h
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

#ifndef __APPS_UAVCAN_NODE_IMU_H
#define __APPS_UAVCAN_NODE_IMU_H

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
#include <uavcan_service/dsdl/equipment/ahrs/SatelliteAndBeacon.h>
#include <uavcan_service/dsdl/equipment/gnss/RawGPS.h>
#include <uavcan_service/dsdl/equipment/ahrs/SatelliteAndBeacon.h>

/* uORB */
#include <uORB/uorb/uORB.h>
#include <uORB/topic/att_body_data_uorb.h>
#include <uORB/topic/gps_data_uorb.h>
#include <uORB/topic/att_estimate_feedback_uorb.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define CANARD_MEMORY_SIZE  1024

enum {
	POLL_MSG_FUSION_IMU = 0,
	POLL_MSG_RTK_GPS,
	POLL_MSG_MAX
};

/****************************************************************************
 * Public Types
 ****************************************************************************/

/*
 * uORB message structure
 */
struct msg_attitude_body_t
{
	int 							  fd;				/* Message file descriptor */
	struct att_body_data_uorb_s       data;				/* Fusion IMU data */
};

struct msg_gps_data_t
{
	int 							  fd;				/* message file descriptor */
	struct gps_data_uorb_s 			  data;				/* RTK GPS */
};

struct pub_msg_att_estimate_feedback_t
{
	 orb_advert_t 							att_estimate_fb_pub;		///< Publish file describer.
	 struct att_estimate_feedback_uorb_s   	att_estimate_fb_data;		///< att_estimate_feedback data.
};

/*
 * UAVCAN node structure
 */
struct uavcan_node_imu_t
{
	/*
	 * Library instance.
	 * In simple applications it makes sense to make it static, but it is not necessary.
	 */
	 CanardInstance 			canard;                  			   ///< The library instance
	 CanardNuttXInstance 		out_ins;							   ///< CanPort instance
	 uint8_t 					canard_memory_pool[CANARD_MEMORY_SIZE];///< Arena for memory allocation, used by the library

	 /*
	  *  UAVCAN message
	  */
	 dsdl_uavcan_equipment_ahrs_FusionIMU 	*fusion_imu;				///< fusion IMU massage structure
	 dsdl_uavcan_equipment_gnss_RawGPS		*raw_gps;					///< Raw GPS data massage structure
	 dsdl_uavcan_equipment_ahrs_SatelliteAndBeacon 	*infos;				///< Satellate and beacon structure

	 /*
	  *  uORB message
	  */
	 struct msg_attitude_body_t 				*uorb_attitude_body;		///< fusion data structure
	 struct msg_gps_data_t						*uorb_gps_data;				///< RTK-GPS data structure
	 struct pub_msg_att_estimate_feedback_t		*uorb_att_estimate_fb;

	 /*
	  * poll structure
	  */
	 struct pollfd fds[POLL_MSG_MAX];								///< array of poll fd


};



/****************************************************************************
 * Public Data
 ****************************************************************************/


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: uavcan_node_imu_initialize
 *
 * Description:
 *   uavcan imu node initialize.
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   the pointer of node.
 *
 ****************************************************************************/
struct uavcan_node_imu_t* uavcan_node_imu_initialize(void);

/****************************************************************************
 * Name: uavcan_node_imu_run
 *
 * Description:
 *   uavcan service runtime for imu node.
 *
 * Input Parameters:
 *   argc  - number of parameters.
 *   argv  - pointer of parameters.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int uavcan_node_imu_run(int argc, FAR char *argv[]);


/****************************************************************************
 * Name: uavcan_node_imu_transmit_run
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
void uavcan_node_imu_transmit_run(pthread_addr_t arg);

#endif /* __APPS_UAVCAN_NODE_IMU_H */
