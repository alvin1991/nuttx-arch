/****************************************************************************
 *
 *   Copyright (C) 2019 SatCommGrop. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <syslog.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <debug.h>
#include <errno.h>

#include <uavcan_service/uavcan_service.h>
#include <uavcan_service/node_imu/uavcan_node_imu.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/*
 * uORB message structure
 */
static struct msg_attitude_body_t g_msg_attitude_body;		//attitude body
static struct msg_gps_data_t 	  g_msg_gps_data;			//gps_data
static struct pub_msg_att_estimate_feedback_t	g_uorb_pub_att_estimate_fb;
/*
 * UAVCAN message structure
 */
static dsdl_uavcan_equipment_ahrs_FusionIMU 	g_dsdl_uavcan_equipment_ahrs_fusion_imu;				//equipment_ahrs_FusionIMU
static dsdl_uavcan_equipment_gnss_RawGPS		g_dsdl_uavcan_equipment_gnss_raw_gps;					//equipment_gnss_raw_gps

/*
 * UAVCAN node structure
 */
static struct uavcan_node_imu_t g_uavcan_node_imu = {
		/* UAVCAN message */
		.fusion_imu = &g_dsdl_uavcan_equipment_ahrs_fusion_imu,
		.raw_gps	= &g_dsdl_uavcan_equipment_gnss_raw_gps,

		/* uORB message */
		.uorb_attitude_body 	= &g_msg_attitude_body,
		.uorb_gps_data			= &g_msg_gps_data,
		.uorb_att_estimate_fb   = &g_uorb_pub_att_estimate_fb,
};



 /****************************************************************************
 * Private Functions
 ****************************************************************************/
static void makeNodeFusionIMUMessage(uint8_t buffer[DSDL_UAVCAN_EQUIPMENT_AHRS_FUSIONIMU_MAX_SIZE],dsdl_uavcan_equipment_ahrs_FusionIMU* source);


 /****************************************************************************
  * Name: getMonotonicTimestampUSec
  *
  * Description:
  *
  ****************************************************************************/
 static uint64_t getMonotonicTimestampUSec(void)
 {
   struct timespec ts;

   memset(&ts, 0, sizeof(ts));
   if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0){
       abort();
   }

   return ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
 }

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
 struct uavcan_node_imu_t* uavcan_node_imu_initialize()
 {
	 return &g_uavcan_node_imu;
 }

 /****************************************************************************
  * Name: uorb_att_estimate_feedback_data_advistise
  *
  * Description:
  *   att estimate feedback data advertise via uORB.
  *
  * Input Parameters:
  *   uavcan_node_core_t  node struct.
  *
  * Returned Value:
  *   the file describer of IMU fusion data.
  *
  ****************************************************************************/
 int uorb_att_estimate_feedback_data_advistise(void *priv)
{
	struct uavcan_node_imu_t 				*node 	= priv;
	struct pub_msg_att_estimate_feedback_t  *msg	= node->uorb_att_estimate_fb;

	memset(&(msg->att_estimate_fb_data), 0, sizeof(msg->att_estimate_fb_data));
	msg->att_estimate_fb_pub = orb_advertise(ORB_ID(att_estimate_feedback_uorb), &(msg->att_estimate_fb_data));

	return OK;
}

 /****************************************************************************
  * Name: uorb_att_estimate_feedback_data_publish
  *
  * Description:
  *   attitude estimate data publish via uORB.
  *
  * Input Parameters:
  *   uavcan_node_core_t  node struct.
  *
  * Returned Value:
  *   the file describer of att_estimate data.
  *
  ****************************************************************************/
 int uorb_att_estimate_feedback_data_publish(struct uavcan_node_imu_t *node,dsdl_uavcan_equipment_ahrs_SatelliteAndBeacon *data)
{
	 struct pub_msg_att_estimate_feedback_t *msg	= node->uorb_att_estimate_fb;

     //copy the fresh data.
	 msg->att_estimate_fb_data.type = ATT_EST_TYPE_SAT;
	 msg->att_estimate_fb_data.att_est_cmd.att_feedback.delta_yaw 	= data->delta_yaw;
	 msg->att_estimate_fb_data.att_est_cmd.att_feedback.s_lon 		= data->s_lon;
	 msg->att_estimate_fb_data.att_est_cmd.att_feedback.strength 	= data->power;
	 //publish the data to uORB.
	return orb_publish(ORB_ID(att_estimate_feedback_uorb), msg->att_estimate_fb_pub, &(msg->att_estimate_fb_data));
}
 /****************************************************************************
  * Name: uavcan_subscribe_attitude
  *
  * Description:
  *   subscribe the orb message.
  *
  * Input Parameters:
  *   priv  	- pointer of message.
  *
  * Returned Value:
  *   statue of subscribe.
  *
  ****************************************************************************/
 int uavcan_subscribe_attitude(void *priv)
 {
	struct uavcan_node_imu_t   *node  	 = priv;
	struct msg_attitude_body_t *att_body = node->uorb_attitude_body;

	/*
	 * Subscribe attitude body via uORB
	 */
 	memset(&(att_body->data), 0, sizeof((att_body->data)));
 	att_body->fd = orb_subscribe(ORB_ID(att_body_data_uorb));

 	return att_body->fd;
 }

 /****************************************************************************
  * Name: uavcan_subscribe_gps_data
  *
  * Description:
  *   subscribe the orb message.
  *
  * Input Parameters:
  *   priv  	- pointer of message.
  *
  * Returned Value:
  *   statue of subscribe.
  *
  ****************************************************************************/
 int uavcan_subscribe_gps_data(void *priv)
 {
	struct uavcan_node_imu_t   *node  	 = priv;
	struct msg_gps_data_t 	   *gps_data = node->uorb_gps_data;

	/*
	 * Subscribe attitude body via uORB
	 */
 	memset(&(gps_data->data), 0, sizeof((gps_data->data)));
 	gps_data->fd = orb_subscribe(ORB_ID(gps_data_uorb));

 	return gps_data->fd;
 }

 /****************************************************************************
  * Name: uavcan_update_attitude_body
  *
  * Description:
  *   update uORB message.
  *
  * Input Parameters:
  *   priv  	- pointer of message.
  *
  * Returned Value:
  *   the size of message.
  *
  ****************************************************************************/
 bool uavcan_update_attitude_body(void *priv)
 {
	struct uavcan_node_imu_t   *node	 		= priv;

 	bool update = FALSE;

 	/* 查看当前是否有新的数据产生 */
	struct msg_attitude_body_t *att_body = node->uorb_attitude_body;
	dsdl_uavcan_equipment_ahrs_FusionIMU *fusion_imu = node->fusion_imu;
 	orb_check(att_body->fd,&update);

 	/* 产生新的数据 */
 	if(update)
 	{
 		/* 拷贝最新鲜的数据 */
 		orb_copy(ORB_ID(att_body_data_uorb), att_body->fd, &(att_body->data));

 		fusion_imu->fusion_roll  = att_body->data.roll;
 		fusion_imu->fusion_pitch = att_body->data.pitch;
 		fusion_imu->fusion_yaw   = att_body->data.yaw;
 		fusion_imu->fusion_roll_velocity  = att_body->data.w_ib_b_x;
 		fusion_imu->fusion_pitch_velocity = att_body->data.w_ib_b_y;
 		fusion_imu->fusion_yaw_velocity   = att_body->data.w_ib_b_z;
 		fusion_imu->q[0]		 = att_body->data.q0;
 		fusion_imu->q[1]		 = att_body->data.q1;
 		fusion_imu->q[2]		 = att_body->data.q2;
 		fusion_imu->q[3]		 = att_body->data.q3;
 	}

 	return update;
 }

 /****************************************************************************
  * Name: uavcan_update_gps_data
  *
  * Description:
  *   update gps data.
  *
  * Input Parameters:
  *   priv  	- pointer of message.
  *
  * Returned Value:
  *   the size of message.
  *
  ****************************************************************************/
 bool uavcan_update_gps_data(void *priv)
 {
	struct uavcan_node_imu_t   *node	 		= priv;

 	bool update = FALSE;

 	/* 查看当前是否有新的数据产生 */
	struct msg_gps_data_t *uorb_gps_data = node->uorb_gps_data;
	dsdl_uavcan_equipment_gnss_RawGPS *uavcan_gps_data = node->raw_gps;
 	orb_check(uorb_gps_data->fd,&update);

 	/* 产生新的数据 */
 	if(update)
 	{
 		/* 拷贝最新鲜的数据 */
 		orb_copy(ORB_ID(gps_data_uorb), uorb_gps_data->fd, &(uorb_gps_data->data));

 		uavcan_gps_data->latitude 	= uorb_gps_data->data.latitude;
 		uavcan_gps_data->longitude 	= uorb_gps_data->data.longitude;
 		uavcan_gps_data->altitude 	= uorb_gps_data->data.altitude;
 		uavcan_gps_data->utc 		= uorb_gps_data->data.time.year    % 2000 * 32140800 \
 									 +uorb_gps_data->data.time.month   * 2678400\
									 +uorb_gps_data->data.time.day 	   * 86400\
									 +uorb_gps_data->data.time.hour    * 3600\
									 +uorb_gps_data->data.time.minutes * 60\
									 +uorb_gps_data->data.time.second;
 		uavcan_gps_data->sats_state = uorb_gps_data->data.gps_state;
 		uavcan_gps_data->sats_visible = uorb_gps_data->data.satellite_no;
 		uavcan_gps_data->fix_type 	  = uorb_gps_data->data.gps_state;
 	}

 	return update;
 }

 ////////////////////////////////////////////////////////////////////////////////////
 /*-------------------------------UAVCAN节点消息接收.--------------------------------*/
 ////////////////////////////////////////////////////////////////////////////////////

 /****************************************************************************
  * Name: onTransferReceived
  *
  * Description:
  *   This callback is invoked by the library when a new message or request or response is received.
  *
  * Input Parameters:
  *   ins  		- UAVCAN library instance.
  *   transfer  - pointer to temporary transfer object.
  *
  * Returned Value:
  *   the status of program.
  *
  ****************************************************************************/
 static void onTransferReceived(CanardInstance* ins,
                                CanardRxTransfer* transfer)
 {
	 	 struct uavcan_node_imu_t* node =  uavcan_node_imu_initialize();
		//请求消息
		if ((transfer->transfer_type == CanardTransferTypeRequest) &&
			(transfer->data_type_id == 1)){

		}

		//广播消息
		if ((transfer->transfer_type == CanardTransferTypeBroadcast) &&
			(transfer->data_type_id == 341)){
			printf("[UAVCAN]:Receive msg from 341\n");
		}

		// Satellite and beacon infomation.
		if ((transfer->transfer_type == CanardTransferTypeBroadcast) &&
			(transfer->data_type_id == DSDL_UAVCAN_EQUIPMENT_AHRS_SATELLITEANDBEACON_ID)){

			dsdl_uavcan_equipment_ahrs_SatelliteAndBeacon dest;

			dsdl_uavcan_equipment_ahrs_SatelliteAndBeacon_decode(transfer,\
												   sizeof(dsdl_uavcan_equipment_ahrs_SatelliteAndBeacon),\
												   &dest,\
												   NULL);
			uorb_att_estimate_feedback_data_publish(node,&dest);
		}
 }

 /****************************************************************************
  * Name: shouldAcceptTransfer
  *
  * Description:
  *   This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received by the local node.
  *	  If the callback returns TRUE, the library will receive the transfer.
  *   If the callback returns FALSE, the library will ignore the transfer.
  *   All transfers that are addressed to other nodes are always ignored.
  *
  * Input Parameters:
  *   ins 						- UAVCAN library instance.
  *   out_data_type_signature 	- node id allocation data type signature.
  *   data_type_id 				- node data type id.
  *   transfer_type 			- Transfer types are defined by the UAVCAN specification.
  *   source_node_id			- source node id.
  *
  * Returned Value:
  *   the status of program.
  *
  ****************************************************************************/
 static bool shouldAcceptTransfer(const CanardInstance* ins,
                                  uint64_t* out_data_type_signature,
                                  uint16_t data_type_id,
                                  CanardTransferType transfer_type,
                                  uint8_t source_node_id)
 {
	  (void)source_node_id;

	  if (canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID){
	      /* If we're in the process of allocation of dynamic node ID, accept
	       * only relevant transfers.
	       */
	  }
	  else{
		  //指定要回应的请求消息ID
	      if ((transfer_type == CanardTransferTypeRequest) &&
	          (data_type_id == 1)){

	          return TRUE;
	        }

	      //指定要接收的广播消息ID
	      if ((transfer_type == CanardTransferTypeBroadcast) &&
	          (data_type_id == 341)){
	          return TRUE;
	        }
	      //指定要接收的广播消息ID
	      if ((transfer_type == CanardTransferTypeBroadcast) &&
	          (data_type_id == DSDL_UAVCAN_EQUIPMENT_AHRS_SATELLITEANDBEACON_ID)){
	    	  *out_data_type_signature = DSDL_UAVCAN_EQUIPMENT_AHRS_SATELLITEANDBEACON_SIGNATURE;
	          return TRUE;
	        }
	 }

	  return FALSE;
 }

 /****************************************************************************
  * Name: processRxOnce
  *
  * Description:
  *   Receive all frames from the bus.
  *
  ****************************************************************************/
 static void processRxOnce(CanardInstance* out_ins,CanardNuttXInstance* nuttxcan,int timeout_msec)
 {
	   /* Receiving */
	   CanardCANFrame rx_frame;
	   const uint64_t timestamp = getMonotonicTimestampUSec();
	   const int rx_res = canardNuttXReceive(nuttxcan, &rx_frame, timeout_msec);

	   if (rx_res < 0){             /* Failure - report */

		   printf("[UAVCAN]:Receive error %d, errno '%s'\n", rx_res,strerror(errno));
	   }
	   else if (rx_res > 0){        /* Success - process the frame */

	       canardHandleRxFrame(out_ins, &rx_frame, timestamp);
	   }
	   else{						/* Timeout - nothing to do */
	       ;
	   }
 }

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
 int uavcan_node_imu_run(int argc, FAR char *argv[])
 {
	 if(argv[0] == NULL || argv[1] == NULL) return ERROR;

	 const char* const can_iface_name = "/dev/can_ext1";
	 struct uavcan_service_t 	*priv = get_uavcan_service_instance();
	 struct uavcan_node_imu_t   *node = (struct uavcan_node_imu_t*)priv->node;

	//初始化can设备驱动
	int16_t res = canardNuttXInit(&(node->out_ins), can_iface_name);
	if (res < 0){

		printf("[UAVCAN]:Failed to open CAN iface '%s'\n", can_iface_name);
		return res;
	}

	//实例化Libcanard设备.
	canardInit(&(node->canard),\
				node->canard_memory_pool,\
				CANARD_MEMORY_SIZE,\
				onTransferReceived,\
				shouldAcceptTransfer,\
				NULL);

	 //设置节点ID
	 canardSetLocalNodeID(&(node->canard),10);

	 priv->should_exit = FALSE;

	//创建副线程（发送线程）
	int ret = creat_uavcan_transmit_thread();
	if(ret<0){
		syslog(LOG_ERR,"[UAVCAN]:ERROR,Failed to create uavcan transmit task:%d\n",ret);
		return ret;
	}

	//Set pthread name.
	ret = pthread_setname_np(priv->thread_tx_pid, "uavcan transmit handler");
	if (ret != 0) {
		syslog(LOG_ERR,"[UAVCAN]:Filaed to set transmit thread name:%d\n.", ret);
	}

	uorb_att_estimate_feedback_data_advistise(node);

    //接收主循环
	while(!priv->should_exit){
		//阻塞接收一帧数据
		processRxOnce(&(node->canard),&(node->out_ins),10);
	}

	return 0;
 }

////////////////////////////////////////////////////////////////////////////////////
/*-------------------------------UAVCAN节点消息发送.--------------------------------*/
////////////////////////////////////////////////////////////////////////////////////

 /****************************************************************************
  * Name: makeNodeFusionIMUMessage
  *
  * Description:
  *
  ****************************************************************************/
 static void makeNodeFusionIMUMessage(uint8_t buffer[DSDL_UAVCAN_EQUIPMENT_AHRS_FUSIONIMU_MAX_SIZE],\
		 	 	 	 	 	 	 	  dsdl_uavcan_equipment_ahrs_FusionIMU* source)
 {
	/*
	 * Clean the buffer to store a message.
	 */
	memset(buffer, 0, DSDL_UAVCAN_EQUIPMENT_AHRS_FUSIONIMU_MAX_SIZE);
	/*
	 * Encode the UAVCAN fusion imu message
	 */
	dsdl_uavcan_equipment_ahrs_FusionIMU_encode(source, (void *)buffer);
 }

 /****************************************************************************
  * Name: makeNodeGPSDataMessage
  *
  * Description:
  *
  ****************************************************************************/
 static void makeNodeGPSDataMessage(uint8_t buffer[DSDL_UAVCAN_EQUIPMENT_GNSS_RAWGPS_MAX_SIZE],\
		 							dsdl_uavcan_equipment_gnss_RawGPS* source)
 {
	/*
	 * Clean the buffer to store a message.
	 */
	memset(buffer, 0, DSDL_UAVCAN_EQUIPMENT_GNSS_RAWGPS_MAX_SIZE);
	/*
	 * Encode the UAVCAN fusion imu message
	 */
	dsdl_uavcan_equipment_gnss_RawGPS_encode(source, (void *)buffer);
 }

 /****************************************************************************
  * Name: processTxOnce
  *
  * Description:
  *   Transmits all frames from the TX queue.
  *
  ****************************************************************************/
 static void processTxOnce(CanardInstance* out_ins,CanardNuttXInstance* nuttxcan,int timeout_msec)
 {
   const CanardCANFrame *txf;

   /* Transmitting */
   for (txf = NULL; (txf = canardPeekTxQueue(out_ins)) != NULL;){
       const int tx_res = canardNuttXTransmit(nuttxcan, txf, 0);
       if (tx_res < 0){           /* Failure - drop the frame and report */
           canardPopTxQueue(out_ins);

           printf("[UAVCAN]:Transmit error %d, frame dropped, errno '%s'\n", tx_res, strerror(errno));
       }
       else if (tx_res > 0){      /* Success - just drop the frame */
           canardPopTxQueue(out_ins);
       }
       else{                      /* Timeout - just exit and try again later */
           break;
       }
     }
 }

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
 void uavcan_node_imu_transmit_run(pthread_addr_t arg)
 {
	 struct uavcan_service_t 	*priv = get_uavcan_service_instance();
	 struct uavcan_node_imu_t   *node = (struct uavcan_node_imu_t*)priv->node;

    /* Subscribe attitude message from uORB and setting poll event */
	node->fds[POLL_MSG_FUSION_IMU].fd	  = uavcan_subscribe_attitude(node);
	node->fds[POLL_MSG_FUSION_IMU].events = POLLIN;

	node->fds[POLL_MSG_RTK_GPS].fd	  	  = uavcan_subscribe_gps_data(node);
	node->fds[POLL_MSG_RTK_GPS].events 	  = POLLIN;

	/* Main loop */
	while(!priv->should_exit){

		/* poll all subscribed uorb message */
//		int poll_ret = poll(node->fds, POLL_MSG_MAX, 5000);
		int poll_ret = poll(node->fds, 1, 5000);


		/* handle the poll result */
		if (poll_ret <= 0){
			/* use a counter to prevent flooding (and slowing us down) */
			syslog(LOG_WARNING, "[UAVCAN]: WARN return value from poll(): %d\n", poll_ret);
//			printf("[UAVCAN]:ERROR return value from poll(): %d, errno %d\n", poll_ret, errno);
		}else{

			/* update attitude body massage from uORB. */
			if(uavcan_update_attitude_body(node) == TRUE){

				/* Transmitting the node status message periodically.*/
			     static uint8_t fusion_imu_transfer_id;  // Note that the transfer ID variable MUST BE STATIC (or heap-allocated)!
			     uint8_t buffer[DSDL_UAVCAN_EQUIPMENT_AHRS_FUSIONIMU_MAX_SIZE];

			     /* Making fusion data of IMU. */
			 	 makeNodeFusionIMUMessage(buffer,node->fusion_imu);

			 	 /* Broadcasting fusion data of IMU. */
			     const int16_t bc_res = canardBroadcast(&(node->canard),\
			    		 	 	 	 	 	 	 	 	DSDL_UAVCAN_EQUIPMENT_AHRS_FUSIONIMU_SIGNATURE,\
														DSDL_UAVCAN_EQUIPMENT_AHRS_FUSIONIMU_ID,\
			                                            &fusion_imu_transfer_id,\
														CANARD_TRANSFER_PRIORITY_MEDIUM,\
			                                            buffer,\
														DSDL_UAVCAN_EQUIPMENT_AHRS_FUSIONIMU_MAX_SIZE);
			     if (bc_res <= 0){
			         syslog(LOG_ERR, "[UAVCAN]: Could not broadcast fusion data of imu: %d\n", bc_res);
			         printf("[UAVCAN]:Could not broadcast fusion data of imu: %d\n", bc_res);
			     }
			}

			/* update gps data massage from uORB. */
			if(uavcan_update_gps_data(node) == TRUE){

				/* Transmitting the node status message periodically.*/
			     static uint8_t gps_data_transfer_id;  // Note that the transfer ID variable MUST BE STATIC (or heap-allocated)!
			     uint8_t buffer[DSDL_UAVCAN_EQUIPMENT_AHRS_FUSIONIMU_MAX_SIZE];

			     /* Making gps data. */
			     makeNodeGPSDataMessage(buffer,node->raw_gps);

			 	 /* Broadcasting fusion data of IMU. */
			     const int16_t bc_res = canardBroadcast(&(node->canard),\
			    		 	 	 	 	 	 	 	    DSDL_UAVCAN_EQUIPMENT_GNSS_RAWGPS_SIGNATURE,\
														DSDL_UAVCAN_EQUIPMENT_GNSS_RAWGPS_ID,\
			                                            &gps_data_transfer_id,\
														CANARD_TRANSFER_PRIORITY_LOW,\
			                                            buffer,\
														DSDL_UAVCAN_EQUIPMENT_GNSS_RAWGPS_MAX_SIZE);
			     if (bc_res <= 0){
			         syslog(LOG_ERR, "[UAVCAN]: Could not broadcast gps data: %d\n", bc_res);
			         printf("[UAVCAN]:Could not broadcast gps data: %d\n", bc_res);
			     }
			}

		    /* Processing a massage from FIFO */
		    processTxOnce(&(node->canard),&(node->out_ins), 5000);
		}
	}
 }

