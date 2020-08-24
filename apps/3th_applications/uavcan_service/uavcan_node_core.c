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
#include <math.h>

#include <debug.h>
#include <errno.h>

#include <uavcan_service/uavcan_service.h>
#include <uavcan_service/node_core/uavcan_node_core.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


/****************************************************************************
 * Public Data
 ****************************************************************************/
/*
 * uORB message structure
 */
//publish
static struct uorb_pub_msg_attitude_body_t g_uorb_pub_msg_attitude_body;		//attitude body
static struct uorb_pub_msg_gps_data_t	   g_uorb_pub_msg_gps_data;				//gps data

//subscribe
static struct uorb_sub_msg_beam_attitude_t g_uorb_sub_msg_beam_attitude;		//beam attitude
static struct uorb_sub_msg_beacon_info_t   g_uorb_sub_msg_beacon_info;			//beacon info

/*
 * UAVCAN message structure
 */
static dsdl_uavcan_equipment_ahrs_SatelliteAndBeacon g_dsdl_uavcan_equipment_ahrs_SatelliteAndBeacon;

/*
 * UAVCAN node structure
 */
static struct uavcan_node_core_t g_uavcan_node_core = {
		/* uORB message */
		.pub_msg_attitude_body = &g_uorb_pub_msg_attitude_body,
		.pub_msg_gps_data	   = &g_uorb_pub_msg_gps_data,
		.sub_msg_beam_attitude = &g_uorb_sub_msg_beam_attitude,
		.sub_msg_beacon_info   = &g_uorb_sub_msg_beacon_info,
		/* uavcan message */
		.infos				   = &g_dsdl_uavcan_equipment_ahrs_SatelliteAndBeacon,
};


 /****************************************************************************
 * Private Functions
 ****************************************************************************/

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
 struct uavcan_node_core_t* uavcan_node_core_initialize()
 {
	 return &g_uavcan_node_core;
 }

 ////////////////////////////////////////////////////////////////////////////////////
 /*-------------------------------UAVCAN节点uORB登记发布.----------------------------*/
 ////////////////////////////////////////////////////////////////////////////////////

 /////////////
 /*IMU*/
 /////////////
 /****************************************************************************
  * Name: uorb_imu_fusion_data_advistise
  *
  * Description:
  *   IMU fusion data advertise via uORB.
  *
  * Input Parameters:
  *   uavcan_node_core_t  node struct.
  *
  * Returned Value:
  *   the file describer of IMU fusion data.
  *
  ****************************************************************************/
 int uorb_imu_fusion_data_advistise(void *priv)
{
	struct uavcan_node_core_t 			*node 	= priv;
	struct uorb_pub_msg_attitude_body_t *msg	= node->pub_msg_attitude_body;

	memset(&(msg->imu_fusion_data), 0, sizeof(msg->imu_fusion_data));
	msg->imu_fusion_pub = orb_advertise(ORB_ID(imu_data_uorb), &(msg->imu_fusion_data));

	return OK;
}

 /****************************************************************************
  * Name: uorb_imu_fusion_data_publish
  *
  * Description:
  *   IMU fusion data publish via uORB.
  *
  * Input Parameters:
  *   uavcan_node_core_t  node struct.
  *
  * Returned Value:
  *   the file describer of IMU fusion data.
  *
  ****************************************************************************/
 int uorb_imu_fusion_data_publish(struct uavcan_node_core_t *node,dsdl_uavcan_equipment_ahrs_FusionIMU *data)
{
	 struct uorb_pub_msg_attitude_body_t *msg	= node->pub_msg_attitude_body;

     //copy the fresh data.
	 msg->imu_fusion_data.fusion_roll  = data->fusion_roll;
	 msg->imu_fusion_data.fusion_pitch = data->fusion_pitch;
	 msg->imu_fusion_data.fusion_yaw   = data->fusion_yaw;
	 msg->imu_fusion_data.q0   = data->q[0];
	 msg->imu_fusion_data.q1   = data->q[1];
	 msg->imu_fusion_data.q2   = data->q[2];
	 msg->imu_fusion_data.q3   = data->q[3];
	 msg->imu_fusion_data.gx   = data->fusion_roll_velocity;
	 msg->imu_fusion_data.gy   = data->fusion_pitch_velocity;
	 msg->imu_fusion_data.gz   = data->fusion_yaw_velocity;

	 //publish the data to uORB.
	return orb_publish(ORB_ID(imu_data_uorb), msg->imu_fusion_pub, &(msg->imu_fusion_data));
}

 /////////////
 /*GPS-RTK*/
 /////////////
 /****************************************************************************
  * Name: uorb_gps_data_advistise
  *
  * Description:
  *   IMU gps data advertise via uORB.
  *
  * Input Parameters:
  *   uavcan_node_core_t  node struct.
  *
  * Returned Value:
  *   the file describer of IMU fusion data.
  *
  ****************************************************************************/
 int uorb_gps_data_advistise(void *priv)
{
	struct uavcan_node_core_t 			*node 	= priv;
	struct uorb_pub_msg_gps_data_t 		*msg	= node->pub_msg_gps_data;

	memset(&(msg->gps_data), 0, sizeof(msg->gps_data));
	msg->gps_data_pub = orb_advertise(ORB_ID(gps_data_uorb), &(msg->gps_data));

	return OK;
}



 /****************************************************************************
  * Name: uorb_gps_data_publish
  *
  * Description:
  *   GPS data publish via uORB.
  *
  * Input Parameters:
  *   uavcan_node_core_t  node struct.
  *
  * Returned Value:
  *   the file describer of IMU fusion data.
  *
  ****************************************************************************/
 int uorb_gps_data_publish(struct uavcan_node_core_t *node,dsdl_uavcan_equipment_gnss_RawGPS *data)
{
	 struct uorb_pub_msg_gps_data_t *msg = node->pub_msg_gps_data;

	//copy the fresh data.
	 msg->gps_data.longitude  = data->longitude;
	 msg->gps_data.latitude   = data->latitude;
	 msg->gps_data.altitude   = data->altitude;
	 msg->gps_data.satellite_no   = data->sats_visible;
	 msg->gps_data.gps_state  = data->sats_state;
	 msg->gps_data.gps_type   = data->fix_type;
	 msg->gps_data.time.second  = data->utc%60;
	 msg->gps_data.time.minutes = data->utc/60%60;
	 msg->gps_data.time.hour    = data->utc/60/60%24;
	 msg->gps_data.time.day   	= data->utc/60/60/24%31;
	 msg->gps_data.time.month   = data->utc/60/60/24/31%12;
	 msg->gps_data.time.year   	= data->utc/60/60/24/31/12%99;

	//publish the data to uORB.
	return orb_publish(ORB_ID(gps_data_uorb), msg->gps_data_pub, &(msg->gps_data));
}

 ////////////////////////////////////////////////////////////////////////////////////
 /*-------------------------------UAVCAN节点uORB订阅.-------------------------------*/
 ////////////////////////////////////////////////////////////////////////////////////
 /////////////
 /*BEAM-ATTITUDE*/
 /////////////
 /****************************************************************************
  * Name: uavcan_subscribe_beam_attitude
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
 int uavcan_subscribe_beam_attitude(void *priv)
 {
	struct uavcan_node_core_t   		*node = priv;
	struct uorb_sub_msg_beam_attitude_t *ba	  = node->sub_msg_beam_attitude;

	/*
	 * Subscribe beam attitude via uORB
	 */
 	memset(&(ba->data), 0, sizeof((ba->data)));
 	ba->fd = orb_subscribe(ORB_ID(beam_attitude_uorb));

 	return ba->fd;
 }
 /****************************************************************************
  * Name: uavcan_update_beam_attitude
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
 bool uavcan_update_beam_attitude(void *priv)
 {
	struct uavcan_node_core_t   *node	 		= priv;

 	bool update = false;

 	/* 查看当前是否有新的数据产生 */
	struct uorb_sub_msg_beam_attitude_t *ba = node->sub_msg_beam_attitude;
	dsdl_uavcan_equipment_ahrs_SatelliteAndBeacon *info = node->infos;
 	orb_check(ba->fd,&update);

 	/* 产生新的数据 */
 	if(update)
 	{
 		/* 拷贝最新鲜的数据 */
 		orb_copy(ORB_ID(beam_attitude_uorb), ba->fd, &(ba->data));
 		info->delta_yaw = (int16_t)(ba->data.n_delta_yaw * 10);
 		info->s_lon		= 1100;
 	}

 	// when attitude controller in run mode
 	if(ba->data.mode > 10 && ba->data.mode < 20)return update;
 	else return false;

 }

 /////////////
 /*BEACON-INFO*/
 /////////////
 /****************************************************************************
  * Name: beacon_info_uorb
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
 int uavcan_subscribe_beacon_info(void *priv)
 {
	struct uavcan_node_core_t   		*node = priv;
	struct uorb_sub_msg_beacon_info_t 	*bi	  = node->sub_msg_beacon_info;

	/*
	 * Subscribe beacon info via uORB
	 */
	memset(&(bi->data), 0, sizeof((bi->data)));
	bi->fd = orb_subscribe(ORB_ID(beacon_info_uorb));

	return bi->fd;
 }
 /****************************************************************************
  * Name: uavcan_update_beacon_info
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
 bool uavcan_update_beacon_info(void *priv)
 {
	struct uavcan_node_core_t   *node	 		= priv;

 	bool update = false;

 	/* 查看当前是否有新的数据产生 */
	struct uorb_sub_msg_beacon_info_t *bi = node->sub_msg_beacon_info;
	dsdl_uavcan_equipment_ahrs_SatelliteAndBeacon *info = node->infos;
 	orb_check(bi->fd,&update);

 	/* 产生新的数据 */
 	if(update)
 	{
 		/* 拷贝最新鲜的数据 */
 		orb_copy(ORB_ID(beacon_info_uorb), bi->fd, &(bi->data));
		/* Beacon/Radio power  */
 		info->power = (int8_t)(bi->data.quality*100);
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
	 struct uavcan_node_core_t* node =  uavcan_node_core_initialize();

	//请求消息
	if ((transfer->transfer_type == CanardTransferTypeRequest) &&
		(transfer->data_type_id == 1)){

	}

	//广播消息
	if ((transfer->transfer_type == CanardTransferTypeBroadcast) &&
		(transfer->data_type_id == DSDL_UAVCAN_EQUIPMENT_AHRS_FUSIONIMU_ID)){

		dsdl_uavcan_equipment_ahrs_FusionIMU dest;

		dsdl_uavcan_equipment_ahrs_FusionIMU_decode(transfer,\
											   sizeof(dsdl_uavcan_equipment_ahrs_FusionIMU),\
											   &dest,\
											   NULL);

		uorb_imu_fusion_data_publish(node,&dest);
	}

	if ((transfer->transfer_type == CanardTransferTypeBroadcast) &&
		(transfer->data_type_id == DSDL_UAVCAN_EQUIPMENT_GNSS_RAWGPS_ID)){

		dsdl_uavcan_equipment_gnss_RawGPS dest;

		dsdl_uavcan_equipment_gnss_RawGPS_decode(transfer,\
											sizeof(dsdl_uavcan_equipment_gnss_RawGPS),\
											&dest,\
											NULL);

		uorb_gps_data_publish(node,&dest);
	}
 }


 /****************************************************************************
  * Name: shouldAcceptTransfer
  *
  * Description:
  *   This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received by the local node.
  *	  If the callback returns true, the library will receive the transfer.
  *   If the callback returns false, the library will ignore the transfer.
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

	  if (canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID)
	    {
	      /* If we're in the process of allocation of dynamic node ID, accept
	       * only relevant transfers.
	       */
	    }
	  else
	    {
	      if ((transfer_type == CanardTransferTypeRequest) &&
	          (data_type_id == 1))
	        {

	          return true;
	        }

	      if ((transfer_type == CanardTransferTypeBroadcast) &&
	          (data_type_id == DSDL_UAVCAN_EQUIPMENT_AHRS_FUSIONIMU_ID))
	        {
	    	  *out_data_type_signature = DSDL_UAVCAN_EQUIPMENT_AHRS_FUSIONIMU_SIGNATURE;
	          return true;
	        }

	      if ((transfer_type == CanardTransferTypeBroadcast) &&
	          (data_type_id == DSDL_UAVCAN_EQUIPMENT_GNSS_RAWGPS_ID))
	        {
	    	  *out_data_type_signature = DSDL_UAVCAN_EQUIPMENT_GNSS_RAWGPS_SIGNATURE;
	          return true;
	        }
	    }

	  return false;
 }

 /****************************************************************************
  * Name: processRxOnce
  *
  * Description:
  *   Transmits all frames from the TX queue, receives up to one frame.
  *
  ****************************************************************************/
 static void processRxOnce(CanardInstance* out_ins,CanardNuttXInstance * nuttxcan,int timeout_msec)
 {
   /* Receiving */
   CanardCANFrame rx_frame;
   const uint64_t timestamp = getMonotonicTimestampUSec();
   const int rx_res = canardNuttXReceive(nuttxcan, &rx_frame, timeout_msec);

   if (rx_res < 0){               /* Failure - report */
            (void)fprintf(stderr, "Receive error %d, errno '%s'\n", rx_res,
                     strerror(errno));
   }
   else if (rx_res > 0){          /* Success - process the frame */
       canardHandleRxFrame(out_ins, &rx_frame, timestamp);
   }
   else{
       ;                         /* Timeout - nothing to do */
   }
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
 int uavcan_node_core_run(int argc, FAR char *argv[])
 {
	 if(argv[0] == NULL) return ERROR;
	 const char* const can_iface_name = "/dev/can_ext2";
	 struct uavcan_service_t 	*priv = get_uavcan_service_instance();
	 struct uavcan_node_core_t  *node = (struct uavcan_node_core_t*)priv->node;

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

	 canardSetLocalNodeID(&(node->canard),\
			 	 	 	  11);

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

	priv->should_exit = false;

	//登记待发送消息
	uorb_imu_fusion_data_advistise(node);
	uorb_gps_data_advistise(node);

    //主循环
	while(!priv->should_exit){

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
 static void makeNodeSattelliteAndBeaconmessage(uint8_t buffer[DSDL_UAVCAN_EQUIPMENT_AHRS_SATELLITEANDBEACON_MAX_SIZE],\
		 dsdl_uavcan_equipment_ahrs_SatelliteAndBeacon* source)
 {
	/*
	 * Clean the buffer to store a message.
	 */
	memset(buffer, 0, DSDL_UAVCAN_EQUIPMENT_AHRS_SATELLITEANDBEACON_MAX_SIZE);
	/*
	 * Encode the UAVCAN fusion imu message
	 */
	dsdl_uavcan_equipment_ahrs_SatelliteAndBeacon_encode(source, (void *)buffer);
 }


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
 void uavcan_node_core_transmit_run(pthread_addr_t arg)
 {
	 struct uavcan_service_t 	*priv = get_uavcan_service_instance();
	 struct uavcan_node_core_t  *node = (struct uavcan_node_core_t*)priv->node;

    /* Subscribe attitude message from uORB and setting poll event */
	node->fds[POLL_MSG_BEAM_ATTITUDE].fd	= uavcan_subscribe_beam_attitude(node);
	node->fds[POLL_MSG_BEAM_ATTITUDE].events= POLLIN;

	node->fds[POLL_MSG_BEACON_POWER].fd	  	= uavcan_subscribe_beacon_info(node);
	node->fds[POLL_MSG_BEACON_POWER].events = POLLIN;

	priv->should_exit = false;

	/* Main loop */
	while(!priv->should_exit){

		/* poll all subscribed uorb message */
		int poll_ret = poll(node->fds, 1, 60000);


		/* handle the poll result */
		if (poll_ret <= 0){
			/* use a counter to prevent flooding (and slowing us down) */
			syslog(LOG_WARNING, "[UAVCAN]: WARN return value from poll(): %d\n", poll_ret);
		}else{

			/* update attitude body massage from uORB. */
			if(uavcan_update_beam_attitude(node) == true){

				/* Transmitting the node status message periodically.*/
			     static uint8_t beam_attitude_transfer_id;  // Note that the transfer ID variable MUST BE STATIC (or heap-allocated)!
			     uint8_t buffer[DSDL_UAVCAN_EQUIPMENT_AHRS_SATELLITEANDBEACON_MAX_SIZE];

			     /* Making uavcan message. */
			     makeNodeSattelliteAndBeaconmessage(buffer,node->infos);

			 	 /* Broadcasting the message via can. */
			     const int16_t bc_res = canardBroadcast(&(node->canard),\
			    		 	 	 	 	 	 	 	 	DSDL_UAVCAN_EQUIPMENT_AHRS_SATELLITEANDBEACON_SIGNATURE,\
														DSDL_UAVCAN_EQUIPMENT_AHRS_SATELLITEANDBEACON_ID,\
			                                            &beam_attitude_transfer_id,\
														CANARD_TRANSFER_PRIORITY_LOW,\
			                                            buffer,\
														DSDL_UAVCAN_EQUIPMENT_AHRS_SATELLITEANDBEACON_MAX_SIZE);
			     if (bc_res <= 0){
			         syslog(LOG_ERR, "[UAVCAN]: Could not broadcast sattellite and beacon: %d\n", bc_res);
			         printf("[UAVCAN]:Could not broadcast sattellite and beacon: %d\n", bc_res);
			     }
			}

			/* update beacon info massage from uORB. */
			if(uavcan_update_beacon_info(node) == true){

				/* Ignore Transmitting the node status message periodically.*/
			}

		    /* Processing a massage from FIFO */
		    processTxOnce(&(node->canard),&(node->out_ins), 5000);
		}
	}

	priv->should_exit = true;
 }


