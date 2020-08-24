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
#include <uavcan_service/node_core/uavcan_node_core.h>
#include <uavcan_service/node_imu/uavcan_node_imu.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


/****************************************************************************
 * Public Data
 ****************************************************************************/
 static struct uavcan_service_t  g_uavcan_service ={
	/* main task parameters */
	.task_pid 			= 0,			///< task pid
	.should_exit 		= true,			///< thread exit flag

	/* sub task(transmit thread) */
	.thread_tx_pid		= 0,			///< sub task(transmit thread) pid

	/* UAVCAN node */
	.node				= NULL,			///< UAVCAN node parameters
 };




/****************************************************************************
 * Public Functions
 ****************************************************************************/
 /****************************************************************************
  * Name: creat_uavcan_transmit_thread
  *
  * Description:
  *   creat transmit thread.
  *
  * Input Parameters:
  *   argc  - number of parameters.
  *   argv  - pointer of parameters.
  *
  * Returned Value:
  *   the status of program.
  *
  ****************************************************************************/
 int creat_uavcan_transmit_thread(void)
 {
 	/* set the tx thread parameters*/
 	pthread_attr_t uavcan_tx_pthread_attr ={
 		  .priority 	= PTHREAD_DEFAULT_PRIORITY,
 		  .policy		= PTHREAD_DEFAULT_POLICY,
 		  .inheritsched = PTHREAD_INHERIT_SCHED,
 		  .stacksize	= PTHREAD_STACK_DEFAULT*2
 	};

 	/* create the tx thread*/
 	int ret = pthread_create(&g_uavcan_service.thread_tx_pid,\
 						   	 &uavcan_tx_pthread_attr,\
							 (pthread_addr_t)g_uavcan_service.thread_tx_func,\
							 NULL);
 	if (ret < 0){
 		syslog(LOG_ERR,"[UAVCAN]:Failed to create transmit thread:%d\n.", ret);
 	}

 	return ret;
 }

 /****************************************************************************
  * Name: creat_uavcan_transmit_thread
  *
  * Description:
  *   creat transmit thread.
  *
  * Input Parameters:
  *   argc  - number of parameters.
  *   argv  - pointer of parameters.
  *
  * Returned Value:
  *   the status of program.
  *
  ****************************************************************************/
 struct uavcan_service_t* get_uavcan_service_instance(void)
 {
	 return &g_uavcan_service;
 }

/****************************************************************************
 * Name: usage
 *
 * Description:
 *   application use infomations.
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   none.
 *
 ****************************************************************************/
static void usage(void)
{
	printf(" uavcan_service_main application\n");
	printf(" <cmd> <start> start the program.\n");
	printf(" <cmd> <stop> stop the program\n");
	printf(" <cmd> <status> print the program status.\n");
	printf(" <cmd> <-s> <parameter> <value> set the parameter\n");
	printf("\n");
}

/****************************************************************************
 * uavcan_service_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int uavcan_service_main(int argc, char *argv[])
#endif
{
	if (argc < 2) {
		usage();
		return -EINVAL;
	}

	/*
	 * Start the application.
	 */
	if (!strcmp(argv[1], "start")) {

		if(argc != 3){
			usage();
			return ERROR;
		}

		if(!g_uavcan_service.should_exit){
			printf("[UAVCAN]:application is already running!\n]");
			return ERROR;
		}

		if(!strcmp(argv[2], "core")){
			//初始化节点参数
			g_uavcan_service.node = (void *)uavcan_node_core_initialize();
			g_uavcan_service.thread_tx_func = (pthread_addr_t)uavcan_node_core_transmit_run;

			//创建主任务（接收线程）
			g_uavcan_service.task_pid = task_create(CONFIG_APP_UAVCAN_SERVICE_PROGNAME, \
													CONFIG_APP_UAVCAN_SERVICE_PRIORITY, \
													CONFIG_APP_UAVCAN_SERVICE_STACKSIZE,\
													uavcan_node_core_run,\
													argv);
			if(g_uavcan_service.task_pid < 0){
				syslog(LOG_ERR,"[UAVCAN]:ERROR,Failed to create uavcan received task:%d\n",g_uavcan_service.task_pid);
				return g_uavcan_service.task_pid;
			}

		}else if(!strcmp(argv[2], "imu")){
			//初始化节点参数
			g_uavcan_service.node = (void *)uavcan_node_imu_initialize();
			g_uavcan_service.thread_tx_func = (pthread_addr_t)uavcan_node_imu_transmit_run;

			//创建主任务（接收线程）
			g_uavcan_service.task_pid = task_create(CONFIG_APP_UAVCAN_SERVICE_PROGNAME, \
													CONFIG_APP_UAVCAN_SERVICE_PRIORITY, \
													CONFIG_APP_UAVCAN_SERVICE_STACKSIZE,\
													uavcan_node_imu_run,\
													argv);
			if(g_uavcan_service.task_pid < 0){
				syslog(LOG_ERR,"[UAVCAN]:ERROR,Failed to create uavcan received task:%d\n",g_uavcan_service.task_pid);
				return g_uavcan_service.task_pid;
			}

		}else{
			usage();
		}

	    if (g_uavcan_service.task_pid < 0){
	        int errcode = g_uavcan_service.task_pid;
	        syslog(LOG_ERR, "[UAVCAN] ERROR: Failed to start uavcan node service task: %d\n",errcode);
	        return errcode;
	    }

		return OK;
	}

	/*
	 * Stop the application.
	 */
	else if (!strcmp(argv[1], "stop")) {
		
	}

	/*
	 * print the application status.
	 */
	else if (!strcmp(argv[1], "status")) {

		return OK;
	}

	/*
	 * Set parameters of the application.
	 */
	else if (!strcmp(argv[1], "-s")) {

		/*
		 * received parameters.
		 */
		float tmp_t = atof(argv[2]);
		if(tmp_t < 0){
			printf("Invalid Parameters:%f!!! parameter must be a positive number.\n",tmp_t);
		}
		return OK;
	}

	/*
	 * Print help information.
	 */
	else{
		usage();
		return -EINVAL;
	}

	return OK;
}
