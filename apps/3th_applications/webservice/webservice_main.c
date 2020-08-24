/****************************************************************************
 *
 *   Copyright (C) 2019 SatCommGrop. All rights reserved.
 *   Author: SatCommGrop <alvin.pengw@gmail.com>
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

#include <sys/ioctl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <debug.h>

#include <net/if.h>
#include <netinet/in.h>

#include <nuttx/net/arp.h>
#include <netutils/netlib.h>
/* Include uIP webserver definitions */

#include <netutils/httpd.h>
#include <webservice/web_cgi.h>
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
struct webservice_t
{
	/* main task parameters */
	pid_t 						task_pid;				/* task pid */
	bool 						should_exit;			/* task loop exit flag */
};
/****************************************************************************
 * Public Data
 ****************************************************************************/
static struct webservice_t  g_webservice ={
	/* main task parameters */
	.task_pid 			= 0,			///< task pid
	.should_exit 		= true,			///< thread exit flag
};
 
 /****************************************************************************
 * Private Functions
 ****************************************************************************/
 /****************************************************************************
  * Name: webservice_run
  *
  * Description:
  *   webservice service runtime for core node.
  *
  * Input Parameters:
  *   argc  - number of parameters.
  *   argv  - pointer of parameters.
  *
  * Returned Value:
  *   the status of program.
  *
  ****************************************************************************/
 int webservice_run(int argc, FAR char *argv[])
 {

#if defined(CONFIG_NET_ETHERNET)
    /* Use the configured, fixed MAC address */
	uint8_t mac[IFHWADDRLEN];
	mac[0] = (CONFIG_NSH_MACADDR >> (8 * 5)) & 0xff;
	mac[1] = (CONFIG_NSH_MACADDR >> (8 * 4)) & 0xff;
	mac[2] = (CONFIG_NSH_MACADDR >> (8 * 3)) & 0xff;
	mac[3] = (CONFIG_NSH_MACADDR >> (8 * 2)) & 0xff;
	mac[4] = (CONFIG_NSH_MACADDR >> (8 * 1)) & 0xff;
	mac[5] = (CONFIG_NSH_MACADDR >> (8 * 0)) & 0xff;

    /* Set the MAC address */

    netlib_setmacaddr("eth0", mac);

    /* Set up our host address */
  	struct in_addr addr;
#ifndef CONFIG_NSH_DHCPC
  	addr.s_addr = HTONL(CONFIG_NSH_IPADDR);
#else
  	addr.s_addr = 0;
#endif
	netlib_set_ipv4addr("eth0", &addr);

	/* Set up the default router address */

	addr.s_addr = HTONL(CONFIG_NSH_DRIPADDR);
	netlib_set_dripv4addr("eth0", &addr);

	/* Setup the subnet mask */

	addr.s_addr = HTONL(CONFIG_NSH_NETMASK);
	netlib_set_ipv4netmask("eth0", &addr);

	/* New versions of netlib_set_ipvXaddr will not bring the network up,
	* So ensure the network is really up at this point.
	*/

	netlib_ifup("eth0");
#endif	/*CONFIG_NET_ETHERNET*/

#ifdef CONFIG_NET_TCP
	  webcgi_register();
	  httpd_listen();
#endif /*CONFIG_NET_TCP*/

	  g_webservice.should_exit = false;

	  while (!g_webservice.should_exit){
	      sleep(3);
	      printf("[web]:Webservice Still running\n");
#if CONFIG_NFILE_DESCRIPTORS > 0
	      fflush(stdout);
#endif /*CONFIG_NFILE_DESCRIPTORS*/
	}

	  return 0;
 }

/****************************************************************************
 * Public Functions
 ****************************************************************************/


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
	printf(" webservice_main application\n");
	printf(" <cmd> <start> start the program.\n");
	printf(" <cmd> <stop> stop the program\n");
	printf("\n");
}

/****************************************************************************
 * webservice_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int webservice_main(int argc, char *argv[])
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
		if(!g_webservice.should_exit){
				printf("[WEB]:application is already running!\n]");
				return ERROR;
		}

		//创建主任务（接收线程）
		g_webservice.task_pid = task_create(CONFIG_APP_WEBSERVICE_PROGNAME, \
											CONFIG_APP_WEBSERVICE_PRIORITY, \
											CONFIG_APP_WEBSERVICE_STACKSIZE,\
											webservice_run,\
											argv);
		if(g_webservice.task_pid < 0){
			syslog(LOG_ERR,"[WEB]:ERROR,Failed to create uavcan received task:%d\n",g_webservice.task_pid);
			return g_webservice.task_pid;
		}

		return OK;
	}

	/*
	 * Stop the application.
	 */
	else if (!strcmp(argv[1], "stop")) {
		
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
