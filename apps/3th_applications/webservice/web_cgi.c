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

#include <stdio.h>
#include <string.h>
#include <sys/socket.h>

#include <netutils/httpd.h>

#include <webservice/web_cgi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_NETUTILS_HTTPDFILESTATS
HTTPD_CGI_CALL(file, "file-stats", file_stats);
#endif


/****************************************************************************
 * Name: net_stats
 ****************************************************************************/


/****************************************************************************
 * Name: file_stats
 ****************************************************************************/

#ifdef CONFIG_NETUTILS_HTTPDFILESTATS
static void file_stats(struct httpd_state *pstate, char *ptr)
{
  char buffer[16];
  char *pcount = strchr(ptr, ' ') + 1;
//  snprintf(buffer, 16, "%5u", httpd_fs_count(pcount));
  send(pstate->ht_sockfd, buffer, strlen(buffer), 0);
}
#endif

/****************************************************************************
 * Name: webcgi_register
 *
 * Description:
 *   webservice cgi register.
 *
 * Input Parameters:
 *   none.
 *
 * Returned Value:
 *   none.
 *
 ****************************************************************************/
void webcgi_register(void)
{
#ifdef CONFIG_NETUTILS_HTTPDFILESTATS
  httpd_cgi_register(&file);
#endif

}


