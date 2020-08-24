/****************************************************************************
 * examples/hello/hello_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <stdio.h>
#include <fcntl.h>


int buzzer_fd = 0;
char buffer1 = 1;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * hello_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{

	if(!strcmp(argv[1], "-h")){
		printf("Hello, World!!\n");
		return OK;
	}else if (!strcmp(argv[1], "-o")) {

		buzzer_fd = open("/dev/buzzer1",O_RDWR);
		printf("open the buzzer1 \n");

		char buffer = 1;
		int wr = write(buzzer_fd,&buffer,sizeof(buffer));
		sleep(2);

		buffer = 0;
		wr = write(buzzer_fd,&buffer,sizeof(buffer));
		sleep(2);

		buffer = 1;
		wr = write(buzzer_fd,&buffer,sizeof(buffer));
		sleep(2);

		buffer = 0;
		wr = write(buzzer_fd,&buffer,sizeof(buffer));
		sleep(2);

		int cr = close(buzzer_fd);
		return OK;
	}
	else if (!strcmp(argv[1], "-cw")){
			int can_fd = open("/dev/can3",O_RDWR);

			char can_buffer[10] = "write can";
			write(can_fd,&can_buffer,sizeof(can_buffer));

			close(can_fd);
			return OK;
	}
	else if (!strcmp(argv[1], "-cr")){
			int can_fd = open("/dev/can3",O_RDWR);

			char can_buffer[10];
			read(can_fd,&can_buffer,sizeof(can_buffer));

			close(can_fd);

			printf(can_buffer);
			printf("\n");
			return OK;
	}
	else{
		printf("Hello, World!!\n");
		return OK;
	}


}
