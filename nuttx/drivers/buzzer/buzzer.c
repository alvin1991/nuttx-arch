/****************************************************************************
 * drivers/buzzer/buzzer.c
 *
 *   Copyright (C) 2008-2009, 2011-2012, 2014-2015, 2017 Gregory Nutt. All
 *     rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Copyright (C) 2016 Omni Hoverboards Inc. All rights reserved.
 *   Author: Paul Alexander Patience <paul-a.patience@polymtl.ca>
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

#include <sys/types.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <fcntl.h>
#include <assert.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/buzzer/buzzer.h>
#include <nuttx/kmalloc.h>


#ifdef CONFIG_CAN_TXREADY
#  include <nuttx/wqueue.h>
#endif

#include <nuttx/irq.h>


/* Character driver methods */
static int            buzzer_open(FAR struct file *filep);
static int            buzzer_close(FAR struct file *filep);
static ssize_t        buzzer_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen);
static ssize_t buzzer_read(FAR struct file *filep,
								FAR char *buffer, size_t buflen);


/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_buzzerops =
{
  buzzer_open,  /* open */
  buzzer_close, /* close */
  buzzer_read,  /* read */
  buzzer_write, /* write */
  NULL,      /* seek */
  NULL,  	 /* ioctl */
  NULL,		 /* poll */
  NULL       /* unlink */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: buzzer_open
 *
 * Description:
 *   This function is called whenever the buzzer device is opened.
 *
 ****************************************************************************/
static int buzzer_open(FAR struct file *filep)
{
	return  OK;
}


static int buzzer_close(FAR struct file *filep)
{
	return  OK;
}


static ssize_t buzzer_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
	return  OK;
}


static ssize_t buzzer_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen)
{
	return  OK;
}




/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: buzzerinitialize
 *
 * Description:
 *   Initialize the buzzer
 *
 * Input Parameters:
 *
 *
 * Returned Value:
 *   Valid buzzer device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

int buzzer_register(FAR const char *path,struct gpio_dev_s *dev)
{
	return OK;
}

