/************************************************************************************
 * include/nuttx/buzzer/buzzer.h
 *
 *   Copyright (C) 2008, 2009, 2011-2012, 2015-2017 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef NUTTX_INCLUDE_NUTTX_BUZZER_BUZZER_H_
#define NUTTX_INCLUDE_NUTTX_BUZZER_BUZZER_H_

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_CAN_TXREADY
#  include <nuttx/wqueue.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/


/************************************************************************************
 * type
 ************************************************************************************/
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif


/************************************************************************************
 * Public Types
 ************************************************************************************/
//struct buzzer_dev_s
//{
//	uint8_t		mode;
//	struct gpio_dev_s *gpio;
//
//};


/************************************************************************************
 * Public Data
 ************************************************************************************/



//int buzzerinitialize(FAR const char *path);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* NUTTX_INCLUDE_NUTTX_BUZZER_BUZZER_H_ */
