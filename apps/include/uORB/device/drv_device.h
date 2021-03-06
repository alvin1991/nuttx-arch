/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

/**
 * @file drv_device.h
 *
 * Generic device / sensor interface.
 */

#ifndef _DRV_DEVICE_H
#define _DRV_DEVICE_H

#include <stdint.h>
#include <sys/ioctl.h>

#ifndef SIOCDEVPRIVATE
#define SIOCDEVPRIVATE 1
#endif

#ifndef DIOC_GETPRIV
#define DIOC_GETPRIV    SIOCDEVPRIVATE
#endif

/*
 * ioctl() definitions
 */

#define _DF_IOC(x,y) _IOC(x,y)


#define _DEVICEIOCBASE          (0x100)
#define _DEVICEIOC(_n)          (_DF_IOC(_DEVICEIOCBASE, _n))

/** ask device to stop publishing */
#define DEVIOCSPUBBLOCK _DEVICEIOC(0)

/** check publication block status */
#define DEVIOCGPUBBLOCK _DEVICEIOC(1)

/**
 * Return device ID, to enable matching of configuration parameters
 * (such as compass offsets) to specific sensors
 */
#define DEVIOCGDEVICEID _DEVICEIOC(2)

#endif /*_DRV_DEVICE_H*/
