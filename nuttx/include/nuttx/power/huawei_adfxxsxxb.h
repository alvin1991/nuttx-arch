/****************************************************************************
 * include/nuttx/power/huawei_adfxxsxxb.h
 *
 *   Author: caihe
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

//#ifndef __INCLUDE_NUTTX_POWER_HUAWEI_ADFXXSXXB_H
//#define __INCLUDE_NUTTX_POWER_HUAWEI_ADFXXSXXB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/* Configuration
 * CONFIG_I2C - Enables support for I2C drivers
 * CONFIG_I2C_HUAWEI_ADFXXSXXB - Enables support for the NCP5623C driver
 */

//#if defined(CONFIG_I2C) && defined(CONFIG_HUAWEI_ADFXXSXXB)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C definitions */

#define I2C_BUS_FREQ_HZ        (100000)

/* adfxxsxxb register addresses */

#define ADFXXSXXB_RGS_OPERATION      (0x01)      /* POWER ON/OFF */
#define ADFXXSXXB_RGS_CLEAR_FAULTS   (0x03)      /* clare faults */
#define ADFXXSXXB_RGS_VOUT_MODE      (0x20)      /* determine the data type and parameters using PMBus cmmand */
#define ADFXXSXXB_RGS_VOUT_CMD       (0x21)      /* set output voltage */
#define ADFXXSXXB_RGS_ALARM          (0x51)      /*  */
#define ADFXXSXXB_RGS_STATUS         (0x79)      /* status */
#define ADFXXSXXB_RGS_READ_VIN       (0x88)      /* read the input voltage */
#define ADFXXSXXB_RGS_READ_VOUT      (0x8B)      /* read the output voltage */
#define ADFXXSXXB_RGS_READ_IOUT      (0x8C)      /* read the ouput current */
#define ADFXXSXXB_RGS_READ_TEMP		 (0X8D)		 /* read the device temperature*/
#define ADFXXSXXB_RGS_READ_POUT		 (0X96)		 /*read the power output */
#define ADFXXSXXB_RGS_READ_PIN		 (0X97)		 /*read the power input*/
#define ADFXXSXXB_RGS_PMBUS_REVISION (0X98)		 /**/
#define ADFXXSXXB_RGS_MFR_STATUS	 (0XE9)
#define ADFXXSXXB_RGS_WRITE_TIME	 (0XEC)		
#define ADFXXSXXB_RGS_READ_ACDROP	 (0XEF)


/****************************************************************************
 * Public Types
 ****************************************************************************/


/* This structure is used in an IOCTL command for setting the PWM of an individual
 * LED. The desired LED is selected by setting the 'led' parameter accordingly
 * whereas the 'led_pwm' field governs the brightness of the selected LED. A value
 * of 0 (0x00) leads to a duty cycle of 0 % = LED off while a value of 255 (0xFF)
 * leads to a duty cycle of 99.6 % = Maximum brightness.
 */



/****************************************************************************
 * Forward declarations
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ncp5623c_register
 *
 * Description:
 *   Register the NCP5623C device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/leddrv0".
 *   i2c     - An instance of the I2C interface to use to communicate
 *             with the LM92.
 *   ncp5623c_i2c_addr
 *           - The I2C address of the NCP5623C.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ncp5623c_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                      uint8_t const ncp5623c_i2c_addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

int adfxxsxxb_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

//#endif /* CONFIG_I2C && CONFIG_I2C_NCP5623C */
//#endif /* __INCLUDE_NUTTX_LEDS_NCP5623C_H */

