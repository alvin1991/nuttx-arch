
/******************************************************************************
 * include/nuttx/sensors/mmc5883.h
 *
 *   Copyright (C) 2017-2018 RAF Research LLC. All rights reserved.
 *   Author: Bob Feretich <bob.feretich@rafresearch.com>
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
 ******************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/cluster_driver.h>

#include "../../../../apps/include/uORB/uorb/uORB.h"
#include "../../../../apps/include/uORB/topic/mmc5883_uorb.h"

/**************************************************************************
I2C ADDRESS/BITS
**************************************************************************/
#define MMC5883_I2C_DEFAULT_ADDRESS                (0x30)	   // 7bits address (0x60 = 0x30<<1 + 0:read) (write:0x61 = 0x19<<1 + 1:write)
#define MMC5883_I2C_STANDARD_FREQUENCY             (100000)	   // SCL Clock Frequency
#define MMC5883_I2C_FAST_FREQUENCY                 (400000)	   // SCL Clock Frequency

/**************************************************************************
REGISTERS DESCRIPTION
**************************************************************************/
#define MMC5883MA_REG_XOUT_LOW      	0x00
#define MMC5883MA_REG_XOUT_HIGH     	0x01
#define MMC5883MA_REG_YOUT_LOW      	0x02
#define MMC5883MA_REG_YOUT_HIGH     	0x03
#define MMC5883MA_REG_ZOUT_LOW      	0x04
#define MMC5883MA_REG_ZOUT_HIGH     	0x05
#define MMC5883MA_REG_TEMPERATURE   	0x06
#define MMC5883MA_REG_STATUS        	0x07
#define MMC5883MA_REG_CONTROL_0     	0x08
#define MMC5883MA_REG_CONTROL_1     	0x09
#define MMC5883MA_REG_CONTROL_2     	0x0A
#define MMC5883MA_REG_X_THRESHOLD   	0x0B
#define MMC5883MA_REG_Y_THRESHOLD   	0x0C
#define MMC5883MA_REG_Z_THRESHOLD   	0x0D
#define MMC5883MA_REG_PRODUCT_ID    	0x2F

#define MMC5883MA_CMD_CTRL0_TM_M      	(1 << 0)
#define MMC5883MA_CMD_CTRL0_TM_T      	(1 << 1)
#define MMC5883MA_CMD_CTRL0_START_MDT 	(1 << 2)
#define MMC5883MA_CMD_CTRL0_SET       	(1 << 3)
#define MMC5883MA_CMD_CTRL0_RESET     	(1 << 4)
#define MMC5883MA_CMD_CTRL0_OTP_READ  	(1 << 6)

#define MMC5883MA_CMD_CTRL1_BW_100    	(0b00 << 0)	//Measurement Time 100hz(10ms)
#define MMC5883MA_CMD_CTRL1_BW_200    	(0b01 << 0) //Measurement Time 200hz(5ms)
#define MMC5883MA_CMD_CTRL1_BW_400    	(0b10 << 0) //Measurement Time 400hz(2.5ms)
#define MMC5883MA_CMD_CTRL1_BW_600    	(0b11 << 0) //Measurement Time 600hz(1.6ms)
#define MMC5883MA_CMD_CTRL1_X_INHIBIT 	(1 << 2)
#define MMC5883MA_CMD_CTRL1_Y_INHIBIT 	(1 << 3)
#define MMC5883MA_CMD_CTRL1_Z_INHIBIT 	(1 << 4)
#define MMC5883MA_CMD_CTRL1_SW_RST    	(1 << 7)

#define MMC5883MA_CMD_CTRL2_FREQ_OFF         (0x00) // assumes BW[1:0] == 0, off = motion detector off
#define MMC5883MA_CMD_CTRL2_FREQ_14HZ        (0x01) // Continuous output frequence 14hz
#define MMC5883MA_CMD_CTRL2_FREQ_5HZ         (0x02) // Continuous output frequence 5hz
#define MMC5883MA_CMD_CTRL2_FREQ_2_2HZ       (0x02) // Continuous output frequence 2.2hz
#define MMC5883MA_CMD_CTRL2_FREQ_1HZ         (0x04) // Continuous output frequence 1hz
#define MMC5883MA_CMD_CTRL2_FREQ_0_5HZ       (0x05) // Continuous output frequence 1/2hz
#define MMC5883MA_CMD_CTRL2_FREQ_0_25HZ      (0x06) // Continuous output frequence 1/4hz
#define MMC5883MA_CMD_CTRL2_FREQ_0_125HZ     (0x07) // Continuous output frequence 1/8hz
#define MMC5883MA_CMD_CTRL2_FREQ_0_0625HZ    (0x08) // Continuous output frequence 1/16hz
#define MMC5883MA_CMD_CTRL2_FREQ_0_03125HZ   (0x09) // Continuous output frequence 1/32hz
#define MMC5883MA_CMD_CTRL2_FREQ_0_015625HZ  (0x09) // Continuous output frequence 1/64hz
#define MMC5883MA_CMD_CTRL2_INT_MDT_EN       (1 << 5)
#define MMC5883MA_CMD_CTRL2_INT_MEAS_DONE_EN (1 << 6)

#define MMC5883MA_DEVICE_OK				 	 (MMC5883MA_VAL_STATUS_MEAS_M_DONE|MMC5883MA_VAL_STATUS_MEAS_T_DONE|MMC5883MA_VAL_STATUS_OTP_READY)
#define MMC5883MA_VAL_STATUS_MEAS_M_DONE 	 (1 << 0)
#define MMC5883MA_VAL_STATUS_MEAS_T_DONE 	 (1 << 1)
#define MMC5883MA_VAL_STATUS_MOTION_DECT 	 (1 << 2)
#define MMC5883MA_VAL_STATUS_PUMP_ON 	 	 (1 << 3)
#define MMC5883MA_VAL_STATUS_OTP_READY 	 	 (1 << 4)

#define MMC5883MA_VAL_OFFSET				32768

/*   mmc5883 ioctl definitions */
#define MMC5883IOC_MEASUREMENT_BANDWIDTH_SET	_SNIOC(1)
#define MMC5883IOC_CONTINUOUS_FREQ_SET			_SNIOC(2)
#define MMC5883IOC_MEASUREMENT_RATE_SET			_SNIOC(3)
#define MMC5883IOC_SOFTWARE_RESET				_SNIOC(4)
#define MMC5883IOC_MEASUREMENT_START			_SNIOC(5)
#define MMC5883IOC_MEASUREMENT_STOP				_SNIOC(6)

struct mmc5883_sample_s
{
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t temp;
};

enum
{
	OMR1 = 0,
	OMR2,
	REG_MAX
};

struct mmc5883_reg_pair_s  /* Utility struct for the below... */
{
  uint8_t addr;            /* SPI register address */
  uint8_t value;           /* Value to be stored in the above reg on open() */
};

/* A reference to a structure of this type must be passed to the HMC6343 driver when the
 * driver is instantiated. This structure provides information about the configuration of the
 * HMC6343 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied by the driver
 * and is presumed to persist while the driver is active. The memory must be writeable
 * because, under certain circumstances, the driver may modify the frequency.
 */

struct mmc5883_config_s
{
  /* Device characterization */

  uint8_t address;     							/* 7-bit I2C address (only bits 0-6 used) */

  uint32_t frequency;  							/* I2C frequency */

  struct mmc5883_reg_pair_s *initial_cr_values; /* Utility struct for the below... */
  uint8_t initial_cr_lens; 						/* mmc5883_reg_pair_s lenths */

  struct sensor_cluster_operations_s *sc_ops; 	/* Cluster driver operations interface */
};


/* This structure represents the state of the HMC6343 driver */

struct mmc5883_dev_s
{
  /* Common fields */

  FAR struct mmc5883_config_s *config; /* Board configuration data */
  FAR struct i2c_master_s *i2c;        /* Saved I2C driver instance */

  /* work queue fields */
  struct work_s _work;                  /* Supports the interrupt handling "bottom half" */
  bool 				_running;		   /* true: measure in work queue false:not measure*/
  unsigned			_mesure_ticks;     /* cycle frequency in work queue  */

  /* semaphore fields */
  sem_t exclsem;                       /* Manages exclusive access to this structure */
  sem_t waitsem;                       /* Used to wait for the availability of data */

  /* status fields */
  uint8_t status;                      /* See HMC6343_STAT_* definitions */
  uint8_t crefs;                       /* Number of times the device has been opened */
  uint8_t nwaiters;                    /* Number of threads waiting for HMC6343 data */

  uint16_t ofsx;                       /* Offset X value */
  uint16_t ofsy;                       /* Offset Y value */
  uint16_t ofsz;                       /* Offset Z value */

  struct mmc5883_sample_s sample;      /* Last sampled Compass data */

  /* ORB data */
  orb_advert_t uorb_pub;			   /* ORB topic advertiser handle  */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: mmc5883_register
 *
 * Description:
 *  This function will register the touchscreen driver as /dev/accelN where N
 *  is the minor device number
 *
 * Input Parameters:
 *   handle    - The handle previously returned by mmc5883_register
 *   minor     - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int mmc5883_register(FAR struct i2c_master_s *dev, int minor);


