/****************************************************************************
 * drivers/power/huawei_adfxxsxxb.c
 * based on drivers/leds/ncp5623c.c
 *
 *   Copyright (C) 2019 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *
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

/* The adfxxsxxbw(adf18s28b,adf9s48b) are Huawei AC-DC Power Management.
 * It can be configured and read through PMBus(IIC)
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/power/huawei_adfxxsxxb.h>
#include <nuttx/power/acdc_power.h>
#define AC_POWER_ERR	_err
#define MAX_IIC_DATA_LONG		5

enum ADFXXSXXB_DATA_TYPE
{
  ADFXXSXXB_DATA_LINEAR11  = 0,   			/* Linear 11 */
  ADFXXSXXB_DATA_LINEAR16,   				/* Linear 16 */
  ADFXXSXXB_DATA_CMD,   					/* Just cmd ,no data */
  ADFXXSXXB_DATA_1BYTE,   					/* one byte */
};
/* This driver requires:
 *
 * 
 * CONFIG_I2C - I2C support
 * 
 */

//I2C_ACDC_POWER_REST
//I2C_ACDC_POWER_OUTPUT_VOLTAGE
//HUAWEI_ADFXXSXXB
//I2C_ACDC_POWER



#if defined(CONFIG_I2C) && defined(CONFIG_HUAWEI_ADFXXSXXB)

typedef union
{
	float         fdata;
	unsigned int  u_intdata;
	int           s_intdata;
    void          *point;
	unsigned char str[4];
}Float_bytes_type;

struct adfxxsxxb_dev_s
{

};


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* I2C support */
/*
static int bq2425x_getreg8(FAR struct bq2425x_dev_s *priv, uint8_t regaddr,
                             FAR uint8_t *regval);
static int bq2425x_putreg8(FAR struct bq2425x_dev_s *priv, uint8_t regaddr,
                             uint8_t regval);

static inline int bq2425x_getreport(FAR struct bq2425x_dev_s *priv,
                                    uint8_t *report);
static inline int bq2425x_reset(FAR struct bq2425x_dev_s *priv);
static inline int bq2425x_watchdog(FAR struct bq2425x_dev_s *priv, bool enable);
static inline int bq2425x_powersupply(FAR struct bq2425x_dev_s *priv, int current);
static inline int bq2425x_setvolt(FAR struct bq2425x_dev_s *priv, int volts);
static inline int bq2425x_setcurr(FAR struct bq2425x_dev_s *priv, int current);
*/
/* Battery driver lower half methods */
/*
static int bq2425x_state(struct battery_charger_dev_s *dev, int *status);
static int bq2425x_health(struct battery_charger_dev_s *dev, int *health);
static int bq2425x_online(struct battery_charger_dev_s *dev, bool *status);
static int bq2425x_voltage(struct battery_charger_dev_s *dev, int value);
static int bq2425x_current(struct battery_charger_dev_s *dev, int value);
static int bq2425x_input_current(struct battery_charger_dev_s *dev, int value);
static int bq2425x_operate(struct battery_charger_dev_s *dev, uintptr_t param);
*/
/****************************************************************************
 * Private Data
 ****************************************************************************/
/*
static const struct adfxxsxxb_ops g_adfxxsxxb_ops =
{
	
  bq2425x_state,
  bq2425x_health,
  bq2425x_online,
  bq2425x_voltage,
  bq2425x_current,
  bq2425x_input_current,
  bq2425x_operate
  
};
*/

/****************************************************************************
 * Name: adfxxsxxb_get_data
 *
 * Description:
 *   Read a Linear 11 data value from a adfxxsxxb register pair.
 *
 *   START <I2C write address> ACK <Reg address> ACK
 *   REPEATED-START <I2C read address> ACK Data0 ACK Data1 NO-ACK STOP
 *
 ****************************************************************************/

static int adfxxsxxb_get_data(FAR struct acdc_power_dev_s *priv, uint8_t regaddr,
                           Float_bytes_type *regval, uint8_t reg_type)
{
	int data_len = 0;
	uint32_t temp_a,temp_b;
	Float_bytes_type temp;
  struct i2c_config_s config;
  uint8_t buffer[MAX_IIC_DATA_LONG];
  int ret = -EBADF;

  if (reg_type == ADFXXSXXB_DATA_LINEAR11)
  {
	  data_len = 3;
  }
  else if (reg_type == ADFXXSXXB_DATA_LINEAR16)
  {
	  data_len = 4;
  }
  else if (reg_type == ADFXXSXXB_DATA_CMD)
  {
	  data_len = 1;
  }
  else if (reg_type == ADFXXSXXB_DATA_1BYTE)
  {
	  data_len = 2;
  }
  else
  {
	  return -EBADF;
  }
  data_len = data_len;
  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address   = priv->i2c_addr;
  config.addrlen   = 7;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
  //ret = -1;
  //usleep(10000);
  if (ret < 0)
    {
      AC_POWER_ERR("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 8-bits from the register */
  
  //ret = i2c_read(priv->i2c, &config, buffer, data_len);
  ret = 0;
  regval->fdata = 0;
  if (ret < 0)
    {
      AC_POWER_ERR("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }
  if (reg_type == ADFXXSXXB_DATA_LINEAR11)
  {
	  temp.str[0] = buffer[0];
	  temp.str[1] = buffer[1];
	  temp_a = temp.u_intdata;
	  temp_b = temp_a;
	  temp_a &= 0x7f;
	  temp_b >>= 10;


	  regval->fdata = temp_a;
  }
  else if (reg_type == ADFXXSXXB_DATA_LINEAR16)
  {
	  
  }
  else if (reg_type == ADFXXSXXB_DATA_CMD)
  {
	  
  }
  else if (reg_type == ADFXXSXXB_DATA_1BYTE)
  {
	  
  }

  /* Copy 8-bit value to be returned */

  return OK;
}

/****************************************************************************
 * Name: adfxxsxxb_send_data
 *
 * Description:
 *   Write a 8-bit value to a BQ2425x register pair.
 *
 *   START <I2C write address> ACK <Reg address> ACK Data0 ACK Data1 ACK STOP
 *
 ****************************************************************************/

static int adfxxsxxb_send_data(FAR struct acdc_power_dev_s *priv, uint8_t regaddr,
                           Float_bytes_type regval, uint8_t reg_type)
{
	int data_len = 0;
  struct i2c_config_s config;
  uint8_t buffer[MAX_IIC_DATA_LONG];

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address   = priv->i2c_addr;
  config.addrlen   = 7;


  /* Set up a 3 byte message to send */

  buffer[0] = regaddr;
  if (reg_type == ADFXXSXXB_DATA_LINEAR11)
  {
	  buffer[1] = regval.fdata;
	  data_len = 3;
  }
  else if (reg_type == ADFXXSXXB_DATA_LINEAR16)
  {
	  buffer[1] = regval.str[0];
	  buffer[2] = regval.str[1];
	  buffer[3] = regval.str[2];
	  data_len = 4;
  }
  else if (reg_type == ADFXXSXXB_DATA_CMD)
  {
	  data_len = 1;
  }
  else if (reg_type == ADFXXSXXB_DATA_1BYTE)
  {
	  buffer[1] = regval.str[0];
	  data_len = 2;
  }
  else
  {
	  return -EBADF;
  }
  
  /* Write the register address followed by the data (no RESTART) */

  return i2c_write(priv->i2c, &config, buffer, data_len);
}


/****************************************************************************
 * Name: acdc_power_ioctl
 *
 *
 ****************************************************************************/

int adfxxsxxb_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
	
	FAR struct inode        *inode = filep->f_inode;
	FAR struct acdc_power_dev_s *priv_adfxxsxxb = inode->i_private;
	//FAR struct adfxxsxxb_dev_s *acdc = priv_adfxxsxxb->;
	FAR struct power_acdc_status_s *read_params;
	Float_bytes_type regval;
	int ret = OK;

	read_params = (struct power_acdc_status_s *)arg; 

	switch (cmd)
    {
		case ACDC_IOC_READ_VOUT://power on
		{
			ret =  adfxxsxxb_get_data(priv_adfxxsxxb, ADFXXSXXB_RGS_READ_VOUT,&regval, ADFXXSXXB_DATA_LINEAR16);
			if (ret < 0)
			{
				
			}
			read_params->vol_out = regval.fdata;
		}
		break;
	case ACDC_IOC_POWER_ON:
		{
			regval.u_intdata = 0;
			ret =  adfxxsxxb_send_data(priv_adfxxsxxb, ADFXXSXXB_RGS_VOUT_CMD,regval, ADFXXSXXB_DATA_LINEAR16);
			if (ret < 0)
			{
				
			}
		}
		break;
	case ACDC_IOC_POWER_OFF:
		{

		}
		break;
	case ACDC_IOC_RESET:
		{

		}
		break;
	case ACDC_IOC_SET_VOL:
		{

		}
		break;
	case ACDC_IOC_READ_VIN:
		{
			ret =  adfxxsxxb_get_data(priv_adfxxsxxb, ADFXXSXXB_RGS_READ_VIN,&regval, ADFXXSXXB_DATA_LINEAR11);
			if (ret < 0)
			{
				syslog(LOG_ERR,"huawei_adfxxsxxb:ERROR: RVIN failed: %d\n", ret );
			}
			read_params->vol_in = regval.fdata;
		}
		break;
	case ACDC_IOC_READ_POUT:
		{

		}
		break;
	case ACDC_IOC_READ_PIN:
		{

		}
		break;
	case ACDC_IOC_READ_IOUT:
		{

		}
		break;
	case ACDC_IOC_READ_TEMP:
		{

		}
		break;

		/* The used ioctl command was invalid */

		default:
		{
			lcderr("ERROR: Unrecognized cmd: %d\n", cmd);
			ret = -ENOTTY;
		}
		break;
    }
	return ret;
}

#endif //defined(CONFIG_I2C) && defined(CONFIG_I2C_HUAWEI_ADFXXSXXB)
