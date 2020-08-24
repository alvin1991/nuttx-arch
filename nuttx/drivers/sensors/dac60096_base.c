/****************************************************************************
 * drivers/sensors/dac60096_base.c
 *
 *   Copyright (C) 2019-2020 Alivn Peng. All rights reserved.
 *   Author: Alivn Peng <alivn.peng@gmail.com.com>
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

#include <errno.h>
#include <debug.h>
#include <string.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <nuttx/config.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

#if defined(CONFIG_SENSORS_DAC60096)
#include <nuttx/sensors/dac60096.h>
#include <nuttx/sensors/dac60096_spi.h>


static int      dac60096_dvr_open(FAR void *instance_handle, int32_t arg);
static int      dac60096_dvr_close(FAR void *instance_handle, int32_t arg);
static ssize_t  dac60096_dvr_read(FAR void *instance_handle,
                                 FAR char *buffer, size_t buflen);
static ssize_t  dac60096_dvr_write(FAR void *instance_handle,
                                  FAR const char *buffer, size_t buflen);
static off_t    dac60096_dvr_seek(FAR void *instance_handle, off_t offset,
                                 int whence);
static int      dac60096_dvr_ioctl(FAR void *instance_handle, int cmd,
                                  unsigned long arg);
static void 	dac60096_dvr_exchange(FAR void *instance_handle,
                                 FAR const void *txbuffer,
                                 FAR void *rxbuffer, size_t nwords);
/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct dac60096_dvr_entry_vector_s g_dac60096_dops =
{
  /* Standard sensor cluster driver entry-vector */

  {
    .driver_open    = dac60096_dvr_open,
    .driver_close   = dac60096_dvr_close,
    .driver_read    = dac60096_dvr_read,
    .driver_write   = dac60096_dvr_write,
    .driver_seek    = dac60096_dvr_seek,
    .driver_ioctl   = dac60096_dvr_ioctl,
    .driver_suspend = 0,
    .driver_resume  = 0,
    },

  /* dac60096 extensions follow */

  .driver_spiexc = dac60096_dvr_exchange,
  .select		 = dac60096_select,			// !!! This function need to implement by board file.
};


/****************************************************************************
 * Private data storage
 ****************************************************************************/

/* Default accelerometer initialization sequence */

/* Synchronous Mode:
 * 		1. Safety power up, read the PWRM Register and return 0xABBA or 0xBAAB.
 * 		2. Ensure the TRIGG pin is left fixed(DC Mode).Only one data registers is used to control DAC output.
 * 		3. Ensure the LDAC pin is held high(Synchronous Mode).In the mode,data registers not output immediately.
 * 		4. Write data to 96-cells(data registers).
 * 		5. Generate a high-to-low transition on LDAC pin and cse all 96 DACs update at same time.(Set CON Registers bit 15 at CS valid.)
 * 		6. Set Filter TAP = 128,and delay sometimes for filter config.
 */

/*
 * DAC60096 Register Configration A
 */
struct dac60096_reg_pair_s g_reg_pairs_a[] =
{
	{DAC60096_REG_CON,\
			DAC60096_CON_SDO2x|						//<<< 2x drive strength
			DAC60096_CON_SDRV_HIZ|					//<<< Hi-Z. STATS pin is disabled
			DAC60096_CON_PHAINV_SCLK_NEG_EDGE|		//<<< SCLK NegEdge
			DAC60096_CON_CLRDAC_NORMAL|				//<<< Normal operating state
			DAC60096_CON_APB_AUTO_ENABLE			//<<< Enable auto populate B feature
	},
};

static struct dac60096_config_s g_dac60096_config_a =
{
	.spi_devid 				= SPIDEV_USER(0),
	.initial_cr  			= g_reg_pairs_a,
	.initial_cr_len			= sizeof(g_reg_pairs_a)/sizeof(struct dac60096_reg_pair_s),
	.enable_atti_delta		= FALSE,
	.sc_ops 				= &g_dac60096_dops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dac60096_soft_reset
 *
 * Description:
 *
 *   Soft reset the DAC60096.
 *
 ****************************************************************************/

static int dac60096_soft_reset(FAR struct dac60096_dev_s *dev)
{
	DEBUGASSERT(dev != NULL);

	for(int j = 0; j < 4; j++){
		// Write sub-system
		dac60096_spi_write(dev,DAC60096_WRITE_REG_SIGNAL(DAC60096_REG_PTR),BIT(j,12));

		// Soft reset the DAC60096.
		dac60096_spi_write(dev,DAC60096_WRITE_REG_SIGNAL(DAC60096_REG_SWR),DAC60096_SWR_SOFT_RESET);
	}

	return OK;
}

/****************************************************************************
 * Name: dac60096_clear_state
 *
 * Description:
 *
 *   Clear the DACs state.
 *
 ****************************************************************************/

static int dac60096_clear_state(FAR struct dac60096_dev_s *dev)
{
	DEBUGASSERT(dev != NULL);

	uint16_t reg = 0;

	for(int j = 0; j < 4; j++){
		// Write sub-system
		dac60096_spi_write(dev,DAC60096_WRITE_REG_SIGNAL(DAC60096_REG_PTR),BIT(j,12));

		// Readback the CONR
		reg = dac60096_spi_read(dev,DAC60096_READ_REG_SIGNAL(DAC60096_REG_CON));

		// Clear the DACs state
		reg |= DAC60096_CON_CLRDAC_CLEAR;
		dac60096_spi_write(dev,DAC60096_WRITE_REG_SIGNAL(DAC60096_REG_CON),reg);
	}

	return OK;
}

/****************************************************************************
 * Name: dac60096_config_default
 *
 * Description:
 *
 *   Initialize the DAC60096 CONR in default configrations.
 *
 ****************************************************************************/

static int dac60096_config_default(FAR struct dac60096_dev_s *dev)
{
	DEBUGASSERT(dev != NULL);

	// configuration CONR of the sub-system.
	struct dac60096_reg_pair_s *drp = dev->config->initial_cr;

	for(int i = 0; i < dev->config->initial_cr_len; i++){
		for(int j = 0; j < 4; j++){
			// Write sub-system
			dac60096_spi_write(dev,DAC60096_WRITE_REG_SIGNAL(DAC60096_REG_PTR),BIT(j,12));

			// Config CONR
			dac60096_spi_write(dev,DAC60096_WRITE_REG_SIGNAL(drp->addr),drp->value);
		}
	}
	return OK;
}

/****************************************************************************
 * Name: dac60096_output_volt
 *
 * Description:
 *   Enable the DAC60096 output voltage.
 *
 * Input Parameters:
 *   dev  - device pointer.
 *
 * Returned Value:
 *   powerstatus.
 *
 ****************************************************************************/
uint16_t dac60096_output_volt(struct dac60096_dev_s *dev)
{
	DEBUGASSERT(dev != NULL);

	// configuration CONR of the sub-system.
	struct dac60096_reg_pair_s *drp = dev->config->initial_cr;

	for(int j = 0; j < 4; j++){
		// Write sub-system
		dac60096_spi_write(dev,DAC60096_WRITE_REG_SIGNAL(DAC60096_REG_PTR),BIT(j,12));

		// Issues an LDAC trigger at CS rising edge
		dac60096_spi_write(dev,DAC60096_WRITE_REG_SIGNAL(drp->addr),drp->value|DAC60096_CON_LDAC_TRIGGER);
	}
	return OK;
}

/****************************************************************************
 * Name: dac60096_set_channel_value
 *
 * Description:
 *   Set the DAC60096 output voltage.
 *
 * Input Parameters:
 *   dev  - device pointer.
 *
 * Returned Value:
 *   powerstatus.
 *
 ****************************************************************************/
int16_t dac60096_set_channel_value(struct dac60096_dev_s *dev,int32_t cv)
{
	DEBUGASSERT(dev != NULL);
	struct dac60096_chan_value_s *dcv =(struct dac60096_chan_value_s *)cv;

	uint16_t ch = dcv->channel;
	int16_t val = dcv->value;

	if(!CHANNEL_CHECK(ch) || !VALUE_CHECK(val)){
		return -EINVAL;
	}else{
		uint16_t ptrr = 0;
		if(ch < 24)		{ptrr = DAC60096_PTR_SID_00 | (ch%24);}
		else if(ch < 48){ptrr = DAC60096_PTR_SID_01 | (ch%24);}
		else if(ch < 70){ptrr = DAC60096_PTR_SID_02 | (ch%24+2);}
		else if(ch ==88){ptrr = DAC60096_PTR_SID_03 | 19;}	//ToDO Hard error!!!!
		else if(ch ==89){ptrr = DAC60096_PTR_SID_03 | 18;}	//ToDO Hard error!!!!
		else			{ptrr = DAC60096_PTR_SID_03 | ((ch-70)%26);}
		uint16_t data = (uint16_t)(val<0?(val+4096):val);
		dac60096_spi_write(dev,DAC60096_WRITE_REG_SIGNAL(DAC60096_REG_PTR),ptrr);
		dac60096_spi_write(dev,DAC60096_WRITE_REG_SIGNAL(DAC60096_REG_BUF_A),data<<4);
	}

	return OK;
}

/****************************************************************************
 * Name: dac60096_get_channel_value
 *
 * Description:
 *   Get the DAC60096 output voltage.
 *
 * Input Parameters:
 *   dev  - device pointer.
 *
 * Returned Value:
 *   powerstatus.
 *
 ****************************************************************************/
int16_t dac60096_get_channel_value(struct dac60096_dev_s *dev,int16_t ch)
{
	DEBUGASSERT(dev != NULL);

	if(!CHANNEL_CHECK(ch)){
		return -EINVAL;
	}else{
		uint16_t ptrr = (ch%24)<<12 | DAC60096_REG_BUF_A;
		dac60096_spi_write(dev,DAC60096_WRITE_REG_SIGNAL(DAC60096_REG_PTR),ptrr);
		return dac60096_spi_read(dev,DAC60096_WRITE_REG_SIGNAL(DAC60096_REG_BUF_A));
	}
}

/****************************************************************************
 * Name: dac60096_get_powerstatus
 *
 * Description:
 *   Read the DAC60096's power monitor registers via SPI
 *
 * Input Parameters:
 *   dev  - device pointer.
 *
 * Returned Value:
 *   powerstatus.
 *
 ****************************************************************************/
uint16_t dac60096_get_powerstatus(struct dac60096_dev_s *dev)
{
	DEBUGASSERT(dev != NULL);

	/* Any register write to PWRM sets PWRM to 0xABBA. */
	dac60096_spi_write(dev,DAC60096_WRITE_REG_SIGNAL(DAC60096_REG_PWRM),0);

	/* Read PWRM Registers */
	return dac60096_spi_read(dev,DAC60096_READ_REG_SIGNAL(DAC60096_REG_PWRM));
}


///////////////////////////driver operation functions////////////////////////

/****************************************************************************
 * Name: dac60096_dvr_open
 *
 * Description:
 *
 *   open the device.
 *   Check the device status is ok!
 *   1)power-on.
 *   2)Wait 800ms.
 *   3)Wait until NOT_READY bit goes to 0.
 *   4)Confirm HARD_ERR bits.
 *
 ****************************************************************************/
static int dac60096_dvr_open(FAR void *instance_handle, int32_t arg)
{
  FAR struct dac60096_dev_s *priv = (FAR struct dac60096_dev_s *)instance_handle;
  int ret = 0;

  DEBUGASSERT(priv != NULL);
  UNUSED(arg);

  sninfo("dac60096_open: entered...\n");

  /* Soft reset the device */
  dac60096_dvr_ioctl(priv,DACIOC_SOFT_RESET,0);

  /* Wait 800ms,the device get in stable. */
  usleep(DAC60096_VAL_WAIT_US_STABLE);

  /* check the device power-on */
  uint16_t ps = dac60096_get_powerstatus(priv);

  if(DAC60096_PWRM_POWER_FAILED == ps || (DAC60096_PWRM_POWER_UP1 != ps && DAC60096_PWRM_POWER_UP2 != ps) ){
	  snerr("ERROR:dac60096 not ready...\n");
	  return ERROR;
  }

  /* Clear the device's state */
  dac60096_dvr_ioctl(priv,DACIOC_CLEAR_STATE,0);

  return ret;
}

/****************************************************************************
 * Name: dac60096_dvr_close
 ****************************************************************************/

static int dac60096_dvr_close(FAR void *instance_handle, int32_t arg)
{
  FAR struct dac60096_dev_s *priv = (FAR struct dac60096_dev_s *)instance_handle;

  DEBUGASSERT(priv != NULL);
  UNUSED(arg);
  UNUSED(priv);

  return OK;
}

/****************************************************************************
 * Name: dac60096_dvr_read
 ****************************************************************************/

static ssize_t dac60096_dvr_read(FAR void *instance_handle, FAR char *buffer,
                                size_t buflen)
{
	return -EINVAL;
}

/****************************************************************************
 * Name: dac60096_dvr_write
 ****************************************************************************/

static ssize_t dac60096_dvr_write(FAR void *instance_handle,
                                 FAR const char *buffer, size_t buflen)
{
	return -EINVAL;
}

/****************************************************************************
 * Name: dac60096_dvr_seek
 ****************************************************************************/

static off_t dac60096_dvr_seek(FAR void *instance_handle, off_t offset,
                              int whence)
{
	return -EINVAL;
}

/****************************************************************************
 * Name: dac60096_dvr_ioctl
 ****************************************************************************/

static int dac60096_dvr_ioctl(FAR void *instance_handle, int cmd,
                             unsigned long arg)
{
	DEBUGASSERT(instance_handle != NULL);

	FAR struct dac60096_dev_s *priv = (FAR struct dac60096_dev_s *)instance_handle;
	int ret = OK;
	switch (cmd)
    {
      /* Command was not recognized */

		case DACIOC_CONFIG_DEFAULT:
			dac60096_config_default(priv);
		break;

		case DACIOC_SET_CHANNEL_VALUE:
			ret = dac60096_set_channel_value(priv,(int32_t) arg);
		break;

		case DACIOC_GET_CHANNEL_VALUE:
			ret = dac60096_get_channel_value(priv,(int16_t) arg);
		break;

		case DACIOC_ENABLE_OUTPUT:
			ret = dac60096_output_volt(priv);
		break;

		case DACIOC_SOFT_RESET:
			ret = dac60096_soft_reset(priv);
		break;

		case DACIOC_CLEAR_STATE:
			ret = dac60096_clear_state(priv);
		break;

		case DACIOC_SELECT_MODE:
			if(priv->mode < DAC60096_MODE_MAX){
				priv->config->sc_ops->select(priv,arg);
				priv->mode = arg;
			}else{
				ret = -EINVAL;
			}
		break;

		default:
		  ret = -EINVAL;
		  break;
    }

	return ret;
}

/****************************************************************************
 * Name: dac60096_dvr_exchange (with SPI DMA capability)
 *
 * Description:
 *   Exchange a block of data on SPI using DMA
 *
 * Input Parameters:
 *   instance_handle - Pointer to struct dac60096_dev_s.
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dac60096_dvr_exchange(FAR void *instance_handle,
                                 FAR const void *txbuffer,
                                 FAR void *rxbuffer, size_t nwords)
{
	DEBUGASSERT(instance_handle != NULL);
	FAR struct dac60096_dev_s *priv = (FAR struct dac60096_dev_s *)instance_handle;
	FAR struct spi_dev_s *spi = priv->spi;

	/* Lock the SPI bus so that only one device can access it at the same time */

	SPI_LOCK(spi, true);

	SPI_SETFREQUENCY(spi, DAC60096_SPI_FREQUENCY);

	SPI_SETMODE(spi, DAC60096_SPI_MODE);

	/* Set CS to low which selects the DAC60096 */

	SPI_SELECT(spi, priv->config->spi_devid, true);

	/* Perform an SPI exchange block operation. */

	SPI_EXCHANGE(spi, txbuffer, rxbuffer, nwords);

	/* Set CS to high to deselect the DAC60096 */

	SPI_SELECT(spi, priv->config->spi_devid, false);

	/* Unlock the SPI bus */

	SPI_LOCK(spi, false);
 }

/****************************************************************************
 * Name: dac60096_select
 *
 * Description:
 *    the device select callback
 *
 * Input Parameters:
 *   instance_handle  - device pointer.
 *	 mode  - device work mode.
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
void __attribute__((weak)) dac60096_select(FAR void *instance_handle,FAR enum dac60096_mode_e mode)
{
	snerr("ERROR: weakfunction,need implement!!!");
}

/****************************************************************************
 * Name: dac60096_spi_config_initialize
 *
 * Description:
 *   Register the dac60096 character device as 'devpath'
 *
 * Input Parameters:
 *   dev      - number of mg_config_s.
 *
 * Returned Value:
 *   config_s pointer.
 *
 ****************************************************************************/

struct dac60096_config_s * dac60096_spi_config_initialize(int dev)
{
	switch(dev)
	{
		case 1:
		{
			return &g_dac60096_config_a;
		}
		break;

		default:
			return NULL;
			break;
	}
	return NULL;
}


#endif /* CONFIG_SPI && CONFIG_SPI_EXCHANGE && CONFIG_SENSORS_DAC60096 */

