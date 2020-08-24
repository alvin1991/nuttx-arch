/****************************************************************************
 * drivers/sensors/mg365_base.c
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
#include <nuttx/config.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

#if defined(CONFIG_SENSORS_EPSON_MG365)
#include <nuttx/sensors/mg365.h>
#include <nuttx/sensors/mg365_spi.h>
#include <nuttx/timers/drv_hrt.h>

static int      mg365_dvr_open(FAR void *instance_handle, int32_t arg);
static int      mg365_dvr_close(FAR void *instance_handle, int32_t arg);
static ssize_t  mg365_dvr_read(FAR void *instance_handle,
                                 FAR char *buffer, size_t buflen);
static ssize_t  mg365_dvr_write(FAR void *instance_handle,
                                  FAR const char *buffer, size_t buflen);
static off_t    mg365_dvr_seek(FAR void *instance_handle, off_t offset,
                                 int whence);
static int      mg365_dvr_ioctl(FAR void *instance_handle, int cmd,
                                  unsigned long arg);
static void 	mg365_dvr_exchange(FAR void *instance_handle,
                                 FAR const void *txbuffer,
                                 FAR void *rxbuffer, size_t nwords);
/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mg365_dvr_entry_vector_s g_mg365_dops =
{
  /* Standard sensor cluster driver entry-vector */

  {
    .driver_open    = mg365_dvr_open,
    .driver_close   = mg365_dvr_close,
    .driver_read    = mg365_dvr_read,
    .driver_write   = mg365_dvr_write,
    .driver_seek    = mg365_dvr_seek,
    .driver_ioctl   = mg365_dvr_ioctl,
    .driver_suspend = 0,
    .driver_resume  = 0,
    },
  /* adis16488 extensions follow */

  .driver_spiexc = mg365_dvr_exchange,
};


/****************************************************************************
 * Private data storage
 ****************************************************************************/
//void 				mg365_cycle_trampoline(void *arg);
//static inline void 	mg365_cycle(void *arg);

/* Default accelerometer initialization sequence */

/* step1:
 * 		1. Safety Measure, Incase Device is in Sampling Mode, goto Config Mode.
 * 		2. Not Inverted the value of Gyros and Accls.
 * 		3. Enable new data flags for Gyros and Accls.
 * 		4. Configure EXT=Reset Counter, EXT_POL=Positive Logic, DRDY_ON=Data_Ready, DRDY_POL=Active Low.
 * 		5. Set samples per second to 400.
 * 		6. Set Filter TAP = 128,and delay sometimes for filter config.
 *
 * step2:
 * 		1. Config AUTO_START=Disable, UART_AUTO=Disable (For SPI Interface)
 * 		2. Enable COUNT and CHKSM data for burst mode
 * 		3. Enable GYRO and ACCL data for burst mode
 * 		4. Enable 32-bit Output for Gyro & Accel
 * 		5. Enable Attitude = Euler mode, ANG1=X, ANG2, Y, ANG3=Z
 */

/*
 * MG365 Register Configration A
 */
struct mg_reg_pair_s g_reg_pairs_a[] =
{
	//step1:
	{CMD_WINDOW1,ADDR_POL_CTRL_LO,0x00},				 //<<< Not Inverted the value of Gyros and Accls.
	{CMD_WINDOW1,ADDR_SIG_CTRL_HI,CMD_EN_NDFLAGS},		 //<<< Enable new data flags for Gyros and Accls.
	{CMD_WINDOW1,ADDR_MSC_CTRL_LO,CMD_RSTCNTR_DRDY},	 //<<< EXT=Reset Counter, EXT_POL=Positive Logic, DRDY_ON=Data_Ready, DRDY_POL = Active Low.
	{CMD_WINDOW1,ADDR_SMPL_CTRL_HI,CMD_RATE100},		 //<<< Set samples per second to 1000hz
	{CMD_WINDOW1,ADDR_FILTER_CTRL_LO,CMD_FLTAP128},		 //<<< Filter TAP = 16.
	//step2:
	{CMD_WINDOW1,ADDR_UART_CTRL_LO,0x00},				 //<<< AUTO_START=Disable, UART_AUTO=Disable (For SPI Interface).
	{CMD_WINDOW1,ADDR_ATTI_CTRL_LO,CMD_ATTI_CONV},		 //<<< Enable Attitude = Euler mode, ANG1=X, ANG2, Y, ANG3=Z.
	{CMD_WINDOW1,ADDR_ATTI_CTRL_HI,CMD_EULER_ATTI_MODE}, //<<< Enable Attitude = Euler mode, ANG1=X, ANG2, Y, ANG3=Z.
};

static struct mg_config_s g_mg365_config_a =
{
	.spi_devid 				= SPIDEV_USER(0),
	.initial_cr  			= g_reg_pairs_a,
	.initial_cr_len			= sizeof(g_reg_pairs_a)/sizeof(struct mg_reg_pair_s),
	.enable_atti_delta		= FALSE,
	.sc_ops 				= &g_mg365_dops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
void mg365_cycle_trampoline(void *arg);

/****************************************************************************
 * Name: mg365_measure
 *
 * Description:
 *   measure sensor from mg365.
 *
 * Input Parameters:
 *   arg  - device pointer
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
static  inline void mg365_measure(void *arg)
{
	FAR struct mg365_dev_s *dev =  arg;

	uint16_t val[2];

	/***********************************************************************
	 * measure the sensor
	 ***********************************************************************/

	// temperature
	mg365_read_reg(dev,CMD_WINDOW0, ADDR_TEMP_HIGH,&val[0]);
	mg365_read_reg(dev,CMD_WINDOW0, ADDR_TEMP_LOW,&val[1]);
	int t = val[0]<<16 | val[1];
	dev->data.temp = (t-172621824)*(EPSON_TEMP_SF) + 25;

	// gyro(x-y-z)
	mg365_read_reg(dev,CMD_WINDOW0, ADDR_XGYRO_HIGH,&val[0]);
	mg365_read_reg(dev,CMD_WINDOW0, ADDR_XGYRO_LOW,&val[1]);
	t = val[0]<<16 | val[1];
	dev->data.gyro[0] = EPSON_GYRO_SF*t;

	mg365_read_reg(dev,CMD_WINDOW0, ADDR_YGYRO_HIGH,&val[0]);
	mg365_read_reg(dev,CMD_WINDOW0, ADDR_YGYRO_LOW,&val[1]);
	t = val[0]<<16 | val[1];
	dev->data.gyro[1] = -EPSON_GYRO_SF*t;

	mg365_read_reg(dev,CMD_WINDOW0, ADDR_ZGYRO_HIGH,&val[0]);
	mg365_read_reg(dev,CMD_WINDOW0, ADDR_ZGYRO_LOW,&val[1]);
	t = val[0]<<16 | val[1];
	dev->data.gyro[2] = -EPSON_GYRO_SF*t;

	// accl(x-y-z)
	mg365_read_reg(dev,CMD_WINDOW0, ADDR_XACCL_HIGH,&val[0]);
	mg365_read_reg(dev,CMD_WINDOW0, ADDR_XACCL_LOW,&val[1]);
	t = val[0]<<16 | val[1];
	dev->data.accl[0] = EPSON_ACCL_SF*t;

	mg365_read_reg(dev,CMD_WINDOW0, ADDR_YACCL_HIGH,&val[0]);
	mg365_read_reg(dev,CMD_WINDOW0, ADDR_YACCL_LOW,&val[1]);
	t = val[0]<<16 | val[1];
	dev->data.accl[1] = -EPSON_ACCL_SF*t;

	mg365_read_reg(dev,CMD_WINDOW0, ADDR_ZACCL_HIGH,&val[0]);
	mg365_read_reg(dev,CMD_WINDOW0, ADDR_ZACCL_LOW,&val[1]);
	t = val[0]<<16 | val[1];
	dev->data.accl[2] = -EPSON_ACCL_SF*t;

	if(dev->config->enable_atti_delta){
		// atti(roll-pitch-yaw)
		mg365_read_reg(dev,CMD_WINDOW0, ADDR_ROLL_HIGH,&val[0]);
		mg365_read_reg(dev,CMD_WINDOW0, ADDR_ROLL_LOW,&val[1]);
		t = val[0]<<16 | val[1];
		dev->data.multi.atti[0] = EPSON_ATTI_SF*t;

		mg365_read_reg(dev,CMD_WINDOW0, ADDR_PITCH_HIGH,&val[0]);
		mg365_read_reg(dev,CMD_WINDOW0, ADDR_PITCH_LOW,&val[1]);
		t = val[0]<<16 | val[1];
		dev->data.multi.atti[1] = EPSON_ATTI_SF*t;

		mg365_read_reg(dev,CMD_WINDOW0, ADDR_YAW_HIGH,&val[0]);
		mg365_read_reg(dev,CMD_WINDOW0, ADDR_YAW_LOW,&val[1]);
		t = val[0]<<16 | val[1];
		dev->data.multi.atti[2] = EPSON_ATTI_SF*t;
	}
	//publish orb message
	orb_publish(ORB_ID(mg365_uorb), dev->uorb_pub, &(dev->data));
}

/****************************************************************************
 * Name: mg365_cycle
 *
 * Description:
 *    cycle call work_queue
 *
 * Input Parameters:
 *   arg  - device pointer
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/

static inline void mg365_cycle(void *arg)
{
	FAR struct mg365_dev_s *dev =  arg;

	/* measure sensor via work queue */
	mg365_measure(arg);

	if (dev->_running) {
		/* schedule a cycle to start things */
		work_queue(HPWORK, &dev->_work, (worker_t)mg365_cycle_trampoline, dev, dev->_mesure_ticks);
	}
}

/****************************************************************************
 * Name: mg365_cycle_trampoline
 *
 * Description:
 *   subscribe the uorb message.
 *
 * Input Parameters:
 *   arg  - device pointer
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
void mg365_cycle_trampoline(void *arg)
{
	FAR struct mg365_dev_s *dev =  arg;

	mg365_cycle(dev);
}

/****************************************************************************
 * Name: mg365_start_cycle
 *
 * Description:
 *    start cycle call work_queue
 *
 * Input Parameters:
 *   arg  - device pointer
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
static void mg365_start_cycle(void *arg)
{
	FAR struct mg365_dev_s *dev =  arg;

	if(!dev->_running){

		/* move to sampling mode */
		mg365_write_reg(dev,CMD_WINDOW0, ADDR_MODE_CTRL_HI,CMD_BEGIN_SAMPLING);

		/* reset the data and state */
		memset(&(dev->data), 0, sizeof(dev->data));

		/* advertise the message topic */
		dev->uorb_pub = orb_advertise(ORB_ID(mg365_uorb), &(dev->data));

		/* schedule a cycle to start things */
		dev->_running = true;
		work_queue(HPWORK, &dev->_work, (worker_t)mg365_cycle_trampoline, dev, 1);
	}

}

/****************************************************************************
 * Name: mg365_stop_cycle
 *
 * Description:
 *    stop cycle call work_queue
 *
 * Input Parameters:
 *   arg  - device pointer
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
static void mg365_stop_cycle(void *arg)
{
	FAR struct mg365_dev_s *dev =  arg;

	int ret = 0;

	if(dev->_running){
		/* stop sampling */
		mg365_write_reg(dev,CMD_WINDOW0, ADDR_MODE_CTRL_HI,CMD_END_SAMPLING);

		/* stop the cycle */
		dev->_running = false;

		work_cancel(HPWORK, &dev->_work);

		/* unadvertise the message topic */
		ret = orb_unadvertise(dev->uorb_pub);
		if(ret <  0){
			syslog(LOG_ERR,"orb_unadvertise: failed to unadvertise uorb topic:%d",ret);
		}
	}
}

///////////////////////////device test functions////////////////////////

/****************************************************************************
 * Name: mg365_config_default
 *
 * Description:
 *
 *   Initialize the MG365 in default configrations.
 *
 * step1:
 * 		1. Safety Measure, Incase Device is in Sampling Mode, goto Config Mode.
 * 		2. Not Inverted the value of Gyros and Accls.
 * 		3. Enable new data flags for Gyros and Accls.
 * 		4. Configure EXT=Reset Counter, EXT_POL=Positive Logic, DRDY_ON=Data_Ready, DRDY_POL=Active Low.
 * 		5. Set samples per second to 400.
 * 		6. Set Filter TAP = 128,and delay sometimes for filter config.
 *
 * step2:
 * 		1. Config AUTO_START=Disable, UART_AUTO=Disable (For SPI Interface)
 * 		2. Enable COUNT and CHKSM data for burst mode
 * 		3. Enable GYRO and ACCL data for burst mode
 * 		4. Enable 32-bit Output for Gyro & Accel
 * 		5. Enable Attitude = Euler mode, ANG1=X, ANG2, Y, ANG3=Z
 *
 ****************************************************************************/

static int mg365_config_default(FAR struct mg365_dev_s *dev)
{
	DEBUGASSERT(dev != NULL);

	struct mg_reg_pair_s *mrp = dev->config->initial_cr;

	// Initialize the MG365 in default configrations.
	for(int i = 0; i< dev->config->initial_cr_len; i++){
		mg365_write_reg(dev,mrp->winnum, mrp->addr,mrp->value);
	}

	return OK;
}

/****************************************************************************
 * Name: mg365_check_prod_id
 *
 * Description:
 *
 *   Read and Check the MG365's ID Registers.
 *   There are one ID Register...
 *
 *     Manufacturer Product ID should be "G365"(in ASCII code).
 *
 ****************************************************************************/

static int mg365_check_prod_id(FAR struct mg365_dev_s *dev)
{
	DEBUGASSERT(dev != NULL);

	uint16_t devid[2];
	uint16_t ret = 0;
	ret = mg365_read_reg(dev,CMD_WINDOW1, ADDR_PROD_ID1,&devid[0]);
	if(ret < 0){
		snerr("ERROR: Failed to read device id[1].\n");
		return ERROR;
	}
	ret = mg365_read_reg(dev,CMD_WINDOW1, ADDR_PROD_ID2,&devid[1]);
	if(ret < 0){
		snerr("ERROR: Failed to read device id[2].\n");
		return ERROR;
	}

	if(VAL_PROD_ID1 == devid[0] && VAL_PROD_ID2 == devid[1]){
		return TRUE;
	}else{
		snerr("ERROR: Unsupported device id:%x,%x\n",devid[0],devid[1]);
		return ERROR;
	}
}

/****************************************************************************
 * Name: mg365_check_ready
 *
 * Description:
 *
 *   Read the MG365's ID Registers.
 *   Check the device status is ok!
 *
 ****************************************************************************/

static int mg365_check_ready(FAR struct mg365_dev_s *dev)
{
	DEBUGASSERT(dev != NULL);

	uint16_t reg_data[2];
	uint16_t ret = 0;

	/* read the global cmd*/
	ret = mg365_read_reg(dev,CMD_WINDOW1, ADDR_GLOB_CMD_LO,&reg_data[0]);

	/* When NOT_READY becomes 0, it ends.  */
	if(ret < 0 || (reg_data[0] & 0x0400)){
		ret = ERROR;
	}

	/* read the diagnosed cmd*/
	ret = mg365_read_reg(dev,CMD_WINDOW0, ADDR_DIAG_STAT,&reg_data[1]);

	/* When hardware check OK at startup,HARD_ERR becomes 0, it ends.  */
	if(ret < 0 || (reg_data[0] & 0x0600)){
		ret = ERROR;
	}

	return OK;
}

///////////////////////////driver operation functions////////////////////////

/****************************************************************************
 * Name: mg365_dvr_open
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
static int mg365_dvr_open(FAR void *instance_handle, int32_t arg)
{
  FAR struct mg365_dev_s *priv = (FAR struct mg365_dev_s *)instance_handle;
  int ret;

  DEBUGASSERT(priv != NULL);
  UNUSED(arg);

  /* Wait 800ms,the device get in stable. */
  usleep(VAL_WAIT_US_STABLE);

  sninfo("mg365_open: entered...\n");

  /* check the device ID */
  ret = mg365_check_prod_id(priv);
  if(ret < 0){
	  return ret;
  }

  /* check the device status */
  ret = mg365_check_ready(priv);
  if(ret < 0){
	  return ret;
  }

  return OK;
}

/****************************************************************************
 * Name: mg365_dvr_close
 ****************************************************************************/

static int mg365_dvr_close(FAR void *instance_handle, int32_t arg)
{
  FAR struct mg365_dev_s *priv = (FAR struct mg365_dev_s *)instance_handle;

  DEBUGASSERT(priv != NULL);
  UNUSED(arg);
  UNUSED(priv);

  return OK;
}

/****************************************************************************
 * Name: mg365_dvr_read
 ****************************************************************************/

static ssize_t mg365_dvr_read(FAR void *instance_handle, FAR char *buffer,
                                size_t buflen)
{
  FAR struct mg365_dev_s *priv = ((FAR struct mg365_dev_s *)instance_handle);

  DEBUGASSERT(priv != NULL);


  /* Permute data out fields */

  int ret = mg365_read_reg(priv,\
		  	  	  	  	   ADDR2WINDOWS(priv->seek_address),\
						   ADDR2REG(priv->seek_address),\
						   (unsigned short *)buffer);
  if (ret < 0){
	  return ret;
  }

  return buflen;
}

/****************************************************************************
 * Name: mg365_dvr_write
 ****************************************************************************/

static ssize_t mg365_dvr_write(FAR void *instance_handle,
                                 FAR const char *buffer, size_t buflen)
{
  FAR struct mg365_dev_s *priv = (FAR struct mg365_dev_s *)instance_handle;

  DEBUGASSERT(priv != NULL);

  if (priv->readonly || buflen != 2){
      return -EROFS;
  }

  uint16_t val = 0;
  memcpy(&val,buffer,buflen);

  int ret = mg365_write_reg(priv,\
		  	  	  	  	  	ADDR2WINDOWS(priv->seek_address),\
							ADDR2REG(priv->seek_address),\
							val);
  if (ret < 0){
	  return ret;
  }
  return buflen;
}

/****************************************************************************
 * Name: mg365_dvr_seek
 ****************************************************************************/

static off_t mg365_dvr_seek(FAR void *instance_handle, off_t offset,
                              int whence)
{
  FAR struct mg365_dev_s *priv = (FAR struct mg365_dev_s *)instance_handle;

  DEBUGASSERT(priv != NULL);

  uint16_t reg;
  switch (whence)
    {
      case SEEK_CUR:  /* Incremental seek */
        reg = priv->seek_address + offset;
        if (0 > reg || reg > SEEK_ADDR(CMD_WINDOW1,ADDR_WIN_CTRL)){
            return -EINVAL;
          }

        priv->seek_address = reg;
        break;

      case SEEK_END:  /* Seek to the 1st X-data register */
        priv->seek_address = SEEK_ADDR(CMD_WINDOW1,ADDR_WIN_CTRL);
        break;

      case SEEK_SET:  /* Seek to designated address */
    	reg = priv->seek_address;
        if (0 > offset || reg > SEEK_ADDR(CMD_WINDOW1,ADDR_WIN_CTRL)){
            return -EINVAL;
          }

        priv->seek_address = offset;
        break;

      default:        /* invalid whence */

        return -EINVAL;
    }

  return priv->seek_address;
}

/****************************************************************************
 * Name: mg365_dvr_ioctl
 ****************************************************************************/

static int mg365_dvr_ioctl(FAR void *instance_handle, int cmd,
                             unsigned long arg)
{
  FAR struct mg365_dev_s *priv = (FAR struct mg365_dev_s *)instance_handle;
  int ret = OK;

  switch (cmd)
    {
      /* Command was not recognized */

		case EPSONIOC_MEASUREMENT_RATE_SET:
			priv->_mesure_ticks = USEC2TICK(arg);
		break;

		case EPSONIOC_MEASUREMENT_START:
			mg365_start_cycle(priv);
		break;

		case EPSONIOC_MEASUREMENT_STOP:
			mg365_stop_cycle(priv);
		break;

		case EPSONIOC_MEASUREMENT_INIT:
			mg365_config_default(priv);
		break;

		case EPSONIOC_MEASUREMENT_ATTI_DELTA:
			priv->config->enable_atti_delta = (char)arg;
		break;

		default:
		  snerr("ERROR: Unrecognized cmd: %d\n", cmd);
		  ret = -ENOTTY;
		  break;
    }

  return ret;
}

/****************************************************************************
 * Name: mg365_dvr_exchange (with SPI DMA capability)
 *
 * Description:
 *   Exchange a block of data on SPI using DMA
 *
 * Input Parameters:
 *   instance_handle - Pointer to struct mg365_dev_s.
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

static void mg365_dvr_exchange(FAR void *instance_handle,
                                 FAR const void *txbuffer,
                                 FAR void *rxbuffer, size_t nwords)
{
  FAR struct mg365_dev_s *priv = (FAR struct mg365_dev_s *)instance_handle;
  FAR struct spi_dev_s *spi = priv->spi;

  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(spi, true);

  SPI_SETFREQUENCY(spi, MG365_SPI_FREQUENCY);

  SPI_SETMODE(spi, MG365_SPI_MODE);

  /* Set CS to low which selects the MG365 */

  SPI_SELECT(spi, priv->config->spi_devid, true);

  /* Perform an SPI exchange block operation. */

  SPI_EXCHANGE(spi, txbuffer, rxbuffer, nwords);

  /* Set CS to high to deselect the MG365 */

  SPI_SELECT(spi, priv->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(spi, false);
 }



/****************************************************************************
 * Name: mg365_spi_config_initialize
 * Description:
 *   return the mg365 config_s
 * Input Parameters:
 *   dev - number of mg_config_s.
 * Returned Value:
 *   config_s pointer
 ****************************************************************************/
struct mg_config_s * mg365_spi_config_initialize(int dev)
{
	switch(dev)
	{
		case 1:
		{
			return &g_mg365_config_a;
		}
		break;

		default:
			return NULL;
			break;
	}
	return NULL;
}

#endif /* CONFIG_SENSORS_EPSON_MG365 */
