/****************************************************************************
 * drivers/sensors/mmc5883_i2c.c
 * Character driver for the ST HMC6343 Three-axis Compass with Algorithms.
 *
 *   Copyright (C) 2017-2018 RAF Research LLC. All rights reserved.
 *   Author: Bob Feretich <bob.feretich@rafresearch.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright+
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
 *****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/random.h>
#include <nuttx/sensors/mmc5883.h>

#if defined(CONFIG_I2C) \
	&& defined(CONFIG_SENSORS_MMC5883)

static struct mmc5883_uorb_s mmc5883_orb_data;


void mmc5883_cycle_trampoline(void *arg);
static inline void mmc5883_cycle(void *arg);



/****************************************************************************
 * Name: mmc5883_read_byte
 *
 * Description:
 *   Read a byte via I2C. Each write operation will be an 'atomic'
 *   operation in the sense that any other I2C actions will be serialized
 *   and pend until this write completes.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   reg  	- Address of register
 *   buff	- memory pointer
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int mmc5883_read_byte(FAR void *dev, uint8_t reg,uint8_t *buff)
{
	FAR struct mmc5883_dev_s *priv = (struct mmc5883_dev_s *)dev;
	struct i2c_msg_s msg[2];

	/* Read reg */
	msg[0].frequency = priv->config->frequency;
	msg[0].addr      = priv->config->address;
	msg[0].flags     = 0 | I2C_M_NOSTOP;
	msg[0].buffer    = &reg;
	msg[0].length    = 1;

	/* Read reg value*/
	msg[1].frequency = priv->config->frequency;
	msg[1].addr      = priv->config->address;
	msg[1].flags     = I2C_M_READ;
	msg[1].buffer    = buff;
	msg[1].length    = 1;

	int ret = priv->i2c->ops->transfer(priv->i2c,msg, 2);
	if(ret < 0){
		snerr("ERROR: Failed to read mmc5883: %d\n", ret);
	}

	return ret;

}


/****************************************************************************
 * Name: mmc5883_write_byte
 *
 * Description:
 *   write a byte via I2C. Each write operation will be an 'atomic'
 *   operation in the sense that any other I2C actions will be serialized
 *   and pend until this write completes.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   reg  	- Address of register
 *   val	- register value
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int mmc5883_write_byte(FAR void *dev, uint8_t reg,uint8_t val)
{
	FAR struct mmc5883_dev_s *priv = (struct mmc5883_dev_s *)dev;
	struct i2c_msg_s msg[2];

	/* Read reg */
	msg[0].frequency = priv->config->frequency;
	msg[0].addr      = priv->config->address;
	msg[0].flags     = I2C_M_NOSTOP;
	msg[0].buffer    = &reg;
	msg[0].length    = 1;

	/* Read reg value*/
	msg[1].frequency = priv->config->frequency;
	msg[1].addr      = priv->config->address;
	msg[1].flags     = 0;
	msg[1].buffer    = &val;
	msg[1].length    = 1;

	int ret = priv->i2c->ops->transfer(priv->i2c,msg, 2);
	if(ret < 0){
		snerr("ERROR: Failed to write mmc5883: %d\n", ret);
	}

	return ret;
}

/****************************************************************************
 * Name: mmc5883_measure
 *
 * Description:
 *   measure sensor from mmc5883.
 *
 * Input Parameters:
 *   arg  - device pointer
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
int mmc5883_measure(void *arg)
{
	FAR struct mmc5883_dev_s *priv =  arg;
	uint8_t buff[7]={0},ret1 = 0, ret2 = 0;


	/* Read device status */
	ret1 = mmc5883_read_byte(priv, MMC5883MA_REG_STATUS,buff);

	if((buff[0]&MMC5883MA_VAL_STATUS_MEAS_M_DONE) != MMC5883MA_VAL_STATUS_MEAS_M_DONE){
		return 0;
	}

	/*
	 * Get device mmc5883 magnetometer
	 */
	ret1 = mmc5883_read_byte(priv, MMC5883MA_REG_XOUT_LOW,&buff[0]);
	ret2 = mmc5883_read_byte(priv, MMC5883MA_REG_XOUT_HIGH,&buff[1]);
	if(ret1	< 0 || ret2 < 0 ){
		snerr("ERROR:Magnetic read error\n");
		return ERROR;
	}else{
		priv->sample.x = (int16_t)((buff[1] << 8) | buff[0]) - MMC5883MA_VAL_OFFSET;
	}
	ret1 = mmc5883_read_byte(priv, MMC5883MA_REG_YOUT_LOW,&buff[2]);
	ret2 = mmc5883_read_byte(priv, MMC5883MA_REG_YOUT_HIGH,&buff[3]);
	if(ret1	< 0 || ret2 < 0 ){
		snerr("ERROR:Magnetic read error\n");
		return ERROR;
	}else{
		priv->sample.y = (int16_t)((buff[3] << 8) | buff[2]) - MMC5883MA_VAL_OFFSET;
	}

	ret1 = mmc5883_read_byte(priv, MMC5883MA_REG_ZOUT_LOW,&buff[4]);
	ret2 = mmc5883_read_byte(priv, MMC5883MA_REG_ZOUT_HIGH,&buff[5]);
	if(ret1	< 0 || ret2 < 0 ){
		snerr("ERROR:Magnetic read error\n");
		return ERROR;
	}else{
		priv->sample.z = (int16_t)((buff[5] << 8) | buff[4]) - MMC5883MA_VAL_OFFSET;
	}

	/*
	 *  Get device mmc5883 temperature data
	 */
	if( mmc5883_read_byte(priv, MMC5883MA_REG_TEMPERATURE,&buff[6]) < 0 ){
		snerr("ERROR:Magnetic read error\n");
		return ERROR;
	}else{
		priv->sample.temp = buff[6] - 75;
	}

	/***********************************************************************
	 * publish orb message
	 ***********************************************************************/
	mmc5883_orb_data.mag[0] = priv->sample.x;
	mmc5883_orb_data.mag[1] = priv->sample.y;
	mmc5883_orb_data.mag[2] = priv->sample.z;
	mmc5883_orb_data.temp = priv->sample.temp;

	return orb_publish(ORB_ID(mmc5883_uorb), priv->uorb_pub, &mmc5883_orb_data);
}


/****************************************************************************
 * Name: mmc5883_cycle
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

static inline void mmc5883_cycle(void *arg)
{
	FAR struct mmc5883_dev_s *dev =  arg;

	/* measure sensor via work queue */
	mmc5883_measure(arg);

	if (dev->_running) {
		/* schedule a cycle to start things */
		work_queue(LPWORK, &dev->_work, (worker_t)mmc5883_cycle_trampoline, dev, dev->_mesure_ticks);
	}
}

/****************************************************************************
 * Name: mmc5883_cycle_trampoline
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
void mmc5883_cycle_trampoline(void *arg)
{
	FAR struct mmc5883_dev_s *dev =  arg;

	mmc5883_cycle(dev);
}

/****************************************************************************
 * Name: mmc5883_start_cycle
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

static void mmc5883_start_cycle(void *arg)
{
	FAR struct mmc5883_dev_s *dev =  arg;

	if(!dev->_running){

		/* reset the data and state */
		memset(&mmc5883_orb_data, 0, sizeof(mmc5883_orb_data));

		/* advertise the message topic */
		dev->uorb_pub = orb_advertise(ORB_ID(mmc5883_uorb), &mmc5883_orb_data);


		/* schedule a cycle to start things */
		dev->_running = true;
		work_queue(LPWORK, &dev->_work, (worker_t)mmc5883_cycle_trampoline, dev, 1);
	}
}


/****************************************************************************
 * Name: mmc5883_stop_cycle
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
static void mmc5883_stop_cycle(void *arg)
{
	FAR struct mmc5883_dev_s *dev =  arg;

	int ret = 0;

	if(dev->_running){

		/* stop the cycle */
		dev->_running = false;

		work_cancel(LPWORK, &dev->_work);

		/* unadvertise the message topic */

		ret = orb_unadvertise(dev->uorb_pub);
		if(ret <  0){
			syslog(LOG_ERR,"orb_unadvertise: failed to unadvertise uorb topic:%d",ret);
		}
	}
}

/****************************************************************************
 * Name: mmc5883_write
 *
 * Description:
 *   Send a block of data on I2C. Each write operation will be an 'atomic'
 *   operation in the sense that any other I2C actions will be serialized
 *   and pend until this write completes.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   config  - Described the I2C configuration
 *   buffer - A pointer to the read-only buffer of data to be written to device
 *   buflen - The number of bytes to send from the buffer
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int mmc5883_dops_open(FAR void *instance_handle, int32_t arg)
{
	FAR struct mmc5883_dev_s *priv = (struct mmc5883_dev_s *)instance_handle;
	uint8_t buff[3];
	int ret = 0;

	/* Read device product ID */
	ret = mmc5883_read_byte(priv, MMC5883MA_REG_PRODUCT_ID,buff);
	if(ret < 0 || buff[0] != 0x0c){
		snerr("ERROR:%d,Unsupport Magnetic Sensor device:%x\n",ret,buff[0]);
	}

	/* Read device status */
	ret = mmc5883_read_byte(priv, MMC5883MA_REG_STATUS,buff);
	if(ret < 0 || ((buff[0] & MMC5883MA_DEVICE_OK)!= MMC5883MA_DEVICE_OK)){
		snerr("ERROR:%d,Magnetic not ready:%x\n",ret,buff[0]);
	}

	return ret;
}


CODE int mmc5883_dops_close(FAR void *instance_handle, int32_t arg)
{

	return OK;
}


CODE ssize_t mmc5883_dops_read(FAR void *instance_handle, FAR char *buffer,
          size_t buflen)
{
	return OK;
}


CODE ssize_t mmc5883_dops_write(FAR void *instance_handle,
          FAR const char *buffer, size_t buflen)
{
	return OK;
}

CODE off_t mmc5883_dops_seek(FAR void *instance_handle, off_t offset,
          int whence)
{
	return OK;
}

CODE int mmc5883_dops_ioctl(FAR void *instance_handle, int cmd,
          unsigned long arg)
{
	FAR struct mmc5883_dev_s *priv = (struct mmc5883_dev_s *)instance_handle;
	int ret = 0;

	switch(cmd){

	case MMC5883IOC_MEASUREMENT_BANDWIDTH_SET:
	{
		/* Set device mmc5883 Measurement band-width */
		ret = mmc5883_write_byte(priv,\
								 MMC5883MA_REG_CONTROL_1,\
								 (uint8_t) arg);
		if(ret < 0){
			snerr("ERROR: Failed to reset mmc5883 magnetometer: %d\n", ret);

		}
		return ret;
	}
		break;

	case MMC5883IOC_CONTINUOUS_FREQ_SET:
	{
		/* Set device mmc5883 magnetometer continuous frequence */
		ret = mmc5883_write_byte(priv,\
								 MMC5883MA_REG_CONTROL_2,\
								 (uint8_t) arg);
		if(ret < 0){
			snerr("ERROR: Failed to reset mmc5883 magnetometer: %d\n", ret);

		}
	}
		break;

	case MMC5883IOC_MEASUREMENT_RATE_SET:
		{
			priv->_mesure_ticks = USEC2TICK(arg);
			ret = OK;
		}
		break;

	case MMC5883IOC_SOFTWARE_RESET:
		{
			/* Reset the magnetometer */
			ret = mmc5883_write_byte(priv,\
									 MMC5883MA_REG_CONTROL_1,\
									 (uint8_t) MMC5883MA_CMD_CTRL1_SW_RST);
			if(ret < 0){
				snerr("ERROR: Failed to reset mmc5883 magnetometer: %d\n", ret);

			}
			return ret;
		}
		break;

	case MMC5883IOC_MEASUREMENT_START:
		{
			/* Reset the magnetometer */
			ret = mmc5883_write_byte(priv,\
									 MMC5883MA_REG_CONTROL_0,\
									 (uint8_t) (MMC5883MA_CMD_CTRL0_TM_M|MMC5883MA_CMD_CTRL0_TM_T|MMC5883MA_CMD_CTRL0_SET));
			if(ret < 0){
				snerr("ERROR: Failed to reset mmc5883 magnetometer: %d\n", ret);

			}

			usleep(100000);

			uint8_t tmp=0;
			/* Reset the magnetometer */
			ret = mmc5883_read_byte(priv,\
									 MMC5883MA_REG_CONTROL_0,\
									 &tmp);
			if(ret < 0){
				snerr("ERROR: Failed to reset mmc5883 magnetometer: %d\n", ret);

			}
			usleep(100000);

			/* Start the measurement */
			mmc5883_start_cycle(priv);
		}
		break;

	case MMC5883IOC_MEASUREMENT_STOP:
		{
			/* Stop the measurement */
			mmc5883_stop_cycle(priv);
		}
		break;

	default:

		break;
	}

	return ret;
}

#endif /* CONFIG_SENSORS_MMC5883 && CONFIG_I2C*/
