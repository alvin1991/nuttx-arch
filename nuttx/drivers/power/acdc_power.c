/****************************************************************************
 * drivers/power/huawei_adfxxsxxb.c

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

/* The acdc_powerw(adf18s28b,adf9s48b) are Huawei AC-DC Power Management.
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
#include <semaphore.h>
#define AC_POWER_ERR	_err
/* This driver requires:
 *
 * 
 * CONFIG_I2C - I2C support
 * 
 */
//ADFXXSXXB_OUTPUT_VOLTAGE
//ADFXXSXXB_RESET
//HUAWEI_ADFXXSXXB
//I2C_HUAWEI_ADFXXSXXB
#if defined(CONFIG_I2C) && defined(CONFIG_I2C_ACDC_POWER)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int acdc_power_open(FAR struct file *filep);
static int acdc_power_close(FAR struct file *filep);
static int acdc_power_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static off_t acdc_power_seek(FAR struct file *filep, off_t offset, int whence);
static ssize_t acdc_power_read(FAR struct file *filep, FAR char *buffer,size_t len);
static ssize_t acdc_power_write(FAR struct file *filep, FAR const char *buffer,size_t len);
static int acdc_power_poll(FAR struct file *filep, struct pollfd *fds, bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_acdc_power_fileops =
{
  acdc_power_open,               /* open */
  acdc_power_close,              /* close */
  acdc_power_read,               /* read */
  acdc_power_write,              /* write */
  acdc_power_seek,               /* seek */
  acdc_power_ioctl,              /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  acdc_power_poll,               /* poll */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Name: acdc_power_semtake
 *
 * Acquire a resource to access the device.
 *
 ****************************************************************************/

static void acdc_power_semtake(FAR struct acdc_power_dev_s *acdc)
{
  /* Take the semaphore (perhaps waiting) */

  //while (sem_wait(&(acdc->sem)) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      //DEBUGASSERT(errno == EINTR || errno == ECANCELED);
    }
}

/****************************************************************************
 * Name: acdc_power_semgive
 *
 * Release a resource to access the device.
 *
 ****************************************************************************/

static inline void acdc_power_semgive(FAR struct acdc_power_dev_s *acdc)
{
  //sem_post(&acdc->sem);
}



/****************************************************************************
 * Name: acdc_power_open
 *
 * Description: Open the acdc_power power device
 *
 ****************************************************************************/

static int acdc_power_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct acdc_power_dev_s *acdc;
  int ret = OK;

  DEBUGASSERT(inode && inode->i_private);
  acdc = (FAR struct acdc_power_dev_s *)inode->i_private;
  acdc_power_semtake(acdc);

  /* Increment the reference count */

  if ((acdc->refs + 1) == 0)
    {
      ret = -EMFILE;
    }
  else
    {
      acdc->refs += 1;
    }

  acdc_power_semgive(acdc);
  return ret;
}


/****************************************************************************
 * Name: acdc_power_close
 *
 * Description: Close the acdc_power device
 *
 ****************************************************************************/

static int acdc_power_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct acdc_power_dev_s *acdc;
  int ret = OK;

  DEBUGASSERT(inode && inode->i_private);
  acdc = (FAR struct acdc_power_dev_s *)inode->i_private;
  acdc_power_semtake(acdc);

  /* Decrement the reference count. I want the entire close operation
   * to be atomic wrt other driver operations.
   */

  if (acdc->refs == 0)
    {
      ret = -EIO;
    }
  else
    {
      acdc->refs -= 1;
    }

  acdc_power_semgive(acdc);
  return ret;
}

/****************************************************************************
 * Name: acdc_power_seek
 *
 *
 ****************************************************************************/

static off_t acdc_power_seek(FAR struct file *filep, off_t offset, int whence)
{
	return -ESRCH;
}

/****************************************************************************
 * Name: acdc_power_read
 ****************************************************************************/

static ssize_t acdc_power_read(FAR struct file *filep, FAR char *buffer,size_t len)
{
    return -ESRCH;
}



/****************************************************************************
 * Name: acdc_power_write
 ****************************************************************************/

static ssize_t acdc_power_write(FAR struct file *filep, FAR const char *buffer,size_t len)
{
	return -ESRCH;
}

/****************************************************************************
 * Name: acdc_power_read
 ****************************************************************************/

static int acdc_power_poll(FAR struct file *filep, struct pollfd *fds, bool setup)
{
    return -ESRCH;
}

/****************************************************************************
 * Name: acdc_power_ioctl
 *
 *
 ****************************************************************************/

static int acdc_power_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
	//FAR struct acdc_power_dev_s *acdc;
	//FAR struct inode        *inode = filep->f_inode;
	//FAR struct ncp5623c_dev_s *priv = inode->i_private;
	//FAR struct power_acdc_status_s *read_params;
	int ret = -ENODEV;

#ifdef CONFIG_HUAWEI_ADFXXSXXB
    ret = adfxxsxxb_ioctl(filep,cmd,arg);
#endif

	return ret;
}
/****************************************************************************
 * Name: huawei_adfxxsxxb_register
 *
 * Description:
 *   Register the HUAWEI acdc_power device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/rgbdrv0".
 *   i2c     - An instance of the I2C interface to use to communicate
 *             with the LED driver.
 *   adfxxsxxb_i2c_addr_i2c_addr
 *           - The I2C address of the acdc_power.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int acdc_power_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                      uint8_t const acdc_i2c_addr)
{
	/* Sanity check */

	DEBUGASSERT(i2c != NULL);

	/* Initialize the NCP5623C device structure */

	FAR struct acdc_power_dev_s *priv =
	(FAR struct acdc_power_dev_s *)kmm_malloc(sizeof(struct acdc_power_dev_s));

	if (priv == NULL)
	{
		lcderr("ERROR: Failed to allocate instance of acdc_power_dev_s\n");
		return -ENOMEM;
	}

	priv->i2c = i2c;
	priv->i2c_addr = acdc_i2c_addr;
	priv->freq = I2C_BUS_FREQ_HZ;
    
	//sem_init(&priv->sem, 0, 1);//////////////////////////////////////////////////////////

	/* Register the character driver */

	int const ret = register_driver(devpath, &g_acdc_power_fileops, 666, priv);
	if (ret != OK)
	{
		lcderr("ERROR: Failed to register driver: %d\n", ret);
		kmm_free(priv);
		return ret;
	}

	return OK;
}



#endif //defined(CONFIG_I2C) && defined(CONFIG_I2C_HUAWEI_ADFXXSXXB)

