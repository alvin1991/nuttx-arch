/****************************************************************************
 * drivers/sensors/adis16488.c
 * Character driver for the ST ADIS16488 Tri-axis accelerometer and gyroscope.
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

#if defined(CONFIG_SPI) \
	&& defined(CONFIG_SPI_EXCHANGE) \
	&& defined(CONFIG_SENSORS_ADIS16488)

#include <errno.h>
#include <debug.h>
#include <string.h>
#include <semaphore.h>
#include <sys/types.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/sensors/adis16488.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define ADIS16488_SPI_DMA
/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Name: adis16488_read_register
 ****************************************************************************/

static uint16_t adis16488_spi_read_register(FAR struct adis16488_dev_s *dev,
                                     uint16_t reg_addr)
{
  uint16_t ret = 0;
  uint16_t reg_data;
  uint8_t buffer[2] = {0xff,0xff};
  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  SPI_SETFREQUENCY(dev->spi, ADIS16488_SPI_FREQUENCY);

  SPI_SETMODE(dev->spi, ADIS16488_SPI_MODE);

  /* Set CS to low to select the ADIS16488 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

#ifdef ADIS16488_SPI_DMA

  /* Transmit the register address from where we want to read. */

  SPI_EXCHANGE(dev->spi, &reg_addr,&ret, 1);

  /* Transmit the content which should be written into the register */

  SPI_EXCHANGE(dev->spi, &buffer,&reg_data, 1);

#else

    /* Transmit the register address from where we want to read. */

    reg_data = SPI_SEND(dev->spi, reg_addr);

    /* Write an idle byte while receiving the requested data */

    reg_data = (uint16_t) (SPI_SEND(dev->spi, 0xff));

#endif

  /* Set CS to high to deselect the ADIS16488 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);

  return reg_data;
}



/****************************************************************************
 * Name: adis16488_spi_write_register
 ****************************************************************************/

static void adis16488_spi_write_register(FAR struct adis16488_dev_s *dev,
                                   uint16_t val)
{

  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  SPI_SETFREQUENCY(dev->spi, ADIS16488_SPI_FREQUENCY);

  SPI_SETMODE(dev->spi, ADIS16488_SPI_MODE);

  /* Set CS to low to select the ADIS16488 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);


  SPI_SEND(dev->spi, val);

  /* Set CS to high to deselect the ADIS16488 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}




/**
 * adis_write_reg() - write 2 bytes from a 16-bit register
 * @dev: The adis device
 * @reg: The address of the lower of the two registers
 * @val: The value write to the device
 */
 int adis_write_reg(struct adis16488_dev_s *dev, unsigned int reg,
	void* val, unsigned int size)
{
	uint32_t value = 0;
	uint16_t tmp = 0;

	unsigned int page = reg / ADIS_PAGE_SIZE;
	int ret;

	memcpy(&value,val,size);

	/* write page */
	if (dev->current_page != page) {
		tmp  = ADIS_WRITE_REG(ADIS_PAGE_SIZE)<<8;
		tmp += page;
		dev->current_page = page;

		/* send message to device */
		adis16488_spi_write_register(dev,tmp);

	}

	/* write data to register */
	switch (size) {
	case 4:
		tmp = ADIS_WRITE_REG(reg + 3)<<8;
		tmp+= (value >> 24) & 0xff;
		adis16488_spi_write_register(dev,tmp);

		tmp = ADIS_WRITE_REG(reg + 2)<<8;
		tmp+= (value >> 16) & 0xff;
		adis16488_spi_write_register(dev,tmp);

	case 2:
		tmp = ADIS_WRITE_REG(reg + 1)<<8;
		tmp+= (value >> 8) & 0xff;
		adis16488_spi_write_register(dev,tmp);

	case 1:
		tmp = ADIS_WRITE_REG(reg)<<8;
		tmp+= value & 0xff;
		adis16488_spi_write_register(dev,tmp);

		ret = OK;
		break;

	default:
		ret = -EINVAL;
		return ret;
	}

	return ret;
}


/**
 * adis_read_reg() - read 2 bytes from a 16-bit register
 * @adis: The adis device
 * @reg: The address of the lower of the two registers
 * @val: The value read back from the device
 */
 int adis_read_reg(struct adis16488_dev_s *dev, unsigned int reg,
	unsigned int *val, unsigned int size)
{
	uint32_t page = reg / ADIS_PAGE_SIZE;
	int ret = 0;
	uint16_t regaddr = 0,regval_low = 0,regval_high = 0,tmp = 0;

	/* write page */
	if (dev->current_page != page) {
		tmp  = ADIS_WRITE_REG(ADIS_PAGE_SIZE)<<8;
		tmp += page;
		dev->current_page = page;

		/* send message to device */
		adis16488_spi_write_register(dev,tmp);

	}

	/* read data from register */
	switch (size) {
	case 4:
		dev->tx[2] = ADIS_READ_REG(reg + 2);
		dev->tx[3] = 0;
		regaddr = (dev->tx[2] << 8) + dev->tx[3];
		regval_high = adis16488_spi_read_register(dev,regaddr);

	case 2:
		dev->tx[4] = ADIS_READ_REG(reg);
		dev->tx[5] = 0;
		regaddr = (dev->tx[4] << 8) + dev->tx[5];
		regval_low = adis16488_spi_read_register(dev,regaddr);

		*val = (regval_high << 16) + regval_low;

		ret = OK;
		break;

	default:
		ret = -EINVAL;
		return ret;
	}

	return ret;
}

#endif /* CONFIG_SPI && CONFIG_SENSORS_ADIS16488 && CONFIG_SPI_EXCHANGE */

