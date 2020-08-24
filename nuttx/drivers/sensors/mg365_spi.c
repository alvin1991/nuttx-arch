/****************************************************************************
 * drivers/sensors/mg365_spi.c
 * Character driver for the epson MG365 Tri-axis accelerometer and gyroscope.
 *
 *   Copyright (C) 2017-2018 RAF Research LLC. All rights reserved.
 *   Author: Peng Wei <alvin.pengw@gmail.com>
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
	&& defined(CONFIG_SENSORS_EPSON_MG365)

#include <errno.h>
#include <debug.h>
#include <string.h>
#include <semaphore.h>
#include <sys/types.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/sensors/mg365.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define MG365_SPI_DMA
/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Name: mg365_spi_read
 *
 * Description:
 *    read word via SPI
 *
 * Input Parameters:
 *   dev  - device pointer.
 *	 reg_addr  - reg address.
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
static uint16_t mg365_spi_read(FAR struct mg365_dev_s *dev,uint16_t reg_addr)
{
  uint16_t ret = 0;
  uint16_t reg_data;
  uint8_t buffer[2] = {0x00,0x00};
  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  SPI_SETFREQUENCY(dev->spi, MG365_SPI_FREQUENCY);

  SPI_SETMODE(dev->spi, MG365_SPI_MODE);

  /* Set CS to low to select the MG365 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

#ifdef MG365_SPI_DMA

  /* Transmit the register address from where we want to read. */

  SPI_EXCHANGE(dev->spi, &reg_addr,&ret, 1);

  /* Transmit the content which should be written into the register */

  SPI_EXCHANGE(dev->spi, &buffer,&reg_data, 1);

#else

    /* Transmit the register address from where we want to read. */

    reg_data = SPI_SEND(dev->spi, reg_addr);

    /* Write an idle byte while receiving the requested data */

    reg_data = (uint16_t) (SPI_SEND(dev->spi, 0x00));

#endif

  /* Set CS to high to deselect the MG365 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);

  return reg_data;
}



/****************************************************************************
 * Name: mg365_spi_write
 *
 * Description:
 *    writr word via SPI
 *
 * Input Parameters:
 *   dev  - device pointer.
 *	 val  - value.
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
static void mg365_spi_write(FAR struct mg365_dev_s *dev,uint16_t val)
{

  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  SPI_SETFREQUENCY(dev->spi, MG365_SPI_FREQUENCY);

  SPI_SETMODE(dev->spi, MG365_SPI_MODE);

  /* Set CS to low to select the MG365 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);


  SPI_SEND(dev->spi, val);

  /* Set CS to high to deselect the MG365 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: mg365_write_reg
 *
 * Description:
 *    writr mg365 register via SPI
 *
 * Input Parameters:
 *   dev  - device pointer.
 *   wn   - windows number.
 *   reg  - register address.
 *	 val  - value.
 *	 size - size of value.
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
 int mg365_write_reg(struct mg365_dev_s *dev,unsigned char wn, unsigned short reg,unsigned short val)
{

	unsigned short word = 0;

	/* windows has changed? */
	if(dev->current_windows != wn){

		dev->current_windows = wn;

		/* msb is 1b for register writes */
		word = ((ADDR_WIN_CTRL | 0x80) << 8) | wn;

		/* write windows numbers */
		mg365_spi_write(dev,word);
	}

	/* msb is 1b for register writes */
	word = (reg | 0x80) << 8 | val;

	/* write command */
	mg365_spi_write(dev,word);

	return OK;
}

 /****************************************************************************
  * Name: mg365_read_reg
  *
  * Description:
  *    read mg365 register via SPI
  *
  * Input Parameters:
  *   dev  - device pointer.
  *   reg  - register address.
  *	  val  - value pointer.
  *	  size - size of value.
  *
  * Returned Value:
  *   NONE.
  *
  ****************************************************************************/
 int mg365_read_reg(struct mg365_dev_s *dev,unsigned char wn, unsigned short reg,unsigned short *val)
{
	unsigned short word = 0;

	/* windows has changed? */
	if(dev->current_windows != wn){

		dev->current_windows = wn;

		/* msb is 1b for register writes */
		word = ((ADDR_WIN_CTRL | 0x80) << 8) | wn;

		/* write windows numbers */
		mg365_spi_write(dev,word);
	}

	/* read data to register */
	word = (reg & 0x7E) << 8 ;
	*val = mg365_spi_read(dev,word);

	return OK;
}

#endif /* CONFIG_SPI && CONFIG_SENSORS_EPSON_MG365 && CONFIG_SPI_EXCHANGE */

