/****************************************************************************
 * drivers/sensors/dac60096.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/sensors/dac60096.h>

#if defined(CONFIG_SPI) \
	&& defined(CONFIG_SPI_EXCHANGE) \
	&& defined(CONFIG_SENSORS_DAC60096)

#define DAC60096_SPI_DMA

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dac60096_spi_read
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
uint16_t dac60096_spi_read(FAR struct dac60096_dev_s *dev,uint16_t reg_addr)
{
  uint16_t ret = 0;
  uint16_t reg_data;
  uint8_t buffer[2] = {0x00,0x00};
  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  SPI_SETFREQUENCY(dev->spi, DAC60096_SPI_FREQUENCY);

  SPI_SETMODE(dev->spi, DAC60096_SPI_MODE);

  /* Set CS to low to select the DAC60096  */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Change to 8-bits */
  SPI_SETBITS(dev->spi, 8);

#ifdef DAC60096_SPI_DMA

  /* Transmit the register address from where we want to read. */

  SPI_EXCHANGE(dev->spi, &reg_addr,&ret, 1);

  /* Transmit the content which should be written into the register */

  SPI_EXCHANGE(dev->spi, &buffer,&reg_data, 2);

#else

    /* Transmit the register address from where we want to read. */

    reg_data = SPI_SEND(dev->spi, reg_addr);

    /* Write an idle byte while receiving the requested data */

    reg_data = (uint16_t) (SPI_SEND(dev->spi, 0x00));

#endif

  /* Set CS to high to deselect the DAC60096  */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);

  return reg_data;
}

/****************************************************************************
 * Name: dac60096_spi_write
 *
 * Description:
 *    writr word via SPI
 *
 * Input Parameters:
 *   dev  - device pointer.
 *	 reg_addr  - reg address.
 *	 val  - value.
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
void dac60096_spi_write(FAR struct dac60096_dev_s *dev,uint16_t reg_addr,uint16_t val)
{

  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  SPI_SETFREQUENCY(dev->spi, DAC60096_SPI_FREQUENCY);

  SPI_SETMODE(dev->spi, DAC60096_SPI_MODE);

  /* Change to 8-bits */
  SPI_SETBITS(dev->spi, 8);

  /* Set CS to low to select the DAC60096  */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);


  /* Transmit the register address from where we want to read. */

  SPI_SEND(dev->spi, reg_addr);

  /* Transmit the content which should be written into the register */

  SPI_SEND(dev->spi, (uint8_t)(val>>8));
  SPI_SEND(dev->spi, (uint8_t)(val&0xff));

  /* Set CS to high to deselect the DAC60096  */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}


#endif /* CONFIG_SPI && CONFIG_SPI_EXCHANGE && CONFIG_SENSORS_DAC60096 */

