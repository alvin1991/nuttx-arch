/******************************************************************************
 * include/nuttx/sensors/dac60096.h
 *
 *   Copyright (C) 2019-2020 Alivn Peng. All rights reserved.
 *   Author: Alivn Peng <alivn.peng@gmail.com.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#ifndef __INCLUDE_NUTTX_SENSORS_DAC60096_H
#define __INCLUDE_NUTTX_SENSORS_DAC60096_H

/******************************************************************************
 * Driver usage notes:
 *
 * This driver is a "kernel sensor leaf driver" that may be used directly from
 * user applications via the file_operations interface or have selected entry
 * points called directly from a "kernel sensor cluster driver".
 *
 * To use this driver via the file_operations interface, the board
 * initialization function should call this driver's registration function.
 * The driver will register itself with Nuttx under the /dev path that is
 * provided by the config structure.  Then user applications may access the
 * driver via the "file descriptor handle" returned by the file_operations
 * open() function.
 *
 * This driver supports the Common Sensor Register Interface.
 * See drivers/sensors/README.txt for details.
 *
 * This driver supports the Sensor Cluster Driver Interface.
 * See drivers/sensors/README.txt for details.
 *
 * It also extends the interface by permitting cluster driver calls to
 * a function that is intended to perform high performance DMA SPI exchange
 * operations. See the usage note on the exchange operation below.
 *
 ****************************************************************************/

/*******************************************************************************
 * Included Files
 *******************************************************************************
 */

#include <nuttx/arch.h>

#include <semaphore.h>
#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/cluster_driver.h>

#if defined(CONFIG_SPI) \
	&& defined(CONFIG_SPI_EXCHANGE) \
	&& defined(CONFIG_SENSORS_DAC60096)

#define DACIOC_CONFIG_DEFAULT		_SNIOC(1)
#define DACIOC_SET_CHANNEL_VALUE	_SNIOC(2)
#define DACIOC_GET_CHANNEL_VALUE	_SNIOC(3)
#define DACIOC_ENABLE_OUTPUT		_SNIOC(4)
#define DACIOC_SOFT_RESET			_SNIOC(5)
#define DACIOC_CLEAR_STATE			_SNIOC(6)
#define DACIOC_SELECT_MODE			_SNIOC(7)
/*******************************************************************************
 * Pre-processor Definitions
 *******************************************************************************
 */
#define BIT(x,nr)					(x << (nr))

#define CHANNEL_CHECK(ch)			(ch<96?1:0)
#define VALUE_CHECK(val)			(abs(val)<2048?1:0)

/* DAC60096 R/W = 0 sets a write operation. R/W = 1 sets a read operation. */
/* DAC60096 S = 0 is used for single command instructions. Bit = 1 is used for streaming operation. */

#define DAC60096_READ_REG_SIGNAL(reg)		(0x80 | (0xbf & (reg<<2)))
#define DAC60096_WRITE_REG_SIGNAL(reg)		(0x7f & (0xbf & (reg<<2)))

#define DAC60096_READ_REG_STREAM(reg)		(0x80 | (0x40 | (reg<<2)))
#define DAC60096_WRITE_REG_STREAM(reg)		(0x7f & (0x40 | (reg<<2)))

/* DAC60096 Register Maps */
#define DAC60096_REG_BUF_A					0x00		///< BUFA Register (address = 0x0) [reset = 0x0000]
#define DAC60096_REG_BUF_B					0x01		///< BUFB Register (address = 0x1) [reset = 0x0000]
#define DAC60096_REG_CON					0x04		///< CON Register (address = 0x4) [reset = 0x0555]
#define DAC60096_REG_CRC					0x05		///< CRC Register (address = 0x5) [reset = 0xFFF]
#define DAC60096_REG_PTR					0x06		///< PTR Register (address = 0x6) [reset = 0x0000]
#define DAC60096_REG_SWR					0x07		///< SWR Register (address = 0x7) [reset = 0x0000]
#define DAC60096_REG_PWRM					0x08		///< PWRM Register (address = 0x8) [reset = 0xCAFE]
#define DAC60096_REG_SDIV					0x09		///< SDIV Register (address = 0x9) [reset = 0x0000]

/* CON Register parameters */
#define DAC60096_CON_LDAC_TRIGGER			BIT(0x01,15)		///< Trigg
#define DAC60096_CON_SDO1x					BIT(0x01,10)		///< 1x drive strength
#define DAC60096_CON_SDO2x					BIT(0x02,10)		///< 2x drive strength
#define DAC60096_CON_SDRV_HIZ				BIT(0x01,8)			///< Hi-Z. STATS pin is disabled (default)
#define DAC60096_CON_SDRV_PP				BIT(0x02,8)			///< CMOS Push-pull output. Should only be enabled for subsystem 1.
#define DAC60096_CON_PHAINV_SCLK_NEG_EDGE	BIT(0x01,6)			///< SCLK NegEdge (default)
#define DAC60096_CON_PHAINV_SCLK_POS_EDGE	BIT(0x02,6)			///< SCLK PosEdge
#define DAC60096_CON_CLRDAC_NORMAL			BIT(0x01,4)			///< Normal operating state (default)
#define DAC60096_CON_CLRDAC_CLEAR			BIT(0x02,4)			///< Clear DAC state
#define DAC60096_CON_APB_AUTO_ENABLE		BIT(0x01,0)			///< Auto-populates BUFB with the negative value of BUFA
#define DAC60096_CON_APB_AUTO_DISABLE		BIT(0x02,0)			///< Disable auto populate B feature

/* PTR Register parameters */
#define DAC60096_PTR_SID_00					BIT(0x00,12)		///< Subsystem 1
#define DAC60096_PTR_SID_01					BIT(0x01,12)		///< Subsystem 2
#define DAC60096_PTR_SID_02					BIT(0x02,12)		///< Subsystem 3
#define DAC60096_PTR_SID_03					BIT(0x03,12)		///< Subsystem 4

/* PWRM Register parameters */
#define DAC60096_SDIV_0						BIT(0x00,0)		///< STATS pin toggling rate
#define DAC60096_SDIV_1						BIT(0x01,0)		///< STATS pin toggling rate
#define DAC60096_SDIV_2						BIT(0x02,0)		///< STATS pin toggling rate
#define DAC60096_SDIV_3						BIT(0x03,0)		///< STATS pin toggling rate
#define DAC60096_SDIV_4						BIT(0x04,0)		///< STATS pin toggling rate
#define DAC60096_SDIV_5						BIT(0x05,0)		///< STATS pin toggling rate
#define DAC60096_SDIV_6						BIT(0x06,0)		///< STATS pin toggling rate

/* SDIV Register parameters */
#define DAC60096_PWRM_POWER_UP1				0xABBA		///< After power-up the PWRM register toggles to 0xABBA.
#define DAC60096_PWRM_POWER_UP2				0xBAAB		///< After power-up the PWRM register toggles to 0xBAAB.
#define DAC60096_PWRM_POWER_FAILED			0xCAFE		///< After power-up the PWRM register is set to 0xCAFE.
#define DAC60096_SWR_SOFT_RESET				0xA5A5		///< Subsystem software reset
#define DAC60096_VAL_WAIT_US_STABLE			800000

/* SPI Bus Parameters */
#define DAC60096_SPI_SLOW				(uint32_t)(300000)
#define DAC60096_SPI_NORMAL				(uint32_t)(1000000)
#define DAC60096_SPI_FAST				(uint32_t)(2000000)
#define DAC60096_SPI_BURST				(uint32_t)(4000000)

#define DAC60096_SPI_FREQUENCY		   	(DAC60096_SPI_FAST)   /* 4 MHz */
#define DAC60096_SPI_MODE				(SPIDEV_MODE3) 		  /* SPI Mode 3: CPOL=1,CPHA=1 */

/****************************************************************************
 * Private structure definitions
 ****************************************************************************/
enum dac60096_mode_e{
	SYNC_DC = 0,
	SYNC_TOGGLE,
	ASYNC_DC,
	ASYNC_TOGGLE,
	DAC60096_MODE_MAX
};

struct dac60096_chan_value_s
{
  uint16_t channel;           				/* Channel Address */
  int16_t value;           					/* Value to be stored */
};

struct dac60096_reg_pair_s
{
  uint16_t addr;           					/* Register Address */
  uint16_t value;           				/* Value to be stored in the above reg on open() */
};

struct dac60096_dvr_entry_vector_s
{
  struct sensor_cluster_operations_s c;

  /* Extend the sensor cluster driver interface with a SPI DMA exchange transfer.
   * The standard driver_read and driver_write perform PIO transfers.
   * The will loop waiting on the SPI hardware and are only appropriate for
   * short data transfers.
   * Note that the first byte in the tx buffer must be a command/address
   * byte. The exchange function does not provide one. Also note that
   * the first byte stored in the rxbuffer is a garbage byte, which
   * is natural for a SPI exchange transfer. Plan your buffer accordingly.
   */

  CODE void (*driver_spiexc)(FAR void *instance_handle,
                             FAR const void *txbuffer,
                             FAR void *rxbuffer, size_t nwords);

  /*
   * Extend the sensor cluster driver interface with work mode select.
   */
  CODE void (*select)(FAR void *instance_handle,
		  	  	  	  FAR enum dac60096_mode_e mode);
};

struct dac60096_config_s
{
	/* Since multiple MG365 can be connected to the same SPI bus we need
	* to use multiple SPI device ids which are employed by NuttX to select/
	* deselect the desired MG365 chip via their chip select inputs.
	*/
	int spi_devid;

	/* The initial value store operations will occur in the order they
	* appear in the array.
	*/
	struct dac60096_reg_pair_s *initial_cr;
	int initial_cr_len;

	char enable_atti_delta;

	/* Pointer to the leaf driver's sensor_cluster_operations_s structure */
	FAR struct dac60096_dvr_entry_vector_s *sc_ops;
};

struct dac60096_dev_s
{
	FAR struct spi_dev_s			*spi;		/* Pointer to the SPI instance */
	FAR struct dac60096_config_s	*config;	/* Pointer to the configuration of the DAC60096 sensor */
	sem_t							devicesem;	/* Manages exclusive access to this device */
	sem_t							datasem;	/* Manages exclusive access to this structure */
	uint8_t							cd_ocount;	/* The number of times the device has been opened */
	uint16_t						seek_address;/* Current device address. */
	uint8_t							readonly;	/* 0 = writing to the device in enabled */
	enum dac60096_mode_e			mode;		/* The device work mode */

};


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

struct dac60096_config_s * dac60096_spi_config_initialize(int dev);

/****************************************************************************
 * Name: dac60096_register
 *
 * Description:
 *   Register the MG365 character device as 'devpath'
 *
 * Input Parameters:
 *   spi      - An instance of the SPI interface to use to communicate with
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int dac60096_register(FAR struct spi_dev_s *spi,FAR int num);


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
void dac60096_select(FAR void *instance_handle,FAR enum dac60096_mode_e mode);

#endif /* CONFIG_SPI && CONFIG_SENSORS_DAC60096 && CONFIG_SPI_EXCHANGE */
#endif /* __INCLUDE_NUTTX_SENSORS_DAC60096_H */
