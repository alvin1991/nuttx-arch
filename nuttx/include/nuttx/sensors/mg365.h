/******************************************************************************
 * include/nuttx/sensors/mg365.h
 *
 *   Copyright (C) 2017-2018 RAF Research LLC. All rights reserved.
 *   Author: Peng Wei <alvin.pengw@gmail.com>
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

#ifndef __INCLUDE_NUTTX_SENSORS_MG365_H
#define __INCLUDE_NUTTX_SENSORS_MG365_H

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
 * If the user desires a different configuration settings, the the user may
 * either provide a pointer to an array of "struct mg365_reg_pair_s" that
 * will be applied to to the sensor upon open(); or dynamically use
 * the lseek() and write() file_operations functions to set the
 * sensor configuration as desired.
 *
 * When using the sensor from the file_operations interface, the sensor is
 * accessed in Programmed I/O (PIO) mode. (i.e. When the read() function is
 * executed, the sensor is read on that thread.) PIO reads and writes block
 * the calling thread until data is available. Since the sensor is on an SPI
 * bus running at near 10 MHz, the read or write operations should only take
 * a few microseconds (about a microsecond per byte of data), so for
 * individual sensor reads and writes, the overhead of using interrupts or
 * DMA is not worthwhile.
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
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <semaphore.h>
#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/cluster_driver.h>

#include "../../../../apps/include/uORB/uorb/uORB.h"
#include "../../../../apps/include/uORB/topic/mg365_uorb.h"


#define DEG2RAD				0.017453292519943
#define RAD2DEG				57.295779513082323

#if defined(CONFIG_SPI) \
	&& defined(CONFIG_SPI_EXCHANGE) \
	&& defined(CONFIG_SENSORS_EPSON_MG365)

#define EPSON_SPI_SLOW						(uint32_t)(500000)
#define EPSON_SPI_NORMAL					(uint32_t)(1000000)
#define EPSON_SPI_FAST						(uint32_t)(2000000)
#define EPSON_SPI_BURST						(uint32_t)(4000000)

#define EPSONIOC_MEASUREMENT_RATE_SET		_SNIOC(1)
#define EPSONIOC_MEASUREMENT_START			_SNIOC(2)
#define EPSONIOC_MEASUREMENT_STOP			_SNIOC(3)
#define EPSONIOC_MEASUREMENT_INIT			_SNIOC(4)
#define EPSONIOC_MEASUREMENT_ATTI_DELTA		_SNIOC(5)

#define SEEK_ADDR(windows,reg)				(uint16_t)(windows<<8|reg)
#define ADDR2WINDOWS(addr)					(uint16_t)(addr>>8)
#define ADDR2REG(addr)						(uint16_t)(addr&0x00FF)
/*******************************************************************************
 * Pre-processor Definitions
 *******************************************************************************
 */
/*
 * #define EPSON_ACCL_SF         				(.400/65536.0)
 * #define EPSON_GYRO_SF         				(.0151515/65536.0)
 * #define EPSON_ATTI_SF         				(.00699411/65536.0)
 * #define EPSON_TEMP_SF         				(-0.0037918/65536.0)
 */
#define EPSON_ACCL_SF         				(6.104e-9)
#define EPSON_GYRO_SF         				(2.31e-7)
#define EPSON_ATTI_SF         				(1.07e-7)
#define EPSON_TEMP_SF         				(-5.8e-8)

// G365 sensor read output is configured for 32-bit
// Define length in 16-bit words
// # Bytes = Gyro(12) + Accel(12) + ATTI(12) + Count(2) + CHKSM(2)
// Do not count Header/Delimiter byte which is discarded by low-level function
#define SENSOR_READ_LEN 20
// This is defined in bytes and includes Header/Delimiter byte
#define BURSTLEN 42


/*                                      -- Commands --
    - ADDR_ address byte of transfer to select the register to access
    - CMD_  data byte of transfer to write to the register selected

    - All accesses are 16 bit transfers
    - For SPI IF:
        - For SPI write accesses - 8-bit address with msb=1b (can be even or odd) + 8-bit write data
                                 - No response
        - For SPI read accesses - 8-bit address with msb=0b(even only) + 8-bit dummy data
                                - Response is transferred on MOSI on next SPI access
                                - Return value is 16-bit read data (high byte + low byte)
    - For UART IF:
        - For UART write accesses - 8-bit address with msb=1b(can be even or odd) + 8-bit write data + Delimiter Byte
                                  - No response
        - For UART read accesses - 8-bit address with msb=0b(even only) + 8-bit dummy data + Delimiter Byte
                                 - Response is transferred immediately
                                 - Return value consists of Register Read Address + 16-bit read data (high byte + low byte) + Delimiter Byte

    - NOTE: G365/G364/G352/G362/G320 have Register Address Maps that depend on the WINDOW_ID (page) */


// WINDOW_ID 0
#define ADDR_MODE_CTRL_LO          0x02     // MODE_CTRL Byte0 (W0)
#define ADDR_MODE_CTRL_HI          0x03     // MODE_CTRL Byte1 (W0)
#define ADDR_DIAG_STAT             0x04     // DIAG_STAT Byte0 (W0)
#define ADDR_FLAG                  0x06     // FLAG(ND/EA) (W0)
#define ADDR_GPIO                  0x08     // GPIO  (W0)
#define ADDR_COUNT                 0x0A     // COUNT (W0)
#define ADDR_TEMP_HIGH             0x0E     // TEMPC HIGH (W0)
#define ADDR_TEMP_LOW              0x10     // TEMPC LOW  (W0)
#define ADDR_XGYRO_HIGH            0x12     // XGYRO HIGH (W0)
#define ADDR_XGYRO_LOW             0x14     // XGYRO LOW  (W0)
#define ADDR_YGYRO_HIGH            0x16     // YGYRO HIGH (W0)
#define ADDR_YGYRO_LOW             0x18     // YGYRO LOW  (W0)
#define ADDR_ZGYRO_HIGH            0x1A     // ZGYRO HIGH (W0)
#define ADDR_ZGYRO_LOW             0x1C     // ZGYRO LOW  (W0)
#define ADDR_XACCL_HIGH            0x1E     // XACCL HIGH (W0)
#define ADDR_XACCL_LOW             0x20     // XACCL LOW  (W0)
#define ADDR_YACCL_HIGH            0x22     // YACCL HIGH (W0)
#define ADDR_YACCL_LOW             0x24     // YACCL LOW  (W0)
#define ADDR_ZACCL_HIGH            0x26     // ZACCL HIGH (W0)
#define ADDR_ZACCL_LOW             0x28     // ZACCL LOW  (W0)
#define ADDR_ROLL_HIGH             0x64     // ROLL HIGH (W0)
#define ADDR_ROLL_LOW              0x66     // ROLL LOW  (W0)
#define ADDR_PITCH_HIGH            0x68     // PITCH HIGH (W0)
#define ADDR_PITCH_LOW             0x6A     // PITCH LOW  (W0)
#define ADDR_YAW_HIGH              0x6C     // YAW HIGH (W0)
#define ADDR_YAW_LOW               0x6E     // YAW LOW  (W0)

// WINDOW_ID 1
#define ADDR_SIG_CTRL_LO           0x00     // SIG_CTRL Byte0 (W1)
#define ADDR_SIG_CTRL_HI           0x01     // SIG_CTRL Byte1 (W1)
#define ADDR_MSC_CTRL_LO           0x02     // MSC_CTRL Byte0 (W1)
#define ADDR_MSC_CTRL_HI           0x03     // MSC_CTRL Byte1 (W1)
#define ADDR_SMPL_CTRL_LO          0x04     // SMPL_CTRL Byte0 (W1)
#define ADDR_SMPL_CTRL_HI          0x05     // SMPL_CTRL Byte1 (W1)
#define ADDR_FILTER_CTRL_LO        0x06     // FILTER_CTRL Byte0 (W1)
#define ADDR_FILTER_CTRL_HI        0x07     // FILTER_CTRL Byte1 (W1)
#define ADDR_UART_CTRL_LO          0x08     // UART_CTRL Byte0 (W1)
#define ADDR_UART_CTRL_HI          0x09     // UART_CTRL Byte1 (W1)
#define ADDR_GLOB_CMD_LO           0x0A     // GLOB_CMD Byte0 (W1)
#define ADDR_GLOB_CMD_HI           0x0B     // GLOB_CMD Byte1 (W1)
#define ADDR_BURST_CTRL1_LO        0x0C     // BURST_CTRL1 Byte0 (W1)
#define ADDR_BURST_CTRL1_HI        0x0D     // BURST_CTRL1 Byte1 (W1)
#define ADDR_BURST_CTRL2_LO        0x0E     // BURST_CTRL2 Byte0 (W1)
#define ADDR_BURST_CTRL2_HI        0x0F     // BURST_CTRL2 Byte1 (W1)
#define ADDR_POL_CTRL_LO           0x10     // POL_CTRL Byte0 (W1)
#define ADDR_POL_CTRL_HI           0x11     // POL_CTRL Byte1 (W1)
#define ADDR_ATTI_CTRL_LO          0x14     // ATTI_CTRL Byte0 (W1)
#define ADDR_ATTI_CTRL_HI          0x15     // ATTI_CTRL Byte1 (W1)

#define ADDR_PROD_ID1              0x6A     // PROD_ID1(W1)
#define ADDR_PROD_ID2              0x6C     // PROD_ID2(W1)
#define ADDR_PROD_ID3              0x6E     // PROD_ID3(W1)
#define ADDR_PROD_ID4              0x70     // PROD_ID4(W1)
#define ADDR_VERSION               0x72     // VERSION(W1)
#define ADDR_SERIAL_NUM1           0x74     // SERIAL_NUM1(W1)
#define ADDR_SERIAL_NUM2           0x76     // SERIAL_NUM2(W1)
#define ADDR_SERIAL_NUM3           0x78     // SERIAL_NUM3(W1)
#define ADDR_SERIAL_NUM4           0x7A     // SERIAL_NUM4(W1)
#define ADDR_WIN_CTRL              0x7E     // WIN_CTRL(W0 or W1)

#define CMD_BURST                  0x80     // Write value to Issue Burst Read

#define CMD_EN_NDFLAGS             0x7E     // Write value for SIG_CTRL_HI to Enables new data (ND) flags in FLAG for Gyros, Accelerometers
#define CMD_EN_BRSTDATA_LO         0x03     // Write value for BURST_CTRL1_LO to enable CHKSM, and COUNT bytes in burst mode
#define CMD_EN_BRSTDATA_HI         0x31     // Write value for BURST_CTRL1_HI to enable GYRO, ACCL, ATTI registers in burst mode
#define CMD_WINDOW0                0x00     // Write value for WIN_CTRL to change to Window 0
#define CMD_WINDOW1                0x01     // Write value for WIN_CTRL to change to Window 1
#define CMD_RSTCNTR_DRDY           0x44     // Write value for MSC_CTRL_LO to enable EXT_SEL to Reset counter and active low DRDY on GPIO1
#define CMD_32BIT                  0x31     // Write value for BURST_CTRL2_HI to enable 32 bit mode for gyro, accl, attitude data
#define CMD_BEGIN_SAMPLING         0x01     // Write value for MODE_CMD_HI to begin sampling
#define CMD_END_SAMPLING           0x02     // Write value for MODE_CMD_HI to stop sampling
#define CMD_SOFTRESET              0x80     // Write value for GLOB_CMD_LO to issue Software Reset
#define CMD_FLASHTEST              0x08     // Write value for MSC_CTRL_HI to issue Flashtest
#define CMD_SELFTEST               0x04     // Write value for MSC_CTRL_HI to issue Selftest
#define CMD_EULER_ATTI_MODE        0x0C     // Write value for ATTI_CTRL_HI to enable Attitude Output in Euler Mode
#define CMD_EULER_DELTA_MODE       0x0A     // Write value for ATTI_CTRL_HI to enable Delta Angle/Delta Velocity in Euler Mode
#define CMD_INCIDENCE_ATTI_MODE    0x04     // Write value for ATTI_CTRL_HI to enable Delta Angle/Delta Velocity in Incidengce Mode
#define CMD_INCIDENCE_DELTA_MODE   0x02     // Write value for ATTI_CTRL_HI to enable Delta Angle/Delta Velocity in Incidengce Mode
#define CMD_ATTI_CONV              0x00     // Write value for ATTI_CTRL_LO to set Attitude Output Order
#define CMD_ATTI_CONV_FRD          0x02     // Write value for ATTI_CTRL_LO to set Attitude Output Order:FRONT-RIGHT-DOWN

// Write values for ADDR_SMPL_CTRL_HI to set Output Rate
#define CMD_RATE2000               0x00     // TAP>=0
#define CMD_RATE1000               0x01     // TAP>=2
#define CMD_RATE500                0x02     // TAP>=4
#define CMD_RATE250                0x03     // TAP>=8
#define CMD_RATE125                0x04     // TAP>=16
#define CMD_RATE62_5               0x05     // TAP>=32
#define CMD_RATE31_25              0x06     // TAP>=64
#define CMD_RATE15_625             0x07     // TAP=128
#define CMD_RATE400                0x08     // TAP>=8
#define CMD_RATE200                0x09     // TAP>=16
#define CMD_RATE100                0x0A     // TAP>=32
#define CMD_RATE80                 0x0B     // TAP>=32
#define CMD_RATE50                 0x0C     // TAP>=64
#define CMD_RATE40                 0x0D     // TAP>=64
#define CMD_RATE25                 0x0E     // TAP=128
#define CMD_RATE20                 0x0F     // TAP=128

// Write values for FILTER_CTRL_LO to set Filter
#define CMD_FLTAP0                 0x00
#define CMD_FLTAP2                 0x01
#define CMD_FLTAP4                 0x02
#define CMD_FLTAP8                 0x03
#define CMD_FLTAP16                0x04
#define CMD_FLTAP32                0x05
#define CMD_FLTAP64                0x06
#define CMD_FLTAP128               0x07
#define CMD_FIRTAP32FC50           0x08
#define CMD_FIRTAP32FC100          0x09
#define CMD_FIRTAP32FC200          0x0A
#define CMD_FIRTAP32FC400          0x0B
#define CMD_FIRTAP64FC50           0x0C
#define CMD_FIRTAP64FC100          0x0D
#define CMD_FIRTAP64FC200          0x0E
#define CMD_FIRTAP64FC400          0x0F
#define CMD_FIRTAP128FC50          0x10
#define CMD_FIRTAP128FC100         0x11
#define CMD_FIRTAP128FC200         0x12
#define CMD_FIRTAP128FC400         0x13

// MODE STAT
#define VAL_SAMPLING_MODE          0x00
#define VAL_CONFIG_MODE            0x04

//PROD_ID
#define VAL_PROD_ID1			   0x3347
#define VAL_PROD_ID2			   0x3536

//PROD_ID
#define VAL_WAIT_US_STABLE		   800000

/* SPI Bus Parameters */

#define MG365_SPI_FREQUENCY        (EPSON_SPI_FAST)   	/* 2 MHz */
#define MG365_SPI_MODE             (SPIDEV_MODE3) 		/* SPI Mode 3: CPOL=1,CPHA=1 */

/****************************************************************************
 * Private structure definitions
 ****************************************************************************/

/*
 *  Utility struct for the below...
 */

struct mg_reg_pair_s
{
  uint16_t winnum;           				/* Window Number */
  uint16_t addr;           					/* Register Address */
  uint16_t value;           				/* Value to be stored in the above reg on open() */
};

struct mg365_dvr_entry_vector_s
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
};

/*
 * Configration of mg365
 */
struct mg_config_s
{
  /* Since multiple MG365 can be connected to the same SPI bus we need
   * to use multiple SPI device ids which are employed by NuttX to select/
   * deselect the desired MG365 chip via their chip select inputs.
   */
  int spi_devid;

  /* The initial value store operations will occur in the order they
   * appear in the array.
   */
  struct mg_reg_pair_s *initial_cr;
  int initial_cr_len;

  char enable_atti_delta;

  /* Pointer to the leaf driver's sensor_cluster_operations_s structure */
  FAR struct mg365_dvr_entry_vector_s *sc_ops;
};

/*
 * EPSON MG365 device struct
 */
struct mg365_dev_s
{
  FAR struct spi_dev_s *spi;         /* Pointer of the SPI instance */

  sem_t devicesem;                   /* Manages exclusive access to this device */
  sem_t datasem;                     /* Manages exclusive access to this structure */
  uint8_t cd_ocount;        		 /* The number of times the device has been opened */
  uint16_t seek_address;             /* Current device address. */
  uint8_t readonly;                  /* 0 = writing to the device in enabled */

  struct mg_config_s	*config;	 /* Configration of spi and register */

  unsigned int		current_windows; /* register current page */
  uint8_t			tx[10];			 /* SPI translate buff */
  uint8_t			rx[4];			 /* SPI received buff */

  /* work queue data */
  struct work_s		_work;
  bool 				_running;		 /* true: measure in work queue false:not measure*/
  unsigned			_mesure_ticks;

  /* ORB data */
  orb_advert_t uorb_pub;			 /* ORB topic advertiser handle  */
  struct mg365_uorb_s   data;
};

/****************************************************************************
 * Name: mg365_spi_config_initialize
 *
 * Description:
 *   return the mg365 config_s
 *
 * Input Parameters:
 *   dev - number of mg365_dev_s.
 *
 * Returned Value:
 *   config_s pointer
 *
 ****************************************************************************/
struct mg_config_s * mg365_spi_config_initialize(int dev);

/****************************************************************************
 * Name: mg365_register
 *
 * Description:
 *   Register the MG365 character device as 'devpath'
 *
 * Input Parameters:
 *   spi     - An instance of the SPI interface to use to communicate with
 *             MG365
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
int mg365_register(FAR struct spi_dev_s *spi,FAR int num);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SPI && CONFIG_SENSORS_EPSON_MG365 && CONFIG_SPI_EXCHANGE */
#endif /* __INCLUDE_NUTTX_SENSORS_MG365_H */
