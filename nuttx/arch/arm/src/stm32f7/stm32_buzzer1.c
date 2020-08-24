/*
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

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/can/can.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include <nuttx/arch/arm/src/chip/stm32_buzzer1.h>
#include "stm32_gpio.h"
#include "stm32f767i-fppa.h"




/* CAN driver methods */

static void stm32buzzer_open(FAR struct buzzer_dev_s *dev);
static void stm32buzzer_close(FAR struct buzzer_dev_s *dev);
static void stm32buzzer_read(FAR struct buzzer_dev_s *dev);
static void stm32buzzer_write(FAR struct buzzer_dev_s *dev,bool enable );

/* private data */
static struct stm32_buzzer_s
{
	uint32_t cfgset;
	uint32_t pinset;
	bool 	 ret;
};

static struct stm32_buzzer_s g_stm32buzzer_priv =
{
		.cfgset = GPIO_BUZZER0,
		.pinset = GPIO_BUZZER0,
		.ret	= NULL,
};

static const struct buzzer_ops_s g_buzzerops =
{
		.bo_open	= &stm32buzzer_open,
		.bo_close   = &stm32buzzer_close,
		.bo_read    = &stm32buzzer_read,
		.bo_write   = &stm32buzzer_write,
};

static struct buzzer_dev_s g_buzzerdev =
{
		.bd_ops		= &g_buzzerops,
		.bd_priv    = &g_stm32buzzer_priv,
};


/* private functions */
static void stm32buzzer_open(FAR struct buzzer_dev_s *dev)
{
    /* config PB7 output, up */
    /* Configure buzzer pins */
	struct stm32_buzzer_s *priv = dev->bd_priv;
    stm32_configgpio(priv->cfgset);
}

static void stm32buzzer_close(FAR struct buzzer_dev_s *dev)
{
	struct stm32_buzzer_s *priv = dev->bd_priv;
	stm32_unconfiggpio(priv->cfgset);
}

static void stm32buzzer_read(FAR struct buzzer_dev_s *dev)
{
	struct stm32_buzzer_s *priv = dev->bd_priv;
	bool tmp = stm32_gpioread(priv->pinset);
	priv->ret = tmp;
}

static void stm32buzzer_write(FAR struct buzzer_dev_s *dev, bool enable)
{
	/* set PB7 1 or 0 */
	struct stm32_buzzer_s *priv = dev->bd_priv;
	stm32_gpiowrite(priv->pinset,enable);
}



/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: stm32_buzzerinitialize
 *
 * Description:
 *   Initialize the buzzer
 *
 * Input Parameters:
 *
 *
 * Returned Value:
 *   Valid buzzer device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct buzzer_dev_s *stm32_buzzerinitialize()
{
	  struct buzzer_dev_s *dev = NULL;

      /* Select the device structure */
      dev = &g_buzzerdev;

      return dev;
}



