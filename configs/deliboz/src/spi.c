/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *         Author: Jukka Laitinen <jukka.laitinen@intel.com>
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

/**
 * @file spi.c
 *
 * Board-specific SPI functions.
 */

/*******************************************************************************
 * Included Files
 ******************************************************************************/

#include <px4_config.h>
#include <px4_log.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/spi/slave.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "chip.h"
#include "stm32.h"
#include "board_config.h"

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/*******************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI for the board.
 *
 ******************************************************************************/

__EXPORT void stm32_spiinitialize(void)
{
	stm32_configgpio(GPIO_SPI_CS1_IMU1);
	stm32_configgpio(GPIO_SPI_CS2_IMU1);
	stm32_configgpio(GPIO_SPI_CS1_IMU2);
	stm32_configgpio(GPIO_SPI_CS2_IMU2);
	stm32_configgpio(GPIO_SPI_CS_TTY_OUT);
	stm32_configgpio(GPIO_SPI_CS_TTY_IN);
}

__EXPORT void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                               bool selected)
{
	/* SPI select is active low, so write !selected to select the device */
	switch(PX4_SPI_DEV_ID((int)devid)) {
	case 0:
		px4_arch_gpiowrite(GPIO_SPI_CS1_IMU1, !selected);
		break;
	case 1:
		px4_arch_gpiowrite(GPIO_SPI_CS2_IMU1, !selected);
		break;
	default:
		PX4_ERR("Unknown SPI device %d\n",devid);
	}
}

__EXPORT uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}

__EXPORT void stm32_spi2select(FAR struct spi_dev_s *dev, uint32_t devid,
                               bool selected)
{
	/* SPI select is active low, so write !selected to select the device */
	switch(PX4_SPI_DEV_ID((int)devid)) {
	case 0:
		px4_arch_gpiowrite(GPIO_SPI_CS1_IMU2, !selected);
		break;
	case 1:
		px4_arch_gpiowrite(GPIO_SPI_CS2_IMU2, !selected);
		break;
	default:
		PX4_ERR("Unknown SPI device %d\n",devid);
	}
}

__EXPORT uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}

__EXPORT void stm32_spi4select(FAR struct spi_dev_s *dev, uint32_t devid,
                               bool selected)
{
	/* SPI select is active low, so write !selected to select the device */
	px4_arch_gpiowrite(GPIO_SPI_CS_TTY_OUT, !selected);
}

__EXPORT uint8_t stm32_spi4status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}

__EXPORT void stm32_spi5select(FAR struct spi_dev_s *dev, uint32_t devid,
                               bool selected)
{
}

__EXPORT uint8_t stm32_spi5status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}

__EXPORT void board_spi_reset(int ms)
{
}

__EXPORT bool px4_spi_bus_external(int bus)
{
	return false;
}
