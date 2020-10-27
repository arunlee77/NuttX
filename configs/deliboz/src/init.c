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

/**
 * @file init.c
 *
 * deliboz-specific early startup code.  This file implements the
 * board_app_initialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialization.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>

#include <stm32.h>
#include "board_config.h"
#include "stm32_uart.h"

#include <arch/board/board.h>

#include "up_arch.h"
#include "stm32_start.h"
/****************************************************************************
 * Private Data
 ****************************************************************************/
static int hw_version = 0;
static int hw_revision = 0;
static char hw_type[4] = HW_VER_TYPE_INIT;

static const uint8_t hw_id_map[] = {
	BOARD_VERSION_CARRIER_EVT,
	BOARD_VERSION_CARRIER_EVT2,
	0,
	0,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: board_detect_hw_version
 *
 * Description: Detect board HW version by checking GPIO pin(s)
 *
 ************************************************************************************/
void board_detect_hw_version(void)
{
	uint8_t id_pins = (stm32_gpioread(GPIO_REDUNDANCY_DETECT_2)^1) |
		( (stm32_gpioread(GPIO_REDUNDANCY_DETECT_3)^1) << 1);

	hw_version = hw_id_map[id_pins];
}

/************************************************************************************
 * Name: board_peripheral_reset
 *
 * Description:
 *
 ************************************************************************************/
void board_peripheral_reset(int ms)
{
}

/****************************************************************************
 * Protected Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void stm32_boardinitialize(void)
{
	/* configure LEDs */

	/* configure SPI interfaces */
	//stm32_spiinitialize();

	/* configure sensor interrupt GPIOs */
	stm32_configgpio(GPIO_DRDY1_IMU1);
	stm32_configgpio(GPIO_DRDY2_IMU1);
	stm32_configgpio(GPIO_DRDY1_IMU2);
	stm32_configgpio(GPIO_DRDY2_IMU2);

	/* configure heaters */
	if (board_is_redundant_fc()) {
		stm32_configgpio(GPIO_IMU_HEATER3);
	} else {
		stm32_configgpio(GPIO_IMU_HEATER2);
		stm32_configgpio(GPIO_IMU_HEATER1);
	}

	/* configure version id pins */
	stm32_configgpio(GPIO_REDUNDANCY_DETECT_1);
	stm32_configgpio(GPIO_REDUNDANCY_DETECT_2);
	stm32_configgpio(GPIO_REDUNDANCY_DETECT_3);

	/* configure PPS and external camera trigger GPIOs */
	stm32_configgpio(GPIO_TIMESYNC1);
	stm32_configgpio(GPIO_TIMESYNC2);
	stm32_configgpio(GPIO_GPS_PPS);

	/* Detect hw version */
	board_detect_hw_version();

	/* Reset all sensors and configure spi */
	//board_spi_reset(0);

	/* Initialize redundancy fc functions */
	board_init_redundancy();

	/* configure rs485 motor control on NAV2 */
	if (board_is_redundant_fc()) {
		/* rs485 */
		stm32_configgpio(GPIO_NAV2_RS485_EN_N);
		stm32_configgpio(GPIO_NAV2_UARTCAN_EN_N);
		//stm32_configgpio(GPIO_USART1_RS485_DIR);

		/* charger */
		stm32_configgpio(GPIO_CHG_DIS);
		stm32_configgpio(GPIO_CHG_DET_H);
		stm32_configgpio(GPIO_CHG_DET_L);

		/* disconnected pin, configure as input */
		stm32_configgpio(GPIO_DISCON1);

		/* battery change indicator */
		stm32_configgpio(GPIO_BAT_CHG_IND);
		stm32_configgpio(GPIO_AUX_IO_1);
		stm32_configgpio(GPIO_AUX_IO_2);

		/* taisync enable */
		stm32_configgpio(GPIO_TAISYNC_EN);
	} else {
		/* rs485 */
		stm32_configgpio(GPIO_NAV1_RS485_EN_N);
		stm32_configgpio(GPIO_NAV1_UARTCAN_EN_N);

		/* configure PWM out pins */
#if 0 // For EVT1 /DJI escs
		stm32_configgpio(GPIO_TIM1_CH1OUT);
		stm32_configgpio(GPIO_TIM1_CH2OUT);
		stm32_configgpio(GPIO_TIM1_CH3OUT);
		stm32_configgpio(GPIO_TIM1_CH4OUT);
		stm32_configgpio(GPIO_TIM8_CH1OUT);
		stm32_configgpio(GPIO_TIM8_CH2OUT);
		stm32_configgpio(GPIO_TIM8_CH3OUT);
		stm32_configgpio(GPIO_TIM8_CH4OUT);
#else
		stm32_configgpio(GPIO_MOTOR_SELECT0);
		stm32_configgpio(GPIO_MOTOR_SELECT1);
		stm32_configgpio(GPIO_MOTOR_SELECT2);
		stm32_configgpio(GPIO_MOTOR_SELECT3);
		stm32_configgpio(GPIO_MOTOR_SELECT4);
		stm32_configgpio(GPIO_MOTOR_SELECT5);
		stm32_configgpio(GPIO_MOTOR_SELECT6);
		stm32_configgpio(GPIO_MOTOR_SELECT7);
#endif
		/* configure XBee select and antenna sel */
		stm32_configgpio(GPIO_ANT_CTRL1);
		stm32_configgpio(GPIO_XBEE_SELECT);

		/* enable gimbal */
		stm32_configgpio(GPIO_GIMBAL_EN);

		/* enable PCIE-USB bridge and USB load switch */
		stm32_configgpio(GPIO_PL1_USB_LOAD_SW);
		stm32_configgpio(GPIO_PL2_PCIE_USB_BRIDGE_EN);
	}
}

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y && CONFIG_NSH_ARCHINIT:
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;

  UNUSED(ret);

#ifdef CONFIG_FS_PROCFS
#ifdef CONFIG_STM32_CCM_PROCFS
  /* Register the CCM procfs entry.  This must be done before the procfs is
   * mounted.
   */

  (void)ccm_procfs_register();
#endif  /* CONFIG_STM32_CCM_PROCFS */

  /* Mount the procfs file system */

  ret = mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d (%d)\n",
             ret, errno);
    }
#endif  /* CONFIG_FS_PROCFS */

#ifdef CONFIG_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif  /* CONFIG_BUTTONS */

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc_setup failed: %d\n", ret);
    }
#endif  /* CONFIG_ADC */

#ifdef CONFIG_SENSORS_LSM6DSL
  ret = stm32_lsm6dsl_initialize("/dev/lsm6dsl0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize LSM6DSL driver: %d\n", ret);
    }
#endif  /* CONFIG_SENSORS_LSM6DSL */

#ifdef CONFIG_SENSORS_LSM303AGR
  ret = stm32_lsm303agr_initialize("/dev/lsm303mag0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize LSM303AGR driver: %d\n", ret);
    }
#endif  /* CONFIG_SENSORS_LSM303AGR */

#ifdef CONFIG_WL_NRF24L01
  ret = stm32_wlinitialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize wireless driver: %d\n", ret);
    }
#endif  /* CONFIG_WL_NRF24L01 */

#if defined(CONFIG_CDCACM) && !defined(CONFIG_CDCACM_CONSOLE)
  /* Initialize CDCACM */

  syslog(LOG_INFO, "Initialize CDCACM device\n");

  ret = cdcacm_initialize(0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: cdcacm_initialize failed: %d\n", ret);
    }
#endif  /* CONFIG_CDCACM & !CONFIG_CDCACM_CONSOLE */

  return OK;
}

/************************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional initialization call
 *   will be performed in the boot-up sequence to a function called
 *   board_late_initialize().  board_late_initialize() will be called immediately after
 *   up_initialize() is called and just before the initial application is started.
 *   This additional initialization phase may be used, for example, to initialize
 *   board-specific device drivers.
 *
 ************************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
#if defined(CONFIG_NSH_LIBRARY) && !defined(CONFIG_LIB_BOARDCTL)
  /* Perform board bring-up here instead of from the board_app_initialize(). */

  (void)stm32_bringup();
#endif
}
#endif

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
#if defined(FLASH_BASED_PARAMS)
	int result;
#endif
#ifdef CONFIG_BOARD_LATE_INITIALIZE
	return OK;
#endif

#if defined(CONFIG_HAVE_CXX) && defined(CONFIG_HAVE_CXXINITIALIZE)

	/* run C++ ctors before we go any further */

	up_cxxinitialize();

#	if defined(CONFIG_EXAMPLES_NSH_CXXINITIALIZE)
#  		error CONFIG_EXAMPLES_NSH_CXXINITIALIZE Must not be defined! Use CONFIG_HAVE_CXX and CONFIG_HAVE_CXXINITIALIZE.
#	endif

#else
#  error platform is dependent on c++ both CONFIG_HAVE_CXX and CONFIG_HAVE_CXXINITIALIZE must be defined.
#endif

#if defined(FLASH_BASED_PARAMS)
	static sector_descriptor_t params_sector_map[] = {
		{12, 128 * 1024, 0x08180000},
		{13, 128 * 1024, 0x081A0000},
		{0, 0, 0},
	};

	/* Initialize the flashfs layer to use heap allocated memory */
	result = parameter_flashfs_init(params_sector_map, NULL, 0);

	if (result != OK) {
		led_on(LED_AMBER);
		return -ENODEV;
	}

#endif

#ifdef CONFIG_MMCSD
	int ret = stm32_sdio_initialize();

	if (ret != OK) {
		return ret;
	}

#endif
#ifdef CONFIG_PWM
	stm32h7_pwm_setup();
#endif

	return OK;
}

/*******************************************************************************
 * Name: board_get_hw_type_name
 *
 * Description:
 *   Optional returns a string defining the HW type
 *
 *
 ******************************************************************************/

const char *board_get_hw_type_name()
{
	return (const char *) hw_type;
}

/*******************************************************************************
 * Name: board_get_hw_version
 *
 * Description:
 *   Optional returns a integer HW version
 *
 *
 ******************************************************************************/

int board_get_hw_version()
{
	return hw_version;
}

/*******************************************************************************
 * Name: board_get_hw_revision
 *
 * Description:
 *   Optional returns a integer HW revision
 *
 *
 ******************************************************************************/

int board_get_hw_revision()
{
	return hw_revision;
}
