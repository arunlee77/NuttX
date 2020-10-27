/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file board_config.h
 *
 * DELIBOZ internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/compiler.h>
#include <stdint.h>

#include "stm32.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Configuration **************************************************************/

#define BOARD_HAS_SIMPLE_HW_VERSIONING 1
#define HW_VER_TYPE_INIT {'V','1',0, 0}


/* PX4FMU GPIOs ***************************************************************/

/* LEDs */

/*
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_50MHz))

/*
 * GPIOs to detect redundancy fc
 */
#define GPIO_REDUNDANCY_DETECT_1 (GPIO_INPUT|GPIO_PULLUP|GPIO_SPEED_2MHz|GPIO_PORTE|GPIO_PIN10)
#define GPIO_REDUNDANCY_DETECT_2 (GPIO_INPUT|GPIO_PULLUP|GPIO_SPEED_2MHz|GPIO_PORTE|GPIO_PIN11)
#define GPIO_REDUNDANCY_DETECT_3 (GPIO_INPUT|GPIO_PULLUP|GPIO_SPEED_2MHz|GPIO_PORTE|GPIO_PIN12)


/* PWM
 *
 * 8  PWM outputs are configured.
 *
 * Pins:
 *
 * FMU_CH1 : PA8 : TIM1_CH1OUT_1
 * FMU_CH2 : PA9 : TIM1_CH2OUT_1
 * FMU_CH3 : PA10 : TIM1_CH3OUT_1
 * FMU_CH4 : PA11 : TIM1_CH4OUT_1
 * FMU_CH5 : PI5 : TIM8_CH1OUT_2
 * FMU_CH6 : PI6 : TIM8_CH2OUT_2
 * FMU_CH7 : PI7 : TIM8_CH3OUT_2
 * FMU_CH8 : PI2 : TIM8_CH4OUT_2
 *
 */
#define GPIO_TIM1_CH1OUT GPIO_TIM1_CH1OUT_1
#define GPIO_TIM1_CH2OUT GPIO_TIM1_CH2OUT_1
#define GPIO_TIM1_CH3OUT GPIO_TIM1_CH3OUT_1
#define GPIO_TIM1_CH4OUT GPIO_TIM1_CH4OUT_1

#define GPIO_TIM8_CH1OUT GPIO_TIM8_CH1OUT_2
#define GPIO_TIM8_CH2OUT GPIO_TIM8_CH2OUT_2
#define GPIO_TIM8_CH3OUT GPIO_TIM8_CH3OUT_2
#define GPIO_TIM8_CH4OUT GPIO_TIM8_CH4OUT_2

#define DIRECT_PWM_OUTPUT_CHANNELS  6
#define BOARD_HAS_PWM    DIRECT_PWM_OUTPUT_CHANNELS

#define GPIO_TIM1_CH1IN GPIO_TIM1_CH1IN_1
#define GPIO_TIM1_CH2IN GPIO_TIM1_CH2IN_1
#define GPIO_TIM1_CH3IN GPIO_TIM1_CH3IN_1
#define GPIO_TIM1_CH4IN GPIO_TIM1_CH4IN_1

#define GPIO_TIM8_CH1IN GPIO_TIM8_CH1IN_2
#define GPIO_TIM8_CH2IN GPIO_TIM8_CH2IN_2
#define GPIO_TIM8_CH3IN GPIO_TIM8_CH3IN_2
#define GPIO_TIM8_CH4IN GPIO_TIM8_CH4IN_2

/* The same timer GPIOs are used for motor selection
   when PWM is not used */
#define GPIO_MOTOR_SELECT0 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)
#define GPIO_MOTOR_SELECT1 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN9)
#define GPIO_MOTOR_SELECT2 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN10)
#define GPIO_MOTOR_SELECT3 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN11)
#define GPIO_MOTOR_SELECT4 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTI|GPIO_PIN5)
#define GPIO_MOTOR_SELECT5 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTI|GPIO_PIN6)
#define GPIO_MOTOR_SELECT6 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTI|GPIO_PIN7)
#define GPIO_MOTOR_SELECT7 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTI|GPIO_PIN2)

#define ASCTEC_ESC_GPIO_MOTOR_SELECT						\
	{														\
		GPIO_MOTOR_SELECT0,									\
		GPIO_MOTOR_SELECT1,									\
		GPIO_MOTOR_SELECT2,									\
		GPIO_MOTOR_SELECT3,									\
		GPIO_MOTOR_SELECT4,									\
		GPIO_MOTOR_SELECT5,									\
		GPIO_MOTOR_SELECT6,									\
		GPIO_MOTOR_SELECT7									\
	}

#define DJI_ESC_POS {1, 2, 3, 4, 5, 0}

/*
 * I2C busses
 *
 * Peripheral   Port

 * compass & baro
 * I2C1_SDA     PB7
 * I2C1_SCL     PB6

 * 2nd compass & baro
 * I2C2_SDA     PF0
 * I2C2_SCL     PF1

 * external sensor module
 * I2C3_SDA     PH8
 * I2C3_SCL     PH7

 * debug i2c
 * I2C4_SDA     PH12
 * I2C4_SCL     PH11
 */

#define BOARD_NUMBER_I2C_BUSES  4
#define BOARD_I2C_BUS_CLOCK_INIT {100000, 100000, 100000, 100000}

#define PX4_I2C_BUS_ONBOARD	1
#define PX4_I2C_BUS_ONBOARD2	2
#define PX4_I2C_BUS_EXPANSION	3


#define BMM150_ONBOARD_ADDR1 0x10
#define BMM150_ONBOARD2_ADDR1 0x10
#define BMM150_ONBOARD2_ADDR2 0x13
#define BMM150_EXPANSION_ADDR1 0x11
#define BMM150_EXPANSION_ADDR2 0x11

#define GPIO_TIMESYNC1 (GPIO_INPUT|GPIO_PULLDOWN|GPIO_EXTI|GPIO_PORTA|GPIO_PIN0)
#define GPIO_TIMESYNC2 (GPIO_INPUT|GPIO_PULLDOWN|GPIO_EXTI|GPIO_PORTA|GPIO_PIN3)
#define GPIO_GPS_PPS (GPIO_INPUT|GPIO_PULLDOWN|GPIO_EXTI|GPIO_PORTG|GPIO_PIN7)

#define GPIO_SPI_CS1_IMU1	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN10)
#define GPIO_SPI_CS2_IMU1	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN4)
#define GPIO_DRDY1_IMU1 (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTG|GPIO_PIN2)
#define GPIO_DRDY2_IMU1 (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTG|GPIO_PIN3)

#define GPIO_SPI_CS1_IMU2	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN0)
#define GPIO_SPI_CS2_IMU2	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN1)
#define GPIO_DRDY1_IMU2 (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTG|GPIO_PIN4)
#define GPIO_DRDY2_IMU2 (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTG|GPIO_PIN5)

#define SPI_BUS_IMU1 1
#define SPI_BUS_IMU2 2
#define SPI_BUS_ETH 2
#define SPI_BUS_TTY_OUT 4
#define SPI_BUS_TTY_IN 5

// TODO: remove this
#define PX4_SPIDEV_ID(type, index)  ((((type) & 0xffff) << 16) | ((index) & 0xffff))

#define PX4_SPI_DEVICE_ID         (1 << 12)
#define PX4_MK_SPI_SEL(b,d)       PX4_SPIDEV_ID(PX4_SPI_DEVICE_ID, ((((b) & 0xff) << 8) | ((d) & 0xff)))
#define PX4_SPI_BUS_ID(devid)     (((devid) >> 8) & 0xff)
#define PX4_SPI_DEV_ID(devid)     ((devid) & 0xff)
#define PX4_CHECK_ID(devid)       ((devid) & PX4_SPI_DEVICE_ID)

#define PX4_SPI_BUS_SENSORS	SPI_BUS_IMU1
#define PX4_SPI_BUS_SENSORS2	SPI_BUS_IMU2

#define PX4_SPIDEV_MPU		PX4_MK_SPI_SEL(SPI_BUS_IMU1, 0) // spi1 cs 0
#define PX4_SPIDEV_LSM		PX4_MK_SPI_SEL(SPI_BUS_IMU1, 1) // spi1 cs 1

#define PX4_SPIDEV_MPU2		PX4_MK_SPI_SEL(SPI_BUS_IMU2, 0) // spi2 cs 0
#define PX4_SPIDEV_LSM2		PX4_MK_SPI_SEL(SPI_BUS_IMU2, 1) // spi2 cs 1
#define PX4_SPIDEV_ETH		PX4_MK_SPI_SEL(SPI_BUS_ETH, 0) // spi2 cs 0
#define PX4_SPIDEV_TTY_OUT  PX4_MK_SPI_SEL(SPI_BUS_TTY_OUT, 0) // spi4 cs 0
#define PX4_SPIDEV_TTY_IN   PX4_MK_SPI_SEL(SPI_BUS_TTY_IN, 0) // spi5 cs 0

#define GPIO_SPI_CS_TTY_OUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)
#define GPIO_SPI_CS_TTY_IN	GPIO_SPI5_NSS_2 /* PH5 */

#define ADC_CHANNELS 0
#define BOARD_NUMBER_BRICKS 0
#define GPIO_IMU_HEATER1 (GPIO_ALT|GPIO_AF2|GPIO_PORTB|GPIO_PIN8)
#define GPIO_IMU_HEATER2 (GPIO_ALT|GPIO_AF2|GPIO_PORTB|GPIO_PIN9)
#define GPIO_IMU_HEATER3 (GPIO_ALT|GPIO_AF2|GPIO_PORTB|GPIO_PIN8)

#define GPIO_SDMMC1_NCD  (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI | GPIO_PORTG | GPIO_PIN6)

// Pins specific to NAV1

/* XBee control pins */
//#define GPIO_ANT_CTRL1 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN8)
//#define GPIO_XBEE_SELECT (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN10)
#define GPIO_ANT_CTRL1 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTI|GPIO_PIN8)
#define GPIO_XBEE_SELECT (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTI|GPIO_PIN10)

// For EVT1 (TODO: automatic detection)
//#define GPIO_NAV1_UARTCAN_EN_N (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN1)
//#define GPIO_NAV1_RS485_EN_N (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)

// For EVT2
#define GPIO_NAV1_UARTCAN_EN_N (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)
#define GPIO_NAV1_RS485_EN_N (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN0)

// Pins specific to NAV2
// For EVT1 (TODO: automatic detection)
//#define GPIO_NAV2_UARTCAN_EN_N (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN11)
// #define GPIO_NAV2_RS485_EN_N (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN9)

// For EVT2
#define GPIO_NAV2_UARTCAN_EN_N (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN11)
#define GPIO_NAV2_RS485_EN_N (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN9)

#define GPIO_CHG_DIS (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)
#define GPIO_CHG_DET_H (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTA|GPIO_PIN9)
#define GPIO_CHG_DET_L (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTA|GPIO_PIN10)

#define GPIO_DISCON1 (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTI|GPIO_PIN2)
#define GPIO_BAT_CHG_IND (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTI|GPIO_PIN5)
#define GPIO_AUX_IO_1 (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTI|GPIO_PIN6)
#define GPIO_AUX_IO_2 (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTI|GPIO_PIN7)
#define GPIO_TAISYNC_EN (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTI|GPIO_PIN8)

#define GPIO_GIMBAL_EN (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN9)

#define GPIO_PL1_USB_LOAD_SW (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN13)
#define GPIO_PL2_PCIE_USB_BRIDGE_EN (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN14)

/*
 * RC Serial port on USART4
 */
//#define RC_SERIAL_PORT         "/dev/ttyS5" /* No HW invert support */

#    define STM32_PROCFS_MOUNTPOINT "/proc"

/*
 * High-resolution timer
 */
#define HRT_TIMER		     3  /* use timer3 for the HRT */
#define HRT_TIMER_CHANNEL    4  /* use capture/compare channel 4 */

/*
 * SDIO
 */
#define SDIO_SLOTNO                    0  /* Only one slot */
#define SDIO_MINOR                     0

/* SD card bringup does not work if performed on the IDLE thread because it
 * will cause waiting.  Use either:
 *
 *  CONFIG_LIB_BOARDCTL=y, OR
 *  CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARD_INITTHREAD=y
 */

#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_LIB_BOARDCTL) && \
   !defined(CONFIG_BOARD_INITTHREAD)
#  warning SDIO initialization cannot be perfomed on the IDLE thread
#endif

/* SPITTY speed. Clocks are selected in a way that possible speeds are 12.5
   and 25MHz. Just selecting a value > 12.5 will result speed of 12.5 */
#define SPITTY_SPI_BUS_SPEED 13000000

#define	BOARD_NAME "DELIBOZ"

#define RADIO_NETWORK_NAME "INTEL_UAV"

// Store everything on SD card
//#define  FLASH_BASED_PARAMS
//##define  FLASH_BASED_DATAMAN

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120
//#define BOARD_CRASHDUMP_RESET_ONLY


/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

enum hw_id_enum
{
	BOARD_VERSION_CARRIER_UNKNOWN = 0,
	BOARD_VERSION_CARRIER_EVT,
	BOARD_VERSION_CARRIER_EVT2,
};

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

int stm32_sdio_initialize(void);

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM
 *
 ****************************************************************************/

int stm32h7_pwm_setup(void);

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

/****************************************************************************************************
 * Name: board_spi_reset board_peripheral_reset
 *
 * Description:
 *   Called to reset SPI and the perferal bus
 *
 ****************************************************************************************************/

void board_spi_reset(int ms);
extern void board_peripheral_reset(int ms);

/************************************************************************************
 * Name: board_init_redundancy board_is_redundant_fc
 *
 * Description:
 *   Called to detect whether running in redundant FC
 *
 ************************************************************************************/
void board_init_redundancy(void);
extern bool board_is_redundant_fc(void);

/************************************************************************************
 * Name: board_get_hw_networkid
 *
 * Description:
 *   Read device networkID number from LFC FFS
 *
 ************************************************************************************/
int board_get_hw_networkid(void);

/************************************************************************************
 * Name: board_get_hw_serial_string
 *
 * Description:
 *   Read device serial number string from LFC FFS
 *   input param 'serial' shall be pointer to allocated buffer with
 *   size of > 16 bytes.
 *
 * Input params:
 *   serial      Pointer to string buffer
 *   size        Size of allocated string buffer
 ************************************************************************************/
int board_get_hw_serial_string(char* serial, uint32_t size);


/************************************************************************************
 * Name: board_set_px4_guid_formated
 *
 * Description:
 *   Set a formatted string of the manufactures Unique ID
 *
 * Input Parameters:
 * format_buffer - A buffer to store the 0 terminated formated px4
 *                 guid string.
 *
 * Returned Value:
 *   The number of printable characters stored.
 *
 ************************************************************************************/
int board_set_px4_guid_formated(char *format_buffer);

//#include <drivers/boards/common/board_common.h>

#endif /* __ASSEMBLY__ */

