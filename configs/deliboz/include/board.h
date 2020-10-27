/************************************************************************************
 * configs/deliboz/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Jukka Laitinen <jukka.laitinen@intel.com>
 *            Arun Ravindran <arun.ravindran@intel.com>
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
 ************************************************************************************/

#ifndef __BOARDS_INTEL_DELIBOZ_INCLUDE_BOARD_H
#define __BOARDS_INTEL_DELIBOZ_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* The deliboz board provides the following clock sources:
 *
 *   Y2100: 24 MHz crystal for HSE
 *
 * So we have these clock source available within the STM32
 *
 *   HSI: 16 MHz RC factory-trimmed
 *   HSE: 24 MHz crystal for HSE
 */

#define STM32_BOARD_XTAL        24000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     0

/* Main PLL Configuration.
 *
 * PLL source is HSE = 24,000,000
 *
 * When STM32_HSE_FREQUENCY / PLLM <= 2MHz VCOL must be selected. VCOH otherwise.
 *
 * PLL_VCOx = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 * Subject to:
 *
 *     1 <= PLLM <= 63
 *     4 <= PLLN <= 512
 *   150 MHz <= PLL_VCOL <= 420MHz
 *   192 MHz <= PLL_VCOH <= 836MHz
 *
 * SYSCLK  = PLL_VCO / PLLP
 * CPUCLK  = SYSCLK / D1CPRE
 * Subject to
 *
 *   PLLP1   = {2, 4, 6, 8, ..., 128}
 *   PLLP2,3 = {2, 3, 4, ..., 128}
 *   CPUCLK <= 400 MHz
 */

#define STM32_BOARD_USEHSE
//#define STM32_HSEBYP_ENABLE

#define STM32_PLLCFG_PLLSRC      RCC_PLLCKSELR_PLLSRC_HSE

/* PLL1, wide 4 - 8 MHz input, enable DIVP, DIVQ, DIVR
 *
 *   PLL1_VCO = (24,000,000 / 6) * 200 = 800 MHz
 *
 *   PLL1P = PLL1_VCO/2 = 800 MHz / 2 = 400 MHz
 *   PLL1Q = PLL1_VCO/4 = 800 MHz / 4 = 200 MHz
 *   PLL1R = PLL1_VCO/8 = 800 MHz / 8 = 100 MHz
 */

#define STM32_PLLCFG_PLL1CFG     (RCC_PLLCFGR_PLL1VCOSEL_WIDE | \
                                  RCC_PLLCFGR_PLL1RGE_4_8_MHZ | \
                                  RCC_PLLCFGR_DIVP1EN | \
                                  RCC_PLLCFGR_DIVQ1EN | \
                                  RCC_PLLCFGR_DIVR1EN)
#define STM32_PLLCFG_PLL1M       RCC_PLLCKSELR_DIVM1(6)
#define STM32_PLLCFG_PLL1N       RCC_PLL1DIVR_N1(200)
#define STM32_PLLCFG_PLL1P       RCC_PLL1DIVR_P1(2)
#define STM32_PLLCFG_PLL1Q       RCC_PLL1DIVR_Q1(4)
#define STM32_PLLCFG_PLL1R       RCC_PLL1DIVR_R1(8)

#define STM32_VCO1_FREQUENCY     ((STM32_HSE_FREQUENCY / 6) * 200)
#define STM32_PLL1P_FREQUENCY    (STM32_VCO1_FREQUENCY / 2)
#define STM32_PLL1Q_FREQUENCY    (STM32_VCO1_FREQUENCY / 4)
#define STM32_PLL1R_FREQUENCY    (STM32_VCO1_FREQUENCY / 8)

/* PLL2 */

#define STM32_PLLCFG_PLL2CFG (RCC_PLLCFGR_PLL2VCOSEL_WIDE | \
                              RCC_PLLCFGR_PLL2RGE_4_8_MHZ | \
                              RCC_PLLCFGR_DIVP2EN)
#define STM32_PLLCFG_PLL2M       RCC_PLLCKSELR_DIVM2(6)
#define STM32_PLLCFG_PLL2N       RCC_PLL2DIVR_N2(200)
#define STM32_PLLCFG_PLL2P       RCC_PLL2DIVR_P2(40)
#define STM32_PLLCFG_PLL2Q       0
#define STM32_PLLCFG_PLL2R       0

#define STM32_VCO2_FREQUENCY     ((STM32_HSE_FREQUENCY / 6) * 200)
#define STM32_PLL2P_FREQUENCY    (STM32_VCO2_FREQUENCY / 2)
#define STM32_PLL2Q_FREQUENCY
#define STM32_PLL2R_FREQUENCY

/* PLL3 */

#define STM32_PLLCFG_PLL3CFG 0
#define STM32_PLLCFG_PLL3M   0
#define STM32_PLLCFG_PLL3N   0
#define STM32_PLLCFG_PLL3P   0
#define STM32_PLLCFG_PLL3Q   0
#define STM32_PLLCFG_PLL3R   0

#define STM32_VCO3_FREQUENCY
#define STM32_PLL3P_FREQUENCY
#define STM32_PLL3Q_FREQUENCY
#define STM32_PLL3R_FREQUENCY

/* SYSCLK = PLL1P = 400 MHz
 * CPUCLK = SYSCLK / 1 = 400 MHz
 */

#define STM32_RCC_D1CFGR_D1CPRE  (RCC_D1CFGR_D1CPRE_SYSCLK)
#define STM32_SYSCLK_FREQUENCY   (STM32_PLL1P_FREQUENCY)
#define STM32_CPUCLK_FREQUENCY   (STM32_SYSCLK_FREQUENCY / 1)

/* Configure Clock Assignments */

/* AHB clock (HCLK) is SYSCLK/2 (200 MHz max)
 * HCLK1 = HCLK2 = HCLK3 = HCLK4
 */

#define STM32_RCC_D1CFGR_HPRE   RCC_D1CFGR_HPRE_SYSCLKd2        /* HCLK  = SYSCLK / 2 */
#define STM32_ACLK_FREQUENCY    (STM32_CPUCLK_FREQUENCY / 2)    /* ACLK in D1, HCLK3 in D1 */
#define STM32_HCLK_FREQUENCY    (STM32_CPUCLK_FREQUENCY / 2)    /* HCLK in D2, HCLK4 in D3 */
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY            /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/4 (50 MHz) */

#define STM32_RCC_D2CFGR_D2PPRE1  RCC_D2CFGR_D2PPRE1_HCLKd4       /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY     (STM32_HCLK_FREQUENCY/4)

/* APB2 clock (PCLK2) is HCLK/4 (50 MHz) */

#define STM32_RCC_D2CFGR_D2PPRE2  RCC_D2CFGR_D2PPRE2_HCLKd4       /* PCLK2 = HCLK / 4 */
#define STM32_PCLK2_FREQUENCY     (STM32_HCLK_FREQUENCY/4)

/* APB3 clock (PCLK3) is HCLK/4 (50 MHz) */

#define STM32_RCC_D1CFGR_D1PPRE   RCC_D1CFGR_D1PPRE_HCLKd4        /* PCLK3 = HCLK / 4 */
#define STM32_PCLK3_FREQUENCY     (STM32_HCLK_FREQUENCY/4)

/* APB4 clock (PCLK4) is HCLK/4 (50 MHz) */

#define STM32_RCC_D3CFGR_D3PPRE   RCC_D3CFGR_D3PPRE_HCLKd4       /* PCLK4 = HCLK / 4 */
#define STM32_PCLK4_FREQUENCY     (STM32_HCLK_FREQUENCY/4)

/* Timer clock frequencies */

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM15_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM16_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM17_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Kernel Clock Configuration
 *
 * Note: look at Table 54 in ST Manual
 */

/* I2C123 clock source - HSI */

#define STM32_RCC_D2CCIP2R_I2C123SRC RCC_D2CCIP2R_I2C123SEL_HSI

/* I2C4 clock source - HSI */

#define STM32_RCC_D3CCIPR_I2C4SRC    RCC_D3CCIPR_I2C4SEL_HSI

/* SPI123 clock source - PLL1Q */

#define STM32_RCC_D2CCIP1R_SPI123SRC RCC_D2CCIP1R_SPI123SEL_PLL1

/* SPI45 clock source - APB (PCLK2?) */

#define STM32_RCC_D2CCIP1R_SPI45SRC  RCC_D2CCIP1R_SPI45SEL_APB

/* SPI6 clock source - APB (PCLK4) */

#define STM32_RCC_D3CCIPR_SPI6SRC    RCC_D3CCIPR_SPI6SEL_PCLK4

/* USB 1 and 2 clock source - HSI48 */

#define STM32_RCC_D2CCIP2R_USBSRC    RCC_D2CCIP2R_USBSEL_HSI48

/* ADC 1 2 3 clock source - pll2_pclk */

#define STM32_RCC_D3CCIPR_ADCSEL     RCC_D3CCIPR_ADCSEL_PLL2

/* FLASH wait states
 *
 *  ------------ ---------- -----------
 *  Vcore        MAX ACLK   WAIT STATES
 *  ------------ ---------- -----------
 *  1.15-1.26 V     70 MHz    0
 *  (VOS1 level)   140 MHz    1
 *                 210 MHz    2
 *  1.05-1.15 V     55 MHz    0
 *  (VOS2 level)   110 MHz    1
 *                 165 MHz    2
 *                 220 MHz    3
 *  0.95-1.05 V     45 MHz    0
 *  (VOS3 level)    90 MHz    1
 *                 135 MHz    2
 *                 180 MHz    3
 *                 225 MHz    4
 *  ------------ ---------- -----------
 */

#define BOARD_FLASH_WAITSTATES 4

/* SDMMC definitions ****************************************************************/

/* Init 400kHz, PLL1Q/(2*250) */

#define STM32_SDMMC_INIT_CLKDIV     (250 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)

/* Set to 25MHz, PLL1Q/(2*4), for default speed of 12.5MB/s */
#define STM32_SDMMC_MMCXFR_CLKDIV   (4 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#define STM32_SDMMC_SDXFR_CLKDIV    (4 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#define STM32_SDMMC_SDHSXFR_CLKDIV  (2 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)

#define STM32_SDMMC_CLKCR_EDGE      STM32_SDMMC_CLKCR_NEGEDGE


/* Ethernet definitions ****************************************************************/
#define GPIO_ETH_RMII_TXD0    GPIO_ETH_RMII_TXD0_2 /* PG13 */
//#define GPIO_ETH_RMII_TXD1    GPIO_ETH_RMII_TXD1_1 /* PB 13 - nucleo board */
#define GPIO_ETH_RMII_TXD1    GPIO_ETH_RMII_TXD1_3 /* PG14 */
#define GPIO_ETH_RMII_TX_EN   GPIO_ETH_RMII_TX_EN_2

/* LED definitions ******************************************************************/
/* The Nucleo-144 board has numerous LEDs but only three, LD1 a Green LED, LD2 a Blue
 * LED and LD3 a Red LED, that can be controlled by software. The following
 * definitions assume the default Solder Bridges are installed.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any way.
 * The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_NLEDS       3

#define BOARD_LED_GREEN   BOARD_LED1
#define BOARD_LED_BLUE    BOARD_LED2
#define BOARD_LED_RED     BOARD_LED3

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)

/* If CONFIG_ARCH_LEDS is defined, the usage by the board port is defined in
 * include/board.h and src/stm32_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                        Red   Green Blue
 *   ----------------------  --------------------------  ------ ------ ----*/

#define LED_STARTED        0 /* NuttX has been started   OFF    OFF   OFF  */
#define LED_HEAPALLOCATE   1 /* Heap has been allocated  OFF    OFF   ON   */
#define LED_IRQSENABLED    2 /* Interrupts enabled       OFF    ON    OFF  */
#define LED_STACKCREATED   3 /* Idle stack created       OFF    ON    ON   */
#define LED_INIRQ          4 /* In an interrupt          N/C    N/C   GLOW */
#define LED_SIGNAL         5 /* In a signal handler      N/C    GLOW  N/C  */
#define LED_ASSERTION      6 /* An assertion failed      GLOW   N/C   GLOW */
#define LED_PANIC          7 /* The system has crashed   Blink  OFF   N/C  */
#define LED_IDLE           8 /* MCU is is sleep mode     ON     OFF   OFF  */

/* Thus if the Green LED is statically on, NuttX has successfully booted and
 * is, apparently, running normally.  If the Red LED is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system
 * has halted.
 */

/* Button definitions ***************************************************************/
/* The NUCLEO board supports one button:  Pushbutton B1, labeled "User", is
 * connected to GPIO PI11.  A high value will be sensed when the button is depressed.
 */

//#define BUTTON_USER        0
//#define NUM_BUTTONS        1
//#define BUTTON_USER_BIT    (1 << BUTTON_USER)

/* Alternate function pin selections ************************************************/

/* IMU HEATER PWM */
#define GPIO_TIM4_CH3OUT GPIO_TIM4_CH3OUT_1  /* PB8  */
#define GPIO_TIM4_CH4OUT GPIO_TIM4_CH4OUT_1  /* PB9  */

/* USART1 (NAVx_MOTORUART) */
#define GPIO_USART1_RX     GPIO_USART1_RX_1  /* PB15 */
#define GPIO_USART1_TX     GPIO_USART1_TX_1  /* PB14 */
#define GPIO_USART1_RS485_DIR (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN12)

/* USART2 (PL_UART) */
#define GPIO_USART2_RX     GPIO_USART2_RX_2  /* PD6 */
#define GPIO_USART2_TX     GPIO_USART2_TX_2  /* PD5 */

/* USART3 (NAVx_CPU_UART) */
#define GPIO_USART3_RX     GPIO_USART3_RX_3  /* PD9 */
#define GPIO_USART3_TX     GPIO_USART3_TX_3  /* PD8 */
#define GPIO_USART3_RTS    GPIO_USART3_RTS_2 /* PD12 */
#define GPIO_USART3_CTS    GPIO_USART3_CTS_NSS_2 /* PD11 */

/* UART4 (GNSS) */
#define GPIO_UART4_RX     GPIO_UART4_RX_5  /* PD0 */
#define GPIO_UART4_TX     GPIO_UART4_TX_5  /* PD1 */

/* UART5 (Battery) */
#define GPIO_UART5_RX     GPIO_UART5_RX_1  /* PB12 */
#define GPIO_UART5_TX     GPIO_UART5_TX_1  /* PB13 */

/* USART6 (NAVx_EXT_UART) */

#define GPIO_USART6_RX     GPIO_USART6_RX_2  /* PG9 */
#define GPIO_USART6_TX     GPIO_USART6_TX_1  /* PC6 */
#define GPIO_USART6_RTS    GPIO_USART6_RTS_1 /* PG12 */
#define GPIO_USART6_CTS    GPIO_USART6_CTS_NSS_2 /* PG15 */

/* USART7 (debug) */

#define GPIO_UART7_RX     GPIO_UART7_RX_4  /* PF6 */
#define GPIO_UART7_TX     GPIO_UART7_TX_2  /* PB4 */

/* UART8 (CTRL_UART, XBEE) */
#define GPIO_UART8_RX     GPIO_UART8_RX_1  /* PE0 */
#define GPIO_UART8_TX     GPIO_UART8_TX_1  /* PE1 */

/* I2C1 SENSOR1 */
#define GPIO_I2C1_SCL GPIO_I2C1_SCL_1 /* PB6 */
#define GPIO_I2C1_SDA GPIO_I2C1_SDA_1 /* PB7 */

/* I2C2 SENSOR2 */
#define GPIO_I2C2_SCL GPIO_I2C2_SCL_2 /* PF1 */
#define GPIO_I2C2_SDA GPIO_I2C2_SDA_2 /* PF0 */

/* I2C3 EXTSENS1 */
#define GPIO_I2C3_SCL GPIO_I2C3_SCL_2 /* PH7 */
#define GPIO_I2C3_SDA GPIO_I2C3_SDA_2 /* PH8 */

/* I2C4 DBG1 */
#define GPIO_I2C4_SCL GPIO_I2C4_SCL_3 /* PH11 */
#define GPIO_I2C4_SDA GPIO_I2C4_SDA_3 /* PH12 */

/* SPI1 IMU1 */
#define GPIO_SPI1_MISO GPIO_SPI1_MISO_1 /* PA6 */
#define GPIO_SPI1_MOSI GPIO_SPI1_MOSI_2 /* PB5 */
#define GPIO_SPI1_SCK  GPIO_SPI1_SCK_1  /* PA5 */
#define GPIO_SPI1_NSS  GPIO_SPI1_NSS_3  /* PG10 (actually driven as gpio) */

/* SPI2 IMU2 */
#define GPIO_SPI2_MISO GPIO_SPI2_MISO_2 /* PC2_C */
#define GPIO_SPI2_MOSI GPIO_SPI2_MOSI_4 /* PI3 */
#define GPIO_SPI2_SCK  GPIO_SPI2_SCK_5  /* PD3 */
#define GPIO_SPI2_NSS  GPIO_SPI2_NSS_4  /* PI0 (actually driven as gpio) */

/* SPI4 NAV1_SPIOUT */
#define GPIO_SPI4_MISO GPIO_SPI4_MISO_1 /* TBD, nc, now E13 */
#define GPIO_SPI4_MOSI GPIO_SPI4_MOSI_1 /* PE14 */
#define GPIO_SPI4_SCK  GPIO_SPI4_SCK_2  /* PE2 */
#define GPIO_SPI4_NSS  GPIO_SPI4_NSS_2  /* PE4 (actually driven as gpio) */

/* SPI5 NAV2_SPIOUT */
#define GPIO_SPI5_MISO GPIO_SPI5_MISO_3 /* TBD, nc, now J11 */
#define GPIO_SPI5_MOSI GPIO_SPI5_MOSI_1 /* PF11 */
#define GPIO_SPI5_SCK  GPIO_SPI5_SCK_1  /* PF7 */
#define GPIO_SPI5_NSS  GPIO_SPI5_NSS_2  /* PH5 */

/* Increase drive strength for SD card GPIOs */

#ifdef GPIO_SDMMC1_CK
#undef GPIO_SDMMC1_CK
#define GPIO_SDMMC1_CK            (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTC|GPIO_PIN12)
#endif

#ifdef GPIO_SDMMC1_CMD
#undef GPIO_SDMMC1_CMD
#define GPIO_SDMMC1_CMD           (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN2)
#endif

#ifdef GPIO_SDMMC1_D0
#undef GPIO_SDMMC1_D0
#define GPIO_SDMMC1_D0            (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN8)
#endif

#ifdef GPIO_SDMMC1_D1
#undef GPIO_SDMMC1_D1
#define GPIO_SDMMC1_D1            (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN9)
#endif

#ifdef GPIO_SDMMC1_D2
#undef GPIO_SDMMC1_D2
#define GPIO_SDMMC1_D2            (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN10)
#endif

#ifdef GPIO_SDMMC1_D3
#undef GPIO_SDMMC1_D3
#define GPIO_SDMMC1_D3            (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN11)
#endif

/* DMA ******************************************************************************/

#define DMAMAP_SDMMC1  DMAMAP_SDMMC1_1

// TODO: the dmamaps for spi 1-3 are just wrong, the spi defaults to non-dma mode
#define DMAMAP_SPI1_RX DMAMAP_DMA12_SPI1RX_0
#define DMAMAP_SPI1_TX DMAMAP_DMA12_SPI1TX_0

#define DMAMAP_SPI2_RX DMAMAP_DMA12_SPI2RX_0
#define DMAMAP_SPI2_TX DMAMAP_DMA12_SPI2TX_0

#define DMAMAP_SPI3_RX DMAMAP_DMA12_SPI3RX_0
#define DMAMAP_SPI3_TX DMAMAP_DMA12_SPI3TX_0

/* These use DMA2 */
#define DMAMAP_SPI4_RX 0
#define DMAMAP_SPI4_TX DMAMAP_DMA12_SPI4TX_1

#define DMAMAP_SPI5_RX DMAMAP_DMA12_SPI5RX_1
#define DMAMAP_SPI5_TX 0

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __BOARDS_INTEL_DELIBOZ_INCLUDE_BOARD_H */
