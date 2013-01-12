/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for ENEL429-12W Group 7 Camera Board
 */

/*
 * Board identifier.
 */
#define BOARD_ENEL429_GRP7_CAMERABOARD
#define BOARD_NAME              "Group 7 Camera Board"

/*
 * Board frequencies.
 * NOTE: The LSE crystal is not fitted by default on the board.
 */
#define STM32_LSECLK            0
#define STM32_HSECLK            8000000

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD               330

/*
 * MCU type as defined in the ST header file stm32f4xx.h.
 */
#define STM32F4XX

/*
 * IO pins assignments.
 */
#define GPIOA_DCMI_HS           4
#define GPIOA_DCMI_CL           6
#define GPIOA_LED2              7       /* User LED on board */
#define GPIOA_SWDIO             13
#define GPIOA_SWCLK             14

#define GPIOB_TRACESWO          3
#define GPIOB_LED1              5       /* IR LED MOSFET gate */
#define GPIOB_DCMI_D5           6
#define GPIOB_DCMI_VS           7
#define GPIOB_DCMI_D6           8
#define GPIOB_DCMI_D7           9
#define GPIOB_I2C_SCL           10      /* I2C for camera config */
#define GPIOB_I2C_SDA           11
#define GPIOB_SPI_nCS           12      /* SPI for communication with comms board */
#define GPIOB_SPI_SCK           13
#define GPIOB_SPI_MISO          14
#define GPIOB_SPI_MOSI          15

#define GPIOC_DCMI_D0           6
#define GPIOC_DCMI_D1           7
#define GPIOC_DCMI_D2           8
#define GPIOC_DCMI_D3           9
#define GPIOC_UART3_TX          10       /* UART for communication with comms board and programming via bootloader */
#define GPIOC_UART3_RX          11
#define GPIOC_DCMI_D9           12

#define GPIOD_DCMI_D11          2
#define GPIOD_EXTCLK            14       /* Camera external clock line (connect to timer PWM channel) */
#define GPIOD_RESET             15       /* Camera RESET line - pull LOW to enable camera, HIGH to disable */

#define GPIOE_DCMI_D4           4

#define GPIOF_DCMI_D12          11

#define GPIOG_DCMI_D13          15

#define GPIOH_OSC_IN            0        /* HSE */
#define GPIOH_OSC_OUT           1

#define GPIOI_DCMI_D8           1
#define GPIOI_DCMI_D10          3

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2))
#define PIN_PUDR_FLOATING(n)        (0U << ((n) * 2))
#define PIN_PUDR_PULLUP(n)          (1U << ((n) * 2))
#define PIN_PUDR_PULLDOWN(n)        (2U << ((n) * 2))
#define PIN_DATA_LOW(n)             (0U << (n))
#define PIN_DATA_HIGH(n)            (1U << (n))
#define PIN_AFIO_AF(n, v)           ((v##U) << ((n % 8) * 4))

/*
 * Port A setup.
 * All input with pull-up except:
 * PA4  - DCMI_HS                    (alternate 13, floating)
 * PA6  - DCMI_CL                    (alternate 13, floating)
 * PA7  - LED2                       (output push-pull, floating)
 * PA13 - SWDIO                      (alternate 0, pull-up)
 * PA14 - SWCLK                      (alternate 0, pull-down)
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(0) |                    \
                                     PIN_MODE_INPUT(1) |                    \
                                     PIN_MODE_INPUT(2) |                    \
                                     PIN_MODE_INPUT(3) |                    \
                                     PIN_MODE_ALTERNATE(GPIOA_DCMI_HS) |    \
                                     PIN_MODE_INPUT(5) |                    \
                                     PIN_MODE_ALTERNATE(GPIOA_DCMI_CL) |    \
                                     PIN_MODE_OUTPUT(GPIOA_LED2) |          \
                                     PIN_MODE_INPUT(8) |                    \
                                     PIN_MODE_INPUT(9) |                    \
                                     PIN_MODE_INPUT(10) |                   \
                                     PIN_MODE_INPUT(11) |                   \
                                     PIN_MODE_INPUT(12) |                   \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |      \
                                     PIN_MODE_INPUT(15))
#define VAL_GPIOA_OTYPER            0x00000000
#define VAL_GPIOA_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOA_PUPDR             (PIN_PUDR_PULLUP(0) |                   \
                                     PIN_PUDR_PULLUP(1) |                   \
                                     PIN_PUDR_PULLUP(2) |                   \
                                     PIN_PUDR_PULLUP(3) |                   \
                                     PIN_PUDR_FLOATING(GPIOA_DCMI_HS) |     \
                                     PIN_PUDR_PULLUP(5) |                   \
                                     PIN_PUDR_FLOATING(GPIOA_DCMI_CL) |     \
                                     PIN_PUDR_FLOATING(GPIOA_LED2) |        \
                                     PIN_PUDR_PULLUP(8) |                   \
                                     PIN_PUDR_PULLUP(9) |                   \
                                     PIN_PUDR_PULLUP(10) |                  \
                                     PIN_PUDR_PULLUP(11) |                  \
                                     PIN_PUDR_PULLUP(12) |                  \
                                     PIN_PUDR_PULLUP(GPIOA_SWDIO) |         \
                                     PIN_PUDR_PULLDOWN(GPIOA_SWCLK) |       \
                                     PIN_PUDR_PULLUP(15))
#define VAL_GPIOA_ODR               (PIN_DATA_LOW(GPIOA_LED2))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_DCMI_HS, 13) |       \
                                     PIN_AFIO_AF(GPIOA_DCMI_CL, 13))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_SWDIO, 0) |          \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0))

/*
 * Port B setup.
 * All input with pull-up except:
 * PB3  - TRACESWO                   (alternate 0, floating)
 * PB5  - LED1                       (alternate 3, push-pull, pull-down)
 * PB6  - DCMI_D5                    (alternate 13, floating)
 * PB7  - DCMI_VS                    (alternate 13, floating)
 * PB8  - DCMI_D6                    (alternate 13, floating)
 * PB9  - DCMI_D7                    (alternate 13, floating)
 * PB10 - I2C_SCL                    (alternate 4, open-drain, pull-up)
 * PB11 - I2C_SDA                    (alternate 4, open-drain, pull-up)
 * PB12 - SPI_nCS                    (alternate 5, push-pull, floating)
 * PB13 - SPI_SCK                    (alternate 5, push-pull, floating)
 * PB14 - SPI_MISO                   (alternate 5, push-pull, floating)
 * PB15 - SPI_MOSI                   (alternate 5, push-pull, floating)
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(0) |                    \
                                     PIN_MODE_INPUT(1) |                    \
                                     PIN_MODE_INPUT(2) |                    \
                                     PIN_MODE_ALTERNATE(GPIOB_TRACESWO) |   \
                                     PIN_MODE_INPUT(4) |                    \
                                     PIN_MODE_ALTERNATE(GPIOB_LED1) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_DCMI_D5) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_DCMI_VS) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_DCMI_D6) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_DCMI_D7) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C_SCL) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C_SDA) |    \
                                     PIN_MODE_OUTPUT(GPIOB_SPI_nCS) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI_SCK) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI_MISO) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI_MOSI))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOB_I2C_SCL) |   \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C_SDA))
#define VAL_GPIOB_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOB_PUPDR             (PIN_PUDR_PULLUP(0) |                   \
                                     PIN_PUDR_PULLUP(1) |                   \
                                     PIN_PUDR_PULLUP(2) |                   \
                                     PIN_PUDR_FLOATING(GPIOB_TRACESWO) |    \
                                     PIN_PUDR_PULLUP(4) |                   \
                                     PIN_PUDR_PULLDOWN(GPIOB_LED1) |        \
                                     PIN_PUDR_FLOATING(GPIOB_DCMI_D5) |     \
                                     PIN_PUDR_FLOATING(GPIOB_DCMI_VS) |     \
                                     PIN_PUDR_FLOATING(GPIOB_DCMI_D6) |     \
                                     PIN_PUDR_FLOATING(GPIOB_DCMI_D7) |     \
                                     PIN_PUDR_PULLUP(GPIOB_I2C_SCL) |       \
                                     PIN_PUDR_PULLUP(GPIOB_I2C_SDA) |       \
                                     PIN_PUDR_FLOATING(GPIOB_SPI_nCS) |     \
                                     PIN_PUDR_FLOATING(GPIOB_SPI_SCK) |     \
                                     PIN_PUDR_FLOATING(GPIOB_SPI_MISO) |    \
                                     PIN_PUDR_FLOATING(GPIOB_SPI_MOSI))
#define VAL_GPIOB_ODR               0xFFFFFFFF
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_TRACESWO, 0) |       \
                                     PIN_AFIO_AF(GPIOB_LED1, 3) |           \
                                     PIN_AFIO_AF(GPIOB_DCMI_D5, 13) |       \
                                     PIN_AFIO_AF(GPIOB_DCMI_VS, 13))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_DCMI_D6, 13) |       \
                                     PIN_AFIO_AF(GPIOB_DCMI_D7, 13) |       \
                                     PIN_AFIO_AF(GPIOB_I2C_SCL, 4) |        \
                                     PIN_AFIO_AF(GPIOB_I2C_SDA, 4) |        \
                                     PIN_AFIO_AF(GPIOB_SPI_SCK, 5) |        \
                                     PIN_AFIO_AF(GPIOB_SPI_MISO, 5) |       \
                                     PIN_AFIO_AF(GPIOB_SPI_MOSI, 5))

/*
 * Port C setup.
 * All input with pull-up except:
 * PC6  - DCMI_D0               (alternate 13, floating)
 * PC7  - DCMI_D1               (alternate 13, floating)
 * PC8  - DCMI_D2               (alternate 13, floating)
 * PC9  - DCMI_D3               (alternate 13, floating)
 * PC10 - UART3_TX              (alternate 7, push-pull, floating)
 * PC11 - UART3_RX              (alternate 7, floating)
 * PC12 - DCMI_D9               (alternate 13, pull-down)
 */
#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(0) |                    \
                                     PIN_MODE_INPUT(1) |                    \
                                     PIN_MODE_INPUT(2) |                    \
                                     PIN_MODE_INPUT(3) |                    \
                                     PIN_MODE_INPUT(4) |                    \
                                     PIN_MODE_INPUT(5) |                    \
                                     PIN_MODE_ALTERNATE(GPIOC_DCMI_D0) |    \
                                     PIN_MODE_ALTERNATE(GPIOC_DCMI_D1) |    \
                                     PIN_MODE_ALTERNATE(GPIOC_DCMI_D2) |    \
                                     PIN_MODE_ALTERNATE(GPIOC_DCMI_D3) |    \
                                     PIN_MODE_ALTERNATE(GPIOC_UART3_TX) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_UART3_RX) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_DCMI_D9) |    \
                                     PIN_MODE_INPUT(13) |                   \
                                     PIN_MODE_INPUT(14) |                   \
                                     PIN_MODE_INPUT(15))
#define VAL_GPIOC_OTYPER            0x00000000
#define VAL_GPIOC_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOC_PUPDR             (PIN_PUDR_PULLUP(0) |                   \
                                     PIN_PUDR_PULLUP(1) |                   \
                                     PIN_PUDR_PULLUP(2) |                   \
                                     PIN_PUDR_PULLUP(3) |                   \
                                     PIN_PUDR_PULLUP(4) |                   \
                                     PIN_PUDR_PULLUP(5) |                   \
                                     PIN_PUDR_FLOATING(GPIOC_DCMI_D0) |     \
                                     PIN_PUDR_FLOATING(GPIOC_DCMI_D1) |     \
                                     PIN_PUDR_FLOATING(GPIOC_DCMI_D2) |     \
                                     PIN_PUDR_FLOATING(GPIOC_DCMI_D3) |     \
                                     PIN_PUDR_FLOATING(GPIOC_UART3_TX) |    \
                                     PIN_PUDR_FLOATING(GPIOC_UART3_RX) |    \
                                     PIN_PUDR_PULLDOWN(GPIOC_DCMI_D9) |     \
                                     PIN_PUDR_PULLUP(13) |                  \
                                     PIN_PUDR_PULLUP(14) |                  \
                                     PIN_PUDR_PULLUP(15))
#define VAL_GPIOC_ODR               0xFFFFFFFF
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_DCMI_D0, 13) |       \
                                     PIN_AFIO_AF(GPIOC_DCMI_D1, 13))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_DCMI_D2, 13) |       \
                                     PIN_AFIO_AF(GPIOC_DCMI_D3, 13) |       \
                                     PIN_AFIO_AF(GPIOC_UART3_TX, 7) |       \
                                     PIN_AFIO_AF(GPIOC_UART3_RX, 7) |       \
                                     PIN_AFIO_AF(GPIOC_DCMI_D9, 13))

/* 
 * Port D setup.
 * All input with pull-up except:
 * PD2  - DCMI_D11            (alternate 13, pull-down)
 * PD14 - EXTCLK              (alternate 2, push-pull, floating)
 * PD15 - RESET               (output push-pull, pull-up)
 */
#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(0) |                    \
                                     PIN_MODE_INPUT(1) |                    \
                                     PIN_MODE_ALTERNATE(GPIOD_DCMI_D11) |   \
                                     PIN_MODE_INPUT(3) |                    \
                                     PIN_MODE_INPUT(4) |                    \
                                     PIN_MODE_INPUT(5) |                    \
                                     PIN_MODE_INPUT(6) |                    \
                                     PIN_MODE_INPUT(7) |                    \
                                     PIN_MODE_INPUT(8) |                    \
                                     PIN_MODE_INPUT(9) |                    \
                                     PIN_MODE_INPUT(10) |                   \
                                     PIN_MODE_INPUT(11) |                   \
                                     PIN_MODE_INPUT(12) |                   \
                                     PIN_MODE_INPUT(13) |                   \
                                     PIN_MODE_ALTERNATE(GPIOD_EXTCLK) |     \
                                     PIN_MODE_OUTPUT(GPIOD_RESET))
#define VAL_GPIOD_OTYPER            0x00000000
#define VAL_GPIOD_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOD_PUPDR             (PIN_PUDR_PULLUP(0) |                   \
                                     PIN_PUDR_PULLUP(1) |                   \
                                     PIN_PUDR_PULLDOWN(GPIOD_DCMI_D11) |    \
                                     PIN_PUDR_PULLUP(3) |                   \
                                     PIN_PUDR_PULLUP(4) |                   \
                                     PIN_PUDR_PULLUP(5) |                   \
                                     PIN_PUDR_PULLUP(6) |                   \
                                     PIN_PUDR_PULLUP(7) |                   \
                                     PIN_PUDR_PULLUP(8) |                   \
                                     PIN_PUDR_PULLUP(9) |                   \
                                     PIN_PUDR_PULLUP(10) |                  \
                                     PIN_PUDR_PULLUP(11) |                  \
                                     PIN_PUDR_PULLUP(12) |                  \
                                     PIN_PUDR_PULLUP(13) |                  \
                                     PIN_PUDR_FLOATING(GPIOD_EXTCLK) |      \
                                     PIN_PUDR_PULLUP(GPIOD_RESET))
#define VAL_GPIOD_ODR               (PIN_DATA_LOW(GPIOD_RESET))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_DCMI_D11, 13))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_EXTCLK, 2))

/*
 * Port E setup.
 * All input with pull-up except:
 * PE4 - DCMI_D4                    (alternate 13, floating)
 */
#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(0) |                    \
                                     PIN_MODE_INPUT(1) |                    \
                                     PIN_MODE_INPUT(2) |                    \
                                     PIN_MODE_INPUT(3) |                    \
                                     PIN_MODE_ALTERNATE(GPIOE_DCMI_D4) |    \
                                     PIN_MODE_INPUT(5) |                    \
                                     PIN_MODE_INPUT(6) |                    \
                                     PIN_MODE_INPUT(7) |                    \
                                     PIN_MODE_INPUT(8) |                    \
                                     PIN_MODE_INPUT(9) |                    \
                                     PIN_MODE_INPUT(10) |                   \
                                     PIN_MODE_INPUT(11) |                   \
                                     PIN_MODE_INPUT(12) |                   \
                                     PIN_MODE_INPUT(13) |                   \
                                     PIN_MODE_INPUT(14) |                   \
                                     PIN_MODE_INPUT(15))
#define VAL_GPIOE_OTYPER            0x00000000
#define VAL_GPIOE_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOE_PUPDR             (PIN_PUDR_PULLUP(0) |                   \
                                     PIN_PUDR_PULLUP(1) |                   \
                                     PIN_PUDR_PULLUP(2) |                   \
                                     PIN_PUDR_PULLUP(3) |                   \
                                     PIN_PUDR_FLOATING(GPIOE_DCMI_D4) |     \
                                     PIN_PUDR_PULLUP(5) |                   \
                                     PIN_PUDR_PULLUP(6) |                   \
                                     PIN_PUDR_PULLUP(7) |                   \
                                     PIN_PUDR_PULLUP(8) |                   \
                                     PIN_PUDR_PULLUP(9) |                   \
                                     PIN_PUDR_PULLUP(10) |                  \
                                     PIN_PUDR_PULLUP(11) |                  \
                                     PIN_PUDR_PULLUP(12) |                  \
                                     PIN_PUDR_PULLUP(13) |                  \
                                     PIN_PUDR_PULLUP(14) |                  \
                                     PIN_PUDR_PULLUP(15))
#define VAL_GPIOE_ODR               0xFFFFFFFF
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_DCMI_D4, 13))
#define VAL_GPIOE_AFRH              0x00000000

/*
 * Port F setup.
 * All input with pull-up except:
 * PF11 - DCMI_D12                  (alternate 13, pull-down)
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(0) |                    \
                                     PIN_MODE_INPUT(1) |                    \
                                     PIN_MODE_INPUT(2) |                    \
                                     PIN_MODE_INPUT(3) |                    \
                                     PIN_MODE_INPUT(4) |                    \
                                     PIN_MODE_INPUT(5) |                    \
                                     PIN_MODE_INPUT(6) |                    \
                                     PIN_MODE_INPUT(7) |                    \
                                     PIN_MODE_INPUT(8) |                    \
                                     PIN_MODE_INPUT(9) |                    \
                                     PIN_MODE_INPUT(10) |                   \
                                     PIN_MODE_ALTERNATE(GPIOF_DCMI_D12) |   \
                                     PIN_MODE_INPUT(12) |                   \
                                     PIN_MODE_INPUT(13) |                   \
                                     PIN_MODE_INPUT(14) |                   \
                                     PIN_MODE_INPUT(15))
#define VAL_GPIOF_OTYPER            0x00000000
#define VAL_GPIOF_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOF_PUPDR             0xFFFFFFFF
#define VAL_GPIOF_ODR               0xFFFFFFFF
#define VAL_GPIOF_AFRL              0x00000000
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_DCMI_D12, 13))

/*
 * Port G setup.
 * All input with pull-up except:
 * PG15 - DCMI_D13                  (alternate 13, pull-down)
 */
#define VAL_GPIOG_MODER             (PIN_MODE_INPUT(0) |                    \
                                     PIN_MODE_INPUT(1) |                    \
                                     PIN_MODE_INPUT(2) |                    \
                                     PIN_MODE_INPUT(3) |                    \
                                     PIN_MODE_INPUT(4) |                    \
                                     PIN_MODE_INPUT(5) |                    \
                                     PIN_MODE_INPUT(6) |                    \
                                     PIN_MODE_INPUT(7) |                    \
                                     PIN_MODE_INPUT(8) |                    \
                                     PIN_MODE_INPUT(9) |                    \
                                     PIN_MODE_INPUT(10) |                   \
                                     PIN_MODE_INPUT(11) |                   \
                                     PIN_MODE_INPUT(12) |                   \
                                     PIN_MODE_INPUT(13) |                   \
                                     PIN_MODE_INPUT(14) |                   \
                                     PIN_MODE_ALTERNATE(GPIOG_DCMI_D13))
#define VAL_GPIOG_OTYPER            0x00000000
#define VAL_GPIOG_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOG_PUPDR             0xFFFFFFFF
#define VAL_GPIOG_ODR               0xFFFFFFFF
#define VAL_GPIOG_AFRL              0x00000000
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_DCMI_D13, 13))

/*
 * Port H setup.
 * All input with pull-up except:
 * PH0  - GPIOH_OSC_IN          (input floating).
 * PH1  - GPIOH_OSC_OUT         (input floating).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) |        \
                                     PIN_MODE_INPUT(2) |                    \
                                     PIN_MODE_INPUT(3) |                    \
                                     PIN_MODE_INPUT(4) |                    \
                                     PIN_MODE_INPUT(5) |                    \
                                     PIN_MODE_INPUT(6) |                    \
                                     PIN_MODE_INPUT(7) |                    \
                                     PIN_MODE_INPUT(8) |                    \
                                     PIN_MODE_INPUT(9) |                    \
                                     PIN_MODE_INPUT(10) |                   \
                                     PIN_MODE_INPUT(11) |                   \
                                     PIN_MODE_INPUT(12) |                   \
                                     PIN_MODE_INPUT(13) |                   \
                                     PIN_MODE_INPUT(14) |                   \
                                     PIN_MODE_INPUT(15))
#define VAL_GPIOH_OTYPER            0x00000000
#define VAL_GPIOH_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOH_PUPDR             (PIN_PUDR_FLOATING(GPIOH_OSC_IN) |      \
                                     PIN_PUDR_FLOATING(GPIOH_OSC_OUT) |     \
                                     PIN_PUDR_PULLUP(2) |                   \
                                     PIN_PUDR_PULLUP(3) |                   \
                                     PIN_PUDR_PULLUP(4) |                   \
                                     PIN_PUDR_PULLUP(5) |                   \
                                     PIN_PUDR_PULLUP(6) |                   \
                                     PIN_PUDR_PULLUP(7) |                   \
                                     PIN_PUDR_PULLUP(8) |                   \
                                     PIN_PUDR_PULLUP(9) |                   \
                                     PIN_PUDR_PULLUP(10) |                  \
                                     PIN_PUDR_PULLUP(11) |                  \
                                     PIN_PUDR_PULLUP(12) |                  \
                                     PIN_PUDR_PULLUP(13) |                  \
                                     PIN_PUDR_PULLUP(14) |                  \
                                     PIN_PUDR_PULLUP(15))
#define VAL_GPIOH_ODR               0xFFFFFFFF
#define VAL_GPIOH_AFRL              0x00000000
#define VAL_GPIOH_AFRH              0x00000000

/*
 * Port I setup.
 * All input with pull-up except:
 *  PI1 - GPIOI_DCMI_D8                 (alternate 13, pull-down)
 *  PI3 - GPIOI_DCMI_D10                (alternate 13, pull-down)
 */
#define VAL_GPIOI_MODER             (PIN_MODE_INPUT(0) |                    \
                                     PIN_MODE_ALTERNATE(GPIOI_DCMI_D8) |    \
                                     PIN_MODE_INPUT(2) |                    \
                                     PIN_MODE_ALTERNATE(GPIOI_DCMI_D10) |   \
                                     PIN_MODE_INPUT(4) |                    \
                                     PIN_MODE_INPUT(5) |                    \
                                     PIN_MODE_INPUT(6) |                    \
                                     PIN_MODE_INPUT(7) |                    \
                                     PIN_MODE_INPUT(8) |                    \
                                     PIN_MODE_INPUT(9) |                    \
                                     PIN_MODE_INPUT(10) |                   \
                                     PIN_MODE_INPUT(11) |                   \
                                     PIN_MODE_INPUT(12) |                   \
                                     PIN_MODE_INPUT(13) |                   \
                                     PIN_MODE_INPUT(14) |                   \
                                     PIN_MODE_INPUT(15))
                                     
#define VAL_GPIOI_OTYPER            0x00000000
#define VAL_GPIOI_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOI_PUPDR             0xFFFFFFFF
#define VAL_GPIOI_ODR               0xFFFFFFFF
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_DCMI_D8, 13) |       \
                                     PIN_AFIO_AF(GPIOI_DCMI_D10, 13))
#define VAL_GPIOI_AFRH              0x00000000

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
