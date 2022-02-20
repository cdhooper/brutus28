/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2022.
 *
 * ---------------------------------------------------------------------
 *
 * Low level STM32 GPIO access.
 */

#ifndef _GPIO_H
#define _GPIO_H

#include <libopencm3/stm32/gpio.h>
#ifdef STM32F4
#include <libopencm3/stm32/f4/gpio.h>
#else
#include <libopencm3/stm32/f1/gpio.h>
#endif

#define USB_CC1_PORT        GPIOA
#define USB_CC1_PIN             GPIO8
#define USB_CC2_PORT        GPIOA
#define USB_CC2_PIN             GPIO10

#define PLD_VCC_PORT        GPIOB
#define PLD_VCC_PIN             GPIO0     // Analog input * 2
#define PLD_GND_PORT        GPIOB
#define PLD_GND_PIN             GPIO1     // Analog input
#define ABORT_BUTTON_PORT   GPIOB
#define ABORT_BUTTON_PIN        GPIO4
#define EN_VCC_PORT         GPIOB
#define EN_VCC_PIN              GPIO5
#define LED_VCC_PORT        GPIOB
#define LED_VCC_PIN             GPIO8
#define LED_POWER_PORT      GPIOB
#define LED_POWER_PIN           GPIO9
#define LED_ALERT_PORT      GPIOB
#define LED_ALERT_PIN           GPIO10
#define LED_BUSY_PORT       GPIOB
#define LED_BUSY_PIN            GPIO11

#define EN_GND_PORT         GPIOC
#define EN_GND_PIN              GPIO12

#define PLD1_PORT           GPIOE  // PLD1-PLD16    = PE0-PE15
#define PLD17_PORT          GPIOC  // PLD17-PLD28   = PC0-PC11
#define PLDD1_PORT          GPIOD  // PLDD1-PLDD16  = PD0-PD15
#define PLDD17_PORT         GPIOA  // PLDD17-PLDD24 = PA0-PA7
#define PLDD25_PORT         GPIOB  // PLDD25-PLDD28 = PB12-PB15


/* Values for gpio_setmode() */
#ifdef STM32F1
#define GPIO_SETMODE_INPUT_ANALOG        0x0  // Analog Input
#define GPIO_SETMODE_INPUT               0x4  // Floating input (reset state)
#define GPIO_SETMODE_INPUT_PULLUPDOWN    0x8  // Input with pull-up / pull-down
#define GPIO_SETMODE_OUTPUT_PPULL_10     0x1  // 10 MHz, Push-Pull
#define GPIO_SETMODE_OUTPUT_ODRAIN_10    0x5  // 10 MHz, Open-Drain
#define GPIO_SETMODE_OUTPUT_AF_PPULL_10  0x9  // 10 MHz, Alt func. Push-Pull
#define GPIO_SETMODE_OUTPUT_AF_ODRAIN_10 0xd  // 10 MHz, Alt func. Open-Drain
#define GPIO_SETMODE_OUTPUT_PPULL_2      0x2  // 2 MHz, Push-Pull
#define GPIO_SETMODE_OUTPUT_ODRAIN_2     0x6  // 2 MHz, Open-Drain
#define GPIO_SETMODE_OUTPUT_AF_PPULL_2   0xa  // 2 MHz, Alt func. Push-Pull
#define GPIO_SETMODE_OUTPUT_AF_ODRAIN_2  0xe  // 2 MHz, Alt func. Open-Drain
#define GPIO_SETMODE_OUTPUT_PPULL_50     0x3  // 50 MHz, Push-Pull
#define GPIO_SETMODE_OUTPUT_ODRAIN_50    0x7  // 50 MHz, Open-Drain
#define GPIO_SETMODE_OUTPUT_AF_PPULL_50  0xb  // 50 MHz, Alt func. Push-Pull
#define GPIO_SETMODE_OUTPUT_AF_ODRAIN_50 0xf  // 50 MHz, Alt func. Open-Drain
#endif

void gpio_setv(uint32_t GPIOx, uint16_t GPIO_Pins, int value);
void gpio_setmode(uint32_t GPIOx, uint16_t GPIO_Pins, uint value);
void gpio_init(void);
void gpio_show(int whichport, int whichpin);
void gpio_assign(int whichport, int whichpin, const char *assign);

#endif /* _GPIO_H */

