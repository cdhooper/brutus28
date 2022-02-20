/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2022.
 *
 * ---------------------------------------------------------------------
 *
 * STM32 LED control
 */

#include "board.h"
#include "main.h"
#include <stdint.h>
#include "led.h"
#include "gpio.h"

#include <libopencm3/stm32/gpio.h>

void
led_alert(int turn_on)
{
    gpio_setv(LED_ALERT_PORT, LED_ALERT_PIN, turn_on);
}

void
led_busy(int turn_on)
{
    gpio_setv(LED_BUSY_PORT, LED_BUSY_PIN, turn_on);
}

void
led_power(int turn_on)
{
    gpio_setv(LED_POWER_PORT, LED_POWER_PIN, turn_on);
}

void
led_pld_vcc(int turn_on)
{
    gpio_setv(LED_VCC_PORT, LED_VCC_PIN, turn_on);
}
