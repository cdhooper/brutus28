/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2022.
 *
 * ---------------------------------------------------------------------
 *
 * main routine.
 */

#include "printf.h"
#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include "board.h"
#include "main.h"
#include "clock.h"
#include "uart.h"
#include "usb.h"
#include "led.h"
#include "gpio.h"
#include "adc.h"
#include "pld.h"
#include "cmdline.h"
#include "readline.h"
#include "timer.h"
#include "utils.h"
#include "version.h"

static void
reset_everything(void)
{
    RCC_APB1ENR  = 0;  // Disable all peripheral clocks
    RCC_APB1RSTR = 0xffffffff;  // Reset APB1
    RCC_APB2RSTR = 0xffffffff;  // Reset APB2
    RCC_APB1RSTR = 0x00000000;  // Release APB1 reset
    RCC_APB2RSTR = 0x00000000;  // Release APB2 reset
}


int
main(void)
{
    reset_check();
    reset_everything();
    clock_init();
    timer_init();
//  timer_delay_msec(10);  // Just for development purposes
    gpio_init();
    led_busy(1);
    uart_init();
    pld_init();

    printf("\r\nBrutus-28 %s\n", version_str);
    identify_cpu();
    show_reset_reason();
    usb_startup();

    rl_initialize();  // Enable command editing and history
    using_history();

    adc_init();


    led_power(1);
    led_busy(0);

    while (1) {
        usb_poll();
        cmdline();
    }

    return (0);
}
