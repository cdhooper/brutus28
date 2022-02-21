/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2022.
 *
 * ---------------------------------------------------------------------
 *
 * PLD-specific code.
 */

#include "board.h"
#include "main.h"
#include <string.h>
#include "printf.h"
#include "uart.h"
#include <stdbool.h>
#include "timer.h"
#include "gpio.h"
#include "adc.h"
#include "led.h"
#include "pld.h"
#include "utils.h"
#include "cmdline.h"
#include "cmds.h"
#include "pcmds.h"
#include "button.h"

#undef DEBUG_DETECT_PART_PRESENT
#undef DEBUG_VCC_AND_GND_JUMPERS

#define PLD_VCC_MISSING 0
#define PLD_VCC_3P3V    1
#define PLD_VCC_5V      2
static uint8_t pld_vcc_jumper;

#define PRESENT_PINS_PLCC28 0x0fdfbf7e
#define PRESENT_PINS_DIP24  0x00ffffff
#define PRESENT_PINS_DIP22  0x00ffe7ff
#define PRESENT_PINS_DIP20  0x00ffc3ff
#define PRESENT_PINS_DIP18  0x00ff81ff
#define PRESENT_PINS_DIP16  0x00ff00ff
#define PRESENT_PINS_DIP14  0x00fe007f
#define PRESENT_PINS_DIP12  0x00fc003f
#define PRESENT_PINS_DIP10  0x00f8001f
#define PRESENT_PINS_DIP8   0x00f0000f
#define PRESENT_PINS_DIP6   0x00e00007
#define PRESENT_PINS_DIP4   0x00c00003

static const struct {
    uint32_t          present;
    const char *const name;
} installed_types[] = {
    { PRESENT_PINS_PLCC28, "PLCC28" },
    { PRESENT_PINS_DIP24,  "DIP24" },
    { PRESENT_PINS_DIP22,  "DIP22" },
    { PRESENT_PINS_DIP20,  "DIP20" },
    { PRESENT_PINS_DIP18,  "DIP18" },
    { PRESENT_PINS_DIP16,  "DIP16" },
    { PRESENT_PINS_DIP14,  "DIP14" },
    { PRESENT_PINS_DIP12,  "DIP12" },
    { PRESENT_PINS_DIP10,  "DIP10" },
    { PRESENT_PINS_DIP8,   "DIP8"  },
    { PRESENT_PINS_DIP6,   "DIP6"  },
    { PRESENT_PINS_DIP4,   "DIP4"  },
};

/*
 * pld_vcc_disable
 * ---------------
 * Turn off PLD VCC supply rail.
 */
static void
pld_vcc_disable(void)
{
    gpio_setv(EN_VCC_PORT, EN_VCC_PIN, 0);  // 0=Off
    gpio_setmode(EN_VCC_PORT, EN_VCC_PIN, GPIO_SETMODE_OUTPUT_PPULL_2);
}

/*
 * pld_vcc_enable
 * --------------
 * Turn on PLD VCC supply rail.
 */
static void
pld_vcc_enable(void)
{
    gpio_setv(EN_VCC_PORT, EN_VCC_PIN, 1);  // 1=On
    gpio_setmode(EN_VCC_PORT, EN_VCC_PIN, GPIO_SETMODE_OUTPUT_PPULL_2);
}

/*
 * pld_gnd_disable
 * ---------------
 * Turn off PLD GND supply rail.
 */
static void
pld_gnd_disable(void)
{
    gpio_setv(EN_GND_PORT, EN_GND_PIN, 0);  // 0=Off
    gpio_setmode(EN_GND_PORT, EN_GND_PIN, GPIO_SETMODE_OUTPUT_PPULL_2);
}

/*
 * pld_gnd_enable
 * --------------
 * Turn on PLD GND supply rail.
 */
static void
pld_gnd_enable(void)
{
    gpio_setv(EN_GND_PORT, EN_GND_PIN, 1);  // 1=On
    gpio_setmode(EN_GND_PORT, EN_GND_PIN, GPIO_SETMODE_OUTPUT_PPULL_2);
}

/*
 * pld_power_disable
 * -----------------
 * Turn off both PLD VDD and GND supply rails.
 */
static void
pld_power_disable(void)
{
    /* Configure power as disabled */
    pld_vcc_disable();
    pld_gnd_disable();
    led_pld_vcc(0);
}

/*
 * pld_power_enable
 * ----------------
 * Turn on both PLD VDD and GND supply rails.
 */
static void
pld_power_enable(void)
{
    pld_vcc_enable();
    pld_gnd_enable();
    led_pld_vcc(1);
}

/*
 * pld_gpio_setmode
 * ----------------
 * Set GPIO pin configuration for the STM32 PLD_* pins.
 */
static void
pld_gpio_setmode(uint32_t pins, uint mode)
{
    gpio_setmode(PLD1_PORT, pins & 0xffff, mode);           // PE0-PE15
    gpio_setmode(PLD17_PORT, (pins >> 16) & 0x0fff, mode);  // PC0-PC11
}

/*
 * pldd_gpio_setmode
 * -----------------
 * Set GPIO pin configuration for the STM32 PLDD_* pins.
 */
static void
pldd_gpio_setmode(uint32_t pins, uint mode)
{
    gpio_setmode(PLDD1_PORT, pins & 0xffff, mode);          // PD0-PD15
    gpio_setmode(PLDD17_PORT, (pins >> 16) & 0xff, mode);   // PA0-PA7
    gpio_setmode(PLDD25_PORT, (pins >> 12) & 0xf000, mode); // PB12-PB15
}

/*
 * pld_output_disable
 * ------------------
 * Configure STM32 PLD_* and PLDD_* pins as inputs (stop driving).
 * In normal operation, the PLD_* pins are always input. They are
 * only set as output when detecting the part or jumper configuration.
 */
static void
pld_output_disable(void)
{
    /* Configure PLD and PLDD pins as input */
    pld_gpio_setmode(0x0fffffff, GPIO_SETMODE_INPUT);
    pldd_gpio_setmode(0x0fffffff, GPIO_SETMODE_INPUT);
}

/*
 * pldd_output_enable
 * ------------------
 * Configure STM32 PLDD_* pins as outputs.
 */
static void
pldd_output_enable(void)
{
    pldd_gpio_setmode(0x0fffffff, GPIO_SETMODE_OUTPUT_PPULL_10);
}

/*
 * pldd_output_value
 * -----------------
 * Report the current value being driven to the PLDD_* pins, so long
 * as they are configured as outputs or pull-up/pull-down.
 */
static uint32_t
pldd_output_value(void)
{
    return (GPIO_ODR(PLDD1_PORT) |
            ((GPIO_ODR(PLDD17_PORT) & 0x00ff) << 16) |
            ((GPIO_ODR(PLDD25_PORT) & 0xf000) << 12));
}

/*
 * pldd_output
 * -----------
 * Drive the PLDD_* pins with the specified 28-bit value.
 */
static void
pldd_output(uint32_t data)
{
    GPIO_ODR(PLDD1_PORT) = data;  // PLDD1-PLDD16

    GPIO_BSRR(PLDD17_PORT) = 0x00ff0000 |             // Clear PLDD17-PLDD24
                             ((data >> 16) & 0x00ff); // Set PLDD17-PLDD24

    GPIO_BSRR(PLDD25_PORT) = 0xf0000000 |             // Clear PLDD25-PLDD28
                             ((data >> 12) & 0xf000); // Set PLDD25-PLDD28
}

/*
 * pldd_input
 * ----------
 * Read the current state of the PLDD_* pins (input).
 */
static uint32_t
pldd_input(void)
{
    return (GPIO_IDR(PLDD1_PORT) |                     // PLDD1-PLDD16
            ((GPIO_IDR(PLDD17_PORT) & 0x00ff) << 16) | // PLDD17-PLDD24
            ((GPIO_IDR(PLDD25_PORT) & 0xf000) << 12)); // PLDD25-PLDD28
}

/*
 * pld_input
 * ---------
 * Read the current state of the PLD_* pins (input).
 */
static uint32_t
pld_input(void)
{
    return (GPIO_IDR(PLD1_PORT) |                     // PLD1-PLD16
            ((GPIO_IDR(PLD17_PORT) & 0x0fff) << 16)); // PLD17-PLD28
}

/*
 * pld_output
 * ----------
 * Drive the specified value on the PLD_* pins. Note that the PLD_*
 * pins must first be manually configured to drive for this function
 * to have any effect.
 */
static void
pld_output(uint32_t data)
{
    /* Set the PLD output value */
    GPIO_ODR(PLD1_PORT) = data;  // PLD1-PLD16

    GPIO_BSRR(PLD17_PORT) = 0x0fff0000 |              // Clear PLD17-PLD24
                             ((data >> 16) & 0x0fff); // Set PLD17-PLD28
}

/*
 * pldd_output_pld_input
 * ---------------------
 * Writes the specified output value to the PLD output and captures
 * resulting input from the PLD. This is done in one step with hopefully
 * sufficient CPU delay in between.
 */
static uint32_t
pldd_output_pld_input(uint32_t wvalue)
{
    pldd_output(wvalue);

    /*
     * Delay to allow for PLD gate latency
     * 72 MHz = 13.9 ns. 2x nop should allow 27 ns of latency.
     */
    __asm__ volatile("nop");
    __asm__ volatile("nop");

    return (pld_input());
}

/*
 * pld_disable
 * -----------
 * Disables all PLD_* and PLDD_* outputs and disables power to the PLD
 * VCC and GND rails.
 */
static void
pld_disable(void)
{
    pld_output_disable();
    pld_power_disable();
    adc_enable();
}

/*
 * pld_enable
 * ----------
 * Enables power to the PLD rails and drives a 0 value to all PLDD_* pins.
 * The PLDD_* pins are connected to the target PLD via 1K resistors.
 */
static void
pld_enable(void)
{
    adc_enable();
    pld_power_enable();
    pldd_output(0);
    pldd_output_enable();
}

/*
 * pld_init
 * --------
 * Configures PLD GPIOs for their default state.
 */
void
pld_init(void)
{
    pld_disable();
}

/*
 * bit_count
 * ---------
 * Simply returns the number of '1' bits in a value. A more optimal version
 * should be written if it were called more than once during a walk.
 */
static uint
bit_count(uint32_t mask)
{
    uint count = 0;
    while (mask != 0) {
        if (mask & 1)
            count++;
        mask >>= 1;
    }
    return (count);
}

/*
 * show_reading
 * ------------
 * Shows a voltage reading, converting from millivolts to volts.fractional V
 */
static void
show_reading(const char *text, uint value)
{
    printf("%s%u.%02uV", text, value / 1000, (value % 1000) / 10);
}

/*
 * print_binary
 * ------------
 * Displays a 28-bit value in human-readable binary.
 */
static void
print_binary(uint32_t value)
{
    int bit;
    for (bit = 27; bit >= 0; bit--) {
        putchar('0' + !!(value & BIT(bit)));
        if ((bit == 24) || (bit == 16) || (bit == 8))
            putchar(':');
    }
}

/*
 * print_binary_buf
 * ----------------
 * Saves a 28-bit value in human-readable binary to the specified buffer.
 */
static int
print_binary_buf(uint32_t value, char *buf)
{
    int bit;
    for (bit = 27; bit >= 0; bit--) {
        *(buf++) = '0' + !!(value & BIT(bit));
        if ((bit == 24) || (bit == 16) || (bit == 8))
            *(buf++) = ':';
    }
    return (31);
}

/*
 * pld_check_vcc_gnd_shorts
 * ------------------------
 * Check the PLD power rails for VCC-GND shorts.
 */
static int
pld_check_vcc_gnd_shorts(void)
{
    uint pld_gnd;
    uint pld_vcc;
    uint temp;
    rc_t rc = RC_SUCCESS;

    pld_disable();   // Start with everything off
    adc_pulldown();  // Drain ADC VCC and GND rails
    timer_delay_msec(50);

    /*
     * Just test floating voltage of PLD VCC and GND.
     * Expected result is that VCC < 0.20V and GND < 2.00V
     */
    adc_enable();
    timer_delay_msec(2);
    pld_vcc = adc_get_pld_readings(&pld_gnd);
    if (pld_vcc >= 200) {
        show_reading("FAIL: PLD VCC=", pld_vcc);
        show_reading(" GND=", pld_gnd);
        printf(" when not driving PLD pins or rails\n"
               "    Expected: PLD VCC < 0.20V & GND < 2.00V\n");
        rc = RC_FAILURE;
    }

    /*
     * Test voltages when PLD GND is driven high by the STM32.
     * VCC should be > 3.00V. Can't check GND voltage, since that
     * is the pin being driven, but can detect short to board
     * GND, as the STM32 input is independent of the output and
     * will report the actual high or low state of the pin.
     *
     * The fact that PLD VCC will be > 3.00V is counter-intuitive
     * to me, as the two nets should not interact if there's no
     * PLD in the socket. Regardless, that does seem to occur, and
     * it might be due to the two capacitors (C7 & C8) which are
     * connected between the rails.
     */
    gpio_setv(PLD_GND_PORT, PLD_GND_PIN, 1);
    gpio_setmode(PLD_GND_PORT, PLD_GND_PIN, GPIO_SETMODE_OUTPUT_PPULL_2);
    timer_delay_msec(2);
    pld_vcc = adc_get_pld_readings(&temp);
    pld_gnd = gpio_get(PLD_GND_PORT, PLD_GND_PIN) ? 1 : 0;
    if ((pld_vcc <= 3000) || (pld_gnd != 1)) {
        show_reading("FAIL: PLD VCC=", pld_vcc);
        printf(" GND=%u when PLD GND driven high by STM32\n"
               "    Expected: PLD VCC > 3.0V & GND=1\n", pld_gnd);
        rc = RC_FAILURE;
    }

    /*
     * Test voltages when PLD GND driven low by STM32.
     * VCC should be < 0.20V and GND pin input should be 0.
     */
    gpio_setv(PLD_GND_PORT, PLD_GND_PIN, 0);
    gpio_setmode(PLD_GND_PORT, PLD_GND_PIN, GPIO_SETMODE_OUTPUT_PPULL_2);
    timer_delay_msec(2);
    pld_vcc = adc_get_pld_readings(&temp);
    pld_gnd = gpio_get(PLD_GND_PORT, PLD_GND_PIN) ? 1 : 0;
    if ((pld_vcc >= 200) || (pld_gnd != 0)) {
        show_reading("FAIL: PLD VCC=", pld_vcc);
        printf(" GND=%u when PLD GND driven low by STM32\n"
               "    Expected: PLD VCC < 0.2V & GND=0\n", pld_gnd);
        rc = RC_FAILURE;
    }

    pld_power_disable();
    adc_enable();

    /*
     * Test voltages when PLD VCC driven high by STM32.
     * analog GND should be > 3.0V and VCC pin input should be 1.
     */
    gpio_setv(PLD_VCC_PORT, PLD_VCC_PIN, 1);
    gpio_setmode(PLD_VCC_PORT, PLD_VCC_PIN, GPIO_SETMODE_OUTPUT_PPULL_2);
    timer_delay_msec(2);
    (void) adc_get_pld_readings(&pld_gnd);
    pld_vcc = gpio_get(PLD_VCC_PORT, PLD_VCC_PIN) ? 1 : 0;
    if ((pld_vcc != 1) || (pld_gnd <= 3000)) {
        printf("FAIL: PLD VCC=%d", pld_vcc);
        show_reading(" GND=", pld_gnd);
        printf(" when PLD VCC driven high by STM32\n"
               "    Expected: PLD VCC=1 & GND > 3.0V\n");
        rc = RC_FAILURE;
    }

    /*
     * Test voltages when PLD VCC driven low by STM32.
     * This might not be a very useful test. GND could float
     * high in this case (will pick an arbitrary < 2.0V limit),
     * but the analog GND and VCC pin input should be 0.
     */
    gpio_setv(PLD_VCC_PORT, PLD_VCC_PIN, 0);
    gpio_setmode(PLD_VCC_PORT, PLD_VCC_PIN, GPIO_SETMODE_OUTPUT_PPULL_2);
    timer_delay_msec(2);
    (void) adc_get_pld_readings(&pld_gnd);
    pld_vcc = gpio_get(PLD_VCC_PORT, PLD_VCC_PIN) ? 1 : 0;
    if ((pld_vcc != 0) || (pld_gnd > 2000)) {
        printf("FAIL: PLD VCC=%d", pld_vcc);
        show_reading(" GND=", pld_gnd);
        printf(" when PLD VCC driven low by STM32\n"
               "    Expected: PLD VCC=0 & GND < 2.0V\n");
        rc = RC_FAILURE;
    }

    /*
     * Test voltages when PLD VCC driven high and EN_PLD_GND enabled.
     * GND should be < 0.1V and VCC pin should be 1.
     */
    gpio_setv(EN_GND_PORT, EN_GND_PIN, 1);  // 1=On
    gpio_setv(PLD_VCC_PORT, PLD_VCC_PIN, 1);
    gpio_setmode(PLD_VCC_PORT, PLD_VCC_PIN, GPIO_SETMODE_OUTPUT_PPULL_2);
    timer_delay_msec(2);
    (void) adc_get_pld_readings(&pld_gnd);
    pld_vcc = gpio_get(PLD_VCC_PORT, PLD_VCC_PIN) ? 1 : 0;
    if ((pld_vcc != 1) || (pld_gnd > 100)) {
        printf("FAIL: PLD VCC=%d", pld_vcc);
        show_reading(" GND=", pld_gnd);
        printf(" when PLD VCC driven high by STM32 and PLD_GND enabled.\n"
               "    Expected: PLD VCC=0 & GND < 0.1V\n");
        rc = RC_FAILURE;
    }

    return (rc);
}

/*
 * pld_report_5v_3p3v_jumper
 * -------------------------
 * Sense and report the setting of the 5V/3.3V jumper. This algorithm
 * is complicated by the fact that a PLD might be installed and jumpers
 * might connect PLD pins with VCC and GND.
 */
static rc_t
pld_report_5v_3p3v_jumper(int verbose)
{
    uint pld_vcc;
    uint pld_gnd;

    /*
     * Test whether 3.3V / 5V jumper is installed,
     */
    pld_power_disable();
    pld_gnd_enable();
    timer_delay_msec(1);
    pld_gnd_disable();
    pld_vcc_enable();
    adc_enable();
    timer_delay_msec(10);
    pld_vcc = adc_get_pld_readings(&pld_gnd);
    pld_vcc_disable();
    pld_gnd_enable();
    timer_delay_msec(10);
    (void) adc_get_pld_readings(&pld_gnd);
    pld_power_disable();

    if ((pld_gnd < 300) && (pld_vcc > 3000) && (pld_vcc < 3600)) {
        pld_vcc_jumper = PLD_VCC_3P3V;
        if (verbose) {
            show_reading("VCC source:  3.3V   PLD VCC=", pld_vcc);
            printf("\n");
        }
    } else if ((pld_gnd < 300) && (pld_vcc >= 4200) && (pld_vcc <= 5500)) {
        pld_vcc_jumper = PLD_VCC_5V;
        if (verbose) {
            show_reading("VCC source:  5V   PLD VCC=", pld_vcc);
            printf("\n");
        }
    } else if ((pld_gnd < 300) && (pld_vcc >= 3500) && (pld_vcc <= 4200)) {
        show_reading("WARNING: PLD VCC=", pld_vcc);
        show_reading(" GND=", pld_gnd);
        printf(" when VCC and GND enabled.\n"
               "Is there a PLD installed?\n");
    } else {
        show_reading("FAIL: PLD VCC=", pld_vcc);
        show_reading(" GND=", pld_gnd);
        printf(" when VCC and GND enabled.\n"
               "    Expected: PLD VCC > 3.0V & GND < 0.3V\n");
        if (pld_vcc < 1000)
            printf("Is the PLD POWER jumper installed?\n");
        return (RC_FAILURE);
    }
    return (RC_SUCCESS);
}

/*
 * pld_report_gnd_and_vcc_jumpers
 * ------------------------------
 * Detect and report where jumpers are installed that provide VCC and GND
 * to the PLD. This is difficult to accomplish without false readings if a
 * PLD is installed.
 */
static rc_t
pld_report_gnd_and_vcc_jumpers(int verbose, uint32_t *vcc_p, uint32_t *gnd_p)
{
    uint pld_gnd;
    uint pld_vcc;
    uint pin;
    uint vcc_pins = 0;
    uint gnd_pins = 0;

    adc_enable();

    /*
     * Find potential VCC and GND pins. This can be difficult because if
     * there is a PLD in a socket, both VCC and GND will show voltage on
     * pins due to backpower through the device.
     *
     * 1. Drain residual power
     * 2. Set all PLDD pins to input
     * 3. On at a time, set PLDD pins to output 1
     * 4. At each set PLDD pin, measure VCC and GND.
     * 5. If no PLD is installed, we should get a very clear > 1.2V reading
     *    for pins connected to VCC or GND.
     * 6. If a PLD is installed, probably both VCC and GND will trigger for
     *    those pins where one or the other is connected. No other pins,
     *    even if connected to the PLD, should trigger this.
     * 7. After iterating all pins, an additional pass with the PLD GND
     *    plane attached is used to differentiate the pins which are connected
     *    to VCC vs those connected to GND.
     */
    pld_disable();
    pldd_output(0x00000000);
    pldd_output_enable();
    timer_delay_msec(10);
    pldd_gpio_setmode(0x0fffffff, GPIO_SETMODE_INPUT);
    pldd_output(0x0fffffff);

    for (pin = 0; pin < 28; pin++) {
        pldd_output(BIT(pin));
        pldd_gpio_setmode(0x0fffffff, GPIO_SETMODE_INPUT);
        pldd_gpio_setmode(BIT(pin), GPIO_SETMODE_OUTPUT_PPULL_2);
        timer_delay_msec(1);
        pld_vcc = adc_get_pld_readings(&pld_gnd);
        if (pld_vcc > 1200)
            vcc_pins |= BIT(pin);  // 1.200V threshold
        if (pld_gnd > 1200)
            gnd_pins |= BIT(pin);  // 1.200V threshold
    }
#ifdef DEBUG_VCC_AND_GND_JUMPERS
    pld_disable();
    printf("PLD_VCC=");
    print_binary(vcc_pins);
    printf("\nPLD_GND=");
    print_binary(gnd_pins);
    printf("\n");
#endif

    /*
     * Second part of the test is just to differentiate VCC and GND
     * connected pins when a PLD is installed in the socket.
     *
     * 1. Enable PLD GND plane
     * 2. Walk the potential VCC and GND pins, setting each one individually
     *    high using the strong drive PLD pins.
     * 3. Observe VCC and GND analog readings. VCC will not be above 1.0V
     *    on the GND-connected pin. GND should be low in all cases.
     * 4. If VCC is above 1.0V, then the pin is connected to VCC and not GND.
     */
    pld_disable();
    pld_gnd_enable();
    pld_output(0x00000000);
    pld_gpio_setmode(0x0fffffff, GPIO_SETMODE_INPUT);

    for (pin = 0; pin < 28; pin++) {
        if ((BIT(pin) & (vcc_pins | gnd_pins)) == 0)
            continue;
        pld_gpio_setmode(0x0fffffff, GPIO_SETMODE_INPUT);
        pld_output(BIT(pin));
        pld_gpio_setmode(BIT(pin), GPIO_SETMODE_OUTPUT_PPULL_2);
        timer_delay_msec(1);

        pld_vcc = adc_get_pld_readings(&pld_gnd);
#ifdef DEBUG_VCC_AND_GND_JUMPERS
        uint input = pld_input();
        printf("Pin%-2u ", pin + 1);
        print_binary(input);
        show_reading(" VCC=", pld_vcc);
        show_reading(" GND=", pld_gnd);
        printf("\n");
#endif
        if (pld_vcc < 1200)
            vcc_pins &= ~BIT(pin);
        else
            gnd_pins &= ~BIT(pin);
    }

    pld_disable();

    printf("VCC jumpers:");
    for (pin = 0; pin < 28; pin++)
        if (vcc_pins & BIT(pin))
            printf(" Pin%u", pin + 1);
    if (vcc_pins == 0)
        printf(" None");
    printf("\nGND jumpers:");
    for (pin = 0; pin < 28; pin++)
        if (gnd_pins & BIT(pin))
            printf(" Pin%u", pin + 1);
    if (gnd_pins == 0)
        printf(" None detected");
    printf("\n");

    if ((vcc_pins == (BIT(28) >> 1)) &&
        (gnd_pins == (BIT(14) >> 1)) &&
        (pld_vcc_jumper == PLD_VCC_5V)) {
        printf("Jumper configuration is standard for a PLCC28 GAL22V10\n");
    } else if ((vcc_pins == (BIT(24) >> 1)) &&
               (gnd_pins == (BIT(12) >> 1)) &&
               (pld_vcc_jumper == PLD_VCC_5V)) {
        printf("Jumper configuration is standard for a DIP GAL22V10\n");
    } else {
        printf("Jumper configuration is not standard\n"
               "    For PLCC GAL22V10, need 5V, VCC=Pin28, GND=Pin14\n"
               "    For DIP GAL22V10, need 5V, VCC=Pin24, GND=Pin12\n");
    }

    *vcc_p = vcc_pins;
    *gnd_p = gnd_pins;
    return (RC_SUCCESS);
}

/*
 * pld_detect_part_present
 * -----------------------
 * Will detect if a part is present and the socket pins into which the
 * part has been inserted.
 */
static void
pld_detect_part_present(uint32_t *pins_present)
{
    uint pin;
    uint pld_indata;
    uint present = 0;
    uint count;
    pld_disable();

    pldd_output(0x0fffffff);
    for (pin = 0; pin < 28; pin++) {
        pldd_gpio_setmode(~BIT(pin), GPIO_SETMODE_OUTPUT_PPULL_2);
        pldd_gpio_setmode(BIT(pin), GPIO_SETMODE_INPUT_PULLUPDOWN);
        pldd_output(~BIT(pin));
        timer_delay_msec(1);
        pld_indata = pld_input();
        if (pld_indata & BIT(pin))
            present |= BIT(pin);
#ifdef DEBUG_DETECT_PART_PRESENT
        printf("Pin%-2u ", pin + 1);
        print_binary(pld_indata);
        printf("\n");
#endif
    }
    pld_disable();

    count = bit_count(present);

    if ((present & PRESENT_PINS_PLCC28) == PRESENT_PINS_PLCC28) {
        printf("Detected %s device inserted\n", installed_types[0].name);
    } else if ((count > 23) && (present & 0x0f000000)) {
        printf("Likely PLCC28 device inserted\n    ");
        print_binary(present);
        printf("\n");
    } else if (count < 4) {
        printf("No part inserted\n");
    } else {
        uint dip;
        for (dip = 0; dip < ARRAY_SIZE(installed_types); dip++) {
            if (present == installed_types[dip].present) {
                printf("Detected %s device inserted\n",
                       installed_types[dip].name);
                break;
            }
        }
        if (dip >= ARRAY_SIZE(installed_types)) {
            printf("Unknown device inserted\n    ");
            print_binary(present);
            printf("\n");
        }
    }
    if (pins_present != NULL)
        *pins_present = present;
}

/*
 * pld_check
 * ---------
 * Implements the "pld check" command. Several checks are performed,
 * including verifying that GND and VCC jumpers are set, the voltage
 * is set, and that there are no shorts or open paths on the PCB.
 * This command does not currently work when a part is installed.
 */
static rc_t
pld_check(void)
{
    uint     pin;
    uint     rep;
    uint32_t vcc_pins;
    uint32_t gnd_pins;
    uint32_t pld_indata;
    uint32_t ignore_pins;
    uint64_t time_start;
    rc_t     rc = RC_SUCCESS;

    pld_detect_part_present(NULL);
    rc = pld_report_5v_3p3v_jumper(1);
    if (rc != RC_SUCCESS)
        return (rc);
    rc = pld_report_gnd_and_vcc_jumpers(1, &vcc_pins, &gnd_pins);
    if (rc != RC_SUCCESS)
        return (rc);
    rc = pld_check_vcc_gnd_shorts();
    if (rc != RC_SUCCESS)
        return (rc);

    ignore_pins = vcc_pins | gnd_pins;

    /*
     * Check CPU PLD_* GPIOs are connected to PLDD_* GPIOs.
     * 1) Disable power to PLDs
     * 2) Set all PLD_* and PLDD_* pins to pulldown
     * 3) Drive PLDD_* pins high one at a time, verifying only the
     *    corresponding PLD_* pin goes high. All other PLDD_* pins
     *    will remain pulldown.
     */
    pld_disable();
    pldd_output(0x00000000);
    pldd_gpio_setmode(0x0fffffff, GPIO_SETMODE_INPUT_PULLUPDOWN);
    pld_output(0x00000000);
    pld_gpio_setmode(0x0fffffff, GPIO_SETMODE_INPUT_PULLUPDOWN);
    timer_delay_msec(10);

    /* Check for pins stuck high */
    pld_indata = pld_input() & ~ignore_pins;
    for (pin = 0; pin < 28; pin++) {
        if (pld_indata & BIT(pin)) {
            if (rc == RC_SUCCESS)
                printf("FAIL when all PLD pins are pulled low\n");
            printf("    Pin%u is high when it should be low - short to VCC?",
                   pin + 1);
            rc = RC_FAILURE;
        }
    }

    for (pin = 0; pin < 28; pin++) {
        uint32_t pldd_indata;
        uint32_t pldd_outdata = BIT(pin);
        uint32_t log_indata[4];

        if (BIT(pin) & ignore_pins)
            continue;

        time_start = timer_tick_get();
        pldd_output(pldd_outdata);
        pldd_gpio_setmode(~BIT(pin), GPIO_SETMODE_INPUT_PULLUPDOWN);
        pldd_gpio_setmode(BIT(pin), GPIO_SETMODE_OUTPUT_PPULL_10);
        timer_delay_msec(1);
#define CHECK_REPS 10000  // ~100 msec
        for (rep = 0; rep < CHECK_REPS; rep++) {
            pld_indata = pld_input() & ~ignore_pins;
            if (pld_indata == pldd_outdata) {
                if ((rep > 0) && ((gnd_pins & BIT(pin)) == 0)) {
                    /* GND pins take longer to settle because of capacitor */
                    uint cur;

                    printf("Pin%-2u took %llu usec to settle\n", pin + 1,
                           timer_tick_to_usec(timer_tick_get() - time_start));
                    if (rep < 4) {
                        cur = 0;
                    } else {
                        cur = rep - 4;
                    }
                    printf("    Most recent states:\n");
                    while (cur < rep) {
                        printf("    ");
                        print_binary(log_indata[cur & 3]);
                        printf("\n");
                        cur++;
                    }
                    printf("    ");
                    print_binary(pld_indata);
                    printf("\n");
                }
                break;
            }
            log_indata[rep & 3] = pld_indata;
            timer_delay_usec(1);
        }
        if (rep >= CHECK_REPS) {
            int  tpin;
            uint pins_high = pld_indata & ~BIT(pin);
            pldd_indata = pldd_input() & ~ignore_pins;
            printf("FAIL when Pin%u driven high\n    ", pin + 1);
            print_binary(pld_indata);
            printf("\n    ");
            for (tpin = 27; tpin > 0; tpin--) {
                char ch = ' ';
                if (tpin == (int)pin) {
                    if ((pld_indata & BIT(tpin)) == 0)
                        ch = '!';
                } else if ((pldd_indata | pld_indata) & BIT(tpin)) {
                    ch = '!';
                }
                putchar(ch);
                if ((tpin == 24) || (tpin == 16) || (tpin == 8))
                    putchar(' ');
            }
            printf("\n");
            for (tpin = 0; tpin < 28; tpin++) {
                if (pins_high & BIT(tpin)) {
                    printf("    Pin%u is high when it should be low\n",
                           tpin + 1);
                }
            }
            if ((pldd_indata & BIT(pin)) == 0) {
                printf("    Pin%u (PLDD) overdriven - short to GND?\n",
                       pin + 1);
            } else if ((pld_indata & BIT(pin)) == 0) {
                /*
                 * Either open circuit or PLD pin shorted to GND.
                 * Attempt to differentiate a short to GND.
                 */
                uint32_t temp_in;
                pld_output(BIT(pin));
                pld_gpio_setmode(BIT(pin), GPIO_SETMODE_OUTPUT_PPULL_10);
                timer_delay_msec(1);
                temp_in = pld_input();
                pld_output(9);
                pld_gpio_setmode(BIT(pin), GPIO_SETMODE_INPUT_PULLUPDOWN);
                printf("    Pin%u (PLD) is low when it should be high - ",
                       pin + 1);
                if (temp_in & BIT(pin))
                    printf("bad connection at resistor?\n");
                else
                    printf("short to GND?\n");
            }
            rc = RC_FAILURE;
        }
    }
    pld_disable();

    return (rc);
}

const char cmd_pld_walk_help[] =
"pld walk options\n"
"  <spin>-<epin>  - specify a range of pins to walk; range 1-28\n"
"  <pin1>,<pin2>  - specify multiple individual pins (-pin removes it)\n"
"  analyze        - perform a quick analysis\n"
"  auto           - automatically probe to select device pins\n"
"  binary         - show binary instead of hex\n"
"  deep           - perform a deep analysis (takes a lot longer)\n"
"  dip            - select standard DIP 22V10 pins\n"
"  invert         - invert ignored pins (make them 1 instead of 0)\n"
"  plcc           - select standard PLCC 22V10 pins\n"
"  raw            - dump raw values (not ASCII)\n"
"  values         - report values (ASCII hex or binary)\n"
"  zero           - perform walking zeros instead of walking ones\n";

#define DIP_22V20_IGNORE_PINS  (((BIT(12) | BIT(24) | BIT(25) | BIT(26) | \
                                  BIT(27) | BIT(28)) >> 1) | 0xf0000000)
#define PLCC_22V20_IGNORE_PINS (((BIT(1) | BIT(8) | BIT(14) | BIT(15) | \
                                  BIT(22) | BIT(28)) >> 1) | 0xf0000000)

#define WALK_FLAG_ANALYZE       0x01  // Do analysis
#define WALK_FLAG_ANALYZE_DEEP  0x02  // Do deep analysis
#define WALK_FLAG_SHOW_BINARY   0x04  // Show binary instead of hex
#define WALK_FLAG_INVERT_IGNORE 0x08  // Make ignored pins 1 instead of 0
#define WALK_FLAG_RAW_BINARY    0x10  // Show raw binary values
#define WALK_FLAG_VALUES        0x20  // Show ASCII values
#define WALK_FLAG_WALK_ZERO     0x48  // Walking zeros

/*
 * cmd_pld_get_ignore_mask
 * -----------------------
 * Capture an ignore_mask and command flags from user-entered command line
 * input. Input can include a footprint type (dip or plcc), pin ranges,
 * and individual pin numbers. Ranges and pin numbers may be negated to
 * remove specific pins from those previously specified.
 */
static rc_t
cmd_pld_get_ignore_mask(int argc, char * const *argv,
                        uint32_t *ignore, uint *flags)
{
    int      arg;
    uint32_t ignore_mask = 0;
    rc_t     rc = RC_SUCCESS;
    uint     ignore_initialized = 0;

    /* Capture bits to walk from command arguments */
    for (arg = 1; arg < argc; arg++) {
        const char *ptr = argv[arg];
        const char *nptr;
        int         mode;
        switch (*ptr) {
            case '?':
                printf("%s", cmd_pld_walk_help);
                return (RC_FAILURE);
            case 'a':
                if (strncmp("auto", ptr, strlen(ptr)) == 0) {
                    uint32_t present;
                    pld_detect_part_present(&present);

                    if ((present & PRESENT_PINS_PLCC28) ==
                                   PRESENT_PINS_PLCC28) {
                        ignore_mask = PLCC_22V20_IGNORE_PINS;
                    } else if (present != 0) {
                        uint32_t vcc_pins;
                        uint32_t gnd_pins;
                        ignore_mask = ~present;
                        rc = pld_report_gnd_and_vcc_jumpers(1, &vcc_pins,
                                                            &gnd_pins);
                        if (rc == RC_SUCCESS)
                            ignore_mask |= vcc_pins | gnd_pins;
                    } else {
                        return (RC_FAILURE);
                    }
                    ignore_initialized = 1;
                    continue;
                }
                if (strncmp("analyze", ptr, strlen(ptr)))
                    goto invalid_argument;
                *flags |= WALK_FLAG_ANALYZE;
                continue;
            case 'b':
                if (strncmp("binary", ptr, strlen(ptr)))
                    goto invalid_argument;
                *flags |= WALK_FLAG_SHOW_BINARY;
                continue;
            case 'd':
                if (strncmp("deep", ptr, strlen(ptr)) == 0) {
                    *flags |= WALK_FLAG_ANALYZE_DEEP | WALK_FLAG_ANALYZE;
                    continue;
                }
                if (strncmp("dip", ptr, strlen(ptr)))
                    goto invalid_argument;
                ignore_mask = DIP_22V20_IGNORE_PINS;
                ignore_initialized = 1;
                continue;
            case 'i':
                if (strncmp("invert", ptr, strlen(ptr)))
                    goto invalid_argument;
                *flags |= WALK_FLAG_INVERT_IGNORE;
                continue;
            case 'p':
                if (strncmp("plcc", ptr, strlen(ptr)))
                    goto invalid_argument;
                ignore_mask = PLCC_22V20_IGNORE_PINS;
                ignore_initialized = 1;
                continue;
            case 'r':
                if (strncmp("raw", ptr, strlen(ptr)))
                    goto invalid_argument;
                *flags |= WALK_FLAG_RAW_BINARY | WALK_FLAG_VALUES;
                continue;
            case 'v':
                if (strncmp("values", ptr, strlen(ptr)))
                    goto invalid_argument;
                *flags |= WALK_FLAG_VALUES;
                continue;
            case 'z':
                if (strncmp("zero", ptr, strlen(ptr))) {
invalid_argument:
                    printf("Invalid argument '%s'\n", ptr);
                    return (RC_FAILURE);
                }
                *flags |= WALK_FLAG_WALK_ZERO;
                continue;
            case '-':
                /* Add to the ignore mask */
                if (ignore_initialized == 0) {
                    ignore_initialized = 1;
                    ignore_mask = 0;
                }
                ptr++;
                mode = 0; // Add
                break;
            default:
                /* Remove from the ignore mask */
                if (ignore_initialized == 0) {
                    ignore_initialized = 1;
                    ignore_mask = 0xffffffff;
                }
                mode = 1;  // Remove
                break;
        }
        while (*ptr != '\0') {
            int pos;
            int start;
            int end;
            if ((sscanf(ptr, "%d%n", &start, &pos) != 1) || (pos == 0) ||
                (start < 1) || (start > 28)) {
                printf("Invalid argument '%s'\n", ptr);
                printf("%s", cmd_pld_walk_help);
                return (RC_FAILURE);
            }
            nptr = ptr + pos;
            if ((*nptr == ',') || (*nptr == '\0')) {
                /* Single position */
                start--;
                ptr = nptr;
                if (mode == 0)
                    ignore_mask |= BIT(start);
                else
                    ignore_mask &= ~BIT(start);
                if (*ptr == '\0')
                    break;
            } else if (*nptr == '-') {
                uint32_t newmask;
                nptr++;
                if ((sscanf(nptr, "%d%n", &end, &pos) != 1) || (pos == 0) ||
                    (end < 1) || (end > 28)) {
                    printf("Invalid argument '%s' at '%s'\n", ptr, nptr);
                    return (RC_USER_HELP);
                }
                nptr += pos;

                /* Allow bits to be specified in either order */
                if (end < start) {
                    int temp = end;
                    end = start;
                    start = temp;
                }
                start--;
                newmask = ((BIT(end) - 1) ^ (BIT(start) - 1));
                if (mode == 0)
                    ignore_mask |= newmask;
                else
                    ignore_mask &= ~newmask;

                if (*nptr == '\0')
                    break;
                if (*nptr != ',') {
                    printf("Invalid argument '%s' at '%s'\n", ptr, nptr);
                    return (RC_USER_HELP);
                }
                ptr = nptr;
            }
            ptr++;
        }
    }

    if (ignore_initialized == 0) {
        printf("You must specify a pin range or part type "
               "(dip / plcc / auto) or ? for help\n");
        return (RC_FAILURE);
    }
    if (ignore_mask == 0)
        ignore_mask = ~0x0000013;  // Debug with just 3 bits

    *ignore = ignore_mask;

    return (rc);
}

/*
 * cmd_pld_walk_analyze
 * --------------------
 * This function implements the "analyze" option of the "walk" command.
 */
static rc_t
cmd_pld_walk_analyze(uint32_t *pins_affected_by, uint flags,
                     uint32_t ignore_mask)
{
    int      bit;
    uint     expected_count;
    uint     count        = 0;
    uint     printed      = 0;
    uint     walk_invert  = flags & WALK_FLAG_INVERT_IGNORE;
    uint     walk_zero    = flags & WALK_FLAG_WALK_ZERO;
    uint     not_deep     = !(flags & WALK_FLAG_ANALYZE_DEEP);
    uint32_t cur_mask     = 0;
    uint32_t last_write_mask;
    uint32_t last_read_mask;
    uint32_t rdiff_mask;
    uint32_t write_mask;
    uint32_t main_write_mask;
    uint32_t read_mask;

    cur_mask = 0;
    expected_count = 1 << (32 - bit_count(ignore_mask));
    do {
        if (not_deep && (cur_mask != 0)) {
            cur_mask = 0x0fffffff & ~ignore_mask;
            /* Only two iterations if not deep */
        }
        if (walk_zero)
            main_write_mask = ~cur_mask;
        else
            main_write_mask = cur_mask;
        if (walk_invert)
            main_write_mask |= ignore_mask;

        for (bit = 0; bit < 28; bit++) {
            if (ignore_mask & BIT(bit))
                continue;

            last_write_mask = main_write_mask;
            write_mask      = main_write_mask ^ BIT(bit);

            last_read_mask = pldd_output_pld_input(last_write_mask);
            read_mask      = pldd_output_pld_input(write_mask);

            /* Calculate pins that were affected by this pin */
            rdiff_mask = (read_mask ^ last_read_mask) & ~BIT(bit);
            pins_affected_by[bit] |= rdiff_mask;

            last_read_mask = read_mask;
        }

        if ((count++ & 0x1f) == 0) {
            if (is_abort_button_pressed() || input_break_pending()) {
                printf("^C Abort\n");
                return (RC_USR_ABORT);
            }

            if (!not_deep && (count & 0x7fff) == 1) {
                printf("\r%u%%", count * 100 / expected_count);
                printed = 1;
            }
        }

        cur_mask = ((cur_mask | ignore_mask) + 1) & ~ignore_mask;
    } while (cur_mask != 0);

    if (printed)
        printf("\r100%%\n");

    return (RC_SUCCESS);
}

/*
 * cmd_pld_walk
 * ------------
 * Run through all binary combinations of inputs to the PLD, analyzing
 * or reporting the resulting output. This function is path through
 * which a user will request analysis of the inserted part.
 */
static rc_t
cmd_pld_walk(int argc, char * const *argv)
{
    uint32_t ignore_mask = 0;
    uint32_t cur_mask = 0;
    uint32_t write_mask;
    uint32_t read_mask;
    uint32_t pins_touched          = 0;
    uint32_t pins_output           = 0;
    uint32_t pins_always_low       = 0xffffffff;
    uint32_t pins_always_high      = 0xffffffff;
    uint32_t pins_always_input     = 0xffffffff;
    uint32_t pins_only_output_high = 0xffffffff;
    uint32_t pins_only_output_low  = 0xffffffff;
    uint32_t pins_affected_by[32];
    uint     count = 0;
    uint     expected_count;
    uint     flags = 0;
    uint     printed = 0;
    int      bit;
    rc_t     rc = RC_SUCCESS;
    char     outbuf[90];

    if (argc < 1) {
        printf("%s", cmd_pld_walk_help);
        return (RC_FAILURE);
    }

    memset(pins_affected_by, 0, sizeof (pins_affected_by));

    rc = cmd_pld_get_ignore_mask(argc, argv, &ignore_mask, &flags);
    if (rc != RC_SUCCESS)
        return (rc);

    if ((flags & (WALK_FLAG_ANALYZE | WALK_FLAG_VALUES)) == 0) {
        printf("walk requires one of: analyze, deep, values, raw\n");
        return (RC_FAILURE);
    }

    if (flags & (WALK_FLAG_SHOW_BINARY | WALK_FLAG_ANALYZE)) {
        print_binary(ignore_mask);
        printf(" ignoring\n");
    }

    /*
     * GAL22V10-25 empirical power-on time is ~500usec
     */
    pld_enable();
    timer_delay_msec(2);

    uint walk_zero = flags & WALK_FLAG_WALK_ZERO;
    uint walk_invert = flags & WALK_FLAG_INVERT_IGNORE;
    uint walk_analyze = flags & WALK_FLAG_ANALYZE;
    uint show_binary = flags & WALK_FLAG_SHOW_BINARY;
    uint raw_binary = (flags & WALK_FLAG_RAW_BINARY);
    uint values = (flags & WALK_FLAG_VALUES);

    expected_count = 1 << (32 - bit_count(ignore_mask));
    if (raw_binary) {
        printf("---- BYTES=0x%x ----\n", expected_count * 8);
    } else if (values) {
        printf("---- LINES=0x%x ----\n", expected_count);
    }

    cur_mask = 0;
    do {
        if (walk_zero)
            write_mask = ~cur_mask;
        else
            write_mask = cur_mask;
        if (walk_invert)
            write_mask |= ignore_mask;

        read_mask = pldd_output_pld_input(write_mask);

        if (walk_analyze) {
            pins_touched      |= write_mask;
            pins_always_low   &= ~read_mask;
            pins_always_high  &= read_mask;
            pins_always_input &= ~(read_mask ^ write_mask);
            pins_output       |= (read_mask ^ write_mask);
            pins_only_output_high &= (read_mask | ~write_mask);
            pins_only_output_low &= (~read_mask | write_mask);
        }

        if (raw_binary) {
            ((uint32_t *)outbuf)[0] = write_mask;
            ((uint32_t *)outbuf)[1] = read_mask;
            puts_binary(outbuf, 8);
        } else if (values) {
            int len = 0;
            if (show_binary) {
                len = print_binary_buf(write_mask, outbuf);
                outbuf[len++] = ' ';
                len += print_binary_buf(read_mask, outbuf + len);
                outbuf[len++] = '\n';
                puts_binary(outbuf, len);
            } else {
                len = sprintf(outbuf, "%07lx %07lx\n", write_mask, read_mask);
                puts_binary(outbuf, len);
            }
        }
        if ((count++ & 0x1f) == 0) {
            if (is_abort_button_pressed() || input_break_pending()) {
                printf("^C Abort\n");
                rc = RC_USR_ABORT;
                goto walk_abort;
            }
            if ((raw_binary || !values) && ((count & 0x7fff) == 1)) {
                char buf[16];
                char *ptr;
                sprintf(buf, "\r%u%%", count * 100 / expected_count);
                if (raw_binary) {
                    for (ptr = buf; *ptr != '\0'; ptr++)
                        uart_putchar(*ptr);
                } else {
                    printf("%s", buf);
                }
                printed = 1;
            }
        }
        cur_mask = ((cur_mask | ignore_mask) + 1) & ~ignore_mask;
    } while (cur_mask != 0);

    if (printed) {
        if (raw_binary)
            uart_putchar('\r');
        else
            putchar('\r');
    }

    if (values)
        printf("---- END ----\n");

    if (walk_analyze) {
        int pin;
        printed = 0;

        pins_touched &= ~ignore_mask;
        pins_only_output_low  &= ~(pins_always_low | pins_always_input);
        pins_only_output_high &= ~(pins_always_high | pins_always_input);
        print_binary(pins_always_input & pins_touched);
        printf(" input\n");
        print_binary(pins_output & pins_touched);
        printf(" output\n");
        print_binary(pins_always_low & pins_touched);
        printf(" output always low\n");
        print_binary(pins_always_high);
        printf(" output always high\n");
        print_binary(pins_only_output_low & pins_touched);
        printf(" open drain: only drives low\n");
        print_binary(pins_only_output_high & pins_touched);
        printf(" open drain: only drives high\n");

        /* Run an analysis pass */
        rc = cmd_pld_walk_analyze(pins_affected_by, flags, ignore_mask);
        if (rc != RC_SUCCESS)
            goto walk_abort;

        for (bit = 0; bit < 28; bit++) {
            uint32_t mask = BIT(bit);
            uint32_t pins_affecting = 0;

            for (pin = 0; pin < 28; pin++) {
                if (pins_affected_by[pin] & mask)
                    pins_affecting |= BIT(pin);
            }
            if ((pins_affected_by[bit] != 0) || (pins_affecting != 0)) {
                if (printed == 0) {
                    printed = 1;
                    printf("\n        %-40sPins affected\n", "Pins affecting");
                }
                if (pins_affecting != 0) {
                    print_binary(pins_affecting);
                    printf(" ->");
                } else {
                    printf("%34s", "");
                }
                printf(" Pin%-2u", bit + 1);
                if (pins_affected_by[bit] != 0) {
                    printf(" -> ");
                    print_binary(pins_affected_by[bit]);
                }
                printf("\n");
            }
        }
    }
walk_abort:
    pld_disable();
    return (rc);
}

const char cmd_pld_help[] =
"pld check          - check GPIOs without PLD attached\n"
"pld disable        - disable PLD power\n"
"pld enable         - enable PLD power\n"
"pld output <value> - drive PLDD pins (resistor-protected GPIOs)\n"
"pld show           - show current PLD pin values\n"
"pld voltage        - show sensor readings\n"
"pld walk [?|opt]   - walk GPIO bits (use 'walk ?' for more help)\n";

/*
 * cmd_pld
 * -------
 * Handle the "pld" command.
 */
rc_t
cmd_pld(int argc, char * const *argv)
{
    uint data;
    rc_t rc;
    if (argc <= 1)
        return (RC_USER_HELP);

    switch (*argv[1]) {
        case 'c':  // check
            return (pld_check());
        case 'e':  // enable
            pld_enable();
            break;
        case 'd':  // disable
            pld_disable();
            break;
        case 'o':  // output value
            if (argc <= 2) {
                printf("Value required\n");
                return (RC_USER_HELP);
            }
            if ((rc = parse_uint(argv[2], &data)) != RC_SUCCESS)
                return (rc);
            pldd_output(data);
            pldd_output_enable();
            break;
        case 'i':  // input
        case 's':  // show value
            printf("Output=%07lx Input=%07lx\n",
                   pldd_output_value(), pld_input());
            break;
        case 'v':  // show value
            adc_show_sensors();
            break;
        case 'w':  // show value
            return (cmd_pld_walk(argc - 1, argv + 1));
        default:
            printf("Unknown argument %s\n", argv[1]);
            return (RC_USER_HELP);
    }
    return (RC_SUCCESS);
}
