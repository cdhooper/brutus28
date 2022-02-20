/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2022.
 *
 * ---------------------------------------------------------------------
 *
 * Analog to digital conversion for sensors.
 */

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "board.h"
#include "cmdline.h"
#include "printf.h"
#include "main.h"
#include "gpio.h"
#include "adc.h"
#include "timer.h"

#define TEMP_BASE          25000     // Base temperature is 25C

#if defined(STM32F407xx)
#define TEMP_V25           760       // 0.76V
#define TEMP_AVGSLOPE      25        // 2.5
#define SCALE_VREF         12100000  // 1.21V

#elif defined(STM32F1)
/* Verified STM32F103xE and STM32F107xC are identical */
#define TEMP_V25           1410      // 1.34V-1.52V; 1.41V seems more accurate
#define TEMP_AVGSLOPE      43        // 4.3
#define SCALE_VREF         12000000  // 1.20V

#else
#error STM32 architecture temp sensor slopes must be known
#endif

#define PLD_VCC_DIVIDER_SCALE 2 / 10000 // (1k / 1k)
#define PLD_GND_DIVIDER_SCALE 1 / 10000 // (no divider)

#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#if defined(STM32F407xx)
#define ADC_CHANNEL_TEMP ADC_CHANNEL_TEMP_F40
#endif

static const uint8_t channel_defs[] = {
    ADC_CHANNEL_VREF,  // 0: Vrefint (used to calibrate other readings
    ADC_CHANNEL_TEMP,  // 1: Vtemp Temperature sensor
    8,                 // 2: PB0 - PLD VCC     (1k/1k divider)
    9,                 // 3: PB1 - PLD GND     (no divider)
};

typedef struct {
    uint32_t gpio_port;
    uint16_t gpio_pin;
} channel_gpio_t;
static const channel_gpio_t channel_gpios[] = {
    { PLD_VCC_PORT, PLD_VCC_PIN },  // PB0 - PLD voltage
    { PLD_GND_PORT, PLD_GND_PIN },  // PB1 - PLD GND (should be 0V)
};

#define CHANNEL_COUNT ARRAY_SIZE(channel_defs)

/* Buffer to store the results of the ADC conversion */
volatile uint16_t adc_buffer[CHANNEL_COUNT];

void
adc_enable(void)
{
    int p;
    for (p = 0; p < ARRAY_SIZE(channel_gpios); p++) {
        gpio_setmode(channel_gpios[p].gpio_port, channel_gpios[p].gpio_pin,
                     GPIO_SETMODE_INPUT_ANALOG);
    }
}

void
adc_pulldown(void)
{
    int p;
    for (p = 0; p < ARRAY_SIZE(channel_gpios); p++) {
        gpio_setv(channel_gpios[p].gpio_port, channel_gpios[p].gpio_pin, 0);
        gpio_setmode(channel_gpios[p].gpio_port, channel_gpios[p].gpio_pin,
                     GPIO_SETMODE_OUTPUT_PPULL_2);
    }
}

void
adc_init(void)
{
    uint32_t adcbase = ADC1;

#ifdef STM32F4
    uint32_t dma     = DMA2;
    uint8_t  stream  = 4;  // STM32F4 UM Table: Summary of DMA2 requests...
    uint32_t channel = 0;

    for (p = 0; p < ARRAY_SIZE(channel_gpios); p++) {
        gpio_mode_setup(channel_gpios[p].gpio_port, GPIO_MODE_ANALOG,
                        GPIO_PUPD_NONE, channel_gpios[p].gpio_pin);
    }

    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_DMA2);
    adc_power_off(adcbase); // Turn off ADC during configuration

    dma_disable_stream(dma, stream);
    dma_set_peripheral_address(dma, stream, (uintptr_t)&ADC_DR(adcbase));
    dma_set_memory_address(dma, stream, (uintptr_t)adc_buffer);
    dma_set_transfer_mode(dma, stream, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_set_number_of_data(dma, stream, CHANNEL_COUNT);
    dma_channel_select(dma, stream, channel);
    dma_disable_peripheral_increment_mode(dma, stream);
    dma_enable_memory_increment_mode(dma, stream);
    dma_set_peripheral_size(dma, stream, DMA_SxCR_PSIZE_16BIT);
    dma_set_memory_size(dma, stream, DMA_SxCR_MSIZE_16BIT);
    dma_enable_circular_mode(dma, stream);
    dma_set_priority(dma, stream, DMA_SxCR_PL_MEDIUM);
    dma_enable_direct_mode(dma, stream);
    dma_set_fifo_threshold(dma, stream, DMA_SxFCR_FTH_2_4_FULL);
    dma_set_memory_burst(dma, stream, DMA_SxCR_MBURST_SINGLE);
    dma_set_peripheral_burst(dma, stream, DMA_SxCR_PBURST_SINGLE);

    dma_enable_stream(dma, stream);

    adc_disable_dma(adcbase);

    adc_set_clk_prescale(ADC_CCR_ADCPRE_BY8);
    adc_set_multi_mode(ADC_CCR_MULTI_INDEPENDENT);

    adc_enable_scan_mode(adcbase);
    adc_set_continuous_conversion_mode(adcbase);
    adc_disable_external_trigger_regular(adcbase);
    adc_disable_external_trigger_injected(adcbase);
    adc_set_right_aligned(adcbase);

    adc_set_sample_time_on_all_channels(adcbase, ADC_SMPR_SMP_28CYC);
    adc_set_resolution(adcbase, ADC_CR1_RES_12BIT);

    /* Assign the channels to be monitored by this ADC */
    adc_set_regular_sequence(adcbase, CHANNEL_COUNT, (uint8_t *)channel_defs);

    adc_power_on(adcbase);
    timer_delay_usec(3); // Wait for ADC start up (3 us)

    /* Enable repeated DMA from ADC */
    adc_set_dma_continue(adcbase);
    adc_enable_dma(adcbase);
    adc_enable_temperature_sensor();
    adc_enable_vbat_sensor();
#else
    /* STM32F1... */
    uint32_t dma = DMA1;
    uint32_t channel = 1;  // STM32F1xx RM Table 78 Summary of DMA1 requests...

    adc_enable();

    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_DMA1);
    adc_power_off(adcbase);  // Turn off ADC during configuration
    rcc_periph_reset_pulse(RST_ADC1);
    adc_disable_dma(adcbase);

    dma_disable_channel(dma, channel);
    dma_channel_reset(dma, channel);
    dma_set_peripheral_address(dma, channel, (uintptr_t)&ADC_DR(adcbase));
    dma_set_memory_address(dma, channel, (uintptr_t)adc_buffer);
    dma_set_read_from_peripheral(dma, channel);
    dma_set_number_of_data(dma, channel, CHANNEL_COUNT);
    dma_disable_peripheral_increment_mode(dma, channel);
    dma_enable_memory_increment_mode(dma, channel);
    dma_set_peripheral_size(dma, channel, DMA_CCR_PSIZE_16BIT);
    dma_set_memory_size(dma, channel, DMA_CCR_MSIZE_16BIT);
    dma_enable_circular_mode(dma, channel);
    dma_set_priority(dma, channel, DMA_CCR_PL_MEDIUM);
    dma_enable_channel(dma, channel);

    adc_set_dual_mode(ADC_CR1_DUALMOD_IND);  // Independent ADCs

    adc_enable_scan_mode(adcbase);

    adc_set_continuous_conversion_mode(adcbase);
    adc_set_sample_time_on_all_channels(adcbase, ADC_SMPR_SMP_28DOT5CYC);
    adc_disable_external_trigger_regular(adcbase);
    adc_disable_external_trigger_injected(adcbase);
    adc_set_right_aligned(adcbase);
    adc_enable_external_trigger_regular(adcbase, ADC_CR2_EXTSEL_SWSTART);

    adc_set_regular_sequence(adcbase, CHANNEL_COUNT, (uint8_t *)channel_defs);
    adc_enable_temperature_sensor();

    adc_enable_dma(adcbase);

    adc_power_on(adcbase);
    adc_reset_calibration(adcbase);
    adc_calibrate(adcbase);
#endif

    /* Start the ADC and triggered DMA */
    adc_start_conversion_regular(adcbase);
}

static void
print_reading(int value, char *suffix)
{
    int  units = value / 1000;
    uint milli = abs(value) % 1000;

    if (*suffix == 'C') {
        printf("%3d.%u %s", units, milli / 100, suffix);
    } else {
        printf("%2d.%02u %s", units, milli / 10, suffix);
    }
}

/*
 * adc_get_scale
 * -------------
 * Captures the current scale value from the sensor table.  This value is
 * based on the internal reference voltage and is then used to appropriately
 * scale all other ADC readings.
 */
static uint
adc_get_scale(uint16_t adc0_value)
{
    if (adc0_value == 0)
        adc0_value = 1;

    return (SCALE_VREF / adc0_value);
}

/*
 * adc_get_pld_readings
 * --------------------
 * Returns the current reading for PLD_VCC and PLD_GND sensor inputs.
 * The pld_vcc value is returned, and the pld_gnd parameter is also
 * updated. Values are expressed in millivolts.
 */
uint
adc_get_pld_readings(uint *pld_gnd)
{
    uint     scale;
    uint16_t adc[CHANNEL_COUNT];
    uint32_t calc_pld_vcc;
    uint32_t calc_pld_gnd;

    memcpy(adc, (void *)adc_buffer, sizeof (adc));
    scale = adc_get_scale(adc[0]);

    calc_pld_vcc =  adc[2] * scale * PLD_VCC_DIVIDER_SCALE;
    calc_pld_gnd =  adc[3] * scale * PLD_GND_DIVIDER_SCALE;

    *pld_gnd = calc_pld_gnd;
    return (calc_pld_vcc);
}

void
adc_show_sensors(void)
{
    uint     scale;
    uint16_t adc[CHANNEL_COUNT];

    /*
     * raw / 4095 * 3V = voltage reading * resistor/div scale (8.5) = reading
     *      10K / 1.33K divider: 10V -> 1.174V (multiply reading by 8.51788756)
     *      So, if reading is 1614:
     *              1608 / 4096 * 3 = 1.177734375V
     *              1.177734375 * 8.51788756 = 10.0318
     *
     * PC4 ADC_U1 IN14 is V10SENSE (nominally 10V)
     *     ADC_U1 IN16 is STM32 Temperature (* 10000 / 25 - 279000)
     *     ADC_U1 IN17 is Vrefint 1.2V
     *     ADC_U1 IN18 is Vbat (* 2)
     *
     *     ADC_CHANNEL_TEMPSENSOR
     *     ADC_CHANNEL_VREFINT
     *     ADC_CHANNEL_VBAT
     *
     * We could use Vrefin_cal to get a more accurate expected Vrefint
     * from factory-calibrated values when Vdda was 3.3V.
     *
     * Temperature sensor formula
     *      Temp = (V25 - VSENSE) / Avg_Slope + 25
     *
     *           STM32F407               STM32F1xx
     *      V25  0.76V                   1.43V
     * AvgSlope  2.5                     4.3
     * Calc      * 10000 / 25 - 279000   * 10000 / 43 - 279000
     *
     * Channel order (STM32F4):
     *     adc_buffer[0] = Vrefint
     *     adc_buffer[1] = Vtemperature
     *     adc_buffer[2] = CH14 V10SENSE   (PC4)
     *     adc_buffer[3] =      V3P3SENSE  (Vbat) (STM32F4xx only)
     *     adc_buffer[4] = CH11 V5SENSE    (PC1) - not connected in Rev1
     *     adc_buffer[5] = CH15 V5CLSENSE  (PC5) - not connected in Rev1
     *     adc_buffer[6] = CH2  V10FBSENSE (PA2) - not connected in Rev1
     *
     * Channel order (STM32F1):
     *     adc_buffer[0] = Vrefint
     *     adc_buffer[1] = Vtemperature
     *     adc_buffer[2] =  CH3 V10SENSE   (PA3)
     *     adc_buffer[3] =  CH1 V3P3SENSE  (PA1) - not connected in Rev2
     *     adc_buffer[4] = CH14 V5SENSE    (PC4) - not connected in Rev2
     *     adc_buffer[5] = CH15 V5CLSENSE  (PC5) - not connected in Rev2
     *     adc_buffer[6] =  CH2 V10FBSENSE (PA2) - not connected in Rev2
     *
     * Algorithm:
     *  * Vrefint tells us what 1.21V (STM32F407) or 1.20V (STM32F1xx) should
     *  be according to ADCs.
     *  1. scale = 1.2 / adc_buffer[0]
     *          Because: reading * scale = 1.2V
     *  2. Report Vbat:
     *          adc_buffer[1] * scale * 2
     *
     * TODO:
     *    Function to compute current sensor values
     *       - including current for V5CL once sensors are implemented
     * Compute current flow with diff of V5CL vs V5?
     *
     * Add test to MX code for overcurrent
     *     1) Check V5CL current with VPP / VCC off
     *     2) Check with VCC on
     *     3) Check with VPP on
     *     4) Check with both on
     */
    memcpy(adc, (void *)adc_buffer, sizeof (adc));
    scale = adc_get_scale(adc[0]);

    uint calc_temp;
    calc_temp = ((int)(TEMP_V25 * 10000 - adc[1] * scale)) / TEMP_AVGSLOPE +
                TEMP_BASE;
    uint32_t calc_pld_vcc =  adc[2] * scale * PLD_VCC_DIVIDER_SCALE;
    uint32_t calc_pld_gnd =  adc[3] * scale * PLD_GND_DIVIDER_SCALE;

    printf("Vrefint=%04x scale=%d\n", adc[0], scale);
    printf("  Vtemp=%04x %8u ", adc[1], adc[1] * scale);
    print_reading(calc_temp, "C\n");
    printf("PLD_VCC=%04x %8u ", adc[2], adc[2] * scale);
    print_reading(calc_pld_vcc, "V\n");
    printf("PLD_GND=%04x %8u ", adc[3], adc[3] * scale);
    print_reading(calc_pld_gnd, "V\n");
}

/*
 * adc_poll() will capture the current readings from the sensors and take
 *            action to maintain the 10V rail as close as possible to 10V.
 */
void
adc_poll(int verbose, int force)
{
    uint16_t        adc[CHANNEL_COUNT];
    static uint64_t next_check = 0;

    if ((timer_tick_has_elapsed(next_check) == false) && (force == false))
        return;
    next_check = timer_tick_plus_msec(1);  // Limit rate to prevent overshoot

    memcpy(adc, (void *)adc_buffer, sizeof (adc));
//  scale = adc_get_scale(adc[0]);
}
