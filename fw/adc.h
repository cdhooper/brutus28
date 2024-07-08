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

#ifndef _ADC_H
#define _ADC_H

void adc_init(void);
void adc_shutdown(void);
void adc_show_sensors(void);
void adc_poll(int verbose, int force);
void dac_setvalue(uint32_t value);
uint adc_get_pld_readings(uint *pld_gnd);
void adc_enable(void);
void adc_pulldown(void);

#endif /* _ADC_H */
