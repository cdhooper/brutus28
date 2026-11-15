/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * ---------------------------------------------------------------------
 *
 * "pld capture": machine-readable PLD captures for host-side analysis.
 */

#ifndef _CAPTURE_H
#define _CAPTURE_H

#include <stdint.h>
#include "main.h"
#include "cmdline.h"

/*
 * Hardware access used by the capture engine. pld.c implements these on
 * the board; a host test harness may implement them against a simulator.
 */
void     cap_hw_power(int on);            /* PLD power and PLDD drivers */
uint32_t cap_hw_io(uint32_t value, uint settle_usec);  /* drive, wait, read */
void     cap_hw_delay_msec(uint msec);
int      cap_hw_abort(void);              /* ^C or abort button */
int      cap_hw_output(const void *buf, uint len);     /* 0 = sent */
int      cap_hw_jumper_pins(uint32_t *vcc_pins, uint32_t *gnd_pins,
                            int verbose);   /* 0 = detected, no supply */
int      cap_hw_supply(uint *rail_mv);      /* 0 = measured */

/*
 * cap_hw_io() drives the socket pins with one port write per group, in
 * this order (pldd_output in pld.c): socket pins 1-16, 17-24, then 25-28.
 * In between, the PLD sees the new values on the groups written so far
 * and the old values on the rest.  cap_hw_power(1) drives every pin low.
 */
#define CAP_PORT_GROUPS { 0x000ffff, 0x0ff0000, 0xf000000 }

rc_t cmd_pld_capture(int argc, char * const *argv);
extern const char cmd_pld_capture_help[];

#endif /* _CAPTURE_H */
