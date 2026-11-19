/*
 * A GAL16V8 / GAL22V10 in the Brutus-28 socket, simulated from its fuse
 * map (galmodel.c).  fwsim.py fills it in from galsim.py's decoding.
 */
#ifndef _GALMODEL_H
#define _GALMODEL_H

#include <stdint.h>

typedef struct gal gal_t;

gal_t   *gal_new(int is16v8, int clock_pin, int ar_row, int sp_row, int ncols,
                 const uint8_t *col_pin, const uint8_t *col_reg, int nrows,
                 const uint64_t *row, const uint8_t *row_ok, uint32_t pins,
                 const uint32_t *sbit, uint32_t vcc_bit, int transients);
int      gal_add_olmc(gal_t *g, int pin, int mode, int active_high,
                      int oe_kind, int oe_row, int nlive, const uint64_t *live);
void     gal_free(gal_t *g);
void     gal_power(gal_t *g, int on);
uint32_t gal_io(gal_t *g, uint32_t value);

#endif /* _GALMODEL_H */
