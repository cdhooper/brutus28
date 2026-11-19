/*
 * A GAL16V8 / GAL20V8 / GAL22V10 in the Brutus-28 socket, simulated from its fuse
 * map: galsim.py's GAL and fwsim.py's SocketDevice in C, for speed.  The
 * Python classes are the reference; tests compare the two read for read.
 *
 * galsim.py decodes the JED and hands over what the simulation needs:
 * per array column the pin it reads (or the register whose feedback it
 * is), the product term rows, and per output macrocell its pin, mode,
 * polarity, enable and the rows that can be true.  Levels and register
 * states are bit masks over the package pin numbers (bit p = pin p).
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "galmodel.h"

#define GAL_MAX_COLS   22
#define GAL_MAX_ROWS   132
#define GAL_MAX_OLMC   10
#define GAL_MAX_LIVE   16

enum { OLMC_INPUT = 0, OLMC_COMB = 1, OLMC_REG = 2 };
enum { OE_ALWAYS = 0, OE_NEVER = 1, OE_ROW = 2, OE_PIN = 3 };  /* OE_PIN: oe_row is the pin */

typedef struct {
    uint8_t  pin;
    uint8_t  mode;
    uint8_t  active_high;
    uint8_t  oe_kind;
    int      oe_row;
    int      nlive;
    uint64_t live[GAL_MAX_LIVE];
} olmc_t;

struct gal {
    int      is16v8;                    /* GAL16V8 or GAL20V8: D = sum ^ XOR, pin = !Q */
    int      clock_pin;                 /* 0: none */
    int      ar_row;                    /* -1: none */
    int      sp_row;
    int      ncols;
    uint8_t  col_pin[GAL_MAX_COLS];
    uint8_t  col_reg[GAL_MAX_COLS];     /* register feedback, not the pin */
    int      nrows;
    uint64_t row[GAL_MAX_ROWS];
    uint8_t  row_ok[GAL_MAX_ROWS];      /* the row exists */
    int      nolmc;
    olmc_t   olmc[GAL_MAX_OLMC];
    uint32_t pins;                      /* the signal pins */
    uint32_t regs;                      /* pins of registered macrocells */
    int      npin;                      /* number of signal pins */
    uint8_t  pin_list[32];              /* ascending */
    uint32_t sbit[32];                  /* socket bit of each pin */
    uint32_t vcc_bit;
    int      transients;
    /* state */
    uint32_t q;                         /* registers, by pin */
    uint32_t levels;
    int      have_levels;
    uint32_t driven;
    uint32_t unstable;
    int      powered;
    uint32_t last;                      /* the socket value last applied */
};

static uint64_t
gal_colvals(const gal_t *g, uint32_t levels)
{
    uint64_t v = 0;
    int      k;

    for (k = 0; k < g->ncols; k++) {
        int bit;

        if (g->col_reg[k])
            bit = !((g->q >> g->col_pin[k]) & 1);
        else
            bit = (levels >> g->col_pin[k]) & 1;
        v |= 1ULL << (2 * k + (bit ? 0 : 1));
    }
    return (v);
}

static int
gal_term(const gal_t *g, int row, uint64_t cv)
{
    return (g->row_ok[row] && ((g->row[row] & ~cv) == 0));
}

static int
gal_sum(const olmc_t *o, uint64_t cv)
{
    int i;

    for (i = 0; i < o->nlive; i++)
        if ((o->live[i] & ~cv) == 0)
            return (1);
    return (0);
}

/* The pins the chip drives and their values, for the given pin levels */
static void
gal_outputs(const gal_t *g, uint32_t levels, uint32_t *mask, uint32_t *val)
{
    uint64_t cv = gal_colvals(g, levels);
    uint32_t m = 0;
    uint32_t v = 0;
    int      i;

    for (i = 0; i < g->nolmc; i++) {
        const olmc_t *o = &g->olmc[i];
        int           bit;

        if (o->mode == OLMC_INPUT)
            continue;
        if (o->oe_kind == OE_PIN) {
            if (((levels >> o->oe_row) & 1) == 0) {
                m |= 1U << o->pin;
                if (!((g->q >> o->pin) & 1))
                    v |= 1U << o->pin;
            }
            continue;
        }
        if (o->oe_kind == OE_NEVER)
            continue;
        if ((o->oe_kind == OE_ROW) && !gal_term(g, o->oe_row, cv))
            continue;
        if (o->mode == OLMC_REG)
            bit = (g->q >> o->pin) & 1;
        else
            bit = gal_sum(o, cv);
        if (!o->active_high)
            bit = !bit;
        m |= 1U << o->pin;
        if (bit)
            v |= 1U << o->pin;
    }
    *mask = m;
    *val = v;
}

/* Apply the external drive ext and settle (GAL.settle) */
static void
gal_settle(gal_t *g, uint32_t ext)
{
    uint32_t levels = g->have_levels ? g->levels : ext;
    uint32_t dmask = 0;
    uint32_t dval = 0;
    uint32_t seen = 0;
    int      it;

    levels = ((levels & g->driven) | (ext & ~g->driven)) & g->pins;
    g->unstable = 0;
    for (it = 0; it < 200; it++) {
        uint32_t next;

        if ((g->ar_row >= 0) && gal_term(g, g->ar_row, gal_colvals(g, levels)))
            g->q &= ~g->regs;
        gal_outputs(g, levels, &dmask, &dval);
        next = ((dval & dmask) | (ext & ~dmask)) & g->pins;
        if (next == levels)
            break;
        if (it < 50) {
            levels = next;
        } else {
            /* no convergence with simultaneous updates: one pin at a time */
            uint32_t ch = next ^ levels;
            int      n = __builtin_popcount(ch);
            int      k = it % n;
            int      i;

            seen |= ch;
            for (i = 0; i < g->npin; i++) {
                int p = g->pin_list[i];

                if ((ch >> p) & 1) {
                    if (k-- == 0) {
                        levels ^= 1U << p;
                        break;
                    }
                }
            }
        }
    }
    if (it == 200)
        g->unstable = seen;
    g->levels = levels;
    g->have_levels = 1;
    g->driven = dmask;
}

/* Like gal_settle, but a rising clock pin clocks the registers with the
 * levels before the edge (GAL.apply) */
static void
gal_apply(gal_t *g, uint32_t ext)
{
    int cp = g->clock_pin;

    if ((cp != 0) && g->have_levels && !((g->levels >> cp) & 1) &&
        ((ext >> cp) & 1)) {
        uint64_t cv = gal_colvals(g, g->levels);
        int      sp = (g->sp_row >= 0) && gal_term(g, g->sp_row, cv);
        uint32_t nq = g->q;
        int      i;

        for (i = 0; i < g->nolmc; i++) {
            const olmc_t *o = &g->olmc[i];
            int           s;

            if (o->mode != OLMC_REG)
                continue;
            s = gal_sum(o, cv);
            if (g->is16v8)
                s ^= o->active_high ? 1 : 0;
            else if (sp)
                s = 1;
            if (s)
                nq |= 1U << o->pin;
            else
                nq &= ~(1U << o->pin);
        }
        g->q = nq;
    }
    gal_settle(g, ext);
}

static uint32_t
gal_apply_socket(gal_t *g, uint32_t value)
{
    uint32_t ext = 0;
    uint32_t out = g->vcc_bit;
    int      i;

    g->last = value;
    for (i = 0; i < g->npin; i++) {
        int p = g->pin_list[i];

        if (value & g->sbit[p])
            ext |= 1U << p;
    }
    gal_apply(g, ext);
    for (i = 0; i < g->npin; i++) {
        int p = g->pin_list[i];

        if ((g->levels >> p) & 1)
            out |= g->sbit[p];
    }
    return (out);
}

gal_t *
gal_new(int is16v8, int clock_pin, int ar_row, int sp_row, int ncols,
        const uint8_t *col_pin, const uint8_t *col_reg, int nrows,
        const uint64_t *row, const uint8_t *row_ok, uint32_t pins,
        const uint32_t *sbit, uint32_t vcc_bit, int transients)
{
    gal_t *g = calloc(1, sizeof (*g));
    int    p;

    if ((g == NULL) || (ncols > GAL_MAX_COLS) || (nrows > GAL_MAX_ROWS)) {
        free(g);
        return (NULL);
    }
    g->is16v8 = is16v8;
    g->clock_pin = clock_pin;
    g->ar_row = ar_row;
    g->sp_row = sp_row;
    g->ncols = ncols;
    memcpy(g->col_pin, col_pin, ncols);
    memcpy(g->col_reg, col_reg, ncols);
    g->nrows = nrows;
    memcpy(g->row, row, nrows * sizeof (*row));
    memcpy(g->row_ok, row_ok, nrows);
    g->pins = pins;
    for (p = 0; p < 32; p++) {
        if ((pins >> p) & 1)
            g->pin_list[g->npin++] = p;
        g->sbit[p] = sbit[p];
    }
    g->vcc_bit = vcc_bit;
    g->transients = transients;
    return (g);
}

int
gal_add_olmc(gal_t *g, int pin, int mode, int active_high, int oe_kind,
             int oe_row, int nlive, const uint64_t *live)
{
    olmc_t *o;

    if ((g->nolmc >= GAL_MAX_OLMC) || (nlive > GAL_MAX_LIVE))
        return (-1);
    o = &g->olmc[g->nolmc++];
    o->pin = pin;
    o->mode = mode;
    o->active_high = active_high;
    o->oe_kind = oe_kind;
    o->oe_row = oe_row;
    o->nlive = nlive;
    memcpy(o->live, live, nlive * sizeof (*live));
    if (mode == OLMC_REG)
        g->regs |= 1U << pin;
    return (0);
}

void
gal_free(gal_t *g)
{
    free(g);
}

/* SocketDevice.power */
void
gal_power(gal_t *g, int on)
{
    g->powered = (on != 0);
    if (on) {
        g->q = 0;
        g->levels = 0;
        g->have_levels = 0;
        g->driven = 0;
        g->unstable = 0;
        gal_settle(g, 0);                /* the drivers come up low */
        g->last = 0;
    }
}

/* SocketDevice.io: one write of the 28 socket pins (three port writes,
 * CAP_PORT_GROUPS), then the read */
uint32_t
gal_io(gal_t *g, uint32_t value)
{
    static const uint32_t groups[] = { 0x000ffff, 0x0ff0000 };
    uint32_t done = 0;
    unsigned i;

    if (!g->powered)
        return (0);
    if (g->transients) {
        for (i = 0; i < sizeof (groups) / sizeof (groups[0]); i++) {
            uint32_t mid;

            done |= groups[i];
            mid = (value & done) | (g->last & ~done);
            if ((mid != g->last) && (mid != value))
                (void) gal_apply_socket(g, mid);
        }
    }
    return (gal_apply_socket(g, value));
}
