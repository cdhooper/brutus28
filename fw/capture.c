/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * ---------------------------------------------------------------------
 *
 * Machine-readable PLD captures ("pld capture") for host-side reverse
 * engineering of registered PLDs such as the GAL16V8 and GAL22V10.
 *
 * The host sets up a job with "pld capture set / path / probe / ret" and
 * runs it with "pld capture walk", "trans" or "state".  Pin masks and
 * vectors are 28-bit socket values in hex, bit n-1 = socket pin n.  Each
 * run powers the PLD up, clocks it along the job's path from the power-up
 * state, does its work, and powers the PLD off again (also on abort).
 *
 * A step with vector v drives v with the clock pins low, then v with the
 * clock pins high: one rising clock edge.  With follow set, the step reads
 * the pins after the first write and drives every read pin at the level
 * it read before the clock edge: where the PLD drives a pin, Brutus then
 * drives the same level instead of fighting it through the 1K, so a pin
 * the clock turns off keeps its level while the edge is still settling.
 * With dclk set, the clock pins are driven directly by the CPU's PLD_*
 * pins (no 1K), for a fast edge; this needs pins the PLD never drives,
 * such as pin 1 of a GAL.
 *
 * A signature is, for each probe vector p (clock pins low), one read of
 * p, then one read per candidate pin c with only c's drive inverted.  A
 * pin the PLD drives reads the same both times; a pin it does not drive
 * follows the 1K drive.  Every read is masked with the job's read mask.
 *
 * walk   For every assignment x of the enumerated pins (the other pins at
 *        base, clock pins low): drive and read; with inv set, read again
 *        with the inv pins inverted.  The i-th vector is x = (i * mul)
 *        mod 2^n: counting order for mul 1, scattered for other (odd)
 *        mul.  Bit b of x drives the b-th enumerated pin, counted from
 *        the lowest socket pin, or with rev set from the highest, so the
 *        pin that changes fastest becomes the one that changes slowest.
 *        With gray set, x runs in Gray code order instead (one pin changes
 *        per vector).  The usual Gray code changes its lowest bit at every
 *        second vector and its highest once.  With mix set (not 0, and
 *        gray), the order is another Gray code that mix decides: the
 *        cube of all vectors is walked as one half, one pin's change, the
 *        other half, and each half again so, down to single pins, with
 *        the pin that splits a part chosen by a hash of mix and the
 *        part's place (cap_mix_next).  The pin that splits the whole walk
 *        still changes once, but which pins change often differs from
 *        part to part, so most pins change about equally often, and
 *        another mix makes another pin the rare one.  With rot set, x is rotated left by rot bits first,
 *        so bit 0 of the order drives the rot-th enumerated pin.  Every
 *        vector is XORed with start, so a Gray walk from the path's last
 *        vector changes one pin at a time from where the path left the
 *        chip.  Vectors v with (v & skm) == skv (skm not 0) are skipped:
 *        nothing is written and their reads are 0.
 *        With quiet set, only the signatures are reported.
 *        The enumerated pins may include the clock pins: after the path,
 *        which clocks as usual, the walk drives them like any other
 *        enumerated pin (start says at which level it begins), so the
 *        chip is clocked whenever the walk raises them.  The host uses
 *        this for logic that reads the clock pin, and for latches in a
 *        registered design.
 *        A result that depends on the order means the chip has state
 *        that changes without a clock (latches, the 22V10's AR).  With
 *        probes set,
 *        the signature is reported before and after the walk, so the host
 *        can tell whether the walk disturbed the register state.
 * trans  Read the signature of the path state S.  Then for each x: step
 *        with x, read the signature, and get back to S.  Nothing to do if
 *        the signature is S's; otherwise clock along the return path the
 *        host gave for that signature (and check S's signature), else
 *        power-cycle and replay the path.  Two bytes per x (low byte
 *        first): the index of its signature.  The first occurrence of a signature is followed
 *        by its reads.  x runs over (i * mul) mod 2^n for i < count.
 *        With chk set, x is also read right after the clock edge (clock
 *        pins low) and again after the signature; if the two reads
 *        differ, reading the signature changed the state, and x goes into
 *        an 'F' chunk, followed by the pins that read differently (the
 *        first CAP_MAX_FLAGS of them).  The read mask decides which pins
 *        count: the host leaves out the pins it knows to change without a
 *        clock (latches).
 *        With pre set, each x's byte (and its signature's reads, if they
 *        follow) is followed by the read of every socket pin with x
 *        driven, just before the clock edge, whatever the read mask: the
 *        levels that the registers load from, for a register input that
 *        reads a latch.
 * state  Report the signature of the path state.
 * psig:  Where a clock edge leads may depend on a latch, and the latch on
 *        the writes since the edge before: the signature reads move the
 *        inputs.  With psig set, power-up and every clocked step (of the
 *        path, of a return path, of trans) are followed by the same
 *        writes: the step's vector with the clock low, the signature,
 *        the vector again, the signature again.  So a step leaves the
 *        latches the same way wherever it is taken, and a path leads to
 *        the state that the sweep saw.  The second signature is the
 *        state's: trans, walk and state report it without reading again.
 *        With chk, trans flags the x whose two reads of the vector
 *        differ; chkm (if not 0) limits that to its pins.
 * Avoid: with avm set, every write takes care never to pass through a
 *        vector v with (v & avm) == avv on the way, although the socket
 *        pins are written in three port writes (CAP_PORT_GROUPS): the
 *        pins that leave the avm/avv cube are written first, then the
 *        others, then those that enter it, each port group on its own.
 *        A target vector in the cube is still reached.  This keeps the
 *        writes from firing an asynchronous reset term through a
 *        momentary mix of old and new values.
 *
 * Preflight: when the job names the socket pins that must carry VCC and
 * GND (set vcc= gnd=), and optionally the PLD supply (set supply=), every
 * run first checks the jumpers and refuses to power the PLD if they do not
 * match.  The VCC and GND pins are found without connecting the supply
 * (cap_hw_jumper_pins); the supply is switched on to measure it only once
 * those pins match, so a wrongly jumpered PLD never sees it.  The job may
 * not enumerate, invert, clock or probe the VCC and GND pins, and every
 * write drives the VCC pin high and the GND pin low.
 *
 * Output: the line "---- CAPTURE ----", then binary chunks
 * <type:1> <length:2 LE> <payload>: 'H' header, 'P' signature reads,
 * 'D' data, 'F' flagged vectors, each with the mask of the pins that
 * changed (trans with chk), 'E' end.  Reads and
 * vectors are 32-bit little endian.  The 'E' payload is status, 3 pad
 * bytes, then 32-bit counts of vectors done, power-cycle restores,
 * return-path restores, failed return paths, and the CRC-32 (as zlib) of
 * every chunk byte before the 'E' chunk.
 */

#include "main.h"
#include "printf.h"
#include "cmdline.h"
#include <string.h>
#include "capture.h"

#define CAP_VERSION      2
#define CAP_MAX_PATH     512
#define CAP_MAX_PROBES   8
#define CAP_MAX_CAND     12
#define CAP_MAX_READS    (CAP_MAX_PROBES * (1 + CAP_MAX_CAND))
#define CAP_MAX_RET      32
#define CAP_MAX_RET_LEN  16
#define CAP_MAX_SIGS     2048
#define CAP_MAX_WALK     24      /* enumerated pins for a walk */
#define CAP_MAX_FLAGS    16
#define CAP_SOCKET_PINS  0x0fffffff
#define CAP_CHUNK        512
#define CAP_NELEM(x)     (sizeof (x) / sizeof ((x)[0]))

#define CAP_STATUS_OK         0
#define CAP_STATUS_ABORT      1
#define CAP_STATUS_TOO_MANY   2  /* more than CAP_MAX_SIGS signatures */
#define CAP_STATUS_LOST       3  /* could not get back to the path state */

typedef struct {
    uint32_t crc;
    uint     len;
    uint32_t vec[CAP_MAX_RET_LEN];
} cap_ret_t;

static struct {
    uint32_t  pins;         /* enumerated pins */
    uint32_t  base;         /* drive of the other pins */
    uint32_t  inv;          /* walk: pins inverted for the second read */
    uint32_t  clk;          /* clock pins */
    uint32_t  cand;         /* signature: pins read with inverted drive */
    uint32_t  mask;         /* read mask */
    uint32_t  count;        /* trans: vectors (0 = all) */
    uint32_t  mul;          /* trans, walk: x = i * mul (odd) */
    uint32_t  rev;          /* walk: bit 0 of x is the highest pin */
    uint32_t  gray;         /* walk: Gray code order */
    uint32_t  mix;          /* walk: which Gray code (0: the usual one) */
    uint32_t  rot;          /* walk: x rotated left by this many bits */
    uint32_t  start;        /* walk: vectors XORed with this */
    uint32_t  skm;          /* walk: skip vectors with (v & skm) == skv */
    uint32_t  skv;
    uint32_t  avm;          /* writes: never pass through (v & avm) == avv */
    uint32_t  avv;
    uint32_t  quiet;        /* walk: report the signatures only */
    uint32_t  chk;          /* trans: flag x whose state the signature changed */
    uint32_t  chkm;         /* trans: the pins that count for chk (0: all read) */
    uint32_t  psig;         /* steps: followed by the signature reads */
    uint32_t  pre;          /* trans: report the read before the clock edge */
    uint32_t  follow;       /* steps: drive the read levels before the edge */
    uint32_t  dclk;         /* clock pins driven directly, not through 1K */
    uint32_t  vcc;          /* preflight: socket pins jumpered to VCC */
    uint32_t  gnd;          /* preflight: socket pins jumpered to GND */
    uint      supply_mv;    /* preflight: PLD supply (mV), 0 = any */
    uint      settle_usec;  /* after each write */
    uint      off_msec;     /* power off time of a power cycle */
    uint      on_msec;      /* power on time before the first vector */
    uint      npath;
    uint      nprobe;
    uint      nret;
    uint32_t  path[CAP_MAX_PATH];
    uint32_t  probe[CAP_MAX_PROBES];
    cap_ret_t ret[CAP_MAX_RET];
} cap;

static uint8_t  cap_buf[3 + CAP_CHUNK];
static uint     cap_buflen;
static uint     cap_buftype;
static uint32_t cap_crc;
static int      cap_out_err;
static uint32_t cap_done;
static uint32_t cap_pcycles;
static uint32_t cap_ret_ok;
static uint32_t cap_ret_fail;
static uint32_t cap_ref[CAP_MAX_READS];
static uint32_t cap_cur[CAP_MAX_READS];
static uint32_t cap_sigs[CAP_MAX_SIGS];
static uint32_t cap_last;            /* the vector last written */
static uint32_t cap_raw;             /* the last read, every socket pin */
static uint32_t cap_pre;             /* the read before the last clock edge */

const char cmd_pld_capture_help[] =
"pld capture clear              - reset the job to defaults\n"
"pld capture set <key>=<value>  - pins base inv clk cand mask count mul rev\n"
"                                 gray mix rot start skm skv avm avv quiet\n"
"                                 chk chkm psig\n"
"                                 pre follow dclk vcc gnd (hex)\n"
"                                 settle (usec) off on (msec) supply (mV)\n"
"pld capture path [+] <v>...    - clocked vectors from power-up to the state\n"
"pld capture probe [+] <v>...   - probe vectors for signatures\n"
"pld capture ret <crc> <v>...   - return path from the state with that\n"
"                                 signature CRC; 'ret clear' drops them all\n"
"pld capture show               - show the job\n"
"pld capture jumpers            - detect the VCC and GND jumpers; if they\n"
"                                 match vcc= gnd=, also measure the supply\n"
"pld capture walk               - every vector of the enumerated pins\n"
"pld capture trans              - clock transitions from the path state\n"
"pld capture state              - signature of the path state\n";

static const uint32_t cap_crc_tab[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c,
};

static uint32_t
cap_crc32(uint32_t crc, const uint8_t *buf, uint len)
{
    crc = ~crc;
    while (len-- > 0) {
        crc ^= *(buf++);
        crc = (crc >> 4) ^ cap_crc_tab[crc & 0xf];
        crc = (crc >> 4) ^ cap_crc_tab[crc & 0xf];
    }
    return (~crc);
}

static uint32_t
cap_crc32_words(const uint32_t *w, uint n)
{
    uint32_t crc = 0;
    uint8_t  b[4];

    while (n-- > 0) {
        b[0] = *w;
        b[1] = *w >> 8;
        b[2] = *w >> 16;
        b[3] = *w >> 24;
        crc = cap_crc32(crc, b, 4);
        w++;
    }
    return (crc);
}

static uint
cap_popcount(uint32_t v)
{
    uint n = 0;
    for (; v != 0; v &= v - 1)
        n++;
    return (n);
}

static void
cap_defaults(void)
{
    memset(&cap, 0, sizeof (cap));
    cap.mask = CAP_SOCKET_PINS;
    cap.mul = 1;
    cap.settle_usec = 1;
    cap.off_msec = 20;
    cap.on_msec = 2;
}

/* -- output ------------------------------------------------------------ */

static void
cap_flush(void)
{
    if (cap_buflen == 0)
        return;
    cap_buf[0] = cap_buftype;
    cap_buf[1] = cap_buflen & 0xff;
    cap_buf[2] = cap_buflen >> 8;
    cap_crc = cap_crc32(cap_crc, cap_buf, 3 + cap_buflen);
    if ((cap_out_err == 0) && cap_hw_output(cap_buf, 3 + cap_buflen))
        cap_out_err = 1;
    cap_buflen = 0;
}

static void
cap_put(uint type, const void *data, uint len)
{
    const uint8_t *ptr = data;

    if (type != cap_buftype) {
        cap_flush();
        cap_buftype = type;
    }
    while (len > 0) {
        uint n = CAP_CHUNK - cap_buflen;
        if (n > len)
            n = len;
        memcpy(cap_buf + 3 + cap_buflen, ptr, n);
        cap_buflen += n;
        ptr += n;
        len -= n;
        if (cap_buflen == CAP_CHUNK)
            cap_flush();
    }
}

static void
cap_put32(uint type, uint32_t v)
{
    uint8_t b[4];
    b[0] = v;
    b[1] = v >> 8;
    b[2] = v >> 16;
    b[3] = v >> 24;
    cap_put(type, b, 4);
}

static void
cap_put_reads(uint type, const uint32_t *r, uint n)
{
    while (n-- > 0)
        cap_put32(type, *(r++));
}

static void
cap_begin(uint kind, uint rec_reads, uint sig_reads, uint nbits,
          uint32_t count)
{
    uint8_t h[4];

    printf("---- CAPTURE ----\n");
    cap_crc = 0;
    cap_out_err = 0;
    cap_buflen = 0;
    cap_buftype = 'H';
    cap_done = 0;
    cap_pcycles = 0;
    cap_ret_ok = 0;
    cap_ret_fail = 0;
    h[0] = CAP_VERSION;
    h[1] = kind;
    h[2] = rec_reads;
    h[3] = sig_reads;
    cap_put('H', h, 4);
    cap_put32('H', nbits);
    cap_put32('H', count);
    cap_put32('H', cap.pins);
    cap_put32('H', cap.base);
    cap_put32('H', cap.inv);
    cap_put32('H', cap.clk);
    cap_put32('H', cap.cand);
    cap_put32('H', cap.mask);
    cap_put32('H', cap.mul);
    cap_flush();
}

static rc_t
cap_end(uint status)
{
    uint8_t e[3 + 24];
    uint32_t v[5];
    uint i;

    cap_flush();
    cap_hw_power(0);
    cap_hw_direct(0);
    v[0] = cap_done;
    v[1] = cap_pcycles;
    v[2] = cap_ret_ok;
    v[3] = cap_ret_fail;
    v[4] = cap_crc;
    e[0] = 'E';
    e[1] = 24;
    e[2] = 0;
    e[3] = status;
    e[4] = e[5] = e[6] = 0;
    for (i = 0; i < 5; i++) {
        e[7 + 4 * i] = v[i];
        e[8 + 4 * i] = v[i] >> 8;
        e[9 + 4 * i] = v[i] >> 16;
        e[10 + 4 * i] = v[i] >> 24;
    }
    if (cap_out_err == 0)
        cap_hw_output(e, sizeof (e));
    if (status == CAP_STATUS_ABORT)
        return (RC_USR_ABORT);
    return ((status == CAP_STATUS_OK) ? RC_SUCCESS : RC_FAILURE);
}

/* -- PLD access ---------------------------------------------------------- */

/*
 * Move from the last vector written towards v without passing through the
 * avm/avv cube: pins leaving the cube first, then the others, then pins
 * entering it, one port group at a time.  Writes every step but v itself.
 */
static void
cap_avoid(uint32_t v)
{
    static const uint32_t groups[] = CAP_PORT_GROUPS;
    uint32_t ch = cap_last ^ v;
    uint32_t cls[3];
    uint32_t cur = cap_last;
    uint     c;
    uint     g;

    cls[0] = ch & cap.avm & (v ^ cap.avv);      /* leave the cube */
    cls[1] = ch & ~cap.avm;
    cls[2] = ch & cap.avm & ~(v ^ cap.avv);     /* enter the cube */
    for (c = 0; c < 3; c++) {
        for (g = 0; g < CAP_NELEM(groups); g++) {
            uint32_t m = cls[c] & groups[g];

            if (m == 0)
                continue;
            cur ^= m;
            if (cur != v)
                (void) cap_hw_io(cur, cap.settle_usec);
        }
    }
}

/* Drive v (supply pins at their rail level), wait, read the masked pins */
static uint32_t
cap_io(uint32_t v)
{
    v = (v & ~cap.gnd) | cap.vcc;
    if (cap.avm != 0)
        cap_avoid(v);
    cap_last = v;
    cap_raw = cap_hw_io(v, cap.settle_usec) & CAP_SOCKET_PINS;
    return (cap_raw & cap.mask);
}

static void
cap_step(uint32_t v)
{
    uint32_t r = cap_io(v & ~cap.clk);

    if (cap.follow) {
        /* r is masked: the read pins take the level they read */
        uint32_t f = ((v & ~cap.mask) | r) & ~cap.clk;

        if (f != (v & ~cap.clk))
            (void) cap_io(f);
        v = f;
    }
    cap_pre = cap_raw;
    (void) cap_io(v | cap.clk);
}

static uint
cap_signature(uint32_t *r)
{
    uint n = 0;
    uint i;

    for (i = 0; i < cap.nprobe; i++) {
        uint32_t w = cap.probe[i] & ~cap.clk;
        uint32_t c;

        r[n++] = cap_io(w);
        for (c = cap.cand; c != 0; c &= c - 1)
            r[n++] = cap_io(w ^ (c & -c));
    }
    return (n);
}

/*
 * What follows power-up and every clocked step with psig: v with the
 * clock low, the signature, v again, the signature again (in cap_cur).
 * Returns the pins that read differently at v before and after the first
 * signature.
 */
static uint32_t
cap_after_step(uint32_t v)
{
    uint32_t r1 = cap_io(v & ~cap.clk);
    uint32_t r2;

    (void) cap_signature(cap_cur);
    r2 = cap_io(v & ~cap.clk);
    (void) cap_signature(cap_cur);
    return (r1 ^ r2);
}

static void
cap_power_up(uint off_msec)
{
    uint i;

    cap_hw_power(0);
    cap_hw_delay_msec(off_msec);
    cap_hw_direct(cap.dclk ? cap.clk : 0);
    cap_hw_power(1);                     /* drives every pin low */
    cap_last = 0;
    cap_hw_delay_msec(cap.on_msec);
    if (cap.psig)
        (void) cap_after_step(cap.base & ~cap.pins);
    for (i = 0; i < cap.npath; i++) {
        cap_step(cap.path[i]);
        if (cap.psig)
            (void) cap_after_step(cap.path[i]);
    }
}

static uint32_t
cap_expand(uint32_t x)
{
    uint32_t w = 0;
    uint32_t p;

    for (p = cap.pins; p != 0; p &= p - 1, x >>= 1)
        if (x & 1)
            w |= p & -p;
    return (w);
}

/* x spread over the enumerated pins from the highest down */
static uint32_t
cap_expand_rev(uint32_t x)
{
    uint32_t w = 0;
    uint32_t p;

    for (p = cap.pins; p != 0; x >>= 1) {
        uint32_t top = 1UL << (31 - __builtin_clz(p));

        if (x & 1)
            w |= top;
        p &= ~top;
    }
    return (w);
}

/* Get back to the path state after a transition; 0 on success */
static int
cap_restore(uint32_t crc, uint nreads)
{
    uint k;
    uint i;

    for (k = 0; k < cap.nret; k++) {
        if (cap.ret[k].crc != crc)
            continue;
        for (i = 0; i < cap.ret[k].len; i++) {
            cap_step(cap.ret[k].vec[i]);
            if (cap.psig)
                (void) cap_after_step(cap.ret[k].vec[i]);
        }
        if (!cap.psig)
            cap_signature(cap_cur);
        if (memcmp(cap_cur, cap_ref, nreads * 4) == 0) {
            cap_ret_ok++;
            return (0);
        }
        cap_ret_fail++;
        break;
    }
    for (i = 0; i < 2; i++) {
        cap_power_up((i == 0) ? cap.off_msec : cap.off_msec * 10);
        if (!cap.psig)
            cap_signature(cap_cur);
        if (memcmp(cap_cur, cap_ref, nreads * 4) == 0) {
            cap_pcycles++;
            return (0);
        }
    }
    return (-1);
}

/* -- preflight ----------------------------------------------------------- */

/*
 * The measured supply may differ from the job's by this much (percent):
 * 4.2 to 5.8 V for 5 V, the window "pld check" uses.
 */
#define CAP_SUPPLY_TOLERANCE 16

static int
cap_supply_ok(uint rail_mv)
{
    return ((rail_mv * 100 >= cap.supply_mv * (100 - CAP_SUPPLY_TOLERANCE)) &&
            (rail_mv * 100 <= cap.supply_mv * (100 + CAP_SUPPLY_TOLERANCE)));
}

/* Whether the detected pins are the job's, so the supply may be switched on */
static int
cap_pins_match(uint32_t vcc, uint32_t gnd)
{
    return ((cap.vcc != 0) && (cap.gnd != 0) &&
            (vcc == cap.vcc) && (gnd == cap.gnd));
}

/* Refuse to power the PLD unless the jumpers match the job */
static rc_t
cap_preflight(void)
{
    uint32_t vcc = 0;
    uint32_t gnd = 0;
    uint     rail_mv = 0;

    if (((cap.vcc | cap.gnd) == 0) && (cap.supply_mv == 0))
        return (RC_SUCCESS);
    if ((cap.vcc == 0) || (cap.gnd == 0)) {
        printf("preflight: set both vcc= and gnd=\n");
        return (RC_BAD_PARAM);
    }
    if ((cap.pins | cap.inv | cap.clk | cap.cand) & (cap.vcc | cap.gnd)) {
        printf("preflight: the job uses a VCC or GND pin\n");
        return (RC_BAD_PARAM);
    }
    if (cap_hw_jumper_pins(&vcc, &gnd, 0) != 0) {
        printf("preflight: could not detect the VCC and GND jumpers\n");
        return (RC_FAILURE);
    }
    if (!cap_pins_match(vcc, gnd)) {
        printf("preflight: jumpers vcc=%07lx gnd=%07lx, job expects "
               "vcc=%07lx gnd=%07lx\n",
               (unsigned long) vcc, (unsigned long) gnd,
               (unsigned long) cap.vcc, (unsigned long) cap.gnd);
        return (RC_FAILURE);
    }
    if (cap.supply_mv == 0)
        return (RC_SUCCESS);
    if (cap_hw_supply(&rail_mv) != 0) {
        printf("preflight: could not measure the supply\n");
        return (RC_FAILURE);
    }
    if (!cap_supply_ok(rail_mv)) {
        printf("preflight: supply %u mV, job expects %u mV\n",
               rail_mv, cap.supply_mv);
        return (RC_FAILURE);
    }
    return (RC_SUCCESS);
}

/*
 * Report the jumpers, with every reading.  The supply is measured only
 * when the VCC and GND pins match the job (vcc= gnd=); otherwise it shows
 * as supply=-.
 */
static rc_t
cap_jumpers(void)
{
    uint32_t vcc = 0;
    uint32_t gnd = 0;
    uint     rail_mv = 0;

    if (cap_hw_jumper_pins(&vcc, &gnd, 1) != 0) {
        printf("jumpers: detection failed\n");
        return (RC_FAILURE);
    }
    if (!cap_pins_match(vcc, gnd)) {
        printf("jumpers vcc=%07lx gnd=%07lx supply=-\n",
               (unsigned long) vcc, (unsigned long) gnd);
        return (RC_SUCCESS);
    }
    if (cap_hw_supply(&rail_mv) != 0) {
        printf("jumpers: could not measure the supply\n");
        return (RC_FAILURE);
    }
    printf("jumpers vcc=%07lx gnd=%07lx supply=%u\n",
           (unsigned long) vcc, (unsigned long) gnd, rail_mv);
    return (RC_SUCCESS);
}

/* -- mixed Gray code ----------------------------------------------------- */

/*
 * A part of the walk at level k covers 2^(k+1) vectors that differ in the
 * k+1 bits of mix_rem[k].  It walks half of them, changes its split bit
 * mix_split[k], and walks the other half; the halves are parts at level
 * k-1 over the other k bits, each with a split bit of its own.  The part
 * at level k that vector number i lies in is number i >> (k+1).
 */
static uint8_t mix_rem[CAP_MAX_WALK][CAP_MAX_WALK];
static uint8_t mix_split[CAP_MAX_WALK];

static uint32_t
cap_mix_hash(uint32_t seed, uint level, uint32_t part)
{
    uint32_t h = seed ^ (part * 0x9e3779b1UL) ^ ((level + 1) * 0x85ebca6bUL);

    h ^= h >> 15;
    h *= 0x2c1b3c6dUL;
    h ^= h >> 12;
    h *= 0x297a2d39UL;
    h ^= h >> 15;
    return (h);
}

/* Choose the split bits of the parts at the levels below `level` that
 * vector number i lies in (level nbits: of all parts, for i = 0) */
static void
cap_mix_enter(uint nbits, uint level, uint32_t i)
{
    uint k;
    uint j;

    if (level == nbits)
        for (j = 0; j < nbits; j++)
            mix_rem[nbits - 1][j] = j;
    for (k = level; k-- > 0; ) {
        uint pick;
        uint n = 0;

        if (k + 1 < nbits)
            for (j = 0; j < k + 2; j++)        /* the part above, less its split bit */
                if (mix_rem[k + 1][j] != mix_split[k + 1])
                    mix_rem[k][n++] = mix_rem[k + 1][j];
        pick = ((cap_mix_hash(cap.mix, k, i >> (k + 1)) >> 16) * (k + 1)) >> 16;
        mix_split[k] = mix_rem[k][pick];
    }
}

/* The bit that changes from vector number i - 1 to i (i not 0) */
static uint
cap_mix_next(uint nbits, uint32_t i)
{
    uint level = __builtin_ctz(i);
    uint bit = mix_split[level];

    cap_mix_enter(nbits, level, i);
    return (bit);
}

/* -- runs ---------------------------------------------------------------- */

static rc_t
cap_walk(void)
{
    uint     nbits = cap_popcount(cap.pins);
    uint32_t xmask = (1UL << nbits) - 1;
    uint     sig_reads = cap.nprobe * (1 + cap_popcount(cap.cand));
    uint32_t i = 0;
    uint32_t x = 0;
    uint32_t r;
    uint32_t cur = 0;
    uint     status = CAP_STATUS_OK;
    rc_t     rc;

    if (nbits > CAP_MAX_WALK) {
        printf("walk: at most %u enumerated pins\n", CAP_MAX_WALK);
        return (RC_BAD_PARAM);
    }
    if ((cap.mul & 1) == 0) {
        printf("walk: mul must be odd\n");
        return (RC_BAD_PARAM);
    }
    if ((cap.rot != 0) && (cap.rot >= nbits)) {
        printf("walk: rot must be below the number of enumerated pins\n");
        return (RC_BAD_PARAM);
    }
    if ((cap.mix != 0) && !cap.gray) {
        printf("walk: mix needs gray\n");
        return (RC_BAD_PARAM);
    }
    if (cap.psig && (cap.nprobe == 0)) {
        printf("walk: psig needs probe vectors\n");
        return (RC_BAD_PARAM);
    }
    if ((rc = cap_preflight()) != RC_SUCCESS)
        return (rc);
    if (cap.mix != 0)
        cap_mix_enter(nbits, nbits, 0);
    cap_begin('W', cap.quiet ? 0 : cap.inv ? 2 : 1, sig_reads, nbits,
              1UL << nbits);
    cap_power_up(cap.off_msec);
    if (cap.nprobe != 0) {
        if (cap.psig)
            memcpy(cap_ref, cap_cur, sig_reads * 4);
        else
            cap_signature(cap_ref);
        cap_put_reads('P', cap_ref, sig_reads);
        cap_flush();
    }
    do {
        uint32_t w = ((cur ^ cap.start) & cap.pins) | (cap.base & ~cap.pins);

        w &= ~(cap.clk & ~cap.pins);
        if ((cap.skm != 0) && ((w & cap.skm) == cap.skv)) {
            if (!cap.quiet) {
                cap_put32('D', 0);
                if (cap.inv != 0)
                    cap_put32('D', 0);
            }
        } else {
            uint32_t rd = cap_io(w);

            if (!cap.quiet)
                cap_put32('D', rd);
            if (cap.inv != 0) {
                rd = cap_io(w ^ cap.inv);
                if (!cap.quiet)
                    cap_put32('D', rd);
            }
        }
        cap_done++;
        if (((cap_done & 0xff) == 0) && (cap_out_err || cap_hw_abort())) {
            status = CAP_STATUS_ABORT;
            break;
        }
        i = (i + 1) & xmask;
        if (cap.gray && (cap.mix != 0))
            x = (i == 0) ? 0 : (x ^ (1UL << cap_mix_next(nbits, i)));
        else
            x = cap.gray ? (i ^ (i >> 1)) : ((x + cap.mul) & xmask);
        r = x;
        if (cap.rot != 0)
            r = ((x << cap.rot) | (x >> (nbits - cap.rot))) & xmask;
        if (cap.rev)
            cur = cap_expand_rev(r);
        else if ((cap.mul == 1) && !cap.gray && (cap.rot == 0))
            cur = ((cur | ~cap.pins) + 1) & cap.pins;
        else
            cur = cap_expand(r);
    } while (i != 0);
    if ((cap.nprobe != 0) && (status == CAP_STATUS_OK)) {
        cap_flush();
        cap_signature(cap_cur);
        cap_put_reads('P', cap_cur, sig_reads);
    }
    return (cap_end(status));
}

static rc_t
cap_trans(void)
{
    uint     nbits = cap_popcount(cap.pins);
    uint32_t xmask = (nbits >= 32) ? 0xffffffff : ((1UL << nbits) - 1);
    uint32_t count = cap.count;
    uint     nreads = cap.nprobe * (1 + cap_popcount(cap.cand));
    uint     nsigs = 0;
    uint     nflags = 0;
    uint     status = CAP_STATUS_OK;
    uint32_t i;
    rc_t     rc;

    if (cap.nprobe == 0) {
        printf("trans: no probe vectors\n");
        return (RC_BAD_PARAM);
    }
    if ((count == 0) || (count > xmask))
        count = xmask + 1;
    if ((count > 1) && ((cap.mul & 1) == 0)) {
        printf("trans: mul must be odd\n");
        return (RC_BAD_PARAM);
    }
    if (cap.pins & cap.clk) {
        printf("trans: pins and clk must not overlap\n");
        return (RC_BAD_PARAM);
    }
    if ((rc = cap_preflight()) != RC_SUCCESS)
        return (rc);
    cap_begin('T', 0, nreads, nbits, count);
    cap_power_up(cap.off_msec);
    if (cap.psig)
        memcpy(cap_ref, cap_cur, nreads * 4);
    else
        cap_signature(cap_ref);
    cap_put_reads('P', cap_ref, nreads);
    cap_flush();

    for (i = 0; i < count; i++) {
        uint32_t x = (i * cap.mul) & xmask;
        uint32_t w = cap_expand(x) | (cap.base & ~cap.pins);
        uint32_t r1 = 0;
        uint32_t pre;
        uint32_t crc;
        uint     k;
        uint8_t  idx[2];

        cap_step(w);
        pre = cap_pre;
        if (cap.psig) {
            r1 = cap_after_step(w);
        } else {
            if (cap.chk)
                r1 = cap_io(w & ~cap.clk);
            cap_signature(cap_cur);
            if (cap.chk)
                r1 ^= cap_io(w & ~cap.clk);
        }
        if (cap.chkm != 0)
            r1 &= cap.chkm;
        if (cap.chk && (r1 != 0) && (nflags++ < CAP_MAX_FLAGS)) {
            cap_put32('F', x);
            cap_put32('F', r1);
        }
        crc = cap_crc32_words(cap_cur, nreads);
        for (k = 0; k < nsigs; k++)
            if (cap_sigs[k] == crc)
                break;
        if ((k == nsigs) && (nsigs == CAP_MAX_SIGS)) {
            status = CAP_STATUS_TOO_MANY;
            break;
        }
        idx[0] = k;
        idx[1] = k >> 8;
        cap_put('D', idx, 2);
        if (k == nsigs) {
            /* First occurrence: the reads follow the index */
            cap_sigs[nsigs++] = crc;
            cap_put_reads('D', cap_cur, nreads);
        }
        if (cap.pre)
            cap_put32('D', pre);
        if (memcmp(cap_cur, cap_ref, nreads * 4) != 0) {
            if (cap_restore(crc, nreads) != 0) {
                status = CAP_STATUS_LOST;
                break;
            }
        }
        cap_done++;
        if (((cap_done & 0x3f) == 0) && (cap_out_err || cap_hw_abort())) {
            status = CAP_STATUS_ABORT;
            break;
        }
    }
    return (cap_end(status));
}

static rc_t
cap_state(void)
{
    uint nreads = cap.nprobe * (1 + cap_popcount(cap.cand));
    rc_t rc;

    if (cap.nprobe == 0) {
        printf("state: no probe vectors\n");
        return (RC_BAD_PARAM);
    }
    if ((rc = cap_preflight()) != RC_SUCCESS)
        return (rc);
    cap_begin('S', 0, nreads, cap_popcount(cap.pins), 1);
    cap_power_up(cap.off_msec);
    if (!cap.psig)
        cap_signature(cap_cur);
    cap_put_reads('P', cap_cur, nreads);
    cap_done = 1;
    return (cap_end(CAP_STATUS_OK));
}

/* -- command line -------------------------------------------------------- */

static int
cap_parse_hex(const char *str, uint32_t *value)
{
    uint32_t v = 0;
    uint     n;

    if ((str[0] == '0') && ((str[1] == 'x') || (str[1] == 'X')))
        str += 2;
    for (n = 0; str[n] != '\0'; n++) {
        char ch = str[n];
        uint d;

        if ((ch >= '0') && (ch <= '9'))
            d = ch - '0';
        else if ((ch >= 'a') && (ch <= 'f'))
            d = ch - 'a' + 10;
        else if ((ch >= 'A') && (ch <= 'F'))
            d = ch - 'A' + 10;
        else
            return (-1);
        if (n >= 8)
            return (-1);
        v = (v << 4) | d;
    }
    if (n == 0)
        return (-1);
    *value = v;
    return (0);
}

static int
cap_parse_dec(const char *str, uint *value)
{
    uint v = 0;
    uint n;

    for (n = 0; str[n] != '\0'; n++) {
        if ((str[n] < '0') || (str[n] > '9') || (n >= 9))
            return (-1);
        v = v * 10 + (str[n] - '0');
    }
    if (n == 0)
        return (-1);
    *value = v;
    return (0);
}

static rc_t
cap_bad(const char *what, const char *arg)
{
    printf("Invalid %s \"%s\"\n", what, arg);
    return (RC_BAD_PARAM);
}

static rc_t
cap_set(int argc, char * const *argv)
{
    static const struct {
        const char *name;
        uint32_t   *hex;
        uint       *dec;
    } keys[] = {
        { "pins",   &cap.pins,  NULL },
        { "base",   &cap.base,  NULL },
        { "inv",    &cap.inv,   NULL },
        { "clk",    &cap.clk,   NULL },
        { "cand",   &cap.cand,  NULL },
        { "mask",   &cap.mask,  NULL },
        { "count",  &cap.count, NULL },
        { "mul",    &cap.mul,   NULL },
        { "rev",    &cap.rev,   NULL },
        { "gray",   &cap.gray,  NULL },
        { "mix",    &cap.mix,   NULL },
        { "rot",    &cap.rot,   NULL },
        { "start",  &cap.start, NULL },
        { "skm",    &cap.skm,   NULL },
        { "skv",    &cap.skv,   NULL },
        { "avm",    &cap.avm,   NULL },
        { "avv",    &cap.avv,   NULL },
        { "quiet",  &cap.quiet, NULL },
        { "chk",    &cap.chk,   NULL },
        { "chkm",   &cap.chkm,  NULL },
        { "psig",   &cap.psig,  NULL },
        { "pre",    &cap.pre,   NULL },
        { "follow", &cap.follow, NULL },
        { "dclk",   &cap.dclk,  NULL },
        { "vcc",    &cap.vcc,   NULL },
        { "gnd",    &cap.gnd,   NULL },
        { "settle", NULL,       &cap.settle_usec },
        { "off",    NULL,       &cap.off_msec },
        { "on",     NULL,       &cap.on_msec },
        { "supply", NULL,       &cap.supply_mv },
    };
    int arg;

    for (arg = 0; arg < argc; arg++) {
        const char *eq = strchr(argv[arg], '=');
        uint        k;
        int         bad;

        if (eq == NULL)
            return (cap_bad("setting", argv[arg]));
        for (k = 0; k < CAP_NELEM(keys); k++)
            if ((strlen(keys[k].name) == (size_t) (eq - argv[arg])) &&
                (strncmp(keys[k].name, argv[arg], eq - argv[arg]) == 0))
                break;
        if (k == CAP_NELEM(keys))
            return (cap_bad("setting", argv[arg]));
        if (keys[k].hex != NULL)
            bad = cap_parse_hex(eq + 1, keys[k].hex);
        else
            bad = cap_parse_dec(eq + 1, keys[k].dec);
        if (bad)
            return (cap_bad("value", argv[arg]));
    }
    if (cap_popcount(cap.cand) > CAP_MAX_CAND) {
        printf("At most %u candidate pins\n", CAP_MAX_CAND);
        cap.cand = 0;
        return (RC_BAD_PARAM);
    }
    if (cap.inv & (cap.pins | cap.clk)) {
        printf("inv must not overlap pins or clk\n");
        return (RC_BAD_PARAM);
    }
    if (cap.dclk && (cap.clk & (cap.vcc | cap.gnd))) {
        printf("dclk: a clock pin is a VCC or GND pin\n");
        return (RC_BAD_PARAM);
    }
    return (RC_SUCCESS);
}

static rc_t
cap_vectors(int argc, char * const *argv, uint32_t *list, uint *count,
            uint max)
{
    uint n = 0;
    int  arg = 0;

    if ((argc > 0) && (strcmp(argv[0], "+") == 0)) {
        n = *count;
        arg++;
    }
    for (; arg < argc; arg++) {
        if (n >= max) {
            printf("At most %u vectors\n", max);
            return (RC_BAD_PARAM);
        }
        if (cap_parse_hex(argv[arg], &list[n]))
            return (cap_bad("vector", argv[arg]));
        n++;
    }
    *count = n;
    return (RC_SUCCESS);
}

static rc_t
cap_ret(int argc, char * const *argv)
{
    cap_ret_t *r;
    uint       n;
    rc_t       rc;

    if ((argc == 1) && (strcmp(argv[0], "clear") == 0)) {
        cap.nret = 0;
        return (RC_SUCCESS);
    }
    if (argc < 1)
        return (RC_USER_HELP);
    if (cap.nret >= CAP_MAX_RET) {
        printf("At most %u return paths\n", CAP_MAX_RET);
        return (RC_BAD_PARAM);
    }
    r = &cap.ret[cap.nret];
    if (cap_parse_hex(argv[0], &r->crc))
        return (cap_bad("CRC", argv[0]));
    n = 0;
    rc = cap_vectors(argc - 1, argv + 1, r->vec, &n, CAP_MAX_RET_LEN);
    if (rc != RC_SUCCESS)
        return (rc);
    r->len = n;
    cap.nret++;
    return (RC_SUCCESS);
}

static void
cap_show_list(const char *name, const uint32_t *list, uint n)
{
    uint i;

    printf("%s (%u):", name, n);
    for (i = 0; i < n; i++)
        printf(" %07lx", (unsigned long) list[i]);
    printf("\n");
}

static void
cap_show(void)
{
    uint k;

    printf("pins=%07lx base=%07lx inv=%07lx clk=%07lx cand=%07lx "
           "mask=%07lx\ncount=%lx mul=%lx rev=%lx gray=%lx mix=%lx rot=%lx "
           "start=%07lx\n"
           "skm=%07lx skv=%07lx avm=%07lx avv=%07lx quiet=%lx chk=%lx\n"
           "chkm=%07lx psig=%lx pre=%lx follow=%lx dclk=%lx\n"
           "settle=%u off=%u on=%u vcc=%07lx gnd=%07lx supply=%u\n",
           (unsigned long) cap.pins, (unsigned long) cap.base,
           (unsigned long) cap.inv, (unsigned long) cap.clk,
           (unsigned long) cap.cand, (unsigned long) cap.mask,
           (unsigned long) cap.count, (unsigned long) cap.mul,
           (unsigned long) cap.rev, (unsigned long) cap.gray,
           (unsigned long) cap.mix, (unsigned long) cap.rot,
           (unsigned long) cap.start, (unsigned long) cap.skm,
           (unsigned long) cap.skv, (unsigned long) cap.avm,
           (unsigned long) cap.avv, (unsigned long) cap.quiet,
           (unsigned long) cap.chk, (unsigned long) cap.chkm,
           (unsigned long) cap.psig, (unsigned long) cap.pre,
           (unsigned long) cap.follow,
           (unsigned long) cap.dclk, cap.settle_usec, cap.off_msec,
           cap.on_msec, (unsigned long) cap.vcc, (unsigned long) cap.gnd,
           cap.supply_mv);
    cap_show_list("path", cap.path, cap.npath);
    cap_show_list("probe", cap.probe, cap.nprobe);
    for (k = 0; k < cap.nret; k++) {
        printf("ret %08lx", (unsigned long) cap.ret[k].crc);
        cap_show_list("", cap.ret[k].vec, cap.ret[k].len);
    }
}

/*
 * cmd_pld_capture
 * ---------------
 * Handle "pld capture <subcommand> ...".  argv[0] is "capture".
 */
rc_t
cmd_pld_capture(int argc, char * const *argv)
{
    static int initialized;
    const char *sub;

    if (initialized == 0) {
        cap_defaults();
        initialized = 1;
    }
    if ((argc < 2) || (strcmp(argv[1], "?") == 0) ||
        (strcmp(argv[1], "help") == 0)) {
        printf("%s", cmd_pld_capture_help);
        return (RC_SUCCESS);
    }
    sub = argv[1];
    argc -= 2;
    argv += 2;
    if (strcmp(sub, "clear") == 0) {
        cap_defaults();
        return (RC_SUCCESS);
    }
    if (strcmp(sub, "set") == 0)
        return (cap_set(argc, argv));
    if (strcmp(sub, "path") == 0)
        return (cap_vectors(argc, argv, cap.path, &cap.npath, CAP_MAX_PATH));
    if (strcmp(sub, "probe") == 0)
        return (cap_vectors(argc, argv, cap.probe, &cap.nprobe,
                            CAP_MAX_PROBES));
    if (strcmp(sub, "ret") == 0)
        return (cap_ret(argc, argv));
    if (strcmp(sub, "show") == 0) {
        cap_show();
        return (RC_SUCCESS);
    }
    if (strcmp(sub, "jumpers") == 0)
        return (cap_jumpers());
    if (strcmp(sub, "walk") == 0)
        return (cap_walk());
    if (strcmp(sub, "trans") == 0)
        return (cap_trans());
    if (strcmp(sub, "state") == 0)
        return (cap_state());
    printf("Unknown capture command \"%s\"\n", sub);
    return (RC_USER_HELP);
}
