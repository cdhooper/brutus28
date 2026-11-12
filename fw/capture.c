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
 * clock pins high: one rising clock edge.
 *
 * A signature is, for each probe vector p (clock pins low), one read of
 * p, then one read per candidate pin c with only c's drive inverted.  A
 * pin the PLD drives reads the same both times; a pin it does not drive
 * follows the 1K drive.  Every read is masked with the job's read mask.
 *
 * walk   For every assignment of the enumerated pins (counting order, the
 *        other pins at base, clock pins low): drive and read; with inv
 *        set, read again with the inv pins inverted.  With probes set,
 *        the signature is reported before and after the walk, so the host
 *        can tell whether the walk disturbed the register state.
 * trans  Read the signature of the path state S.  Then for each x: step
 *        with x, read the signature, and get back to S.  Nothing to do if
 *        the signature is S's; otherwise clock along the return path the
 *        host gave for that signature (and check S's signature), else
 *        power-cycle and replay the path.  One byte per x: the index of
 *        its signature.  The first occurrence of a signature is followed
 *        by its reads.  x runs over (i * mul) mod 2^n for i < count.
 * state  Report the signature of the path state.
 *
 * Output: the line "---- CAPTURE ----", then binary chunks
 * <type:1> <length:2 LE> <payload>: 'H' header, 'P' signature reads,
 * 'D' data, 'E' end.  Reads are 32-bit little endian.  The 'E' payload is
 * status, 3 pad bytes, then 32-bit counts of vectors done, power-cycle
 * restores, return-path restores, failed return paths, and the CRC-32
 * (as zlib) of every chunk byte before the 'E' chunk.
 */

#include "main.h"
#include "printf.h"
#include "cmdline.h"
#include <string.h>
#include "capture.h"

#define CAP_VERSION      1
#define CAP_MAX_PATH     48
#define CAP_MAX_PROBES   8
#define CAP_MAX_CAND     12
#define CAP_MAX_READS    (CAP_MAX_PROBES * (1 + CAP_MAX_CAND))
#define CAP_MAX_RET      32
#define CAP_MAX_RET_LEN  16
#define CAP_MAX_SIGS     255
#define CAP_MAX_WALK     24      /* enumerated pins for a walk */
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
    uint32_t  mul;          /* trans: x = i * mul */
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

const char cmd_pld_capture_help[] =
"pld capture clear              - reset the job to defaults\n"
"pld capture set <key>=<value>  - pins base inv clk cand mask count mul (hex)\n"
"                                 settle (usec) off on (msec)\n"
"pld capture path [+] <v>...    - clocked vectors from power-up to the state\n"
"pld capture probe [+] <v>...   - probe vectors for signatures\n"
"pld capture ret <crc> <v>...   - return path from the state with that\n"
"                                 signature CRC; 'ret clear' drops them all\n"
"pld capture show               - show the job\n"
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

static void
cap_step(uint32_t v)
{
    (void) cap_hw_io(v & ~cap.clk, cap.settle_usec);
    (void) cap_hw_io(v | cap.clk, cap.settle_usec);
}

static void
cap_power_up(uint off_msec)
{
    uint i;

    cap_hw_power(0);
    cap_hw_delay_msec(off_msec);
    cap_hw_power(1);
    cap_hw_delay_msec(cap.on_msec);
    for (i = 0; i < cap.npath; i++)
        cap_step(cap.path[i]);
}

static uint
cap_signature(uint32_t *r)
{
    uint n = 0;
    uint i;

    for (i = 0; i < cap.nprobe; i++) {
        uint32_t w = cap.probe[i] & ~cap.clk;
        uint32_t c;

        r[n++] = cap_hw_io(w, cap.settle_usec) & cap.mask;
        for (c = cap.cand; c != 0; c &= c - 1)
            r[n++] = cap_hw_io(w ^ (c & -c), cap.settle_usec) & cap.mask;
    }
    return (n);
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

/* Get back to the path state after a transition; 0 on success */
static int
cap_restore(uint32_t crc, uint nreads)
{
    uint k;
    uint i;

    for (k = 0; k < cap.nret; k++) {
        if (cap.ret[k].crc != crc)
            continue;
        for (i = 0; i < cap.ret[k].len; i++)
            cap_step(cap.ret[k].vec[i]);
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
        cap_signature(cap_cur);
        if (memcmp(cap_cur, cap_ref, nreads * 4) == 0) {
            cap_pcycles++;
            return (0);
        }
    }
    return (-1);
}

/* -- runs ---------------------------------------------------------------- */

static rc_t
cap_walk(void)
{
    uint     nbits = cap_popcount(cap.pins);
    uint     sig_reads = cap.nprobe * (1 + cap_popcount(cap.cand));
    uint32_t cur = 0;
    uint     status = CAP_STATUS_OK;

    if (nbits > CAP_MAX_WALK) {
        printf("walk: at most %u enumerated pins\n", CAP_MAX_WALK);
        return (RC_BAD_PARAM);
    }
    cap_begin('W', cap.inv ? 2 : 1, sig_reads, nbits, 1UL << nbits);
    cap_power_up(cap.off_msec);
    if (cap.nprobe != 0) {
        cap_signature(cap_ref);
        cap_put_reads('P', cap_ref, sig_reads);
        cap_flush();
    }
    do {
        uint32_t w = (cur | (cap.base & ~cap.pins)) & ~cap.clk;

        cap_put32('D', cap_hw_io(w, cap.settle_usec) & cap.mask);
        if (cap.inv != 0)
            cap_put32('D', cap_hw_io(w ^ cap.inv, cap.settle_usec) & cap.mask);
        cap_done++;
        if (((cap_done & 0xff) == 0) && (cap_out_err || cap_hw_abort())) {
            status = CAP_STATUS_ABORT;
            break;
        }
        cur = ((cur | ~cap.pins) + 1) & cap.pins;
    } while (cur != 0);
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
    uint     status = CAP_STATUS_OK;
    uint32_t i;

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
    cap_begin('T', 0, nreads, nbits, count);
    cap_power_up(cap.off_msec);
    cap_signature(cap_ref);
    cap_put_reads('P', cap_ref, nreads);
    cap_flush();

    for (i = 0; i < count; i++) {
        uint32_t x = (i * cap.mul) & xmask;
        uint32_t crc;
        uint     k;
        uint8_t  idx;

        cap_step(cap_expand(x) | (cap.base & ~cap.pins));
        cap_signature(cap_cur);
        crc = cap_crc32_words(cap_cur, nreads);
        for (k = 0; k < nsigs; k++)
            if (cap_sigs[k] == crc)
                break;
        if ((k == nsigs) && (nsigs == CAP_MAX_SIGS)) {
            status = CAP_STATUS_TOO_MANY;
            break;
        }
        idx = k;
        cap_put('D', &idx, 1);
        if (k == nsigs) {
            /* First occurrence: the reads follow the index */
            cap_sigs[nsigs++] = crc;
            cap_put_reads('D', cap_cur, nreads);
        }
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

    if (cap.nprobe == 0) {
        printf("state: no probe vectors\n");
        return (RC_BAD_PARAM);
    }
    cap_begin('S', 0, nreads, cap_popcount(cap.pins), 1);
    cap_power_up(cap.off_msec);
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
        { "settle", NULL,       &cap.settle_usec },
        { "off",    NULL,       &cap.off_msec },
        { "on",     NULL,       &cap.on_msec },
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
    if ((cap.pins & cap.clk) || (cap.inv & (cap.pins | cap.clk))) {
        printf("pins, clk and inv must not overlap\n");
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
           "mask=%07lx\ncount=%lx mul=%lx settle=%u off=%u on=%u\n",
           (unsigned long) cap.pins, (unsigned long) cap.base,
           (unsigned long) cap.inv, (unsigned long) cap.clk,
           (unsigned long) cap.cand, (unsigned long) cap.mask,
           (unsigned long) cap.count, (unsigned long) cap.mul,
           cap.settle_usec, cap.off_msec, cap.on_msec);
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
    if (strcmp(sub, "walk") == 0)
        return (cap_walk());
    if (strcmp(sub, "trans") == 0)
        return (cap_trans());
    if (strcmp(sub, "state") == 0)
        return (cap_state());
    printf("Unknown capture command \"%s\"\n", sub);
    return (RC_USER_HELP);
}
