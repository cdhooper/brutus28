/*
 * Host test harness for fw/capture.c: the PLD is a simulator reached
 * through callbacks, and all output (text and binary) is collected the
 * way the Brutus-28 console would send it.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "capture.h"

typedef uint32_t (*io_cb_t)(uint32_t value);
typedef void (*power_cb_t)(int on);

static io_cb_t        io_cb;
static power_cb_t     power_cb;
static unsigned char *out;
static size_t         out_len;
static size_t         out_cap;

static void
out_put(const void *p, size_t n)
{
    if (out_len + n > out_cap) {
        out_cap = (out_len + n) * 2 + 4096;
        out = realloc(out, out_cap);
    }
    memcpy(out + out_len, p, n);
    out_len += n;
}

int
harness_printf(const char *fmt, ...)
{
    char    buf[2048];
    va_list ap;
    int     n;
    int     i;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof (buf), fmt, ap);
    va_end(ap);
    for (i = 0; (i < n) && (buf[i] != '\0'); i++) {
        if (buf[i] == '\n')
            out_put("\r", 1);       /* the firmware's putchar adds CR */
        out_put(&buf[i], 1);
    }
    return (n);
}

void
cap_hw_power(int on)
{
    power_cb(on);
}

uint32_t
cap_hw_io(uint32_t value, uint settle_usec)
{
    (void) settle_usec;
    return (io_cb(value));
}

void
cap_hw_delay_msec(uint msec)
{
    (void) msec;
}

int
cap_hw_abort(void)
{
    return (0);
}

int
cap_hw_output(const void *buf, uint len)
{
    out_put(buf, len);
    return (0);
}

void
harness_init(io_cb_t io, power_cb_t power)
{
    io_cb = io;
    power_cb = power;
}

/* Run one "pld capture ..." line; returns the length of its output */
size_t
harness_run(const char *line)
{
    char *copy = strdup(line);
    char *argv[72];
    int   argc = 0;
    char *tok;

    out_len = 0;
    for (tok = strtok(copy, " "); (tok != NULL) && (argc < 71);
         tok = strtok(NULL, " "))
        argv[argc++] = tok;
    argv[argc] = NULL;
    if ((argc >= 2) && (strcmp(argv[0], "pld") == 0))
        (void) cmd_pld_capture(argc - 1, argv + 1);
    free(copy);
    return (out_len);
}

const unsigned char *
harness_output(void)
{
    return (out);
}
