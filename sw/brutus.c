/*
 * PLD brute force analyzer by Chris Hooper
 *
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2022.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <err.h>
#include <stdarg.h>

/* Options for debug output */
#undef  DEBUG_LIMIT_BITS
#undef  DEBUG_BIT_FLIP
#undef  DEBUG_ADD_OR_MASK
#undef  DEBUG_ADD_OR_MASK_2
#undef  DEBUG_IS_CONTAINED_WITHIN
#define DEBUG_ELIMINATE_COMMON_TERMS_1
#undef  DEBUG_ELIMINATE_COMMON_TERMS_2
#undef  DEBUG_ELIMINATE_COMMON_TERMS_3
#undef  DEBUG_COLLECT_OR_MASKS
#define  DEBUG_MERGE_COMMON_SUBEXPRESSIONS

#ifdef  DEBUG_LIMIT_BITS
#define LIMIT_BITS (BIT(23) | BIT(24) | BIT(25))  // U202 CINH SLAVE SLAVEOE
#endif

#define CONTENT_UNKNOWN       0  // Unknown content type
#define CONTENT_RAW_BINARY    1  // Raw binary data
#define CONTENT_ASCII_UNKNOWN 2  // Unknown ASCII (hex or binary)
#define CONTENT_ASCII_BINARY  3  // ASCII binary
#define CONTENT_ASCII_HEX     4  // ASCII hex

#define KEYWORD_UNKNOWN 0
#define KEYWORD_END     1 // No more content
#define KEYWORD_DEVICE  2 // DEVICE <name>
#define KEYWORD_PIN     3 // PIN <num> = <name>

#define ARRAY_SIZE(x) ((sizeof (x) / sizeof ((x)[0])))
#define BIT(x)      (1 << (x))

typedef unsigned int uint;

static uint      total_lines = 0;    // Number of lines expected
static uint      read_lines  = 0;    // Number of lines read
static uint32_t *pld_in;             // Pin input to the PLD
static uint32_t *pld_out;            // Pin output from the PLD
static uint8_t   bit_flip_pos[32];   // Bit flip offsets into pld_in[] records
static uint32_t  pins_affecting_pin[32];  // Mask of other pins affecting this
static uint32_t  pins_always_input     = 0xffffffff;  // Pins which are inputs
static uint32_t  pins_output           = 0x00000000;  // Pins which are outputs
static uint32_t  pins_only_output_high = 0xffffffff;  // Open drain, drive high
static uint32_t  pins_only_output_low  = 0xffffffff;  // Open drain, drive low
static uint32_t  ignore_mask           = 0x00000000;  // Pins to ignore
static const char *cfg_filename        = NULL;        // config filename
static const char *cfg_file_map        = NULL;        // memory-mapped config
static const char *cfg_file_end        = NULL;        // end of mapped config


typedef struct {
    uint     pie_line;
    uint8_t  pie_result_bit;
    uint32_t pie_input_bits;
    uint32_t pie_affecting_bits;
} pi_ent_t;

static struct {
    uint        pi_affecting_bits;
    uint        pi_count;
    uint        pi_count_max;
    uint8_t     pi_invert;  // Pin is inverted at input/output
    uint8_t     pi_num;     // Pin number at input/output
    const char *pi_name;    // Pin virtual name
    pi_ent_t   *pi_ent;
} pinfo[32];

static const uint8_t bit_to_pin_g22v10[] =
{
     0,  1,  2,  3,  4,  5,  6,  0,
     7,  8,  9, 10, 11, 12,  0, 13,
    14, 15, 16, 17, 18,  0, 19, 20,
    21, 22, 23, 24,
};

static const uint8_t bit_to_pin_dip24[] =
{
     1,  2,  3,  4,  5,  6,  7,  8,
     9, 10, 11, 12, 13, 14, 15, 16,
    17, 18, 19, 20, 21, 22, 23, 24,
     0,  0,  0,  0,
};

static const uint8_t bit_to_pin_dip22[] =
{
     1,  2,  3,  4,  5,  6,  7,  8,
     9, 10, 11,  0,  0, 12, 13, 14,
    15, 16, 17, 18, 19, 20, 21, 22,
     0,  0,  0,  0,
};

static const uint8_t bit_to_pin_dip20[] =
{
     1,  2,  3,  4,  5,  6,  7,  8,
     9, 10,  0,  0,  0,  0, 11, 12,
    13, 14, 15, 16, 17, 18, 19, 20,
     0,  0,  0,  0,
};

static const uint8_t bit_to_pin_dip18[] =
{
     1,  2,  3,  4,  5,  6,  7,  8,
     9,  0,  0,  0,  0,  0,  0, 10,
    11, 12, 13, 14, 15, 16, 17, 18,
     0,  0,  0,  0,
};

static const uint8_t bit_to_pin_dip16[] =
{
     1,  2,  3,  4,  5,  6,  7,  8,
     0,  0,  0,  0,  0,  0,  0,  0,
     9, 10, 11, 12, 13, 14, 15, 16,
     0,  0,  0,  0,  0,  0,  0,  0,
};

static const uint8_t bit_to_pin_dip14[] =
{
     1,  2,  3,  4,  5,  6,  7,  0,
     0,  0,  0,  0,  0,  0,  0,  0,
     0,  8,  9, 10, 11, 12, 13, 14,
     0,  0,  0,  0,  0,  0,  0,  0,
};

static const uint8_t bit_to_pin_dip12[] =
{
     1,  2,  3,  4,  5,  6,  0,  0,
     0,  0,  0,  0,  0,  0,  0,  0,
     0,  0,  7,  8,  9, 10, 11, 12,
     0,  0,  0,  0,  0,  0,  0,  0,
};

static const uint8_t bit_to_pin_dip10[] =
{
     1,  2,  3,  4,  5,  0,  0,  0,
     0,  0,  0,  0,  0,  0,  0,  0,
     0,  0,  0,  6,  7,  8,  9, 10,
     0,  0,  0,  0,  0,  0,  0,  0,
};

static const uint8_t bit_to_pin_dip8[] =
{
     1,  2,  3,  4,  0,  0,  0,  0,
     0,  0,  0,  0,  0,  0,  0,  0,
     0,  0,  0,  0,  5,  6,  7,  8,
     0,  0,  0,  0,  0,  0,  0,  0,
};

static const uint8_t bit_to_pin_dip6[] =
{
     1,  2,  3,  0,  0,  0,  0,  0,
     0,  0,  0,  0,  0,  0,  0,  0,
     0,  0,  0,  0,  0,  4,  5,  6,
     0,  0,  0,  0,  0,  0,  0,  0,
};

static const uint8_t bit_to_pin_dip4[] =
{
     1,  2,  0,  0,  0,  0,  0,  0,
     0,  0,  0,  0,  0,  0,  0,  0,
     0,  0,  0,  0,  0,  0,  3,  4,
     0,  0,  0,  0,  0,  0,  0,  0,
};

static const uint8_t *bit_to_pin = NULL;

/*
 * pin_name
 * --------
 * Returns the human-readable pin name which corresponds to the
 * specified bit number. This function knows how to deal with pins
 * which were configured as inverted, and inversions of those.
 */
static const char *
pin_name(uint pin, uint invert)
{
    static char  buf[2][32];
    static uint8_t whichbuf = 0;

    invert ^= pinfo[pin].pi_invert;
    if (pinfo[pin].pi_name != NULL) {
        if (invert) {
            whichbuf ^= whichbuf;
            sprintf(buf[whichbuf], "!%s", pinfo[pin].pi_name);
            return (buf[whichbuf]);
        }
        return (pinfo[pin].pi_name);
    }
    whichbuf ^= whichbuf;
    sprintf(buf[whichbuf], "%sP%u", invert ? "!" : "", pinfo[pin].pi_num);
    return (buf[whichbuf]);
}

/*
 * incoming_data
 * -------------
 * Process a single incoming data line into the pld_in[] (pins driven to
 * the PLD) and pld_out[] (pins driven by the PLD combined with inputs
 * to the PLD).
 */
void
incoming_data(uint32_t in, uint32_t out)
{
    if (read_lines < total_lines) {
        pld_in[read_lines] = in;
        pld_out[read_lines] = out;
    }
    read_lines++;
}

/*
 * bcdbinary
 * ---------
 * Convert binary coded decimal binary value to binary.
 * Example: the value 0x11111111 becomes 0xff
 * Example: the value 0x10100101 becomes 0xa5
 */
static uint8_t
bcdbinary(uint32_t value)
{
    return (((value & BIT(28)) >> 21) |
            ((value & BIT(24)) >> 18) |
            ((value & BIT(20)) >> 15) |
            ((value & BIT(16)) >> 12) |
            ((value & BIT(12)) >> 9) |
            ((value & BIT(8)) >> 6) |
            ((value & BIT(4)) >> 3) |
             (value & BIT(0)));
}

/*
 * read_cap_file
 * -------------
 * Read a capture file from disk and process all records present.
 * The beginning and end of the capture is automatically determined
 * based on expected output text from the Brutus firmware, so it's
 * not necessary to manually trim the input file.
 */
void
read_cap_file(const char *filename)
{
    char line[256];
    FILE *fp;
    char *ptr;
    int line_num = 0;
    int data_line_num = 0;
    int content_type = CONTENT_UNKNOWN;

    fp = fopen(filename, "r");
    if (fp == NULL)
        err(EXIT_FAILURE, "Unable to open %s for read", filename);

    while (fgets(line, sizeof (line), fp) != NULL) {
        if (line_num++ > 100)
            break;
        ptr = strstr(line, "---- BYTES=");
        if (ptr != NULL) {
            /* Content is raw data */
            uint32_t bytes;
            content_type = CONTENT_RAW_BINARY;
            sscanf(ptr + 11, "%x", &bytes);
            total_lines = bytes / 8;
            break;
        }
        ptr = strstr(line, "---- LINES=");
        if (ptr != NULL) {
            /* Content is either hex or binary data */
            content_type = CONTENT_ASCII_UNKNOWN;
            sscanf(ptr + 11, "%x", &total_lines);
            break;
        }
    }

    if (content_type == CONTENT_UNKNOWN)
        errx(EXIT_FAILURE, "Could not find start marker in %s", filename);

    pld_in  = malloc(total_lines * 4);
    pld_out = malloc(total_lines * 4);
    if ((pld_in == NULL) || (pld_out == NULL))
        err(EXIT_FAILURE, "Unable to allocate %u bytes", total_lines * 4);

    data_line_num = 1;
    if (content_type == CONTENT_RAW_BINARY) {
        uint32_t buf[2];
        while (fread(buf, sizeof (buf), 1, fp) == 1) {
            if ((buf[0] == 0x2d2d2d2d) && (buf[1] == 0x444e4520)) {
                /* ---- END */
                break;
            }
            incoming_data(buf[0], buf[1]);
            line_num++;
            data_line_num++;
        }
    } else {
        /* ASCII unknown content type */
        while (fgets(line, sizeof (line), fp) != NULL) {
            if (content_type == CONTENT_ASCII_UNKNOWN) {
                ptr = strchr(line, ':');
                if (ptr != NULL) {
                    ptr = strchr(ptr + 1, ':');
                    if (ptr != NULL)
                        content_type = CONTENT_ASCII_BINARY;
                }
                if (ptr == NULL)
                    content_type = CONTENT_ASCII_HEX;
            }
            if (strstr(line, "---- END ----") != NULL)
                break;
            switch (content_type) {
                case CONTENT_ASCII_BINARY: {
                    uint32_t v1a, v1b, v1c, v1d;
                    uint32_t v2a, v2b, v2c, v2d;
                    if (sscanf(line, "%04x:%08x:%08x:%08x %04x:%08x:%08x:%08x",
                               &v1a, &v1b, &v1c, &v1d,
                               &v2a, &v2b, &v2c, &v2d) != 8) {
                        warnx("line %u invalid: %s\n", line_num, line);
                    } else {
                        uint32_t v1 = (bcdbinary(v1a) << 24) |
                                      (bcdbinary(v1b) << 16) |
                                      (bcdbinary(v1c) << 8) |
                                      (bcdbinary(v1d));
                        uint32_t v2 = (bcdbinary(v2a) << 24) |
                                      (bcdbinary(v2b) << 16) |
                                      (bcdbinary(v2c) << 8) |
                                      (bcdbinary(v2d));
                        incoming_data(v1, v2);
                    }
                    break;
                }
                case CONTENT_ASCII_HEX: {
                    uint32_t v1;
                    uint32_t v2;
                    if (sscanf(line, "%08x %08x", &v1, &v2) != 2) {
                        warnx("line %u invalid: %s\n", line_num, line);
                    } else {
                        incoming_data(v1, v2);
                    }
                    break;
                }
            }
            line_num++;
            if (data_line_num++ == total_lines)
                break;
        }
    }
    fclose(fp);
    if (read_lines != total_lines) {
        warnx("Read %u lines of data, but expected %u lines",
              read_lines, total_lines);
    }
}

/*
 * fatal_cfg
 * ---------
 * Print a fatal error message relative to a particular configuration
 * file line and exit with a non-zero status.
 */
static void
fatal_cfg(uint sline, uint eline, const char *fmt, ...)
{
    va_list ap;

    fprintf(stderr, "%s:%u", cfg_filename, sline);
    if (sline != eline)
        fprintf(stderr, "-%u ", eline);
    else
        fprintf(stderr, " ");

    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fprintf(stderr, "\n");

    exit(1);
}

/*
 * strnchr
 * -------
 * Find a specific character in a string, up to a specified length.
 * This function is similar to strchr() except for the length limit.
 * NULL is returned if the character is not found.
 */
static const char *
strnchr(const char *str, size_t len, char ch)
{
    while (len-- > 0) {
        if (*str == ch)
            return (str);
        str++;
    }
    return (NULL);
}

/*
 * find_next_keyword
 * -----------------
 * Locate the next configuration keyword in the specified string.
 */
static const char *
find_next_keyword(const char *sptr, const char *eptr, uint *keyword)
{
    while (sptr < eptr) {
        if (strncasecmp(sptr, "DEVICE", 6) == 0) {
            *keyword = KEYWORD_DEVICE;
            return (sptr);
        } else if (strncasecmp(sptr, "PIN", 3) == 0) {
            *keyword = KEYWORD_PIN;
            return (sptr);
        }
        sptr++;
    }
    *keyword = KEYWORD_UNKNOWN;
    return (NULL);
}

/*
 * pin_to_bit
 * ----------
 * Converts a given config file pin number to bit position.
 * The current version only supports GAL22V10 parts.
 */
static uint
pin_to_bit(uint pin)
{
    uint bit;

    if (bit_to_pin == NULL)
        return (pin - 1);

    for (bit = 0; bit < 32; bit++)
        if (bit_to_pin[bit] == pin)
            return (bit);
    return (0);
}

/*
 * pin_to_bit
 * ----------
 * Handle "DEVICE" keyword in a config file.
 * The current version only supports GAL22V10 parts.
 */
static void
cfg_keyword_device(const char *sptr, const char *eptr, uint line)
{
    uint bit;
    char devname[32];
    char *ptr;

    sptr += 6;  // DEVICE
    if ((*sptr == ' ') || (*sptr == '\t'))
        sptr++;

    strncpy(devname, sptr, sizeof (devname));
    devname[sizeof (devname) - 1] = '\0';
    for (ptr = devname; *ptr != '\0'; ptr++) {
        if (((*ptr < '0') || (*ptr > 'z')) ||
            ((*ptr > '9') && (*ptr < 'A')) ||
            ((*ptr > 'Z') && (*ptr < 'a'))) {
            *ptr = '\0';
            break;
        }
    }

    if (strncasecmp(devname, "G22V10", 6) == 0)
        bit_to_pin = bit_to_pin_g22v10;
    else if (strcasecmp(devname, "DIP24") == 0)
        bit_to_pin = bit_to_pin_dip24;
    else if (strcasecmp(devname, "DIP22") == 0)
        bit_to_pin = bit_to_pin_dip22;
    else if (strcasecmp(devname, "DIP20") == 0)
        bit_to_pin = bit_to_pin_dip20;
    else if (strcasecmp(devname, "DIP18") == 0)
        bit_to_pin = bit_to_pin_dip18;
    else if (strcasecmp(devname, "DIP16") == 0)
        bit_to_pin = bit_to_pin_dip16;
    else if (strcasecmp(devname, "DIP14") == 0)
        bit_to_pin = bit_to_pin_dip14;
    else if (strcasecmp(devname, "DIP12") == 0)
        bit_to_pin = bit_to_pin_dip12;
    else if (strcasecmp(devname, "DIP10") == 0)
        bit_to_pin = bit_to_pin_dip10;
    else if (strcasecmp(devname, "DIP8") == 0)
        bit_to_pin = bit_to_pin_dip8;
    else if (strcasecmp(devname, "DIP6") == 0)
        bit_to_pin = bit_to_pin_dip6;
    else if (strcasecmp(devname, "DIP4") == 0)
        bit_to_pin = bit_to_pin_dip4;
    else
        fatal_cfg(line, line, "invalid device '%s'", devname);

    if (bit_to_pin != NULL) {
        for (bit = 0; bit < 28; bit++)
            pinfo[bit].pi_num = bit_to_pin[bit];
    } else {
        for (bit = 0; bit < 28; bit++)
            pinfo[bit].pi_num = bit + 1;
    }
}

/*
 * cfg_keyword_pin
 * ---------------
 * Handle "PIN" keyword in a config file.
 */
static void
cfg_keyword_pin(const char *sptr, const char *eptr, uint line)
{
    uint pin;
    uint pos;
    uint bit;
    const char *nptr;

    sptr += 3;  // PIN
    if ((*sptr == ' ') || (*sptr == '\t'))
        sptr++;
    if (sscanf(sptr, "%d%n", &pin, &pos) != 1)
        fatal_cfg(line, line, "invalid pin number '%s'", sptr);

    bit = pin_to_bit(pin);
    if (bit == 0)
        fatal_cfg(line, line, "invalid pin number '%u'", pin);

    for (sptr += pos; *sptr != '='; sptr++)
        if (sptr >= eptr)
            fatal_cfg(line, line, "no '=' sign in PIN statement");

    for (sptr++; (*sptr == ' ') || (*sptr == '\t'); sptr++)
        if (sptr >= eptr)
            fatal_cfg(line, line, "no '=' sign in PIN statement");

    nptr = sptr;
    for (; (*sptr != ' ') && (*sptr != '\t') && (*sptr != ';'); sptr++)
        if (sptr >= eptr)
            fatal_cfg(line, line, "no ';' at end of PIN statement");
    if (*nptr == '!') {
        nptr++;
        pinfo[bit].pi_invert = 1;
    }
    pinfo[bit].pi_name = strndup(nptr, sptr - nptr);
}

/*
 * parse_cfg_file
 * --------------
 * Reads a config file and processes all known keywords present.
 * This function currently supports very few keywords and should probably
 * be modified to simply ignore keywords it doesn't recognize.
 */
static void
parse_cfg_file(const char *filename)
{
    uint        line = 1;
    const char *sptr = cfg_file_map;
    const char *eptr;
    const char *tptr;
    const char *kptr;
    uint        keyword;

    while (sptr < cfg_file_end) {
        uint tline;
        uint kline;
        eptr = strnchr(sptr, cfg_file_end - sptr, ';');
        if (eptr == NULL) {
            if (sptr < cfg_file_end)
                eptr = cfg_file_end;
            else
                break;
        }
        for (tline = line, tptr = sptr; tptr <= eptr; tptr++) {
            if (tptr == eptr)
                break;

            if (*tptr == '\n')
                tline++;
        }
        kptr = find_next_keyword(sptr, eptr, &keyword);
        if (kptr == NULL) {
            /* If just whitespace or comments remain, then these can be ignored */
            const char *ptr;
            for (ptr = sptr; ptr < eptr; ptr++) {
                if ((*sptr != '\n') || (*sptr != ' ') || (*sptr != '\t'))
                    continue;
                break;
            }
            if (ptr < eptr)
                fatal_cfg(line, tline, "missing keyword");
        }
        for (kline = line, tptr = sptr; tptr <= eptr; tptr++) {
            if (tptr == eptr)
                break;
            if (*tptr == '\n')
                kline++;
        }

        switch (keyword) {
            case KEYWORD_DEVICE:
                cfg_keyword_device(kptr, eptr, kline);
                break;
            case KEYWORD_PIN:
                cfg_keyword_pin(kptr, eptr, kline);
                break;
            case KEYWORD_END:
                sptr = cfg_file_end;
                break;
        }

        while (sptr < eptr + 1) {
            if (*sptr == '\n')
                line++;
            sptr++;
        }
    }
}

/*
 * read_cfg_file
 * -------------
 * Reads/maps the config file into memory.
 */
static void
read_cfg_file(const char *filename)
{
    size_t      cfg_file_size = 0;  // size of config file
    struct stat statbuf;
    int         fd;

    if (filename == NULL)
        return;

    fd = open(filename, O_RDONLY);
    if (fd < 0)
        err(EXIT_FAILURE, "Unable to open %s for read", filename);

    if (fstat(fd, &statbuf) < 0)
        err(EXIT_FAILURE, "fstat %s failed", filename);

    cfg_file_size = statbuf.st_size;
    cfg_file_map = mmap(0, cfg_file_size, PROT_READ, MAP_SHARED, fd, 0);
    if (cfg_file_map == NULL)
        err(EXIT_FAILURE, "mmap of %s failed", filename);
    cfg_file_end = cfg_file_map + cfg_file_size;

    parse_cfg_file(filename);
}

/*
 * print_cfg_file
 * --------------
 * Sends the contents of the config file to stdout.
 */
void
print_cfg_file(void)
{
    if (cfg_file_map == NULL) {
        /* Need to print our own config file */
        uint pin = 1;
        uint bit;

        for (bit = 0; bit < 32; bit++) {
            if (ignore_mask & BIT(bit))
                continue;
            printf("PIN %u = %s;\n", pinfo[bit].pi_num, pin_name(bit, 0));
            pin++;
        }
    }
    printf("%.*s", cfg_file_end - cfg_file_map, cfg_file_map);
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
        printf("%d", !!(value & BIT(bit)));
        if ((bit == 24) || (bit == 16) || (bit == 8))
            printf(":");
    }
}

/*
 * build_ignore_mask
 * -----------------
 * Walks all data read in from the file and builds a mask of bits which
 * were never touched in the input file. This is an easy way to figure
 * out the bits which do not matter.
 */
static void
build_ignore_mask(void)
{
    uint     line;
    uint32_t saw_0 = 0x00000000;
    uint32_t saw_1 = 0x00000000;

    for (line = 0; line < read_lines; line++) {
        saw_0 |= ~pld_in[line];
        saw_1 |= pld_in[line];
    }
    ignore_mask = ~(saw_0 & saw_1);
    print_binary(ignore_mask);
    printf(" ignore_mask = %08x\n", ignore_mask);
}

/*
 * build_bit_flip_offsets
 * ----------------------
 * This function uses the previously computed ignore_mask (the bits which
 * were not sequenced in the input file) to compute power-of-two offsets
 * for where the equivalent input pattern should exist with only the
 * relevant bit flipped. This is useful for comparing the output result
 * of, for example:
 *     0000:00001101:10010000:00000000
 * with that of:
 *     0000:00001101:10010000:00000010
 *
 * The strong assumption for this to work is that the input file was
 * sequenced in a normal binary counting order (..000, ..001, ..010, ...).
 */
static void
build_bit_flip_offsets(void)
{
    uint bit;
    uint nbit = 0;
    for (bit = 0; bit < 32; bit++) {
        if (ignore_mask & BIT(bit))
            continue;
        bit_flip_pos[bit] = nbit;
#ifdef DEBUG_BIT_FLIP
        printf("bitflip_pos %2d=%-2d ", bit, nbit);
        print_binary(pld_in[BIT(nbit)]);
        printf("\n");
#endif
        nbit++;
    }
}

/*
 * walk_find_affected
 * ------------------
 * This function will walk every bit of all input lines, comparing the
 * input line with it's bit-flipped version at each bit. Any bit difference
 * in the output will be recorded as the flipped bit caused that. This is
 * not necessarily true, as registered (latched) and clocked values will
 * confuse this logic.
 */
static void
walk_find_affected(uint32_t *pins_affected_by)
{
    int      bit;
    uint     line;
    uint     oline;
    uint     count        = 0;
    uint     not_deep     = 0;
    uint32_t cur_mask     = 0;
    uint32_t last_write_mask;
    uint32_t last_read_mask;
    uint32_t rdiff_mask;
    uint32_t wdiff_mask;
    uint32_t write_mask;
    uint32_t main_write_mask;
    uint32_t read_mask;

    cur_mask = 0;
    for (line = 0; line < total_lines; line++) {
        for (bit = 0; bit < 28; bit++) {
            if (ignore_mask & BIT(bit))
                continue;

            oline = line ^ BIT(bit_flip_pos[bit]);
            /* Calculate pins that were affected by this pin */
            rdiff_mask = (pld_out[line] ^ pld_out[oline]);
            if (pins_always_input & BIT(bit))
                rdiff_mask &= ~BIT(bit);
            pins_affected_by[bit] |= rdiff_mask;

            /* Verify inputs to PLD were as expected (a single bit flip) */
            wdiff_mask = (pld_in[line] ^ pld_in[oline]);
            if (wdiff_mask != BIT(bit)) {
                printf("PLD input unexpected (multiple bits differ):\n  ");
                print_binary(pld_in[line]);
                printf(" ^ Pin%u != ", bit + 1);
                print_binary(pld_in[oline]);
            }
        }
    }
}

/*
 * bit_count
 * ---------
 * Simply returns the number of '1' bits in a value. A more optimal version
 * should be written if it were called more than 32 times during a program
 * run.
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
 * print_ent
 * ---------
 * Displays a given entry of input bits and resulting output bits as
 * binary text.
 */
static void
print_ent(pi_ent_t *ent)
{
    print_binary(ent->pie_input_bits);
    printf("->%d ", ent->pie_result_bit);
    print_binary(ent->pie_affecting_bits);
    printf("\n");
}

/*
 * print_ents
 * ----------
 * Displays all entries of input bits and output bits for each pin
 * in the device. Entries are displayed in binary.
 */
static void
print_ents(void)
{
    uint bit;
    uint cur;
    for (bit = 0; bit < 32; bit++) {
        for (cur = 0; cur < pinfo[bit].pi_count; cur++) {
            if (pinfo[bit].pi_ent[cur].pie_affecting_bits == 0)
                continue;
            printf("Pin=%-6s e=%-4u ", pin_name(bit, 0), cur);
            print_ent(&pinfo[bit].pi_ent[cur]);
        }
    }
}

/*
 * print_ent_ops
 * -------------
 * Displays a single output pin and the logic required to generate that output.
 */
static void
print_ent_ops(uint32_t affecting_bits, uint32_t input_bits)
{
    uint bit;
    uint printed = 0;

    for (bit = 0; bit < 32; bit++) {
        if (affecting_bits & BIT(bit)) {
            if (printed)
                printf(" & ");
            printed = 1;
            printf("%s", pin_name(bit, (input_bits & BIT(bit)) == 0));
        }
    }
}

/*
 * print_ents_as_ops
 * -----------------
 * Displays all output pins and the logic required to generate that output.
 * Some outputs are special, such as those implementing open drain signals.
 * The current version of this code does not do a good job on anything but
 * purely combinatorial logic.
 */
static void
print_ents_as_ops(uint result_bit)
{
    uint        bit;
    uint        cur;
    uint32_t    affecting_bits;
    const char *indent = (result_bit == 0) ? "   " : "";

    for (bit = 0; bit < 32; bit++) {
        uint        search_bit = result_bit ^ pinfo[bit].pi_invert;
        const char *pname = pin_name(bit, search_bit == 0);
        uint        pname_len = strlen(pname);
        uint        printed = 0;
        for (cur = 0; cur < pinfo[bit].pi_count; cur++) {
            if (pinfo[bit].pi_ent[cur].pie_result_bit != search_bit)
                continue;
            affecting_bits = pinfo[bit].pi_ent[cur].pie_affecting_bits;
#if 1
            /*
             * XXX: This might not work correctly in all cases.
             *      Need further testing.
             */
            if ((pins_only_output_low | pins_only_output_high) & BIT(bit))
                affecting_bits &= ~BIT(bit);
#endif
            if (affecting_bits == 0)
                continue;
            if (printed == 0) {
                if (pins_only_output_high & BIT(bit)) {
                    printf("%s%s    = 'b'%u;\n", indent, pname, search_bit);
                    printf("%s%s.OE = ", indent, pname);
                    pname_len += 3;
                } else if (pins_only_output_low & BIT(bit)) {
                    printf("%s%s    = 'b'%u;\n", indent, pname, search_bit ^ 1);
                    printf("%s%s.OE = ", indent, pname);
                    pname_len += 3;
                } else {
                    printf("%s%s = ", indent, pname);
                }
            } else {
                printf("\n%s%*s # ", indent, pname_len, "");
            }
            printed = 1;
            print_ent_ops(affecting_bits,
                          pinfo[bit].pi_ent[cur].pie_input_bits);
        }
        if (printed)
            printf(";\n");
    }
}

/*
 * add_or_mask
 * -----------
 * This function simply adds a new "or" mask entry to the specified
 * pin. It automatically filters out duplicates.
 */
static void
add_or_mask(uint bit, uint bit_state, uint32_t input_bits, uint line)
{
    uint cur;

    for (cur = 0; cur < pinfo[bit].pi_count; cur++) {
        if ((pinfo[bit].pi_ent[cur].pie_input_bits == input_bits) &&
            (pinfo[bit].pi_ent[cur].pie_result_bit == bit_state)) {
            /* Duplicate -- discard */
            return;
        }
    }
#ifdef DEBUG_ADD_OR_MASK
    printf("aom bit=%u state=%u ", bit, bit_state);
    print_binary(input_bits);
    printf(" new %u\n", cur);
#endif
    if (cur >= pinfo[bit].pi_count_max) {
#ifdef DEBUG_ADD_OR_MASK
        printf("aom bit=%u state=%u ", bit, bit_state);
        print_binary(input_bits);
        printf(" %u > max count\n", cur);
#endif
#ifdef DEBUG_ADD_OR_MASK_2
        printf(" line %-5u ", line);
        print_binary(pld_in[line]);
        printf(" -> ");
        print_binary(pld_out[line]);
        printf("\n");
#endif
#ifdef DEBUG_ADD_OR_MASK
        for (int cur2 = 0; cur2 < cur; cur2++) {
            printf("       dup state=%u ",
                   pinfo[bit].pi_ent[cur2].pie_result_bit);
            print_binary(pinfo[bit].pi_ent[cur2].pie_input_bits);
            printf(" %u\n", pinfo[bit].pi_ent[cur2].pie_line);
        }
#endif
        return;
    }
    pinfo[bit].pi_ent[cur].pie_input_bits     = input_bits;
    pinfo[bit].pi_ent[cur].pie_result_bit     = bit_state;
    pinfo[bit].pi_ent[cur].pie_line           = line;
    pinfo[bit].pi_ent[cur].pie_affecting_bits = pins_affecting_pin[bit];
    pinfo[bit].pi_count++;
}

/*
 * show_counts
 * -----------
 * Displays the count of "or" entries for each pin.
 */
static void
show_counts(void)
{
    uint bit;
    for (bit = 0; bit < 32; bit++) {
        if (pinfo[bit].pi_count > 0) {
            printf("P%d count=%u\n", pin_name(bit, 0), pinfo[bit].pi_count);
        }
    }
}

/*
 * collect_or_masks
 * ----------------
 * For each output pin, record all unique masks, filtering for
 * just the relevant input pins. This step captures from the
 * global data read from the input file and stores that data
 * in per output pin. After this point, the global data read from
 * the input file is no longer used.
 *
 * There should be no input mask which duplicates another input
 * mask, yet yields a different value in the result pin. This
 * indicates an inconsistency which might be caused by an internal
 * register in the PLD which is not reflected in a pin. No code
 * checks for this right now.
 */
static void
collect_or_masks(void)
{
    uint line;
    uint bit;

    /* Allocate memory for or masks */
    for (bit = 0; bit < 32; bit++) {
        uint32_t bitcount;
        if (ignore_mask & BIT(bit))
            continue;
        if ((pins_output & BIT(bit)) == 0)
            continue;
        bitcount = bit_count(pins_affecting_pin[bit]);
#ifdef DEBUG_COLLECT_OR_MASKS
        printf("for bit=%u affecting_bitcount=%u\n", bit, bitcount);
#endif
        pinfo[bit].pi_count      = 0;
        pinfo[bit].pi_count_max  = 1 << bitcount;
        pinfo[bit].pi_ent        = malloc(sizeof (pi_ent_t) << bitcount);
    }

    for (line = 0; line < read_lines; line++) {
        uint32_t write_mask = pld_in[line];
        uint32_t read_mask  = pld_out[line];
        for (bit = 0; bit < 32; bit++) {
            if (ignore_mask & BIT(bit))
                continue;
            if ((pins_output & BIT(bit)) == 0)
                continue;
#ifdef DEBUG_LIMIT_BITS
            if ((bit & LIMIT_BITS) == 0)
                continue;
#endif
            add_or_mask(bit, !!(pld_out[line] & BIT(bit)),
                        pld_in[line] & pins_affecting_pin[bit], line);
        }
    }
}

/*
 * collapse_duplicates
 * -------------------
 * This function eliminates "or" entries from a pin's list which no longer
 * have relevant bits. This is done to speed up walks through the list.
 */
static void
collapse_duplicates(void)
{
    uint bit;

    /* Collapse all duplicates */
    for (bit = 0; bit < 32; bit++) {
        uint pi_count = pinfo[bit].pi_count;
        uint cur;
        uint scur = 0;
        for (cur = 0; cur < pi_count; cur++) {
            if (pinfo[bit].pi_ent[cur].pie_affecting_bits != 0) {
                if (cur != scur) {
                    memcpy(&pinfo[bit].pi_ent[scur], &pinfo[bit].pi_ent[cur],
                           sizeof (pi_ent_t));
                }
                scur++;
            }
        }
        pinfo[bit].pi_count = scur;
    }
}

/*
 * merge_or_masks
 * --------------
 * This function merges all expressions where bit differences don't change
 * the state of the result. This function operates on a single output pin
 * at a time.
 */
static void
merge_or_masks(void)
{
    uint bit;
    uint pin;

    for (bit = 0; bit < 32; bit++) {
        if (ignore_mask & BIT(bit))
            continue;
        if ((pins_output & BIT(bit)) == 0)
            continue;
        if (pinfo[bit].pi_count == 0)
            continue;

        /*
         * For every pin which affects this bit, walk the entire sequence
         * of bits and merge any cases where this bit is both 0 and 1,
         * yet all other bits remain the same.
         *
         * In the first pass, just clear the bits which are not relevant
         * in the affecting_bits mask. In the next stage, those duplicates
         * will be joined.
         */
        for (pin = 0; pin < 32; pin++) {
            uint scur;
            uint cur;
            uint pinmask = BIT(pin);
            if (ignore_mask & BIT(bit))
                continue;
            if ((pins_affecting_pin[bit] & BIT(pin)) == 0)
                continue;
            for (scur = 0; scur < pinfo[bit].pi_count - 1; scur++) {
                for (cur = scur + 1; cur < pinfo[bit].pi_count; cur++) {
                    if ((pinfo[bit].pi_ent[scur].pie_affecting_bits != 0) &&
                        (pinfo[bit].pi_ent[scur].pie_affecting_bits ==
                         pinfo[bit].pi_ent[cur].pie_affecting_bits) &&
                        (pinfo[bit].pi_ent[scur].pie_result_bit ==
                         pinfo[bit].pi_ent[cur].pie_result_bit) &&
                        ((pinfo[bit].pi_ent[scur].pie_input_bits & ~pinmask) ==
                         (pinfo[bit].pi_ent[cur].pie_input_bits & ~pinmask))) {
                        /*
                         * Found an entry which is identical other than the
                         * bit. Remove the duplicate entry and remove this
                         * pin from the surviving entry.
                         */
                        pinfo[bit].pi_ent[cur].pie_affecting_bits = 0;
                        pinfo[bit].pi_ent[scur].pie_affecting_bits &= ~pinmask;
                    }
                }
            }
        }
    }
    /*
     * XXX: There should be no entries where the value of the input bits
     *      is the same, but the result bit is different. Add a check for
     *      this?
     */
}

/*
 * eliminate_common_terms
 * ----------------------
 * This function walks the definition of a single output at a time,
 * eliminating common terms within that definition in order to
 * simplify the logic required to generate that output.
 */
static uint
eliminate_common_terms(void)
{
    uint pin;
    uint cur;
    uint scur;
    uint count = 0;

    /* If there are expressions such as the following:
     *
     * P1 = !P2
     *    # P3 & P2
     *    # !P3 & P4
     *    # P3 & P5
     *
     * then this can be reduced to:
     * P1 = !P2
     *    # P3             (because if !P2, then true; otherwise it's P2)
     *    # !P3 & P4
     *    # P3 & P5
     *
     * which can be reduced further:
     * P1 = !P2
     *    # P3
     *    # P4             (because if P3, then true; otherwise it's !P3)
     *                     (because if P3, then P5 is irrelevant)
     */
    for (pin = 0; pin < 32; pin++) {
        if (ignore_mask & BIT(pin))
            continue;
        if ((pins_output & BIT(pin)) == 0)
            continue;
        if (pinfo[pin].pi_count == 0)
            continue;
        for (cur = 0; cur < pinfo[pin].pi_count; cur++) {
            uint32_t top_result = pinfo[pin].pi_ent[cur].pie_result_bit;
            uint32_t top_aff    = pinfo[pin].pi_ent[cur].pie_affecting_bits;
            uint32_t top_input  = (pinfo[pin].pi_ent[cur].pie_input_bits &
                                   top_aff);

            if (top_aff == 0)
                continue;

            /* Search for these bits in other expressions */
            for (scur = 0; scur < pinfo[pin].pi_count; scur++) {
                if (scur == cur)
                    continue;
                if (pinfo[pin].pi_ent[scur].pie_result_bit != top_result)
                    continue;
                if ((pinfo[pin].pi_ent[scur].pie_affecting_bits &
                                                       top_aff) != top_aff) {
                    continue;
                }

                /*
                 * If the input bits are the same as the input bits in
                 * the top expression, then we have a case where the second
                 * expression can be eliminated.
                 * Example:                         or, more complicated
                 *    P1 = P2                       P1 = P2 & !P3
                 *       # P2 & P4                     # P2 & !P3 & P4
                 *
                 * Eliminate that second expression:
                 *    P1 = P2                       P1 = P2 & !P3
                 */
                if ((top_aff & pinfo[pin].pi_ent[scur].pie_input_bits) ==
                    top_input) {
                    /*
                     * Likely should not see reuse because that would
                     * have been filtered out in the initial analysis.
                     * However, it might happen on a subsequent pass of
                     * this function.
                     */
#ifdef DEBUG_ELIMINATE_COMMON_TERMS_1
                    printf("Found reuse of %s ", pin_name(pin, 0));
                    print_binary(top_aff);
                    printf(" from entry %u in entry %u\n", cur, scur);
#endif
                    pinfo[pin].pi_ent[scur].pie_affecting_bits = 0;
                }

                /*
                 * If the input bits are the opposite of the input bits in
                 * the top expression, then we can remove those bits from
                 * the second expression.
                 * Example:                         or, more complicated
                 *    P1 = P2                       P1 = P2 & !P3
                 *       # !P2 & P4                    # !P2 & P3 & P4
                 *
                 * We can reduce to
                 *    P1 = P2                       P1 = P2 & !P3
                 *       # P4                          # P4
                 */
                if (((top_input ^ pinfo[pin].pi_ent[scur].pie_input_bits) &
                     top_aff) == top_aff) {
#ifdef DEBUG_ELIMINATE_COMMON_TERMS_2
                    printf("Found inverse of %s ", pin_name(pin, 0));
                    print_binary(top_aff);
                    printf(" from entry %u in entry %u\n", cur, scur);
#endif
                    pinfo[pin].pi_ent[scur].pie_affecting_bits &= ~top_aff;
                    count++;
                }
            }
        }
    }
    return (count);
}

/*
 * XXX: * Further logic simplication should be possible.
 *
 * For example:
 *     (A & B) | (!A & B & C)
 *  =  B & (A | !A & C)
 *  =  B & (A | C)
 *  =  (A & B) | (B & C)
 *
 * In the above, the !A term could be removed from the second term
 * of the expression. I don't know how to describe this. Maybe that
 * if we have the first expression fully contained in the second
 * expression except one bit is different, then that bit can be
 * eliminated from the second expression.
 *
 * I think the below can be optimized using this idea
 * INTSPC = !A3 & A2 & FC1 & FC0 & CFGIN & !RST & SHUNT
 *        # !A3 & A2 & FC1 & FC0 & CFGIN & !RST & !SHUNT & !BERR
 */


/*
 * is_contained_within
 * -------------------
 * Reports whether the given element (subbit) of a binary expression is
 * fully contained within another expression (supbit).
 */
static int
is_contained_within(uint supbit, uint subbit, uint result_bit)
{
    /*
     * For each element in the subbit, if there is a matching
     * element in the supbit, then this function will return true.
     * More and/or conditions may be present in the supbit.
     */
    uint supcur;
    uint subcur;
    uint affecting = 0;
    uint matched = 0;

    for (subcur = 0; subcur < pinfo[subbit].pi_count; subcur++) {
        uint matched = 0;
        if (pinfo[subbit].pi_ent[subcur].pie_result_bit != result_bit)
            continue;
        for (supcur = 0; supcur < pinfo[supbit].pi_count; supcur++) {
            if (pinfo[supbit].pi_ent[supcur].pie_result_bit != result_bit)
                continue;
#ifdef DEBUG_IS_CONTAINED_WITHIN
            printf("Comp %u  supa=%x suba=%x  supi=%x subi=%x\n", result_bit,
                pinfo[supbit].pi_ent[supcur].pie_affecting_bits,
                pinfo[subbit].pi_ent[subcur].pie_affecting_bits,
                pinfo[supbit].pi_ent[supcur].pie_input_bits,
                pinfo[subbit].pi_ent[subcur].pie_input_bits);
#endif
            if (matched &&
                ((pinfo[supbit].pi_ent[supcur].pie_affecting_bits &
                                                affecting) != affecting)) {
                /*
                 * Affecting bits of the first match are not present in this
                 * match. Ignore this entry.
                 */
                continue;
            }
            if (((pinfo[supbit].pi_ent[supcur].pie_affecting_bits &
                  pinfo[subbit].pi_ent[subcur].pie_affecting_bits) ==
                 pinfo[subbit].pi_ent[subcur].pie_affecting_bits) &&
                ((pinfo[supbit].pi_ent[supcur].pie_input_bits &
                  pinfo[subbit].pi_ent[subcur].pie_input_bits) ==
                 pinfo[subbit].pi_ent[subcur].pie_input_bits)) {
                if (matched == 0) {
                    affecting =
                        pinfo[supbit].pi_ent[supcur].pie_affecting_bits &
                        ~pinfo[subbit].pi_ent[subcur].pie_affecting_bits;
                }
                matched = 1;
                break;
            }
        }
        if (supcur >= pinfo[supbit].pi_count) {
            /* Did not find a match */
            return (0);
        }
    }
    return (1);
}

/*
 * merge_common_subexpression
 * --------------------------
 * This function replaces subexpressions in a given output which can
 * be described by the inclusion of another output.
 */
static void
merge_common_subexpression(uint supbit, uint subbit, uint result_bit)
{
    uint matched = 0;
    /*
     * Replace all subbit subexpressions in superbit with just subbit.
     *
     * Example:
     *   P1   = P3 & P4 & P6
     *        # P5 & P6;
     *   P2   = P3 & P4
     *        # P5;
     *
     * Result (P2 is the subexpression):
     *
     *   P1   = P2 & P6
     *        # P2 & P6;
     *   P2   = P3 & P4
     *        # P5;
     *
     * As can be seen in the above, all identical expressions in P1 can
     * then be combined:
     *   P1   = P2 & P6;
     *   P2   = P3 & P4
     *        # P5;
     */
    uint supcur;
    uint subcur;

    for (subcur = 0; subcur < pinfo[subbit].pi_count; subcur++) {
        if (pinfo[subbit].pi_ent[subcur].pie_result_bit != result_bit)
            continue;   // Doesn't match the desired result
#if 0
        if ((pinfo[subbit].pi_ent[subcur].pie_affecting_bits &
            (pinfo[subbit].pi_ent[subcur].pie_affecting_bits - 1)) == 0)
            continue;   // No point in swapping -- it's only a single bit
        /* Might need the above to allow for combining logic */
#endif

        for (supcur = 0; supcur < pinfo[supbit].pi_count; supcur++) {
            if (pinfo[supbit].pi_ent[supcur].pie_result_bit != result_bit)
                continue;
            if (((pinfo[supbit].pi_ent[supcur].pie_affecting_bits &
                  pinfo[subbit].pi_ent[subcur].pie_affecting_bits) ==
                 pinfo[subbit].pi_ent[subcur].pie_affecting_bits) &&
                ((pinfo[supbit].pi_ent[supcur].pie_input_bits &
                  pinfo[subbit].pi_ent[subcur].pie_input_bits) ==
                 pinfo[subbit].pi_ent[subcur].pie_input_bits)) {
                if (matched == 0) {
                    /* First match -- reduce this expression */
                    pinfo[supbit].pi_ent[supcur].pie_affecting_bits =
                        (pinfo[supbit].pi_ent[supcur].pie_affecting_bits &
                        ~pinfo[subbit].pi_ent[subcur].pie_affecting_bits) |
                        BIT(subbit);
                    if (result_bit) {
                        pinfo[supbit].pi_ent[supcur].pie_input_bits |=
                            BIT(subbit);
                    } else {
                        pinfo[supbit].pi_ent[supcur].pie_input_bits &=
                            ~BIT(subbit);
                    }
                } else {
                    /* Subsequent match -- eliminate this expression */
                    pinfo[supbit].pi_ent[supcur].pie_affecting_bits = 0;
                }
                matched = 1;
                break;
            }
        }
    }
}

/*
 * merge_common_subexpression
 * --------------------------
 * This function walks all outputs looking for and replacing subexpressions
 * in a given output which be described by the inclusion of another output.
 */
static uint
merge_common_subexpressions(void)
{
    uint supbit;
    uint subbit;
    uint merge_count = 0;
    uint pin_state;
    /*
     * Find expressions where all conditions of PIN A are
     * fully captured by PIN B. The expression for Pin B
     * can then be simplified to include just PIN A with
     * whatever additional constraints are required by PIN A.
     * This is complicated because you could have a multiple
     * entry expression, for example:
     *
     * P25  = !P19
     *      # !P16 && !P18 && P19
     * P26  = !P19 && P26
     *      # !P16 && !P18 && P19 && P26
     *
     * the above should resolve to:
     *
     * P25  = !P19
     *      # !P16 && !P18 && P19
     * P26  = P25 && P26
     *
     * Argh! Except the above example is not right because P26 is
     * actually an open drain that should end up being
     * P26    = 'b'2;
     * P26.OE = P25;
     */

    for (supbit = 0; supbit < 32; supbit++) {
        if (ignore_mask & BIT(supbit))
            continue;
        if ((pins_output & BIT(supbit)) == 0)
            continue;
        if (pinfo[supbit].pi_count == 0)
            continue;
        for (subbit = 0; subbit < 32; subbit++) {
            if (supbit == subbit)
                continue;
            if (ignore_mask & BIT(subbit))
                continue;
            if ((pins_output & BIT(subbit)) == 0)
                continue;
            if (pinfo[subbit].pi_count == 0)
                continue;

            for (pin_state = 0; pin_state <= 1; pin_state++) {
                if (is_contained_within(supbit, subbit, pin_state)) {
#ifdef DEBUG_MERGE_COMMON_SUBEXPRESSIONS
                    printf("%s contains %s\n",
                           pin_name(supbit, !pin_state),
                           pin_name(subbit, !pin_state));
#endif
                    merge_common_subexpression(supbit, subbit, pin_state);
                    merge_count++;
                }
            }
        }
    }
    return (merge_count);
}

/*
 * analyze
 * -------
 * Walk all values read from the file and provide an analysis summary.
 * The output is nearly identical to what is available in the Brutus
 * firmware.
 */
void
analyze(void)
{
    uint     line;
    uint     bit;
    uint     pin;
    uint     printed = 0;
    uint32_t pins_touched          = 0;
    uint32_t pins_always_low       = 0xffffffff;
    uint32_t pins_always_high      = 0xffffffff;
    uint32_t pins_affected_by[32];

    pins_only_output_high = 0xffffffff;
    pins_only_output_low  = 0xffffffff;

    build_ignore_mask();
    build_bit_flip_offsets();

    memset(pins_affected_by, 0, sizeof (pins_affected_by));
    memset(pins_affecting_pin, 0, sizeof (pins_affecting_pin));

    for (line = 0; line < read_lines; line++) {
        uint32_t write_mask = pld_in[line];
        uint32_t read_mask  = pld_out[line];
        pins_touched          |= write_mask;
        pins_always_low       &= ~read_mask;
        pins_always_high      &= read_mask;
        pins_always_input     &= ~(read_mask ^ write_mask);
        pins_output           |= (read_mask ^ write_mask);
        pins_only_output_high &= (read_mask | ~write_mask);
        pins_only_output_low  &= (~read_mask | write_mask);

        /*
         * pins_always_input means that the read_mask is always the same
         * as the write_mask.
         *
         * pins_output means that the read_mask is at least sometimes
         * different than the write_mask.
         *
         * pins_only_output_high is unusual open drain, where the driver
         * can only drive high and relies on an external pull-down to
         * pull the line low when it's not driving.
         *     If read_mask is low, then it better not be the case that
         *         write_mask is high
         *
         * pins_only_output_low is typical open drain, where the driver
         * can only drive low and relies on an external pull-up to pull
         * the line high when it's not driving.
         *     If read_mask is high, then it better not be the case that
         *         write_mask is low
         */
    }

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

    walk_find_affected(pins_affected_by);
    for (bit = 0; bit < 28; bit++) {
        uint32_t mask = BIT(bit);
        uint32_t pins_affecting = 0;

        for (pin = 0; pin < 28; pin++) {
            if (pins_affected_by[pin] & mask)
                pins_affecting |= BIT(pin);
        }
        pins_affecting_pin[bit] = pins_affecting;
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

/*
 * initialize_pinfo
 * ----------------
 * Sets up default bit to pin number mappings. These numbers are
 * used when reporting pin names if a config file is not provided.
 */
static void
initialize_pinfo(void)
{
    uint pin;
    memset(pinfo, 0, sizeof (pinfo));
    for (pin = 0; pin < 32; pin++)
        pinfo[pin].pi_num = pin + 1;  // Pin number at input / output
}

int
main(int argc, char *argv[])
{
    int arg;
    const char *cap_filename = NULL;
    char line[256];
    FILE *fp;
    char *ptr;
    int line_num = 0;
    int count = 0;
    int content_type = CONTENT_UNKNOWN;

    for (arg = 1; arg < argc; arg++) {
        if (cap_filename == NULL) {
            cap_filename = argv[arg];
        } else if (cfg_filename == NULL) {
            cfg_filename = argv[arg];
        } else {
            errx(EXIT_FAILURE, "Unknown argument %s", argv[arg]);
        }
    }

    if (cap_filename == NULL)
        errx(EXIT_FAILURE, "You must specify a cap_filename to read");

    initialize_pinfo();

    read_cfg_file(cfg_filename);
    read_cap_file(cap_filename);
    analyze();
    collect_or_masks();
    merge_or_masks();
    collapse_duplicates();

    printf("after merge or masks\n");
    print_ents();
    print_ents_as_ops(1);
    print_ents_as_ops(0);

    count = 0;
    while (merge_common_subexpressions() != 0) {
        merge_or_masks();
        if (count++ > 5) {
            printf("Too many iterations merging common subexpressions\n");
            break;
        }
    }

#ifdef DEBUG_MERGE_COMMON_SUBEXPRESSIONS
    printf("after merge common subexpressions\n");
    print_ents();
    print_ents_as_ops(1);
    print_ents_as_ops(0);
    printf("\n");
#endif

    count = 0;
    while (eliminate_common_terms() > 1) {
        if (count++ > 10) {
            printf("Too many iterations eliminating single terms\n");
            break;
        }
    }
#ifdef DEBUG_ELIMINATE_COMMON_TERMS_3
    print_ents_as_ops(1);
    print_ents_as_ops(0);
#endif

    print_cfg_file();
    printf("\n");
    print_ents_as_ops(1);
    printf("/*\n"
           "   Inverted logic for reference purposes\n"
           "   -------------------------------------\n");
    print_ents_as_ops(0);
    printf("*/\n");
}
