/*
 * Copyright 2020 Chris Hooper, All rights reserved.
 * -------------------------------------------------
 *
 * This program source is public domain and may be used for any purpose.
 *
 * Compiling on Linux:
 *     cc -O2 -o /home/cdh/bin/term term.c -lpthread
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <signal.h>
#include <string.h>
#include <getopt.h>
#include <limits.h>
#include <pthread.h>
#include <errno.h>
#include <err.h>
#include <poll.h>
#include <unistd.h>

/* Enable for gdb debug */
#undef DEBUG_CTRL_C_KILL

/* Enable for non-blocking tty input */
#undef USE_NON_BLOCKING_TTY

#ifndef EXIT_USAGE
#define EXIT_USAGE 2
#endif

#ifdef __APPLE__
#define MAC_OS
#endif

/*
 * ARRAY_SIZE() - Provide a count of the number of elements in an array.
 * This macro works the same as the Linux kernel header definition of the
 * same name.
 */
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) ((size_t) (sizeof (array) / sizeof ((array)[0])))
#endif

/*
 * Path to tty FTDI info:
 *     /sys/class/tty/ttyUSB0/../../../../serial
 *          example: AM01F9T1
 *     /sys/class/tty/ttyUSB0/../../../../idVendor
 *          example: 0403
 *     /sys/class/tty/ttyUSB0/../../../../idProduct
 *          example: 6001
 *     /sys/class/tty/ttyUSB0/../../../../uevent
 *          has DEVNAME (example: "bus/usb/001/031")
 *          has BUSNUM (example: "001")
 *          has DEVNUM (example: "031")
 *
 * Also, for all serial devices,:
 *     ls /dev/serial/by-id/
 */

enum {
    CH_FLOW_HW=1,
    CH_FLOW_SW,
    CH_FLOW_NO,
    CH_IC_DELAY,
};

/** Program long format options */
static const struct option long_opts[] = {
    { "help",     no_argument,       NULL, 'h' },
    { "bits",     required_argument, NULL, 'b' },
    { "capture",  required_argument, NULL, 'c' },
    { "hwflow",   no_argument,       NULL, CH_FLOW_HW },
    { "swflow",   no_argument,       NULL, CH_FLOW_SW },
    { "noflow",   no_argument,       NULL, CH_FLOW_NO },
    { "icdelay",  no_argument,       NULL, CH_IC_DELAY },
    { "parity",   required_argument, NULL, 'p' },
    { "rts",      required_argument, NULL, 'r' },
    { "speed",    required_argument, NULL, 's' },
    { "stopbits", required_argument, NULL, 't' },
    { NULL,       no_argument,       NULL,  0  }
};

static char short_opts[] = {
    ':',         // Missing argument
    CH_FLOW_HW,  // --hwflow
    CH_FLOW_SW,  // --swflow
    CH_FLOW_NO,  // --noflow
    'b', ':',    // --bits 5|6|7|8
    'c', ':',    // --capture <filename>
    'D', ':',    // --ic_delay <num>
    'h',         // --help
    'p', ':',    // --parity odd|even|none
    'r', ':',    // --rts <num>
    's', ':',    // --speed <num>
    't', ':',    // --stopbits 0|1
    '\0'
};

/** Program help text */
static const char usage_text[] =
"term <opts> <dev>\n"
"    -b --bits <num>         data bits: 5, 6, 7, or 8 (default)\n"
"    -c --capture <filename> capture output to a file\n"
"    -D --delay              pacing delay between sent characters (ms)\n"
"    -h --help               display usage\n"
"       --hwflow             hardware flow control\n"
"       --swflow             software flow control\n"
"       --noflow             no flow control (default)\n"
"    -p --parity <num>       even, odd, or none (default)\n"
"    -r --rts <num>          drive RTS (0=low, 1=high)\n"
"    -s --speed <num>        specify bps rate (115200 default)\n"
"    -t --stopbits <num>     stop bits: 1 (default) or 2\n"
"\n"
"Specify the TTY name to open\n"
"Example:\n"
"    term /dev/ttyACM0\n"
"    term -s 9600 /dev/ttyUSB0\n"
"";

typedef unsigned int uint;

typedef enum {
    RC_SUCCESS = 0,
    RC_FAILURE = 1,
} rc_t;

typedef enum {
    TRUE  = 1,
    FALSE = 0,
} bool_t;

#define INPUT_RING_SIZE 32
static uint8_t         input_rb[INPUT_RING_SIZE];
static uint            input_rb_producer = 0;
static uint            input_rb_consumer = 0;
static int             dev_fd            = -1;
static int             got_terminfo      = 0;
static volatile uint   got_input         = 0;
static int             running           = 1;
static uint            ic_delay          = 0;  // Pacing delay (ms)
static uint            serial_speed      = 115200;
static uint            serial_bits       = 8;  // 8 bits
static uint            serial_parity     = 0;  // No parity
static uint            serial_stop_bits  = 1;  // 1 stop bit
static char            flow              = CH_FLOW_NO;
static char           *capture_file      = NULL;
static char            device_name[PATH_MAX];
static struct termios  saved_term;  // good terminal settings

typedef struct {
    uint    bps;
    speed_t speed;
} serial_speed_t;

static const serial_speed_t serial_speeds[] = {
    { 300, B300 },
    { 600, B600 },
    { 1200, B1200 },
    { 2400, B2400 },
    { 4800, B4800 },
    { 9600, B9600 },
    { 19200, B19200 },
    { 38400, B38400 },
    { 57600, B57600 },
    { 115200, B115200 },
#ifdef B230400
    { 230400, B230400 },
#endif
#ifdef B460800
    { 460800, B460800 },
#endif
#ifdef B921600
    { 921600, B921600 },
#endif
#ifdef B1000000
    { 1000000, B1000000 },
#endif
#ifdef B1500000
    { 1500000, B1500000 },
#endif
#ifdef B2000000
    { 2000000, B2000000 },
#endif
#ifdef B2500000
    { 2500000, B2500000 },
#endif
#ifdef B3000000
    { 3000000, B3000000 },
#endif
#ifdef B3500000
    { 3500000, B3500000 },
#endif
#ifdef B4000000
    { 4000000, B4000000 },
#endif
};

static speed_t
lookup_speed(uint bps)
{
    size_t pos;

    for (pos = 0; pos < ARRAY_SIZE(serial_speeds); pos++)
        if (serial_speeds[pos].bps == bps)
            return (serial_speeds[pos].speed);

    return (0);
}

static uint
atou(const char *str)
{
    uint value;
    if (sscanf(str, "%u", &value) != 1)
        errx(EXIT_FAILURE, "'%s' is not an integer value", str);
    return (value);
}

/**
 * usage() displays command usage.
 *
 * This function requires no arguments.
 *
 * @return      None.
 */
static void
usage(FILE *fp)
{
    (void) fputs(usage_text, fp);
}

/*
 * input_rb_put() stores a next character in the tty input ring buffer.
 *
 * @param [in]  ch - The character to store in the tty input ring buffer.
 *
 * @return      None.
 */
static void
input_rb_put(int ch)
{
    uint new_prod = (input_rb_producer + 1) % sizeof (input_rb);

    if (new_prod == input_rb_consumer)
        return;  // Discard input because ring buffer is full

    input_rb[input_rb_producer] = (uint8_t) ch;
    input_rb_producer = new_prod;
}

/*
 * input_rb_get() returns the next character in the tty input ring buffer.
 *                A value of -1 is returned if there are no characters waiting
 *                to be received in the tty input ring buffer.
 *
 * This function requires no arguments.
 *
 * @return      The next input character.
 * @return      -1 = No input character is pending.
 */
static int
input_rb_get(void)
{
    int ch;

    if (input_rb_consumer == input_rb_producer)
        return (-1);  // Ring buffer empty

    ch = input_rb[input_rb_consumer];
    input_rb_consumer = (input_rb_consumer + 1) % sizeof (input_rb);
    return (ch);
}

/*
 * input_rb_space() returns a count of the number of characters remaining
 *                  in the input ring buffer before the buffer is completely
 *                  full.  A value of 0 means the buffer is already full.
 */
static uint
input_rb_space(void)
{
    uint diff = input_rb_consumer - input_rb_producer;
    return (diff + sizeof (input_rb) - 1) % sizeof (input_rb);
}


/**
 * time_delay_msec() - Delay for specified milliseconds.
 *
 * @param [in]  msec - Milliseconds from now.
 *
 * @return      None.
 */
static void
time_delay_msec(int msec)
{
    if (poll(NULL, 0, msec) < 0)
        warn("poll() failed");
}


static rc_t
config_dev(int fd)
{
    speed_t        sspeed;
    struct termios tty;

    if (flock(fd, LOCK_EX | LOCK_NB) < 0)
        warnx("Failed to get exclusive lock on %s", device_name);

#ifdef MAC_OS
    /* Disable non-blocking */
    if (fcntl(fd, F_SETFL, 0) < 0)
        warnx("Failed to enable blocking on %s", device_name);
#endif

    sspeed = lookup_speed(serial_speed);
    if (sspeed == 0) {
        warnx("Unsupported speed %u", serial_speed);
        close(fd);
        return (RC_FAILURE);
    }

    (void) memset(&tty, 0, sizeof (tty));

    if (tcgetattr(fd, &tty) != 0) {
        /* Failed to get terminal information */
        warn("Failed to get tty info for %s", device_name);
        close(fd);
        return (RC_FAILURE);
    }

#undef DEBUG_TTY
#ifdef DEBUG_TTY
    printf("tty: pre  c=%x i=%x o=%x l=%x\n",
           tty.c_cflag, tty.c_iflag, tty.c_oflag, tty.c_lflag);
#endif

    if (cfsetispeed(&tty, sspeed) ||
        cfsetospeed(&tty, sspeed)) {
        warn("failed to set %s speed to %d BPS", device_name, serial_speed);
        close(fd);
        return (RC_FAILURE);
    }

    if (flow == CH_FLOW_HW)
        tty.c_cflag |= CRTSCTS;               // hw flow on
    else
        tty.c_cflag &= ~CRTSCTS;              // hw flow off

    tty.c_iflag &= IXANY;
    if (flow == CH_FLOW_SW)
        tty.c_iflag |= IXON | IXOFF;          // sw flow on
    else
        tty.c_iflag &= (IXON | IXOFF);        // sw flow off

    tty.c_cflag &= (uint)~CSIZE;              // no bits
    switch (serial_bits) {
        case 5:
            tty.c_cflag |= CS5;               // 5 bits
            break;
        case 6:
            tty.c_cflag |= CS6;               // 6 bits
            break;
        case 7:
            tty.c_cflag |= CS7;               // 7 bits
            break;
        default:
        case 8:
            tty.c_cflag |= CS8;               // 8 bits
            break;
    }

    tty.c_cflag &= (uint)~(PARENB | PARODD);  // no parity
    tty.c_cflag |= serial_parity;             // add in even or odd parity
    if (serial_stop_bits == 1)
        tty.c_cflag &= (uint)~CSTOPB;         // one stop bit
    else
        tty.c_cflag |= CSTOPB;                // two stop bits

    tty.c_iflag  = IGNBRK;                    // raw, no echo
    tty.c_lflag  = 0;
    tty.c_oflag  = 0;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~ECHOPRT;                  // CR is not newline

    tty.c_cc[VINTR]    = 0;  // Ctrl-C
    tty.c_cc[VQUIT]    = 0;  // Ctrl-Backslash
    tty.c_cc[VERASE]   = 0;  // Del
    tty.c_cc[VKILL]    = 0;  // @
    tty.c_cc[VEOF]     = 4;  // Ctrl-D
    tty.c_cc[VTIME]    = 0;  // Inter-character timer unused
    tty.c_cc[VMIN]     = 1;  // Blocking read until 1 character arrives
#ifdef VSWTC
    tty.c_cc[VSWTC]    = 0;  // '\0'
#endif
    tty.c_cc[VSTART]   = 0;  // Ctrl-Q
    tty.c_cc[VSTOP]    = 0;  // Ctrl-S
    tty.c_cc[VSUSP]    = 0;  // Ctrl-Z
    tty.c_cc[VEOL]     = 0;  // '\0'
    tty.c_cc[VREPRINT] = 0;  // Ctrl-R
    tty.c_cc[VDISCARD] = 0;  // Ctrl-u
    tty.c_cc[VWERASE]  = 0;  // Ctrl-W
    tty.c_cc[VLNEXT]   = 0;  // Ctrl-V
    tty.c_cc[VEOL2]    = 0;  // '\0'

#ifdef DEBUG_TTY
    printf("tty: post c=%x i=%x o=%x l=%x cc=%02x %02x %02x %02x\n",
           tty.c_cflag, tty.c_iflag, tty.c_oflag, tty.c_lflag,
           tty.c_cc[0], tty.c_cc[1], tty.c_cc[2], tty.c_cc[3]);
#endif
    if (tcsetattr(fd, TCSANOW, &tty)) {
        warn("failed to set %s attributes", device_name);
        close(fd);
        return (RC_FAILURE);
    }
    return (RC_SUCCESS);
}

static void
reopen_dev(void)
{
    int           temp      = dev_fd;
    static time_t last_time = 0;
    time_t        now       = time(NULL);
    bool_t        printed   = FALSE;
    int           oflags    = O_NOCTTY;

#ifdef MAC_OS
    oflags |= O_NONBLOCK;
#endif

    dev_fd = -1;
    if (temp != -1) {
        if (flock(temp, LOCK_UN | LOCK_NB) < 0)
            warnx("Failed to release exclusive lock on %s", device_name);
        close(temp);
    }
    if (now - last_time > 5) {
        printed = TRUE;
        printf("\n<< Closed %s >>", device_name);
        fflush(stdout);
    }
top:
    do {
        if (running == 0)
            return;
        time_delay_msec(400);
    } while ((temp = open(device_name, oflags | O_RDWR)) == -1);

    if (config_dev(temp) != RC_SUCCESS)
        goto top;

    /* Hand off the new I/O fd */
    dev_fd = temp;

    now = time(NULL);
    if (now - last_time > 5) {
        if (printed == FALSE)
            printf("\n");
        printf("\r<< Reopened %s >>\n", device_name);
    }
    last_time = now;
}

/**
 * th_serial_reader() is a thread to read from serial port and store it in
 *                    a circular buffer.  The buffer's contents are retrieved
 *                    asynchronously by another thread.
 *
 * @param [in]  arg - Unused argument.
 *
 * @return      NULL pointer (unused)
 *
 * @see         serial_in_snapshot(), serial_in_count(), serial_in_advance(),
 *              serial_in_flush()
 */
static void *
th_serial_reader(void *arg)
{
    int         ch;
    const char *log_file;
    FILE       *log_fp = NULL;
    FILE       *capture_fp = NULL;

    if ((log_file = getenv("TERM_DEBUG")) != NULL) {
        /*
         * Examples:
         *     TERM_DEBUG=/dev/pts/4 term /dev/ttyUSB0
         *     TERM_DEBUG=/tmp/term_debug term /dev/ttyUSB0
         */
        log_fp = fopen(log_file, "w");
        if (log_fp == NULL)
            warn("Unable to open %s for log", log_file);
    }

    if (capture_file != NULL) {
        capture_fp = fopen(capture_file, "w");
        if (capture_fp == NULL)
            warn("Unable to open %s for capture", capture_file);
    }

    while (running) {
        ssize_t len;
        while ((len = read(dev_fd, &ch, 1)) >= 0) {
            if (len == 0) {
#ifdef USE_NON_BLOCKING_TTY
                /* No input available */
                time_delay_msec(10);
                continue;
#else
                /* Error reading */
                break;
#endif
            }
            if (running == 0)
                break;
            fputc(ch, stdout);
            fflush(stdout);
            if (log_fp != NULL) {
                fprintf(log_fp, "%c", ch);
                fflush(log_fp);
            }
            if (capture_fp != NULL) {
                fprintf(capture_fp, "%c", ch);
                fflush(capture_fp);
            }
            got_input++;
        }
        if (running == 0)
            break;
        reopen_dev();
    }
    printf("not running\n");

    if (log_fp != NULL)
        fclose(log_fp);
    if (capture_fp != NULL)
        fclose(capture_fp);
    return (NULL);
}

/**
 * th_serial_writer() is a thread to read from the tty input ring buffer and
 *                    write data to the serial port.  The separation of tty
 *                    input from serial writes allows the program to still be
 *                    responsive to user interaction even when blocked on
 *                    serial writes.
 *
 * @param [in]  arg - Unused argument.
 *
 * @return      NULL pointer (unused)
 *
 * @see         serial_in_snapshot(), serial_in_count(), serial_in_advance(),
 *              serial_in_flush()
 */
static void *
th_serial_writer(void *arg)
{
    int ch;
    uint pos = 0;
    char lbuf[64];

    while (running) {
        ch = input_rb_get();
        if (ch >= 0)
            lbuf[pos++] = ch;
        if (((ch < 0) && (pos > 0)) ||
             (pos >= sizeof (lbuf)) || (ic_delay != 0)) {
            ssize_t count;
            if (dev_fd == -1) {
                time_delay_msec(500);
                continue;
            } else if ((count = write(dev_fd, lbuf, pos)) < 0) {
                /* Wait for reader thread to close / reopen */
                time_delay_msec(500);
                continue;
            } else if (ic_delay) {
                /* Inter-character pacing delay was specified */
                time_delay_msec(ic_delay);
            }
            if (count < pos) {
                printf("sent only %ld of %u\n", count, pos);
            }
            pos = 0;
        } else if (ch < 0) {
            time_delay_msec(10);
        }
    }
    return (NULL);
}

/**
 * serial_open() initializes a serial port for communication with a device.
 *
 * This function requires no arguments.
 *
 * @return      None.
 */
static rc_t
serial_open(bool_t verbose)
{
    struct termios tty;
    int            oflags = O_NOCTTY;

#ifdef MAC_OS
    oflags |= O_NONBLOCK;
#endif

    /* First verify the file exists */
    dev_fd = open(device_name, oflags | O_RDONLY);
    if (dev_fd == -1) {
        warn("Failed to open %s for read", device_name);
        return (RC_FAILURE);
    }
    close(dev_fd);

    dev_fd = open(device_name, oflags | O_RDWR);
    if (dev_fd == -1) {
        warn("Failed to open %s for write", device_name);
        return (RC_FAILURE);
    }
    return (config_dev(dev_fd));
}

/**
 * set_rts() sets the state of the hardware RTS signal.
 *
 * @param [in]  rts_value - 1 for RTS=1; 0 for RTS=0.
 *
 * @return      RC_SUCCESS - Operation succeeded.
 * @return      RC_FAILURE - Operation failed.
 */
static int
set_rts(int rts_value)
{
    int cm_status;

    if (ioctl(dev_fd, TIOCMGET, &cm_status) == -1) {
        warn("set_rts() failed TIOCMSET");
        return (RC_FAILURE);
    }
    if (rts_value)
        cm_status |= TIOCM_RTS;
    else
        cm_status &= ~TIOCM_RTS;

    if (ioctl(dev_fd, TIOCMSET, &cm_status) == -1) {
        warn("set_rts() failed TIOCMSET");
        return (RC_FAILURE);
    }
    return (RC_SUCCESS);
}

/**
 * at_exit_func() cleans up the terminal.  This function is necessary because
 *                the terminal is put in raw mode in order to receive
 *                non-blocking character input which is not echoed to the
 *                console.  It is necessary because some libdevaccess
 *                functions may exit on a fatal error.
 *
 * This function requires no arguments.
 *
 * @return      None.
 */
static void
at_exit_func(void)
{
    if (got_terminfo) {
        got_terminfo = 0;
        tcsetattr(0, TCSANOW, &saved_term);
    }
}

/**
 * do_exit() exits gracefully.
 *
 * @param [in]  rc - The exit code with which to terminate the program.
 *
 * @return      This function does not return.
 */
static void __attribute__((noreturn))
do_exit(int rc)
{
    putchar('\n');
    exit(rc);
}

static void
sig_exit(int sig)
{
    do_exit(EXIT_FAILURE);
}

static void
create_threads(void)
{
    pthread_attr_t thread_attr;
    pthread_t      thread_id;

    /* Create thread */
    pthread_attr_init(&thread_attr);
    pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED);
    if (pthread_create(&thread_id, &thread_attr, th_serial_reader, NULL))
        err(EXIT_FAILURE, "failed to create %s reader thread", device_name);
    if (pthread_create(&thread_id, &thread_attr, th_serial_writer, NULL))
        err(EXIT_FAILURE, "failed to create %s writer thread", device_name);
}

/**
 * main() is the entry point of the term utility.
 *
 * @param [in]  argc - Count of user arguments.
 * @param [in]  argv - Array of user arguments.
 *
 * @return      EXIT_USAGE   - Command argument invalid.
 * @return      EXIT_FAILURE - Command failed.
 * @return      EXIT_SUCCESS - Command completed.
 */
int
main(int argc, char * const *argv)
{
    struct termios   term;
    int              arg;
    int              ch;
    int              drive_rts  = -1;
    int              enable     = 1;
    int              long_index = 0;
    bool_t           literal    = FALSE;
    struct sigaction sa;

    memset(&sa, 0, sizeof (sa));
    sa.sa_handler = sig_exit;
    (void) sigaction(SIGTERM, &sa, NULL);
    (void) sigaction(SIGINT,  &sa, NULL);
    (void) sigaction(SIGQUIT, &sa, NULL);
    (void) sigaction(SIGPIPE, &sa, NULL);

    device_name[0] = '\0';

    while ((ch = getopt_long(argc, argv, short_opts, long_opts,
                             &long_index)) != EOF) {
        switch (ch) {
            case ':':
                warnx("The -%c flag requires an argument", optopt);
                usage(stderr);
                exit(EXIT_FAILURE);
                break;
            case 'b':
                serial_bits = atou(optarg);
                if ((serial_bits < 5) || (serial_bits > 8))
                    errx(EXIT_FAILURE, "invalid bits '%s'", optarg);
                break;
            case 'c':
                capture_file = optarg;
                break;
            case CH_IC_DELAY:
            case 'D':
                ic_delay = atou(optarg);
                break;
            case CH_FLOW_HW:
                flow = CH_FLOW_HW;
                break;
            case CH_FLOW_SW:
                flow = CH_FLOW_SW;
                break;
            case CH_FLOW_NO:
                flow = CH_FLOW_NO;
                break;
            case 'h':
            case '?':
                usage(stdout);
                exit(EXIT_SUCCESS);
                break;
            case 'p':
                if (strcasecmp(optarg, "odd") == 0)
                    serial_parity = PARENB | PARODD;
                else if (strcasecmp(optarg, "even") == 0)
                    serial_parity = PARENB;
                else if (strcasecmp(optarg, "none") == 0)
                    serial_parity = 0;
                else
                    errx(EXIT_FAILURE, "invalid parity '%s'", optarg);
                break;
            case 's':
                serial_speed = atou(optarg);
                break;
            case 't':
                serial_stop_bits = atou(optarg);
                if ((serial_stop_bits < 1) || (serial_stop_bits > 2))
                    errx(EXIT_FAILURE, "invalid stop bits '%s'", optarg);
                break;
            case 'r':
                drive_rts = atou(optarg);
                break;
            default:
                warnx("Unknown option -%c %x", ch, ch);
                usage(stderr);
                exit(EXIT_USAGE);
        }
    }

    argc -= optind;
    argv += optind;

    for (arg = 0; arg < argc; arg++) {
        if (argc == 2)
            errx(EXIT_USAGE, "Too many arguments: %s", argv[arg]);

        strcpy(device_name, argv[arg]);
    }

    if (device_name[0] == '\0') {
        warnx("You must specify a device to open");
        usage(stderr);
        exit(EXIT_USAGE);
    }

    if (isatty(fileno(stdin))) {
        if (tcgetattr(0, &saved_term))
            errx(EXIT_FAILURE, "Could not get terminal information");

        got_terminfo = 1;

        term = saved_term;
        cfmakeraw(&term);
        term.c_oflag |= OPOST;
#ifdef DEBUG_CTRL_C_KILL
        term.c_lflag |= ISIG;   // Enable to not trap ^C
#endif
        tcsetattr(0, TCSANOW, &term);
#ifdef USE_NON_BLOCKING_TTY
        if (ioctl(fileno(stdin), FIONBIO, &enable))  // Set input non-blocking
            warn("FIONBIO failed for stdin");
#endif
    }

    atexit(at_exit_func);

    if (serial_open(TRUE) != RC_SUCCESS)
        do_exit(EXIT_FAILURE);

    if (drive_rts >= 0) {
        set_rts(drive_rts);
    } else if (flow == CH_FLOW_HW) {
        set_rts(1);
        time_delay_msec(1);
        set_rts(0);
    }

    create_threads();

    if (isatty(fileno(stdin)))
        printf("<< Type ^X to exit.  Opened %s >>\n", device_name);

    while (running) {
        int ch = 0;
        ssize_t len;

        while (input_rb_space() == 0)
            time_delay_msec(20);

        if ((len = read(0, &ch, 1)) <= 0) {
            if (len == 0) {
                /* End of input */
                fprintf(stderr, "EOF - waiting for output to end\n");
                do {
                    got_input = 0;
                    time_delay_msec(400);
                } while (got_input);
                do_exit(EXIT_SUCCESS);
            }
            if (errno != EAGAIN) {
                warn("read failed");
                do_exit(EXIT_FAILURE);
            }
            ch = -1;
        }
#ifdef USE_NON_BLOCKING_TTY
        if (ch == 0) {                   // EOF
            time_delay_msec(400);
            do_exit(EXIT_SUCCESS);
        }
#endif
        if (literal == TRUE) {
            literal = FALSE;
            input_rb_put(ch);
            continue;
        }
        if (ch == 0x16) {                  // ^V
            literal = TRUE;
            continue;
        }

        if (ch == 0x18)  // ^X
            do_exit(EXIT_SUCCESS);

        if (ch >= 0)
            input_rb_put(ch);
    }
    printf("not running\n");
    running = 0;
}

/*
 * TODO
 *
 * th_serial_writer() should block on cv rather than spin waiting for
 *                    input to appear in the ring buffer.
 *
 * Use inotify to tell when another process has the device_name open.
 * Or... use flock() on the device_name.
 *
 * Add an option to capture to file
 */
