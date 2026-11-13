/* Host build of fw/capture.c: stand-in for fw/cmdline.h */
#ifndef _CMDLINE_H
#define _CMDLINE_H
typedef enum {
    RC_SUCCESS   = 0,
    RC_FAILURE   = 1,
    RC_USER_HELP = 2,
    RC_USR_ABORT = 3,
    RC_BAD_PARAM = 6,
} rc_t;
#endif
