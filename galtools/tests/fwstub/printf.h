/* Host build of fw/capture.c: printf goes to the harness output buffer */
#ifndef _PRINTF_H
#define _PRINTF_H
int harness_printf(const char *fmt, ...);
#define printf harness_printf
#endif
