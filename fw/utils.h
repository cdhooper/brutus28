/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2022.
 *
 * ---------------------------------------------------------------------
 *
 * Generic POSIX function emulation.
 */

#ifndef _UTILS_H
#define _UTILS_H

#define ADDR8(x)    ((uint8_t *)  ((uintptr_t)(x)))
#define ADDR16(x)   ((uint16_t *) ((uintptr_t)(x)))
#define ADDR32(x)   ((uint32_t *) ((uintptr_t)(x)))

#define MIN(x, y)   ((x) < (y) ? (x) : (y))
#define MAX(x, y)   ((x) > (y) ? (x) : (y))

#define BIT(x)      (1 << (x))
#define BITRANGE32(x, y) ((0xffffffff >> (31 - MAX(x, y))) & \
                          (0xffffffff << MIN(x, y)))

#define ARRAY_SIZE(x) (int)((sizeof (x) / sizeof ((x)[0])))

void reset_dfu(void);
void reset_cpu(void);
void reset_check(void);
void show_reset_reason(void);
void identify_cpu(void);

#endif /* _UTILS_H */
