#ifndef SEEKFREE_SCCB_COMPAT_H
#define SEEKFREE_SCCB_COMPAT_H
#include <assert.h>
#include "seekfree_soft_iic.h"
#define zf_assert(condition) assert(condition)
#define gpio_high(pin) gpio_set((PIN_enum)(pin), 1U)
#define gpio_low(pin) gpio_set((PIN_enum)(pin), 0U)
#define gpio_get_level(pin) gpio_get((PIN_enum)(pin))
#define gpio_set_dir(pin, direction, config) gpio_dir((PIN_enum)(pin), (direction))
#define GPO_OPEN_DTAIN (GPIO_PIN_CONFIG | OD_EN)
#define GPO_PUSH_PULL GPIO_PIN_CONFIG
#define GPI_FLOATING_IN GPIO_PIN_CONFIG
#endif
