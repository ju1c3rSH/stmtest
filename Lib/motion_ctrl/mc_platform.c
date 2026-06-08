/**
 * @file    mc_platform.c
 * @brief   Platform abstraction storage
 */

#include "mc_platform.h"

static mc_hal_t s_hal;
static uint8_t  s_hal_registered = 0;

void mc_hal_register(const mc_hal_t *hal)
{
    if (hal)
    {
        s_hal = *hal;
        s_hal_registered = 1;
    }
}

const mc_hal_t *mc_hal_get(void)
{
    return s_hal_registered ? &s_hal : (void *)0;
}
