//Delay
#pragma once

#include <hardware_declarations.h>
#include <Timers.h>

/*
 * Blocking millisecond delay using Timers_GetMilliSeconds().
 * Handles uint32_t wraparound correctly via unsigned subtraction.
 */
static void delay_ms(uint32_t ms)
{
    uint32_t start = Timers_GetMilliSeconds();
    while ((Timers_GetMilliSeconds() - start) < ms);
}
