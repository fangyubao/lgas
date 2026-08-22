#include "loop_profiler.h"

#include "main.h"
#include "usbd_cdc_if.h"

#include <stdio.h>

typedef struct
{
    uint32_t print_period_ms;
    volatile uint32_t print_tick;
    uint32_t begin_cycle;
    uint32_t last_cycles;
    uint8_t initialized;
} loop_profiler_t;

static loop_profiler_t g_loop_profiler = {0U, 0U, 0U, 0U, 0U};

void loop_profiler_init(uint32_t print_period_ms)
{
    g_loop_profiler.print_period_ms = print_period_ms;
    g_loop_profiler.print_tick = print_period_ms;
    g_loop_profiler.begin_cycle = 0U;
    g_loop_profiler.last_cycles = 0U;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0U;

    g_loop_profiler.initialized = 1U;
}

void loop_profiler_taskDelay(void)
{
    if ((g_loop_profiler.initialized != 0U) && (g_loop_profiler.print_tick > 0U))
    {
        g_loop_profiler.print_tick--;
    }
}

void loop_profiler_begin(void)
{
    if (g_loop_profiler.initialized == 0U)
    {
        return;
    }

    g_loop_profiler.begin_cycle = DWT->CYCCNT;
}

void loop_profiler_end(void)
{
    char usb_buf[64];
    uint32_t loop_us;
    int len;

    if (g_loop_profiler.initialized == 0U)
    {
        return;
    }

    g_loop_profiler.last_cycles = DWT->CYCCNT - g_loop_profiler.begin_cycle;

    if (g_loop_profiler.print_tick > 0U)
    {
        return;
    }

    g_loop_profiler.print_tick = g_loop_profiler.print_period_ms;
    loop_us = g_loop_profiler.last_cycles / (SystemCoreClock / 1000000U);
    len = snprintf(usb_buf, sizeof(usb_buf), "main while: %lu us\r\n", (unsigned long)loop_us);
    if (len > 0)
    {
        usb_cdc_send_packet((uint8_t *)usb_buf, (uint16_t)len);
    }
}
