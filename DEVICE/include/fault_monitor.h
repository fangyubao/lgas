#ifndef __FAULT_MONITOR_H
#define __FAULT_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define FAULT_MONITOR_STALL_MS 3000U

typedef enum
{
    FAULT_MONITOR_EVT_LOOP = 0U,
    FAULT_MONITOR_EVT_SYSTICK,
    FAULT_MONITOR_EVT_HARDFAULT,
    FAULT_MONITOR_EVT_MEMMANAGE,
    FAULT_MONITOR_EVT_BUSFAULT,
    FAULT_MONITOR_EVT_USAGEFAULT,
    FAULT_MONITOR_EVT_ERROR
} fault_monitor_event_t;

extern volatile uint32_t g_fault_monitor_loop_age_ms;
extern volatile uint8_t g_fault_monitor_stall_reported;

static inline void fault_monitor_kick(void)
{
    g_fault_monitor_loop_age_ms = 0U;
    g_fault_monitor_stall_reported = 0U;
}

void fault_monitor_tick(void);
void fault_monitor_capture(uint32_t event, uint32_t arg0, uint32_t arg1);

#ifdef __cplusplus
}
#endif

#endif /* __FAULT_MONITOR_H */
