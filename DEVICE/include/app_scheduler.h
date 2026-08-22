#ifndef __APP_SCHEDULER_H
#define __APP_SCHEDULER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef void (*app_scheduler_handler_t)(void);

typedef struct
{
    uint8_t enable;
    volatile uint32_t next_ms;
    uint32_t period_ms;
    app_scheduler_handler_t handler;
} app_scheduler_task_t;

void app_scheduler_tick(void);
uint32_t app_scheduler_millis(void);
void app_scheduler_set_millis(uint32_t ms);
void app_scheduler_delay_ms(uint32_t delay_ms);
void app_scheduler_run(app_scheduler_task_t *tasks, uint8_t task_count);
uint8_t app_scheduler_expired(uint32_t deadline_ms);

#ifdef __cplusplus
}
#endif

#endif 