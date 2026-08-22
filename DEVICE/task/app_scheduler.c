#include "app_scheduler.h"
#include <stddef.h>

static volatile uint32_t s_app_scheduler_ms = 0U;
static app_scheduler_task_t *s_current_task = NULL;

void app_scheduler_tick(void)
{
    s_app_scheduler_ms++;
}

uint32_t app_scheduler_millis(void)
{
    return s_app_scheduler_ms;
}

void app_scheduler_set_millis(uint32_t ms)
{
    s_app_scheduler_ms = ms;
}

uint8_t app_scheduler_expired(uint32_t deadline_ms)
{
    return ((int32_t)(app_scheduler_millis() - deadline_ms) >= 0) ? 1U : 0U;
}

void app_scheduler_delay_ms(uint32_t delay_ms)
{
    if (s_current_task != NULL)
    {
        s_current_task->next_ms = app_scheduler_millis() + delay_ms;
    }
}

void app_scheduler_run(app_scheduler_task_t *tasks, uint8_t task_count)
{
    uint8_t i;
    uint32_t now_ms = app_scheduler_millis();

    if (tasks == NULL)
    {
        return;
    }

    for (i = 0U; i < task_count; i++)
    {
        if ((tasks[i].enable == 0U) || (tasks[i].handler == NULL) ||
            (app_scheduler_expired(tasks[i].next_ms) == 0U))
        {
            continue;
        }

        s_current_task = &tasks[i];
        tasks[i].handler();
        if ((tasks[i].period_ms > 0U) && (app_scheduler_expired(tasks[i].next_ms) != 0U))
        {
            tasks[i].next_ms = now_ms + tasks[i].period_ms;
        }
        s_current_task = NULL;
    }
}