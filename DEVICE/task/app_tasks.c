#include "app_tasks.h"

#include "app_scheduler.h"
#include "bus485_task.h"
#include "dataup.h"
#include "dgusii.h"
#include "fault_monitor.h"
#include "HMI_task.h"
#include "usb_task.h"

#define APP_TASK_COUNT(tasks) ((uint8_t)(sizeof(tasks) / sizeof((tasks)[0])))

static void app_task_fault_monitor(void);
static void app_task_usb(void);
static void app_task_bus485(void);
static void app_task_dataup(void);
static void app_task_dgusii(void);
static void app_task_hmi(void);

static app_scheduler_task_t s_running_tasks[] =
{
    {1U, 0U, 0U,  app_task_fault_monitor},
    {1U, 0U, 0U,  app_task_usb},
    {1U, 0U, 0U,  app_task_bus485},
    {1U, 0U, 50U, app_task_dataup},
    {1U, 0U, 0U,  app_task_dgusii},
    {1U, 0U, 0U,  app_task_hmi}
};

static app_scheduler_task_t s_stop_tasks[] =
{
    {1U, 0U, 0U,  app_task_usb},
    {1U, 0U, 0U,  app_task_bus485},
    {1U, 0U, 50U, app_task_dataup},
    {1U, 0U, 0U,  app_task_dgusii},
    {1U, 0U, 0U,  app_task_hmi}
};

void app_tasks_run_running(void)
{
    app_scheduler_run(s_running_tasks, APP_TASK_COUNT(s_running_tasks));
}

void app_tasks_run_stop(void)
{
    app_scheduler_run(s_stop_tasks, APP_TASK_COUNT(s_stop_tasks));
}

static void app_task_fault_monitor(void)
{
    fault_monitor_kick();
}

static void app_task_usb(void)
{
    usb_task_process();
}

static void app_task_bus485(void)
{
    bus485_run();
}

static void app_task_dataup(void)
{
    (void)updata_sensor();
}

static void app_task_dgusii(void)
{
    dgusii_poll();
}

static void app_task_hmi(void)
{
    HMI_task_poll();
}