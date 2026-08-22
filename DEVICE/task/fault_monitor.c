#include "fault_monitor.h"

#include "main.h"
#include "uart.h"

#include <stdio.h>
#include <string.h>

#define FAULT_MONITOR_MAGIC    0x464C5444UL

typedef struct
{
    uint32_t magic;
    uint32_t type;
    uint32_t tick;
    uint32_t exc_return;
    uint32_t msp;
    uint32_t psp;
    uint32_t r[13];
    uint32_t lr;
    uint32_t pc;
    uint32_t xpsr;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t afsr;
    uint32_t shcsr;
    uint32_t icsr;
    uint32_t heartbeat_ms;
} fault_monitor_dump_t;

volatile uint32_t g_fault_monitor_r4_r11[8];
volatile uint32_t g_fault_monitor_loop_age_ms = 0U;
volatile uint8_t g_fault_monitor_stall_reported = 0U;
static volatile fault_monitor_dump_t s_fault_dump = {0U};
static volatile uint8_t s_fault_monitor_fault_reported = 0U;

static const char *fault_monitor_type_name(uint32_t type)
{
    switch ((fault_monitor_event_t)type)
    {
    case FAULT_MONITOR_EVT_HARDFAULT:
        return "HardFault";

    case FAULT_MONITOR_EVT_MEMMANAGE:
        return "MemManage";

    case FAULT_MONITOR_EVT_BUSFAULT:
        return "BusFault";

    case FAULT_MONITOR_EVT_USAGEFAULT:
        return "UsageFault";

    case FAULT_MONITOR_EVT_ERROR:
        return "Error";

    default:
        return "LoopStall";
    }
}

static void fault_monitor_send_line(const char *text)
{
    if (text == NULL)
    {
        return;
    }

    (void)uart_sendData(UART_PORT_USART1, (const uint8_t *)text, (uint16_t)strlen(text), 200U);
}

static void fault_monitor_emit_dump(void)
{
    char line[160];
    int len;

    len = snprintf(line, sizeof(line),
                   "\r\n[FAULT] %s tick=%lu beat=%lu exc=0x%08lX\r\n",
                   fault_monitor_type_name(s_fault_dump.type),
                   (unsigned long)s_fault_dump.tick,
                   (unsigned long)s_fault_dump.heartbeat_ms,
                   (unsigned long)s_fault_dump.exc_return);
    if (len > 0)
    {
        fault_monitor_send_line(line);
    }

    len = snprintf(line, sizeof(line),
                   "R0=%08lX R1=%08lX R2=%08lX R3=%08lX\r\n",
                   (unsigned long)s_fault_dump.r[0],
                   (unsigned long)s_fault_dump.r[1],
                   (unsigned long)s_fault_dump.r[2],
                   (unsigned long)s_fault_dump.r[3]);
    if (len > 0)
    {
        fault_monitor_send_line(line);
    }

    len = snprintf(line, sizeof(line),
                   "R4=%08lX R5=%08lX R6=%08lX R7=%08lX\r\n",
                   (unsigned long)s_fault_dump.r[4],
                   (unsigned long)s_fault_dump.r[5],
                   (unsigned long)s_fault_dump.r[6],
                   (unsigned long)s_fault_dump.r[7]);
    if (len > 0)
    {
        fault_monitor_send_line(line);
    }

    len = snprintf(line, sizeof(line),
                   "R8=%08lX R9=%08lX R10=%08lX R11=%08lX\r\n",
                   (unsigned long)s_fault_dump.r[8],
                   (unsigned long)s_fault_dump.r[9],
                   (unsigned long)s_fault_dump.r[10],
                   (unsigned long)s_fault_dump.r[11]);
    if (len > 0)
    {
        fault_monitor_send_line(line);
    }

    len = snprintf(line, sizeof(line),
                   "R12=%08lX LR=%08lX PC=%08lX xPSR=%08lX\r\n",
                   (unsigned long)s_fault_dump.r[12],
                   (unsigned long)s_fault_dump.lr,
                   (unsigned long)s_fault_dump.pc,
                   (unsigned long)s_fault_dump.xpsr);
    if (len > 0)
    {
        fault_monitor_send_line(line);
    }

    len = snprintf(line, sizeof(line),
                   "MSP=%08lX PSP=%08lX CFSR=%08lX HFSR=%08lX\r\n",
                   (unsigned long)s_fault_dump.msp,
                   (unsigned long)s_fault_dump.psp,
                   (unsigned long)s_fault_dump.cfsr,
                   (unsigned long)s_fault_dump.hfsr);
    if (len > 0)
    {
        fault_monitor_send_line(line);
    }

    len = snprintf(line, sizeof(line),
                   "MMFAR=%08lX BFAR=%08lX AFSR=%08lX SHCSR=%08lX ICSR=%08lX\r\n",
                   (unsigned long)s_fault_dump.mmfar,
                   (unsigned long)s_fault_dump.bfar,
                   (unsigned long)s_fault_dump.afsr,
                   (unsigned long)s_fault_dump.shcsr,
                   (unsigned long)s_fault_dump.icsr);
    if (len > 0)
    {
        fault_monitor_send_line(line);
    }
}

static void fault_monitor_capture_fault(uint32_t event, const uint32_t *stack_frame, uint32_t exc_return)
{
    uint32_t i;

    memset((void *)&s_fault_dump, 0, sizeof(s_fault_dump));
    s_fault_dump.magic = FAULT_MONITOR_MAGIC;
    s_fault_dump.type = event;
    s_fault_dump.tick = HAL_GetTick();
    s_fault_dump.exc_return = exc_return;
    s_fault_dump.msp = __get_MSP();
    s_fault_dump.psp = __get_PSP();
    s_fault_dump.cfsr = SCB->CFSR;
    s_fault_dump.hfsr = SCB->HFSR;
    s_fault_dump.mmfar = SCB->MMFAR;
    s_fault_dump.bfar = SCB->BFAR;
    s_fault_dump.afsr = SCB->AFSR;
    s_fault_dump.shcsr = SCB->SHCSR;
    s_fault_dump.icsr = SCB->ICSR;
    s_fault_dump.heartbeat_ms = g_fault_monitor_loop_age_ms;

    for (i = 0U; i < 8U; i++)
    {
        s_fault_dump.r[4U + i] = g_fault_monitor_r4_r11[i];
    }

    if (stack_frame != NULL)
    {
        s_fault_dump.r[0] = stack_frame[0];
        s_fault_dump.r[1] = stack_frame[1];
        s_fault_dump.r[2] = stack_frame[2];
        s_fault_dump.r[3] = stack_frame[3];
        s_fault_dump.r[12] = stack_frame[4];
        s_fault_dump.lr = stack_frame[5];
        s_fault_dump.pc = stack_frame[6];
        s_fault_dump.xpsr = stack_frame[7];
    }
    else
    {
        s_fault_dump.lr = exc_return;
    }
}

static void fault_monitor_capture_stall(void)
{
    memset((void *)&s_fault_dump, 0, sizeof(s_fault_dump));
    s_fault_dump.magic = FAULT_MONITOR_MAGIC;
    s_fault_dump.type = 0xFFFFFFFFUL;
    s_fault_dump.tick = HAL_GetTick();
    s_fault_dump.msp = __get_MSP();
    s_fault_dump.psp = __get_PSP();
    s_fault_dump.cfsr = SCB->CFSR;
    s_fault_dump.hfsr = SCB->HFSR;
    s_fault_dump.mmfar = SCB->MMFAR;
    s_fault_dump.bfar = SCB->BFAR;
    s_fault_dump.afsr = SCB->AFSR;
    s_fault_dump.shcsr = SCB->SHCSR;
    s_fault_dump.icsr = SCB->ICSR;
    s_fault_dump.heartbeat_ms = g_fault_monitor_loop_age_ms;
}

void fault_monitor_tick(void)
{
    if (g_fault_monitor_stall_reported != 0U)
    {
        return;
    }

    if (g_fault_monitor_loop_age_ms < FAULT_MONITOR_STALL_MS)
    {
        g_fault_monitor_loop_age_ms++;
        return;
    }

    if (s_fault_monitor_fault_reported != 0U)
    {
        return;
    }

    fault_monitor_capture_stall();
    g_fault_monitor_stall_reported = 1U;
    fault_monitor_emit_dump();
}

void fault_monitor_capture(uint32_t event, uint32_t arg0, uint32_t arg1)
{
    fault_monitor_capture_fault(event, (const uint32_t *)arg0, arg1);
    s_fault_monitor_fault_reported = 1U;
    fault_monitor_emit_dump();
}
