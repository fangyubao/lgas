#include "main.h"
#include "bus485_task.h"
#include "CH4.h"
#include "dataup.h"
#include "rtc.h"
#include "rs485.h"
#include <stdio.h>
#include <string.h>
#include "C2H2.h"
#include "uart.h"
#include "H2.h"
#include "usbd_cdc_if.h"
#define BUS485_CH4_PARAM_MAX 20U
#define BUS485_RESULT_HOLD_MS 500U

typedef struct
{
    uint8_t enable;
    uint8_t proto;
    uint8_t module;
    uint8_t cmd;
    uint8_t param_len;
    uint8_t param[BUS485_CH4_PARAM_MAX];
    uint8_t state;
} bus485_task_t;

typedef struct{
    uart_baud_sel_t baud_sel;
    uint8_t _id;
}uart_dev_info_t;
uart_dev_info_t uart_dev_info[UART_PORT_COUNT] = {
    {UART_BAUD_SEL_115200,   BUS_MOD_C2H2},
    {UART_BAUD_SEL_19200 ,   BUS_MOD_H2},
    {UART_BAUD_SEL_115200  ,   BUS_MOD_CH4},
    {UART_BAUD_SEL_115200  ,   BUS_UPDATA},
};
static bus485_task_t s_bus485_task = {0U, 0U, 0U, 0U, 0U, {0U}, BUS485_IDLE};
static uint32_t s_bus485_result_deadline = 0U;

static uint8_t bus485_start(const uint8_t *req, uint8_t len);
static uint8_t bus485_get_state(void);
static void bus485_clear(void);
static uint8_t bus485_request_match(const uint8_t *req, uint8_t len);
static void bus485_run_modbus(void);
static void bus485_run_updata(void);
static void bus485_run_modbus_ch4(void);
static void bus485_run_updata_sensor(void);
static void bus485_run_modbus_c2h2(void);
static void bus485_run_modbus_H2(void);
void bus485_run(void)
{
    if (s_bus485_task.enable == 0U)
    {
        return;
    }

    switch (s_bus485_task.proto)
    {
    case BUS_MODBUS:
        bus485_run_modbus();
        break;

    case BUS_UPDATA:
        bus485_run_updata();
        break;

    default:
        s_bus485_task.state = BUS485_FAIL;
        s_bus485_task.enable = 0U;
        break;
    }

    if (((s_bus485_task.state == BUS485_DONE) ||
         (s_bus485_task.state == BUS485_FAIL)) &&
        (s_bus485_result_deadline == 0U))
    {
        s_bus485_result_deadline = app_scheduler_millis() + BUS485_RESULT_HOLD_MS;
    }
}

static uint8_t bus485_start(const uint8_t *req, uint8_t len)
{
    uint8_t param_len;

    if (s_bus485_task.enable != 0U)
    {
        return 0U;
    }

    if ((req == NULL) || (len < 3U))
    {
        char msg[64];
        sprintf(msg, "Invalid bus485 request: len=%u\r\n", len);
        usb_cdc_send_packet((uint8_t *)msg, (uint16_t)strlen(msg));
        return 0U;
    }

    param_len = (uint8_t)(len - 3U);
    if (param_len > BUS485_CH4_PARAM_MAX)
    {
        char msg[64];
        sprintf(msg, "Bus485 request param too long: len=%u\r\n", len   );
        usb_cdc_send_packet((uint8_t *)msg, (uint16_t)strlen(msg));
        return 0U;
    }

    s_bus485_task.enable = 1U;
    s_bus485_task.proto = req[0];
    s_bus485_task.module = req[1];
    s_bus485_task.cmd = req[2];
    s_bus485_task.param_len = param_len;
    if (param_len > 0U)
    {
        char msg[64];
        sprintf(msg, "Bus485 request: proto=%u, module=%u, cmd=%u, param_len=%u\r\n", s_bus485_task.proto, s_bus485_task.module, s_bus485_task.cmd, s_bus485_task.param_len);
        usb_cdc_send_packet((uint8_t *)msg, (uint16_t)strlen(msg));
        memcpy(s_bus485_task.param, &req[3], param_len);
    }
    #if 0
    for(uint8_t i=0;i<sizeof(uart_dev_info)/sizeof(uart_dev_info[0]);i++){
        if(uart_dev_info[i]._id == s_bus485_task.module){
            if(uart_getBaudRate() != uart_dev_info[i].baud_sel){
                if (uart_setBaudRate((uart_port_t)UART_PORT_USART2, uart_dev_info[i].baud_sel) != HAL_OK)
                {
                    HAL_Delay(50);
                    Error_Handler();
                    return 0U;
                }
            }
            break;
        }
    }
    #endif
    s_bus485_task.state = BUS485_BUSY;
    return 1U;
}

uint8_t bus485_request(const uint8_t *req, uint8_t len)
{
    uint8_t state = bus485_get_state();

    if ((req == NULL) || (len < 3U) ||
        ((uint8_t)(len - 3U) > BUS485_CH4_PARAM_MAX))
    {
        return BUS485_FAIL;
    }

    if (state == BUS485_BUSY)
    {
        return BUS485_BUSY;
    }
    if ((state == BUS485_DONE) || (state == BUS485_FAIL))
    {
        if (bus485_request_match(req, len) != 0U)
        {
            bus485_clear();
            return state;
        }

        /* Preserve a completed result briefly for the state machine that
         * started it.  Without this, an unrelated USB/HMI request executed
         * earlier in the main loop can discard the owner's DONE/FAIL result. */
        if ((s_bus485_result_deadline != 0U) &&
            (app_scheduler_expired(s_bus485_result_deadline) == 0U))
        {
            return BUS485_BUSY;
        }
        bus485_clear();
        
    }

    if (bus485_start(req, len) == 0U)
    {
        if (s_bus485_task.state == BUS485_FAIL)
        {
            return BUS485_FAIL;
        }
        return BUS485_BUSY;
    }
    return BUS485_BUSY;
}

static uint8_t bus485_get_state(void)
{
    return s_bus485_task.state;
}

static void bus485_clear(void)
{
    s_bus485_task.enable = 0U;
    s_bus485_task.proto = 0U;
    s_bus485_task.module = 0U;
    s_bus485_task.cmd = 0U;
    s_bus485_task.param_len = 0U;
    memset(s_bus485_task.param, 0, sizeof(s_bus485_task.param));
    s_bus485_task.state = BUS485_IDLE;
    s_bus485_result_deadline = 0U;
}

static uint8_t bus485_request_match(const uint8_t *req, uint8_t len)
{
    uint8_t param_len;

    if ((req == NULL) || (len < 3U))
    {
        return 0U;
    }

    param_len = (uint8_t)(len - 3U);
    if ((s_bus485_task.proto != req[0]) ||
        (s_bus485_task.module != req[1]) ||
        (s_bus485_task.cmd != req[2]) ||
        (s_bus485_task.param_len != param_len))
    {
        return 0U;
    }

    if (param_len == 0U)
    {
        return 1U;
    }

    return (memcmp(s_bus485_task.param, &req[3], param_len) == 0) ? 1U : 0U;
}

static void bus485_run_modbus(void)
{
    switch (s_bus485_task.module)
    {
    case BUS_MOD_CH4:
        bus485_run_modbus_ch4();
        break;
    case BUS_MOD_C2H2:
        bus485_run_modbus_c2h2();
    break;

    case BUS_MOD_H2:
        bus485_run_modbus_H2();
    break;
    
    default:
        s_bus485_task.state = BUS485_FAIL;
        s_bus485_task.enable = 0U;
        break;
    }
}

static void bus485_run_updata(void)
{
    switch (s_bus485_task.module)
    {
    case BUS_UPDATA_SENSOR:
        bus485_run_updata_sensor();
        break;

    default:
        s_bus485_task.state = BUS485_FAIL;
        s_bus485_task.enable = 0U;
        break;
    }
}

static void bus485_run_modbus_ch4(void)
{
    uint8_t req[1U + BUS485_CH4_PARAM_MAX];
    uint8_t ret;
    CH4_t ch4_data;

    req[0] = s_bus485_task.cmd;
    if (s_bus485_task.param_len > 0U)
    {
        memcpy(&req[1], s_bus485_task.param, s_bus485_task.param_len);
    }

    ret = CH4_exec(req, &ch4_data);
    s_bus485_task.state = (ret == CH4_RUN_BUSY) ? BUS485_BUSY :
                          (ret == CH4_RUN_DONE) ? BUS485_DONE : BUS485_FAIL;
    if (ret != CH4_RUN_BUSY)
    {
        s_bus485_task.enable = 0U;
    }
}

static void bus485_run_modbus_c2h2(void)
{
    uint8_t ret;
    ret = C2H2_Getdata(&s_bus485_task.cmd);
    s_bus485_task.state = (ret == C2H2_RUN_BUSY) ? BUS485_BUSY :
                          (ret == C2H2_RUN_DONE) ? BUS485_DONE : BUS485_FAIL;
    if (ret != C2H2_RUN_BUSY)
    {
        s_bus485_task.enable = 0U;

    }    
}

static void bus485_run_modbus_H2(void)
{
    uint8_t ret;
    ret = H2_Getdata(&s_bus485_task.cmd);
    s_bus485_task.state = (ret == H2_RUN_BUSY) ? BUS485_BUSY :
                          (ret == H2_RUN_DONE) ? BUS485_DONE : BUS485_FAIL;
    if (ret != H2_RUN_BUSY)    {
        s_bus485_task.enable = 0U;

    }
}
static void bus485_run_updata_sensor(void)
{
    char tx_buf[128];
    int str_len;
    tx_buf[0]=0x05;
    tx_buf[1]=0x10;
    uint8_t cmd = s_bus485_task.cmd;
    if (rtc_get_datetime((rtc_datetime_t *)&sensordata.time_data) != HAL_OK)
    {
        s_bus485_task.state = BUS485_FAIL;
        s_bus485_task.enable = 0U;
        return;
    }
    switch(cmd){
        case sCH4: 
            str_len = snprintf(&tx_buf[2],
                        sizeof(tx_buf)-2,
                        "time:%04u-%02u-%02u %02u:%02u:%02u,CH4:%.3fppm\r\n",
                        sensordata.time_data.year,
                        sensordata.time_data.month,
                        sensordata.time_data.date,
                        sensordata.time_data.hour,
                        sensordata.time_data.min,
                        sensordata.time_data.sec,
                        (double)sensordata.CH4);
        break;
        case sC2H6:
            str_len = snprintf(&tx_buf[2],
                sizeof(tx_buf)-2,
                "time:%04u-%02u-%02u %02u:%02u:%02u,C2H6:%.3fppm\r\n",
                sensordata.time_data.year,
                sensordata.time_data.month,
                sensordata.time_data.date,
                sensordata.time_data.hour,
                sensordata.time_data.min,
                sensordata.time_data.sec,
                (double)sensordata.C2H6);
        break;
        case sH2:
            str_len = snprintf(&tx_buf[2],
                sizeof(tx_buf)-2,
                "time:%04u-%02u-%02u %02u:%02u:%02u,H2:%.3fppm\r\n",
                sensordata.time_data.year,
                sensordata.time_data.month,
                sensordata.time_data.date,
                sensordata.time_data.hour,
                sensordata.time_data.min,
                sensordata.time_data.sec,
                (double)sensordata.H2);
        break;
        case sC2H2:
            str_len = snprintf(&tx_buf[2],
                sizeof(tx_buf)-2,
                "time:%04u-%02u-%02u %02u:%02u:%02u,C2H2:%.3fppm\r\n",
                sensordata.time_data.year,
                sensordata.time_data.month,
                sensordata.time_data.date,
                sensordata.time_data.hour,
                sensordata.time_data.min,
                sensordata.time_data.sec,
                (double)sensordata.C2H2);
        break;
        default:
            str_len = snprintf(&tx_buf[2],
                sizeof(tx_buf)-2,
                "time:%04u-%02u-%02u %02u:%02u:%02u,NO GAS\r\n",
                sensordata.time_data.year,
                sensordata.time_data.month,
                sensordata.time_data.date,
                sensordata.time_data.hour,
                sensordata.time_data.min,
                sensordata.time_data.sec);
        break;
    }

    if ((str_len <= 0) || (str_len > (int)(sizeof(tx_buf) - 2U)) ||
        (rs485_write((const uint8_t *)tx_buf, (uint16_t)(str_len + 2), 1000U) != HAL_OK))
    {
        s_bus485_task.state = BUS485_FAIL;
    }
    else
    {
        s_bus485_task.state = BUS485_DONE;
    }
    #ifdef DEBUG_MODE
    usb_cdc_send_packet((const uint8_t *)tx_buf,(uint16_t)(str_len + 2));
    #endif
    s_bus485_task.enable = 0U;
}
