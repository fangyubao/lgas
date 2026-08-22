#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "rtc.h"
#include "fan.h"
#include "pump.h"
#include "uart.h"
#include "modbus_master.h"
#include "rs485.h"
#include "stats.h"
#include "usb_task.h"
#include "main.h"
#include "CH4.h"
#include "C2H6.h"
#include "bus485_task.h"
#include "DAC.h"
#include "solenoid.h"
#include "dg408.h"
#include "dataup.h"
#include "C2H2.h"
#include "H2.h"
#include "dgusii.h"
#include "HMI_task.h"
#include "start_task.h"
#include "flash_storage.h"
#include "dev_addr.h"
/* Poll flags are manipulated only as masks.  Keep an explicit byte instead
 * of relying on implementation-defined C bit-field layout. */
static uint8_t usb_poll;
static rtc_datetime_t rtc_data = {2026, 3, 10, 11, 7, 23};
volatile usb_delay_t usb_delay = {0U, IDLE, IDLE};
static usb_task_t usb_task;
static uint8_t usb_fail_code;
static uint32_t _cmd_error = 0;

static uint8_t usb_485_id;
static uint8_t c2h6_temp[10];

static uint8_t usb_cmd_set_time(void);
static uint8_t usb_cmd_get_sysinfo(void);
static uint8_t usb_cmd_get_time(void);
static uint8_t usb_cmd_set_baudrate(void);
static uint8_t usb_cmd_setCH4id(void);
static uint8_t usb_cmd_getCH4info(void);
static uint8_t usb_cmd_getC2H6data(void);
static uint8_t usb_cmd_setDAC(void);
static uint8_t usb_cmd_setFANduty(void);
static uint8_t usb_cmd_setPUMPduty(void);
static uint8_t usb_cmd_setSOLduty(void);
static uint8_t usb_cmd_getC2H2data(void);
static uint8_t usb_cmd_getCH4conc(void);
static uint8_t usb_cmd_getC2H2conc(void);
static uint8_t usb_cmd_getC2H6conc(void);
static uint8_t usb_cmd_setDG408add(void);
static uint8_t usb_cmd_getH2conc(void);
static uint8_t usb_cmd_getH2temp(void);
static uint8_t usb_cmd_intomenu(void);
static uint8_t usb_cmd_touch_ctrl(void);
static uint8_t usb_cmd_base(void);
static uint8_t usb_cmd_fixFLASHbase(void);
static uint8_t usb_cmd_read_param(void);
static uint8_t usb_cmd_clear_param(void);
static uint8_t usb_cmd_read_log(void);
static uint8_t usb_cmd_write_param(void);
static uint8_t usb_cmd_erase_log(void);
static uint8_t usb_cmd_read_log_raw(void);
static uint8_t usb_is_bcd(uint8_t value);
static uint8_t _usb_cmd_getC2H6data(void);
static uint8_t _cmd_getCH4info(void);
static uint8_t _usb_cmd_setCH4id(void);
static uint8_t _usb_cmd_getC2H2data(void);
static uint8_t _usb_cmd_getH2temp(void);
static uint8_t _usb_cmd_intomenu(void);
static const usb_cmd_entry_t *usb_find_cmd(uint8_t cmd);
static uint8_t usb_fail(uint8_t f_id);
static void usb_task_delay_to(uint32_t delay_ms, uint8_t next_status);

static void usb_cmd_oder(void);
static void usb_cmd_polling(void);

static uint8_t (*usb_poll_fun[])(void)={
    _cmd_getCH4info,
    _usb_cmd_setCH4id,
    _usb_cmd_getC2H6data,
    _usb_cmd_getC2H2data,
    _usb_cmd_getH2temp,
    _usb_cmd_intomenu,
};
static const usb_cmd_entry_t usb_cmd_table[] =
{
    {TASK_SETTIME, 9U, 300U, UT_SUCCESS, 0U, usb_cmd_set_time},
    {TASK_SYSINFO, 2U, 300U, UT_SUCCESS, 0U, usb_cmd_get_sysinfo},
    {TASK_GETTIME, 2U, 300U, UT_SUCCESS, 0U, usb_cmd_get_time},
    {TASK_SETBAUD, 4U, 300U, UT_SUCCESS, 0U, usb_cmd_set_baudrate},
    {TASK_CH4INFO, 2U, 300U, IDLE,       1U, usb_cmd_getCH4info},
    {TASK_CH4ID  , 3U, 300U, IDLE,       1U, usb_cmd_setCH4id},
    {TASK_C2H6  ,  10U,100U, IDLE,       1U, usb_cmd_getC2H6data},
    {TASK_DAC   ,  4U, 50U,  UT_SUCCESS, 0U, usb_cmd_setDAC},
    {TASK_FAN   ,  3U, 50U,  UT_SUCCESS, 0U, usb_cmd_setFANduty},
    {TASK_PUMP  ,  3U, 50U,  UT_SUCCESS, 0U, usb_cmd_setPUMPduty},
    {TASK_SOL  ,   3U, 50U,  UT_SUCCESS, 0U, usb_cmd_setSOLduty},
    {TASK_DG408,   3U, 50U,  UT_SUCCESS, 0U, usb_cmd_setDG408add},
    {TASK_C2H2 ,   2U, 300U, IDLE,       1U, usb_cmd_getC2H2data},
    {TASK_CH4C,    2U, 100,  UT_SUCCESS, 1U, usb_cmd_getCH4conc},
    {TASK_C2H2C,   2U, 100,  UT_SUCCESS, 1U, usb_cmd_getC2H2conc},
    {TASK_C2H6C,   2U, 100,  UT_SUCCESS, 1U, usb_cmd_getC2H6conc},
    {TASK_H2C ,    2U, 100,  IDLE,       1U, usb_cmd_getH2conc},
    {TASK_H2TEMP,  2U, 100,  IDLE,       1U, usb_cmd_getH2temp},
    {TASK_MENU,    2U, 100,  IDLE,       1u, usb_cmd_intomenu},
    {TASK_TOUCH,   3U, 50U,  UT_SUCCESS, 0U, usb_cmd_touch_ctrl},
    {TASK_BASE,    2U, 50U,  UT_SUCCESS, 0U, usb_cmd_base},
    {TASK_FLASHLOG,6U, 50U,  UT_SUCCESS, 0U, usb_cmd_fixFLASHbase},
    {TASK_READ_PARAM,2U, 50U, UT_SUCCESS, 0U, usb_cmd_read_param},
    {TASK_CLEAR_PARAM,2U, 50U, UT_SUCCESS, 0U, usb_cmd_clear_param},
    {TASK_READ_LOG,  2U, 50U, UT_SUCCESS, 0U, usb_cmd_read_log},
    {TASK_WRITE_PARAM,11U,50U, UT_SUCCESS, 0U, usb_cmd_write_param},
    {TASK_ERASE_LOG,   2U,50U, UT_SUCCESS, 0U, usb_cmd_erase_log},
    {TASK_READ_LOG_RAW,2U,50U, UT_SUCCESS, 0U, usb_cmd_read_log_raw},
};

static uint8_t usb_task_is_idle(void)
{
    if (usb_delay.usb_status != IDLE) return 0U;
    if (usb_poll != 0U) return 0U;
    if (getUSB_rev_state() != 0U) return 0U;
    return 1U;
}

void usb_task_process(void)
{
    switch (usb_delay.usb_status)
    {
        case IDLE:           
            usb_cmd_oder();
            usb_cmd_polling();
            if(_cmd_error!=0)
            {
                char _cmd_Einfo[32];
                sprintf(_cmd_Einfo, "USB fail%2lX \r\n", (unsigned long)_cmd_error);
                _cmd_error = 0;
                usb_cdc_send_packet((uint8_t *)_cmd_Einfo,(uint16_t)strlen(_cmd_Einfo));
            }
            break;

        case DELAY:
            if (ndelay_expired(&usb_delay) != 0U)
            {
                usb_delay.usb_status = usb_delay.next_id;
            }
            break;

        case UT_SUCCESS:
#ifdef DEBUG_MODE
        {
            char usb_ok_buf[64];
            sprintf(usb_ok_buf, "USB Task ID %02X success\r\n", usb_task.usb_task_id);
            usb_cdc_send_packet((uint8_t *)usb_ok_buf, (uint16_t)strlen(usb_ok_buf));
        }
#endif
            usb_delay.usb_status = IDLE;
            break;

        case UT_FAIL:
            usb_fail(usb_fail_code);
            usb_delay.usb_status = IDLE;
            break;

        default:
            usb_delay.usb_status = IDLE;
            break;
    }
    // if (usb_task_is_idle())
    // {
    //     __WFI();
    // }
}

static void usb_cmd_oder(void)
{
    if (getUSB_rev_state())
    {
        static const usb_cmd_entry_t *cmd_entry;

        usb_task.temp_usb_len = getUSB_rx_buflen();

        if ((usb_task.temp_usb_buf != NULL) && (usb_task.temp_usb_len >= 2U) &&
            (usb_task.temp_usb_buf[0] == USB_HEAD))
        {
            usb_task.usb_task_id = usb_task.temp_usb_buf[1];
            cmd_entry = usb_find_cmd(usb_task.usb_task_id);

            if (cmd_entry == 0)
            {
                usb_fail_code = ECODE;
                usb_delay.next_id = UT_FAIL;
                usb_delay.usb_status = UT_FAIL;
            }
            else if (usb_task.temp_usb_len < cmd_entry->min_len)
            {
                usb_fail_code = LEN_F;
                usb_task_delay_to(50U, UT_FAIL);
            }
            else
            {
                usb_fail_code = cmd_entry->handler();
                if (usb_fail_code == 0U)
                {
                    if (cmd_entry->async_mode != 0U)
                    {
                        usb_delay.usb_status = IDLE;
                    }
                    else
                    {
                        usb_task_delay_to(cmd_entry->delay_ms, cmd_entry->next_id);
                    }
                }
                else
                {
                    usb_task_delay_to(50U, UT_FAIL);
                }
            }
        }

        cleanUSB_rev_data();
    }
}
uint8_t usb_task_init(void)
{
    usb_task.temp_usb_buf = USB_rx_buffer();
    usb_task.temp_usb_len = 0U;
    if (usb_task.temp_usb_buf == NULL)
    {
        usb_cdc_send_packet((const uint8_t *)"USB task init fail\r\n", 20U);
        return (uint8_t)-1;
    }
    return 0U;
}

static void usb_task_delay_to(uint32_t delay_ms, uint8_t next_status)
{
    usb_delay.usb_status = next_status;
    app_scheduler_delay_ms(delay_ms);
}

void ndelay_ms(volatile usb_delay_t *task,uint32_t t_ms, uint8_t next_id)
{
    if (task == NULL)
    {
        return;
    }
    task->usb_status = DELAY;
    task->usb_tick = app_scheduler_millis() + t_ms;
    task->next_id = next_id;
}

uint8_t ndelay_expired(volatile usb_delay_t *task)
{
    if (task == NULL)
    {
        return 1U;
    }
    return ((int32_t)(app_scheduler_millis() - task->usb_tick) >= 0) ? 1U : 0U;
}

static uint8_t usb_cmd_set_time(void)
{
    uint16_t year_temp;

    if ((usb_task.temp_usb_buf == NULL) || (usb_task.temp_usb_len != 9U))
    {
        return RTC_F;
    }

    for (uint8_t i = 2U; i < 9U; i++)
    {
        if (usb_is_bcd(usb_task.temp_usb_buf[i]) == 0U)
        {
            return RTC_F;
        }
    }

    year_temp = (uint16_t)bcd2dec(usb_task.temp_usb_buf[2]) * 100U + bcd2dec(usb_task.temp_usb_buf[3]);
    rtc_data.year = year_temp;
    rtc_data.month = bcd2dec(usb_task.temp_usb_buf[4]);
    rtc_data.date = bcd2dec(usb_task.temp_usb_buf[5]);
    rtc_data.hour = bcd2dec(usb_task.temp_usb_buf[6]);
    rtc_data.min = bcd2dec(usb_task.temp_usb_buf[7]);
    rtc_data.sec = bcd2dec(usb_task.temp_usb_buf[8]);

    if ((rtc_data.month > 12U) || (rtc_data.month < 1U) ||
        (rtc_data.date < 1U) || (rtc_data.date > 31U) ||
        (rtc_data.hour > 23U) || (rtc_data.min > 59U) || (rtc_data.sec > 59U))
    {
        return RTC_F;
    }

    return (rtc_set_datetime(&rtc_data) == HAL_OK) ? 0U : RTC_F;
}

static uint8_t usb_is_bcd(uint8_t value)
{
    return ((((value >> 4U) & 0x0FU) <= 9U) && ((value & 0x0FU) <= 9U)) ? 1U : 0U;
}

static uint8_t usb_cmd_get_sysinfo(void)
{
    char *sys_info = get_sysVersion();

    usb_cdc_send_packet((uint8_t *)sys_info, (uint16_t)strlen(sys_info));
    return 0U;
}

static uint8_t usb_cmd_get_time(void)
{
    rtc_datetime_t now;
    char usb_buf[64];
    int len;

    if (rtc_get_datetime(&now) != HAL_OK)
    {
        return RTC_F;
    }

    len = snprintf(usb_buf, sizeof(usb_buf),
                   "%04u-%02u-%02u %02u:%02u:%02u\r\n",
                   now.year, now.month, now.date,
                   now.hour, now.min, now.sec);
    if (len <= 0)
    {
        return ECODE;
    }

    usb_cdc_send_packet((uint8_t *)usb_buf, (uint16_t)len);
    return 0U;
}

static const usb_cmd_entry_t *usb_find_cmd(uint8_t cmd)
{
    uint16_t i;

    for (i = 0U; i < (sizeof(usb_cmd_table) / sizeof(usb_cmd_table[0])); i++)
    {
        if (usb_cmd_table[i].cmd == cmd)
        {
            return &usb_cmd_table[i];
        }
    }

    return 0;
}

static uint8_t usb_cmd_set_baudrate(void)
{
    if(usb_task.temp_usb_buf[2]>3||usb_task.temp_usb_buf[3]<1||usb_task.temp_usb_buf[3]>8)
    {
        return BAUD;
    }
    return (uart_setBaudRate((uart_port_t)usb_task.temp_usb_buf[2], (uart_baud_sel_t)usb_task.temp_usb_buf[3]) == HAL_OK) ? 0U : BAUD;
}

uint8_t cmd_set_enable(uint8_t p, uint8_t en)
{
    if (p >= 8) return 1;
    
    en ? (usb_poll |= (uint8_t)(1U << p)) : (usb_poll &= (uint8_t)~(1U << p));
    return 0;
}

static uint8_t cmd_get_enable(uint8_t p)
{
    return (uint8_t)(usb_poll & (uint8_t)(1U << p));
}
static uint8_t usb_cmd_getCH4info(void){
    return cmd_set_enable(CMD_CH4INFO,TRUE);
}
static uint8_t usb_cmd_getC2H6data(void){
    uint8_t payload_len;

    if ((usb_task.temp_usb_buf == NULL) || (usb_task.temp_usb_len < 3U))
    {
        return C2H6_INIT;
    }

    payload_len = usb_task.temp_usb_buf[2];
    if (((uint16_t)payload_len + 3U > usb_task.temp_usb_len) ||
        ((uint16_t)payload_len + 1U > sizeof(c2h6_temp)))
    {
        return C2H6_INIT;
    }

    C2H6_reset();
    for(uint8_t i=0;i<=payload_len;i++)
    {
        c2h6_temp[i]=usb_task.temp_usb_buf[2+i];
    }
    return cmd_set_enable(CMD_C2H6,TRUE);
}

static uint8_t usb_cmd_setDAC(void){
    uint8_t hi;
    uint8_t lo;
    uint16_t dac_v;

    hi = usb_task.temp_usb_buf[2];
    lo = usb_task.temp_usb_buf[3];

    if (((hi & 0x0FU) > 9U) || (((hi >> 4) & 0x0FU) > 9U) ||
        ((lo & 0x0FU) > 9U) || (((lo >> 4) & 0x0FU) > 9U))
    {
        return BAUD;
    }

    dac_v = (uint16_t)bcd2dec(hi) * 100U + bcd2dec(lo);
    return DAC_OutputSetMv(dac_v);
}
static uint8_t usb_cmd_setCH4id(void){
    if(usb_task.temp_usb_buf[2]>247||usb_task.temp_usb_buf[2]<1){
        char info[64];
        sprintf(info,"buf=%d",usb_task.temp_usb_buf[2]);
        usb_cdc_send_packet((uint8_t *)info, (uint16_t)strlen(info));
        return CH4_ID ;
    }
    usb_485_id = usb_task.temp_usb_buf[2];
    return cmd_set_enable(CMD_CH4ID,TRUE);
}

static uint8_t usb_cmd_setFANduty(void)
{
    if(usb_task.temp_usb_buf[2]>100U)
        return 1;
    else
        return fan_set_speed(usb_task.temp_usb_buf[2])==HAL_OK?0:1;
}

static uint8_t usb_cmd_setPUMPduty(void)
{
    if(usb_task.temp_usb_buf[2]>100U)
        return 1;
    else
        return pump_set_strength(usb_task.temp_usb_buf[2])==HAL_OK?0:1;
}

static uint8_t usb_cmd_setSOLduty(void)
{
    if(usb_task.temp_usb_buf[2]>100U)
        return 1;
    else
        return solenoid_set_duty(usb_task.temp_usb_buf[2])==HAL_OK?0:1;
}

static uint8_t usb_cmd_setDG408add(void){
	return dg408_select(usb_task.temp_usb_buf[2])==HAL_OK?0:1;
}

static uint8_t usb_cmd_getC2H2data(void){
    
    return cmd_set_enable(CMD_C2H2,TRUE);
}

static uint8_t usb_cmd_getC2H2conc(void){
    dataup_set_enable(TRUE,sC2H2);
    return 0;
}

static uint8_t usb_cmd_getC2H6conc(void){
    dataup_set_enable(TRUE,sC2H6);
    return 0;  
}

static uint8_t usb_cmd_getH2conc(void){
    dataup_set_enable(TRUE,sH2);
    return 0;  
}

static uint8_t usb_cmd_getH2temp(void){
    return cmd_set_enable(CMD_H2TEMP,TRUE);
}

static uint8_t usb_cmd_getCH4conc(void){
    dataup_set_enable(TRUE,sCH4);
    return 0;
}

static uint8_t usb_cmd_intomenu(void){
    cmd_set_enable(CMD_MENU,TRUE);
    return 0;
}

static uint8_t usb_cmd_touch_ctrl(void)
{
    if (usb_task.temp_usb_buf[2] == 0x01U)
    {
        HMI_openTouch();
        return 0U;
    }
    if (usb_task.temp_usb_buf[2] == 0x02U)
    {
        HMI_closeTouch();
        return 0U;
    }
    return ECODE;
}

static uint8_t usb_cmd_base(void)
{
    char buf[256];
	sprintf(buf,"--------baseInfo: conc || std-----------\r\n\
        H2=%f,%f\r\nC2H2=%f,%f\r\nC2H6=%f,%f\r\n0CH4=%f,%f\
        \r\n----------------\r\n",base_data.h2.value,base_data.h2.std[0],base_data.c2h2.value,base_data.c2h2.std[0],base_data.c2h6.value,base_data.c2h6.std[0],base_data.ch4.value,base_data.ch4.std[0]);
    usb_cdc_send_packet((uint8_t*)buf,strlen(buf));
    return 0;
}

static uint8_t usb_cmd_fixFLASHbase(void)
{
    uint32_t addr = (uint32_t)usb_task.temp_usb_buf[2]
                  | (uint32_t)usb_task.temp_usb_buf[3] << 8U
                  | (uint32_t)usb_task.temp_usb_buf[4] << 16U
                  | (uint32_t)usb_task.temp_usb_buf[5] << 24U;

    if ((addr & 0x7FFU) != 0U)
        return 1U;
    if (addr < 0x08000000U)
        return 2U;

    extern uint32_t _flash_image_end;
    uint32_t code_end = (uint32_t)&_flash_image_end;
    uint32_t code_end_page = (code_end + 2047U) & ~2047U;
    if (addr < code_end_page)
        return 3U;

    if ((addr + 0x5000U) > 0x0803F800U)
        return 4U;
    if (dev_addr_save(addr) != 0)
        return 5U;

    storage_set_addr(addr);

    return 0U; 
}

static uint8_t _usb_cmd_intomenu(void){
    enum {MENU_OPENTOUCH, MENU_TOMENU, MENU_DELAY};
    static uint8_t status = MENU_OPENTOUCH;
    static uint8_t send_data[20]={0};
    static uint16_t _tick_nop = 0;
    send_data[0] = 0x82;
    send_data[1] = 0;
    switch(status){
        case MENU_OPENTOUCH:
            send_data[2] = 0xfc;
            send_data[3] = 0;
            send_data[4] = 0;
            send_data[5] = 0;
            send_data[6] = 0;
            dgusii_send(send_data,7);
            status = MENU_DELAY;
        break;
        case MENU_TOMENU:
            send_data[2] = 0x84;
            send_data[3] = 0x5a;
            send_data[4] = 0x01;
            send_data[5] = 0x00;
            send_data[6] = 0x01;
            dgusii_send(send_data,7);
            status = MENU_OPENTOUCH;
            cmd_set_enable(CMD_MENU,FALSE);
        break;
        case MENU_DELAY:
            if(_tick_nop<20){
                _tick_nop++;
                return 0;
            }
            _tick_nop = 0;
            status = MENU_TOMENU;
        break;
    }
    return 0;
}

static uint8_t _usb_cmd_getC2H2data(void){
    uint8_t ret;
    uint8_t req[3]={0};
    C2H2_t* c2h2temp =_C2H2_getdata();
    req[0] = BUS_MODBUS;
    req[1] = BUS_MOD_C2H2;
    req[2] = C2H2_INFO;
    ret = bus485_request(req, (uint8_t)sizeof(req));
    if(ret == BUS485_FAIL){
        cmd_set_enable(CMD_C2H2,FALSE);
        return 1U;
    }
    if (ret == BUS485_BUSY)
    {
        return 0U;
    }
    
    char buf[128];
    sprintf(buf,"C2H2 Concent=%dppm,relight=%d,status=0x%lx\r\n",c2h2temp->conc,c2h2temp->relight,(unsigned long)((((uint32_t)c2h2temp->statusH)<<16)|c2h2temp->statusL));
    usb_cdc_send_packet((uint8_t*)buf,strlen(buf));
    cmd_set_enable(CMD_C2H2,FALSE);
    return 0;
}
static uint8_t _usb_cmd_setCH4id(void)
{
    uint8_t ret;
    uint8_t req[4];

    req[0] = BUS_MODBUS;
    req[1] = BUS_MOD_CH4;
    req[2] = CH4_CMD_SET_ID;
    req[3] = usb_485_id;

    ret = bus485_request(req, (uint8_t)sizeof(req));
    if (ret == BUS485_BUSY)
    {
        return 0U;
    }
    if (ret == BUS485_DONE)
    {
        cmd_set_enable(CMD_CH4ID,FALSE);
        usb_task_delay_to(300U, UT_SUCCESS);
        return 0U;
    }
    if (ret == BUS485_FAIL)
    {
        cmd_set_enable(CMD_CH4ID,FALSE);
        usb_fail_code = CH4_ID;
        usb_task_delay_to(50U, UT_FAIL);
        return 0U;
    }
    return 0U;
}



static uint8_t _usb_cmd_getC2H6data(void)
{
    static C2H6_frame_t frame;
    uint8_t ret;

    ret = C2H6_run(c2h6_temp, &frame);
    if (ret == 1U)
    {
        char info[128];
        uint16_t used;
        uint8_t data_len;
        uint8_t i;
        if (frame.len < 1U)
        {
            cmd_set_enable(CMD_C2H6,FALSE);
            usb_fail_code = C2H6_INIT;
            return 1U;
        }
        data_len = (uint8_t)(frame.len - 1U);
        used = (uint16_t)snprintf(info, sizeof(info), "C2H6 len=%u,opc=%02X,data=", frame.len, frame.opc);
        for (i = 0U; (i < data_len) && (used < (sizeof(info) - 4U)); i++)
        {
            used += (uint16_t)snprintf(&info[used], sizeof(info) - used, "%02X ", frame.data[i]);
        }
        (void)snprintf(&info[used], sizeof(info) - used, "\r\n");
        usb_cdc_send_packet((uint8_t *)info, (uint16_t)strlen(info));
        cmd_set_enable(CMD_C2H6,FALSE);
        return 0U;
    }
    if (ret == 0U)
    {
        cmd_set_enable(CMD_C2H6,FALSE);
        usb_fail_code = C2H6_INIT;
        
        return 1U;
    }
    return 0U;
}

static uint8_t _cmd_getCH4info(void)
{
    static int inf =0;
    uint8_t ret;
    uint8_t req[3];

    req[0] = BUS_MODBUS;
    req[1] = BUS_MOD_CH4;
    req[2] = (uint8_t)inf;
    ret = bus485_request(req, (uint8_t)sizeof(req));
    if (ret == BUS485_FAIL)
    {
        cmd_set_enable(CMD_CH4INFO,FALSE);
        inf=0;
        return 1U;
    }
    if (ret == BUS485_BUSY)
    {
        return 0U;
    }
    inf = (inf+1);
    if(inf>=CH4_INFO_COUNT)
    {
        char info[128]  ;
        CH4_t *ch4_data = CH4_getCH4data();
        cmd_set_enable(CMD_CH4INFO,FALSE);
        inf=0;
        sprintf(info,"CH4 ID:0x%x,PointNum:%u,Ucode:%u,Range:%u,ConcCode:%u,DA:%u,Conc:%u,sensorType=0x%x\r\n" \
                ,ch4_data->id, ch4_data->pointbit, ch4_data->ucode, ch4_data->crang, ch4_data->conc_code, ch4_data->CH4da, ch4_data->conc_m, ch4_data->ctype);
        usb_cdc_send_packet((uint8_t *)info, (uint16_t)strlen(info));
        usb_task_delay_to(300U, UT_SUCCESS);
    }else{
        usb_task_delay_to(300U, IDLE);
    }
    
    return 0U;
}

static uint8_t _usb_cmd_getH2temp(void){
    const H2_data_t *h2_data = H2_getData();
    static float temp[2]={0.0f,0.0f};
    enum H2_status {SENSOR , BOARD,COMPLETE};
    static uint8_t H2_task_status = SENSOR;
    uint8_t req[3];
    uint8_t ret;

    req[0] = BUS_MODBUS;
    req[1] = BUS_MOD_H2;
    switch(H2_task_status)
    {
        case SENSOR:{
            req[2] = H2_CMD_TEMP_SENSOR;
            ret = bus485_request(req, (uint8_t)sizeof(req));
            if(ret == BUS485_FAIL){
                cmd_set_enable(CMD_H2TEMP,FALSE);
                H2_task_status = SENSOR;
                return 1U;
            }
            if (ret == BUS485_BUSY){
                return 0U;
            }
            if(ret == BUS485_DONE){
                temp[0] = h2_data->temp_sensor;
                H2_task_status = BOARD;
                return 0U;
            }                
            break;
					}
        case BOARD: {
            req[2] = H2_CMD_TEMP_BOARD;
            ret = bus485_request(req, (uint8_t)sizeof(req));
            if(ret == BUS485_FAIL){
                cmd_set_enable(CMD_H2TEMP,FALSE);
                H2_task_status = SENSOR;
                return 1U;
            }
            if (ret == BUS485_BUSY){
                return 0U;
            }
            if(ret == BUS485_DONE){
                temp[1] = h2_data->temp_board;
                H2_task_status = COMPLETE;
                return 0U;
            } 
            break;
					}
        case COMPLETE:{
            char info[128];
            sprintf(info,"H2 Temp: Sensor=%.2fC, Board=%.2fC\r\n", temp[0], temp[1]);
            usb_cdc_send_packet((uint8_t *)info, (uint16_t)strlen(info));
            H2_task_status = SENSOR;
            cmd_set_enable(CMD_H2TEMP,FALSE);
            break;
				}
        default:
            H2_task_status = SENSOR;
            cmd_set_enable(CMD_H2TEMP,FALSE);
            break;
    }
    return 0U;  
}
static void usb_cmd_polling(void){
    static uint8_t i =0;
    if(((usb_poll >> i) & 0x01U) != 0U)
    {
        if(usb_poll_fun[i] != NULL) {
            if(usb_poll_fun[i]())
                _cmd_error |= (1<<i);
        }
    }
    i = (i + 1) % (sizeof(usb_poll_fun)/sizeof(usb_poll_fun[0]));
}

static uint8_t usb_cmd_read_param(void)
{
    float kb[4][2];
    char buf[128];
    int len;

    if (storage_param_read_all(kb) != 0)
        return 1U;

    len = snprintf(buf, sizeof(buf), "id=%d:%f,%f\r\nid=%d:%f,%f\r\nid=%d:%f,%f\r\nid=%d:%f,%f\r\n",
            0x01,kb[0][0], kb[0][1], 
            0x02,kb[1][0], kb[1][1],
            0x03,kb[2][0], kb[2][1], 
            0x04,kb[3][0], kb[3][1]);
    if (len <= 0)
        return 1U;
    if (len >= (int)sizeof(buf))
        len = (int)sizeof(buf) - 1;
    usb_cdc_send_packet((uint8_t *)buf, (uint16_t)len);
    return 0U;
}

/* USB frame: 78 1B.  Clear the four persisted (k, b) parameter groups. */
static uint8_t usb_cmd_clear_param(void)
{
    float kb[PARAM_GROUP_CNT][PARAM_FLOATS_PER_GROUP] = {{0.0F}};
    static const uint8_t ok_msg[] = "param cleared\r\n";

    if (storage_param_write(kb, sizeof(kb)) != 0)
    {
        return 1U;
    }

    usb_cdc_send_packet(ok_msg, sizeof(ok_msg) - 1U);
    return 0U;
}

static uint8_t usb_cmd_read_log(void)
{
    int total = storage_log_size();
    if (total == 0)
    {
        usb_cdc_send_packet((uint8_t *)"empty\r\n", 7);
        return 0U;
    }

    uint32_t off = 0;
    char line[256];

    while (off + 8 <= (uint32_t)total)
    {
        uint8_t hdr[8];
        uint32_t data_len, aligned, entry_total, magic;

        if (storage_log_read_offset(off, hdr, 8) < 8)
            break;

        magic = (uint32_t)hdr[0] | (uint32_t)hdr[1] << 8
              | (uint32_t)hdr[2] << 16 | (uint32_t)hdr[3] << 24;
        if (magic != 0xA5A5A5A5U)
            break;

        data_len = (uint32_t)hdr[4] | (uint32_t)hdr[5] << 8
                 | (uint32_t)hdr[6] << 16 | (uint32_t)hdr[7] << 24;
        if (data_len == 0U || data_len == 0xFFFFFFFFU)
            break;

        aligned = (data_len + 3U) & ~3U;
        entry_total = 8U + aligned;
        if (off + entry_total > (uint32_t)total)
            break;

        if (data_len == 15U)
        {
            uint8_t data[15];
            if (storage_log_read_offset(off + 8, data, 15) < 15)
                break;

            uint16_t year = 2000U + ((uint16_t)(data[0] - 208U) & 0xFFU);
            float conc;
            char gas_name[5];
            int line_len;

            memcpy(&conc, &data[11], 4);
            memcpy(gas_name, &data[6], 4);
            gas_name[4] = '\0';

            line_len = snprintf(line, sizeof(line),
                "%04u-%02u-%02u %02u:%02u:%02u %s conc=%f\r\n",
                year, data[1], data[2], data[3], data[4], data[5],
                gas_name, conc);
            if (line_len > 0)
                usb_cdc_send_packet((uint8_t *)line, (uint16_t)line_len);
        }
        else
        {
            uint8_t data[64];
            uint32_t rdlen = data_len > 64 ? 64 : data_len;

            if (storage_log_read_offset(off + 8, data, rdlen) < (int)rdlen)
                break;

            /* Output as text (preserve GBK multi-byte chars, stop at null padding) */
            int pos = 0;
            for (uint32_t i = 0; i < rdlen && pos < (int)sizeof(line) - 1; i++)
            {
                uint8_t c = data[i];
                if (c == 0x00)
                    break;
                if (c >= 0x20U && c < 0x7FU)
                    line[pos++] = (char)c;
                else if (c == '\r' || c == '\n')
                    line[pos++] = (char)c;
                else if (c >= 0x80U)
                    line[pos++] = (char)c;  /* GBK multi-byte / extended ASCII */
                else
                    line[pos++] = '.';
            }
            if (pos + 2 <= (int)sizeof(line)) {
                line[pos++] = '\r';
                line[pos++] = '\n';
            }
            usb_cdc_send_packet((uint8_t *)line, (uint16_t)pos);
        }

        off += entry_total;
    }

    return 0U;
}

static uint8_t usb_cmd_read_log_raw(void)
{
    int total = storage_log_size();
    if (total == 0)
    {
        usb_cdc_send_packet((uint8_t *)"empty\r\n", 7);
        return 0U;
    }

    uint8_t buf[256];
    uint32_t off = 0;
    while (off < (uint32_t)total)
    {
        uint32_t rdlen = (uint32_t)total - off;
        if (rdlen > sizeof(buf)) rdlen = sizeof(buf);
        if (storage_log_read_offset(off, buf, rdlen) < (int)rdlen)
            break;
        usb_cdc_send_packet(buf, (uint16_t)rdlen);
        off += rdlen;
    }
    return 0U;
}

static uint8_t usb_cmd_write_param(void)
{
    uint8_t group = usb_task.temp_usb_buf[2];
    float k, b;

    memcpy(&k, &usb_task.temp_usb_buf[3], 4);
    memcpy(&b, &usb_task.temp_usb_buf[7], 4);

    return (uint8_t)storage_param_write_group(group, k, b);
}

static uint8_t usb_cmd_erase_log(void)
{
    int ret = storage_log_erase();
    if (ret != 0)
        return 1U;

    usb_cdc_send_packet((uint8_t *)"log erased\r\n", 12);
    return 0U;
}

char* get_sysVersion(void)
{
    static char sys_info[64];
    sprintf(sys_info, "%s System Version: %d.%d.%d\r\n", SYS_NAME, SYS_MAJOR, SYS_MINOR, SYS_PATCH);
    return sys_info;
}

static uint8_t usb_fail(uint8_t f_id)
{
    char usb_fail_buf[64];
    sprintf(usb_fail_buf, "USB Task ID %02X failed\r\n", f_id);
    usb_cdc_send_packet((uint8_t *)usb_fail_buf, (uint16_t)strlen(usb_fail_buf));
    return (uint8_t)-1;
}
