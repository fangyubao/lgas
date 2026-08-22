#ifndef __USB_TASK_H
#define __USB_TASK_H
#include "stdlib.h"
#include "stdint.h"

#define USB_HEAD 0X78U

#define TASK_SETTIME 0X00
#define TASK_SYSINFO 0X01
#define TASK_GETTIME 0X02
#define TASK_SETBAUD 0X03
#define TASK_CH4INFO 0X04
#define TASK_CH4ID   0X05
#define TASK_C2H6    0X06
#define TASK_DAC     0X07
#define TASK_FAN     0X08
#define TASK_PUMP    0X09
#define TASK_SOL     0X0A
#define TASK_DG408   0x0B
#define TASK_C2H2    0X0C
#define TASK_CH4C    0X0D
#define TASK_C2H2C   0X0E
#define TASK_C2H6C   0X0F
#define TASK_H2C     0X10
#define TASK_H2TEMP  0X11
#define TASK_MENU    0X12
#define TASK_TOUCH   0X13
#define TASK_BASE    0X14
#define TASK_FLASHLOG   0X15
#define TASK_READ_PARAM 0X16
#define TASK_READ_LOG   0X17
#define TASK_WRITE_PARAM 0X18
#define TASK_ERASE_LOG   0X19
#define TASK_READ_LOG_RAW 0X1A
#define TASK_CLEAR_PARAM 0X1B

#define CMD_CH4INFO     0
#define CMD_CH4ID       1
#define CMD_C2H6        2
#define CMD_C2H2        3
#define CMD_H2TEMP      4
#define CMD_MENU        5
#define USB_taskDelay() do { } while (0)

typedef uint8_t (*usb_cmd_handler_t)(void);

typedef enum{
    IDLE,
    DELAY,
    WAIT,
    UT_FAIL,
    UT_SUCCESS,
    TASK_NUM
}usb_task_status;

typedef enum{
    OK = 0,
    RTC_F,
    LEN_F,
    BAUD,
    CH4_ID,
    CH4_INFO,
    C2H6_INIT,
    NOCODE ,
    ECODE
}usb_ecode;

typedef struct{
    volatile uint32_t usb_tick;
    volatile uint8_t usb_status;
    volatile uint8_t next_id;
}usb_delay_t;

typedef struct 
{
    uint8_t usb_task_id;
    uint8_t *temp_usb_buf;
    uint16_t temp_usb_len;
    uint8_t uart_len;
}usb_task_t;

typedef struct
{
    uint8_t cmd;
    uint16_t min_len;
    uint32_t delay_ms;
    uint8_t next_id;
    uint8_t async_mode;
    usb_cmd_handler_t handler;
} usb_cmd_entry_t;
void ndelay_ms(volatile usb_delay_t *task, uint32_t t_ms, uint8_t next_id);
uint8_t ndelay_expired(volatile usb_delay_t *task);
uint8_t usb_task_init(void);
void usb_task_process(void);
char* get_sysVersion(void);
uint8_t cmd_set_enable(uint8_t p, uint8_t en);
extern volatile usb_delay_t usb_delay;
#endif
