#include "stdint.h"
#include "rtc.h"
#include "usb_task.h"

#define sH2  0X01
#define sC2H2 0X02
#define sCH4 0X03
#define sC2H6 0X04
#define sUPDATA 0x05
#define sNOGAS 0x06

typedef struct 
{
    rtc_datetime_t time_data;
    float CH4;
    float C2H6;
    float H2;
    float C2H2;
    volatile uint32_t _tick;
    uint8_t status;
    uint8_t last_status;
    volatile uint32_t wait_tick;
    uint8_t wait_flag;
    uint16_t wait_sec;
    volatile uint32_t collect_start;
}dataup;

extern dataup sensordata;
void dataup_taskDelay(void);
uint8_t updata_sensor(void);
void dataup_set_enable(uint8_t enable,uint8_t gas);
uint8_t dataup_get_enable(void);

extern dataup sensordata;
