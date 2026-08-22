#ifndef __BUS485_TASK_H
#define __BUS485_TASK_H

#include <stdint.h>

#define BUS_MODBUS 0X01
#define BUS_UPDATA 0X02

#define BUS_MOD_CH4 0X01
#define BUS_MOD_H2  0X02
#define BUS_MOD_C2H2 0X03

#define BUS_UPDATA_SENSOR 0X01

typedef enum
{
    BUS485_IDLE = 0U,
    BUS485_BUSY,
    BUS485_DONE,
    BUS485_FAIL
} bus485_state_t;

void bus485_run(void);
uint8_t bus485_request(const uint8_t *req, uint8_t len);

#endif 
