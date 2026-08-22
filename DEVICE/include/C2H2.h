#ifndef __C2H2_H
#define __C2H2_H
#include "stdint.h"
#include "usb_task.h"
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif

#define C2H2_MODBUS_SLAVE_ID                0x03
#define C2H2_MODBUS_TIMEOUT_MS              1000U

#define C2H2_RUN_DONE 0
#define C2H2_RUN_FAIL 0x01
#define C2H2_RUN_BUSY 0X02


#define C2H2_INFO    0X01
#define C2H2_CONC    0X02
typedef struct{
    uint16_t conc;
    uint16_t statusH;
    uint16_t statusL;
    uint16_t relight;
}C2H2_t;

uint8_t C2H2_Getdata(uint8_t *cmd);
C2H2_t* _C2H2_getdata(void);
HAL_StatusTypeDef C2H2_readinfo(float *c2h2_conc);
#ifdef __cplusplus
}
#endif

#endif /* __CH4_H */
