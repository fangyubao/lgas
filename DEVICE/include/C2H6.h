#ifndef __C2H6_H
#define __C2H6_H

#include "main.h"
#include "stdint.h"
#include "uart.h"
#include "usb_task.h"

#define C2H6_FW_VER      0X01
#define C2H6_PJ_SN       0X02
#define C2H6_READ_CONC   0X03
#define C2H6_CABR        0X04
#define C2H6_CABRPAM     0X05
#define C2H6_ZEROCABR    0X06
#define C2H6_RANGCABR    0X07

#define C2H6_TX_FRAME_HEAD  0x10U
#define C2H6_RX_FRAME_HEAD  0x20U
#define C2H6_TIMEOUT_MS     1000U
#define C2H6_DATA_MAX_LEN   60U

#define CH2_TIMEOUT_MS     1000U

#define C2H6_taskDelay() do { } while (0)

typedef struct {
    uint8_t len;
    uint8_t data[C2H6_DATA_MAX_LEN];
} C2H6_info_t;

typedef struct {
    uint8_t hand;
    uint8_t len;
    uint8_t opc;
    uint8_t *data;
    uint8_t crc;
} C2H6_frame_t;

uint8_t C2H6_run(uint8_t *c6h6,C2H6_frame_t* data);
void   C2H6_reset(void);
uint8_t* C2H6_SNverify(void);
uint8_t* C2H6_conc(void);
extern volatile usb_delay_t c2h6_delay;
#endif
