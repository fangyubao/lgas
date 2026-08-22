#ifndef __H2_H
#define __H2_H

#include "main.h"
#include <stdint.h>

#define H2_CMD_CONC           0x01U
#define H2_CMD_TEMP_SENSOR    0x02U
#define H2_CMD_TEMP_BOARD     0x03U

#define H2_RUN_DONE 0U
#define H2_RUN_FAIL 0x01U
#define H2_RUN_BUSY 0x02U

#define H2_MODBUS_SLAVE_ID   0x01U
#define H2_MODBUS_TIMEOUT_MS 1000U

/* All H2 sensor registers are holding registers. */
#define H2_REG_CONC_H        0x04U
#define H2_REG_TEMP_SENSOR   0x06U
#define H2_REG_TEMP_BOARD    0x07U

typedef struct
{
    uint32_t conc_raw;
    float conc;
    uint16_t temp_sensor_raw;
    float temp_sensor;
    uint16_t temp_board_raw;
    float temp_board;
} H2_data_t;

uint8_t H2_Getdata(uint8_t *cmd);
const H2_data_t *H2_getData(void);
HAL_StatusTypeDef H2_readConc(float *h2_conc);

#endif
