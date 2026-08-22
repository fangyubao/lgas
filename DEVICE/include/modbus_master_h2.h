#ifndef __MODBUS_MASTER_H2_H
#define __MODBUS_MASTER_H2_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef enum
{
    MODBUS_MASTER_H2_IDLE = 0U,
    MODBUS_MASTER_H2_BUSY,
    MODBUS_MASTER_H2_DONE,
    MODBUS_MASTER_H2_FAIL
} modbus_master_h2_state_t;

void modbus_master_h2_init(void);
void modbus_master_h2_taskDelay(void);
void modbus_master_h2_poll(void);
void modbus_master_h2_clear(void);
modbus_master_h2_state_t modbus_master_h2_get_state(void);

HAL_StatusTypeDef modbus_master_h2_start_read_holding(uint8_t slave_addr,
                                                      uint16_t reg_addr,
                                                      uint16_t reg_count,
                                                      uint16_t *out_regs,
                                                      uint32_t timeout_ms);

HAL_StatusTypeDef modbus_master_h2_read_holding(uint8_t slave_addr,
                                                uint16_t reg_addr,
                                                uint16_t reg_count,
                                                uint16_t *out_regs,
                                                uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_MASTER_H2_H */
