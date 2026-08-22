#ifndef __MODBUS_SLAVE_H
#define __MODBUS_SLAVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

void modbus_slave_init(uint8_t slave_addr, uint16_t *holding_regs, uint16_t holding_reg_count);
void modbus_slave_taskDelay(void);
void modbus_slave_set_coils(uint8_t *coils, uint16_t coil_count);
void modbus_slave_set_discrete_inputs(const uint8_t *discrete_inputs, uint16_t discrete_input_count);
void modbus_slave_set_input_regs(const uint16_t *input_regs, uint16_t input_reg_count);
void modbus_slave_set_slave_id(const uint8_t *slave_id, uint8_t slave_id_len, uint8_t run_status);
void modbus_slave_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_SLAVE_H */
