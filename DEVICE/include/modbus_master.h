#ifndef __MODBUS_MASTER_H
#define __MODBUS_MASTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef enum
{
    MODBUS_MASTER_IDLE = 0U,
    MODBUS_MASTER_BUSY,
    MODBUS_MASTER_DONE,
    MODBUS_MASTER_FAIL
} modbus_master_state_t;

typedef enum
{
    MODBUS_MASTER_ERR_NONE = 0U,
    MODBUS_MASTER_ERR_INVALID_PARAM,
    MODBUS_MASTER_ERR_TX_FAIL,
    MODBUS_MASTER_ERR_RX_OVERFLOW,
    MODBUS_MASTER_ERR_TIMEOUT,
    MODBUS_MASTER_ERR_FRAME_SHORT,
    MODBUS_MASTER_ERR_CRC,
    MODBUS_MASTER_ERR_EXCEPTION,
    MODBUS_MASTER_ERR_RESPONSE_MISMATCH,
    MODBUS_MASTER_ERR_UNSUPPORTED_OP
} modbus_master_error_t;

typedef void (*modbus_master_log_writer_t)(const char *msg);

void modbus_master_init(void);
void modbus_master_taskDelay(void);
void modbus_master_poll(void);
void modbus_master_clear(void);
modbus_master_state_t modbus_master_get_state(void);
modbus_master_error_t modbus_master_get_last_error(void);
void modbus_master_set_log_writer(modbus_master_log_writer_t writer);

HAL_StatusTypeDef modbus_master_start_read_holding(uint8_t slave_addr,
                                                   uint16_t reg_addr,
                                                   uint16_t reg_count,
                                                   uint16_t *out_regs,
                                                   uint32_t timeout_ms);

HAL_StatusTypeDef modbus_master_start_read_input(uint8_t slave_addr,
                                                 uint16_t reg_addr,
                                                 uint16_t reg_count,
                                                 uint16_t *out_regs,
                                                 uint32_t timeout_ms);

HAL_StatusTypeDef modbus_master_start_write_multi(uint8_t slave_addr,
                                                  uint16_t reg_addr,
                                                  const uint16_t *values,
                                                  uint16_t reg_count,
                                                  uint32_t timeout_ms);

HAL_StatusTypeDef modbus_master_read_coils(uint8_t slave_addr,
                                           uint16_t coil_addr,
                                           uint16_t coil_count,
                                           uint8_t *out_bits,
                                           uint32_t timeout_ms);

HAL_StatusTypeDef modbus_master_read_discrete_inputs(uint8_t slave_addr,
                                                     uint16_t input_addr,
                                                     uint16_t input_count,
                                                     uint8_t *out_bits,
                                                     uint32_t timeout_ms);

HAL_StatusTypeDef modbus_master_read_holding(uint8_t slave_addr,
                                             uint16_t reg_addr,
                                             uint16_t reg_count,
                                             uint16_t *out_regs,
                                             uint32_t timeout_ms);

HAL_StatusTypeDef modbus_master_read_input(uint8_t slave_addr,
                                           uint16_t reg_addr,
                                           uint16_t reg_count,
                                           uint16_t *out_regs,
                                           uint32_t timeout_ms);

HAL_StatusTypeDef modbus_master_write_single_coil(uint8_t slave_addr,
                                                  uint16_t coil_addr,
                                                  uint8_t value,
                                                  uint32_t timeout_ms);

HAL_StatusTypeDef modbus_master_write_single(uint8_t slave_addr,
                                             uint16_t reg_addr,
                                             uint16_t value,
                                             uint32_t timeout_ms);

HAL_StatusTypeDef modbus_master_write_multi_coils(uint8_t slave_addr,
                                                  uint16_t coil_addr,
                                                  const uint8_t *values,
                                                  uint16_t coil_count,
                                                  uint32_t timeout_ms);

HAL_StatusTypeDef modbus_master_write_multi(uint8_t slave_addr,
                                            uint16_t reg_addr,
                                            const uint16_t *values,
                                            uint16_t reg_count,
                                            uint32_t timeout_ms);

HAL_StatusTypeDef modbus_master_mask_write_register(uint8_t slave_addr,
                                                    uint16_t reg_addr,
                                                    uint16_t and_mask,
                                                    uint16_t or_mask,
                                                    uint32_t timeout_ms);

HAL_StatusTypeDef modbus_master_read_write_multi(uint8_t slave_addr,
                                                 uint16_t read_addr,
                                                 uint16_t read_count,
                                                 uint16_t *out_regs,
                                                 uint16_t write_addr,
                                                 const uint16_t *write_values,
                                                 uint16_t write_count,
                                                 uint32_t timeout_ms);

HAL_StatusTypeDef modbus_master_report_slave_id(uint8_t slave_addr,
                                                uint8_t *out_buf,
                                                uint16_t out_buf_max,
                                                uint16_t *out_len,
                                                uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_MASTER_H */
