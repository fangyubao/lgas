#ifndef __RS485_H2_H
#define __RS485_H2_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

void rs485_h2_init(void);
uint16_t rs485_h2_read(uint8_t *buf, uint16_t max_len);
HAL_StatusTypeDef rs485_h2_write(const uint8_t *data, uint16_t len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __RS485_H2_H */
