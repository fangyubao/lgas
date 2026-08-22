#ifndef __DEVICE_DG408_H
#define __DEVICE_DG408_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

HAL_StatusTypeDef dg408_init(void);
HAL_StatusTypeDef dg408_enable(uint8_t enable);
HAL_StatusTypeDef dg408_select(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __DEVICE_DG408_H */
