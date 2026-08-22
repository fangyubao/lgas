#ifndef __DEVICE_FAN_H
#define __DEVICE_FAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

HAL_StatusTypeDef fan_init(void);
HAL_StatusTypeDef fan_set_speed(uint8_t percent);

#ifdef __cplusplus
}
#endif

#endif /* __DEVICE_FAN_H */
