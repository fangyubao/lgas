#ifndef __DEVICE_SOLENOID_H
#define __DEVICE_SOLENOID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

HAL_StatusTypeDef solenoid_init(void);
HAL_StatusTypeDef solenoid_set_duty(uint8_t percent);

#ifdef __cplusplus
}
#endif

#endif /* __DEVICE_SOLENOID_H */
