#ifndef __DEVICE_PUMP_H
#define __DEVICE_PUMP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define PUMP_RUNING_DUTY 60U
#define PUMP_START_DUTY  30U

#define PUMP_CLEAN_TIME_DEFAULT 100U /* seconds */
HAL_StatusTypeDef pump_init(void);
HAL_StatusTypeDef pump_set_strength(uint8_t percent);

#ifdef __cplusplus
}
#endif

#endif /* __DEVICE_PUMP_H */
