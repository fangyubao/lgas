#ifndef __DEVICE_RTC_H
#define __DEVICE_RTC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define RTC_BACKEND_INTERNAL       0U
#define RTC_BACKEND_DS1302         1U

#ifndef RTC_BACKEND_SELECT
#define RTC_BACKEND_SELECT         RTC_BACKEND_DS1302
#endif

typedef struct
{
    uint16_t year;
    uint8_t month;
    uint8_t date;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
} rtc_datetime_t;

HAL_StatusTypeDef rtc_init(void);
HAL_StatusTypeDef rtc_get_datetime(rtc_datetime_t *datetime);
HAL_StatusTypeDef rtc_set_datetime(const rtc_datetime_t *datetime);

#ifdef __cplusplus
}
#endif

#endif /* __DEVICE_RTC_H */
