#include "fan.h"

extern TIM_HandleTypeDef htim1;

static uint8_t s_fan_started = 0U;

static uint32_t fan_get_compare(uint8_t percent)
{
    uint32_t period;

    if (percent > 100U)
    {
        percent = 100U;
    }

    period = __HAL_TIM_GET_AUTORELOAD(&htim1);
    return (period * percent) / 100U;
}

HAL_StatusTypeDef fan_init(void)
{
    if (htim1.Instance != TIM1)
    {
        return HAL_ERROR;
    }

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    s_fan_started = 1U;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    return HAL_OK;
}

HAL_StatusTypeDef fan_set_speed(uint8_t percent)
{
    if (s_fan_started == 0U)
    {
        if (fan_init() != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, fan_get_compare(percent));
    return HAL_OK;
}
