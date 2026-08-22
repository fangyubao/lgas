#include "pump.h"

extern TIM_HandleTypeDef htim2;

static uint8_t s_pump_started = 0U;

static uint32_t pump_get_compare(uint8_t percent)
{
    uint32_t period;

    if (percent > 100U)
    {
        percent = 100U;
    }

    period = __HAL_TIM_GET_AUTORELOAD(&htim2);
    return (period * percent) / 100U;
}

HAL_StatusTypeDef pump_init(void)
{
    if (htim2.Instance != TIM2)
    {
        return HAL_ERROR;
    }

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    s_pump_started = 1U;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
    return HAL_OK;
}

HAL_StatusTypeDef pump_set_strength(uint8_t percent)
{
    if (s_pump_started == 0U)
    {
        if (pump_init() != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pump_get_compare(percent));
    return HAL_OK;
}
