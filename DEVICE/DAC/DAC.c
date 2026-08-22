#include "main.h"
#include "DAC.h"

#define DAC_VREF_MV     3300U
#define DAC_12B_MAX     4095U
#define CAB             5U

extern DAC_HandleTypeDef hdac;

uint8_t DAC_OutputInit(void)
{
    if (HAL_DAC_Start(&hdac, DAC_CHANNEL_2) != HAL_OK)
    {
        return 1U;
    }
    return 0U;
}

uint8_t DAC_OutputSetMv(uint16_t target_mv)
{
    uint32_t dac_code;

    if (target_mv > DAC_VREF_MV)
    {
        target_mv = DAC_VREF_MV;
    }

    dac_code = ((uint32_t)target_mv * DAC_12B_MAX) / DAC_VREF_MV;
    dac_code += CAB;
    if (dac_code > DAC_12B_MAX)
    {
        dac_code = DAC_12B_MAX;
    }

    if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_code) != HAL_OK)
    {
        return 1U;
    }

    return 0U;
}
