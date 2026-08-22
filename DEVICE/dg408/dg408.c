#include "dg408.h"
#include "main.h"

static void dg408_write_addr(uint8_t channel)
{
		uint8_t A0t = (uint8_t)(channel&0x01);
		uint8_t A1t = (uint8_t)((channel&0x02)>>1);
		uint8_t A2t = (uint8_t)((channel&0x04)>>2);
    HAL_GPIO_WritePin(DG408_A0_GPIO_Port, DG408_A0_Pin, A0t);
    HAL_GPIO_WritePin(DG408_A1_GPIO_Port, DG408_A1_Pin, A1t);
    HAL_GPIO_WritePin(DG408_A2_GPIO_Port, DG408_A2_Pin, A2t);
}

HAL_StatusTypeDef dg408_init(void)
{
    HAL_GPIO_WritePin(DG408_EN_GPIO_Port, DG408_EN_Pin, GPIO_PIN_RESET);
    dg408_write_addr(0U);
    return HAL_OK;
}

HAL_StatusTypeDef dg408_enable(uint8_t enable)
{
    HAL_GPIO_WritePin(DG408_EN_GPIO_Port,
                      DG408_EN_Pin,
                      (enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return HAL_OK;
}

HAL_StatusTypeDef dg408_select(uint8_t channel)
{
    if (channel > 7U)
    {
				dg408_write_addr(0);
        dg408_enable(FALSE);
        return HAL_ERROR;
    }
    dg408_enable(TRUE);
    

    return HAL_OK;
}
