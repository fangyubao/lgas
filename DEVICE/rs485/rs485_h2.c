#include "rs485_h2.h"

#include "main.h"
#include "uart.h"

static void rs485_h2_set_tx_mode(void)
{
    HAL_GPIO_WritePin(RS485_2_ENABLE_PORT, RS485_2_ENABLE_PIN, GPIO_PIN_SET);
}

static void rs485_h2_set_rx_mode(void)
{
    HAL_GPIO_WritePin(RS485_2_ENABLE_PORT, RS485_2_ENABLE_PIN, GPIO_PIN_RESET);
}

void rs485_h2_init(void)
{
    uart_startReceiveIT(UART_PORT_USART1);
    rs485_h2_set_rx_mode();
}

uint16_t rs485_h2_read(uint8_t *buf, uint16_t max_len)
{
    return uart_getData(UART_PORT_USART1, buf, max_len);
}

HAL_StatusTypeDef rs485_h2_write(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    extern UART_HandleTypeDef huart1;
    uint32_t start_tick;

    if ((data == NULL) || (len == 0U))
    {
        return HAL_OK;
    }

    rs485_h2_set_tx_mode();

    if (uart_sendData(UART_PORT_USART1, data, len, timeout_ms) != HAL_OK)
    {
        rs485_h2_set_rx_mode();
        return HAL_ERROR;
    }

    start_tick = HAL_GetTick();
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET)
    {
        if ((HAL_GetTick() - start_tick) >= timeout_ms)
        {
            rs485_h2_set_rx_mode();
            return HAL_ERROR;
        }
    }

    rs485_h2_set_rx_mode();
    return HAL_OK;
}
