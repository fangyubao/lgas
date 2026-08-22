#include "rs485.h"

#include "main.h"
#include "usbd_cdc_if.h"
#include "uart.h"

#include <stdio.h>

static void rs485_set_tx_mode(void)
{
  HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);
}

static void rs485_set_rx_mode(void)
{
  HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
}

void rs485_init(void)
{
  uart_startReceiveIT(UART_PORT_USART2);
  rs485_set_rx_mode();
}

uint16_t rs485_read(uint8_t *buf, uint16_t max_len)
{
  return uart_getData(UART_PORT_USART2, buf, max_len);
}

HAL_StatusTypeDef rs485_write(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
  extern UART_HandleTypeDef huart2;
  uint32_t start_tick;
#if 0
  char dbg_buf[96];
  int dbg_len;
  uint16_t i;
  uint16_t max_dump;
#endif

  if ((data == NULL) || (len == 0U))
  {
    return HAL_OK;
  }

#if 0
  max_dump = (len > 16U) ? 16U : len;
  dbg_len = snprintf(dbg_buf, sizeof(dbg_buf), "485 TX len=%u data=", (unsigned)len);
  for (i = 0U; (i < max_dump) && (dbg_len > 0) && (dbg_len < (int)(sizeof(dbg_buf) - 4U)); i++)
  {
    dbg_len += snprintf(&dbg_buf[dbg_len], sizeof(dbg_buf) - (size_t)dbg_len, "%02X ", data[i]);
  }
  if (len > max_dump)
  {
    dbg_len += snprintf(&dbg_buf[dbg_len], sizeof(dbg_buf) - (size_t)dbg_len, "...");
  }
  dbg_len += snprintf(&dbg_buf[dbg_len], sizeof(dbg_buf) - (size_t)dbg_len, "\r\n");
  if (dbg_len > 0)
  {
    (void)usb_cdc_send_packet((uint8_t *)dbg_buf, (uint16_t)dbg_len);
  }
#endif

  rs485_set_tx_mode();

  if (uart_sendData(UART_PORT_USART2, data, len, timeout_ms) != HAL_OK)
  {
    rs485_set_rx_mode();
    return HAL_ERROR;
  }

  start_tick = HAL_GetTick();
  while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET)
  {
    if ((HAL_GetTick() - start_tick) >= timeout_ms)
    {
      rs485_set_rx_mode();
      return HAL_ERROR;
    }
  }

  rs485_set_rx_mode();
  return HAL_OK;
}
