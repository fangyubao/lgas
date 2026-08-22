#ifndef __UART_H
#define __UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE 256
#endif

typedef enum
{
  UART_PORT_USART1 = 0,
  UART_PORT_USART2 = 1,//modbus
  UART_PORT_USART3 = 2,
  UART_PORT_UART4  = 3,
  UART_PORT_COUNT
} uart_port_t;

typedef enum
{
  UART_BAUD_SEL_9600 = 1,
  UART_BAUD_SEL_19200,
  UART_BAUD_SEL_38400,
  UART_BAUD_SEL_57600,
  UART_BAUD_SEL_115200,
  UART_BAUD_SEL_230400,
  UART_BAUD_SEL_460800,
  UART_BAUD_SEL_921600
} uart_baud_sel_t;

void uart_startReceiveIT(uart_port_t port);
void uart_startAllReceiveIT(void);
uint8_t uart_getBaudRate(void);
uint16_t uart_getDataLength(uart_port_t port);
uint16_t uart_getData(uart_port_t port, uint8_t *out, uint16_t max_len);

HAL_StatusTypeDef uart_sendData(uart_port_t port, const uint8_t *data, uint16_t len, uint32_t timeout_ms);
HAL_StatusTypeDef uart_setBaudRate(uart_port_t port, uart_baud_sel_t baud_sel);

#ifdef __cplusplus
}
#endif

#endif /* __UART_H */
