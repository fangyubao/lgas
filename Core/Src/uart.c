#include "uart.h"
#include "usbd_cdc_if.h"
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;

typedef struct
{
  volatile uint16_t head;
  volatile uint16_t tail;
  uint8_t buf[UART_RX_BUFFER_SIZE];
} uart_fifo_t;

static uart_fifo_t s_fifos[UART_PORT_COUNT];

static UART_HandleTypeDef *uart_get_handle(uart_port_t port)
{
  switch (port)
  {
    case UART_PORT_USART1:
      return &huart1;

    case UART_PORT_USART2:
      return &huart2;

    case UART_PORT_USART3:
      return &huart3;

    case UART_PORT_UART4:
      return &huart4;

    default:
      return NULL;
  }
}

static uint32_t uart_lock(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void uart_unlock(uint32_t primask)
{
  __set_PRIMASK(primask);
}

static void uart_fifo_push(uart_port_t port, uint8_t byte)
{
  uart_fifo_t *fifo = &s_fifos[(uint32_t)port];
  uint16_t next = (uint16_t)((fifo->head + 1U) % UART_RX_BUFFER_SIZE);

  if (next == fifo->tail)
  {
    fifo->tail = (uint16_t)((fifo->tail + 1U) % UART_RX_BUFFER_SIZE);
  }

  fifo->buf[fifo->head] = byte;
  fifo->head = next;
}

static void uart_enable_rx_irq(UART_HandleTypeDef *huart)
{
  __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(huart, UART_IT_ERR);
}

static void uart_rxne_service(USART_TypeDef *instance, uart_port_t port)
{
  uint32_t sr = instance->SR;

  if ((sr & USART_SR_RXNE) != 0U)
  {
    uint8_t data = (uint8_t)(instance->DR & 0x00FFU);
    uart_fifo_push(port, data);
    return;
  }

  if ((sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U)
  {
    volatile uint32_t dr = instance->DR;
    (void)dr;
  }
}

void uart_startReceiveIT(uart_port_t port)
{
  switch (port)
  {
    case UART_PORT_USART1:
      uart_enable_rx_irq(&huart1);
      break;

    case UART_PORT_USART2:
      uart_enable_rx_irq(&huart2);
      break;

    case UART_PORT_USART3:
      uart_enable_rx_irq(&huart3);
      break;

    case UART_PORT_UART4:
      uart_enable_rx_irq(&huart4);
      break;

    default:
      break;
  }
}

void uart_startAllReceiveIT(void)
{
  uart_startReceiveIT(UART_PORT_USART1);
  uart_startReceiveIT(UART_PORT_USART2);
  uart_startReceiveIT(UART_PORT_USART3);
  uart_startReceiveIT(UART_PORT_UART4);
}

void USART1_IRQHandler(void)
{
  uart_rxne_service(huart1.Instance, UART_PORT_USART1);
}

void USART2_IRQHandler(void)
{
  uart_rxne_service(huart2.Instance, UART_PORT_USART2);
}

void USART3_IRQHandler(void)
{
  uart_rxne_service(huart3.Instance, UART_PORT_USART3);
}

void UART4_IRQHandler(void)
{
  uart_rxne_service(huart4.Instance, UART_PORT_UART4);
}

uint16_t uart_getDataLength(uart_port_t port)
{
  if ((uint32_t)port >= (uint32_t)UART_PORT_COUNT)
  {
    return 0U;
  }

  uint16_t len;
  uint32_t key = uart_lock();
  {
    uart_fifo_t *fifo = &s_fifos[(uint32_t)port];
    if (fifo->head >= fifo->tail)
    {
      len = (uint16_t)(fifo->head - fifo->tail);
    }
    else
    {
      len = (uint16_t)(UART_RX_BUFFER_SIZE - fifo->tail + fifo->head);
    }
  }
  uart_unlock(key);

  return len;
}

uint16_t uart_getData(uart_port_t port, uint8_t *out, uint16_t max_len)
{
  if ((uint32_t)port >= (uint32_t)UART_PORT_COUNT || out == NULL || max_len == 0U)
  {
    return 0U;
  }

  uint16_t read_len = 0U;
  uint32_t key = uart_lock();
  {
    uart_fifo_t *fifo = &s_fifos[(uint32_t)port];
    while ((fifo->tail != fifo->head) && (read_len < max_len))
    {
      out[read_len++] = fifo->buf[fifo->tail];
      fifo->tail = (uint16_t)((fifo->tail + 1U) % UART_RX_BUFFER_SIZE);
    }
  }
  uart_unlock(key);

  return read_len;
}

HAL_StatusTypeDef uart_sendData(uart_port_t port, const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
  if (data == NULL || len == 0U)
  {
    return HAL_OK;
  }

  switch (port)
  {
    case UART_PORT_USART1:
      return HAL_UART_Transmit(&huart1, (uint8_t *)data, len, timeout_ms);

    case UART_PORT_USART2:
      return HAL_UART_Transmit(&huart2, (uint8_t *)data, len, timeout_ms);

    case UART_PORT_USART3:
      return HAL_UART_Transmit(&huart3, (uint8_t *)data, len, timeout_ms);

    case UART_PORT_UART4:
      return HAL_UART_Transmit(&huart4, (uint8_t *)data, len, timeout_ms);

    default:
      return HAL_ERROR;
  }
}

HAL_StatusTypeDef uart_setBaudRate(uart_port_t port, uart_baud_sel_t baud_sel)
{
  UART_HandleTypeDef *huart;
  uint32_t key;
  HAL_StatusTypeDef status;
  uint32_t baudrate;

  switch (baud_sel)
  {
    case UART_BAUD_SEL_9600:
      baudrate = 9600U;
      break;

    case UART_BAUD_SEL_19200:
      baudrate = 19200U;
      break;

    case UART_BAUD_SEL_38400:
      baudrate = 38400U;
      break;

    case UART_BAUD_SEL_57600:
      baudrate = 57600U;
      break;

    case UART_BAUD_SEL_115200:
      baudrate = 115200U;
      break;

    case UART_BAUD_SEL_230400:
      baudrate = 230400U;
      break;

    case UART_BAUD_SEL_460800:
      baudrate = 460800U;
      break;

    case UART_BAUD_SEL_921600:
      baudrate = 921600U;
      break;

    default:
      return HAL_ERROR;
  }

  huart = uart_get_handle(port);
  if (huart == NULL)
  {
    return HAL_ERROR;
  }

  key = uart_lock();
  huart->Init.BaudRate = baudrate;
  status = HAL_UART_Init(huart);
  if (status == HAL_OK)
  {
    uart_enable_rx_irq(huart);
  }
  uart_unlock(key);
  uint8_t msg[64];
  snprintf((char *)msg, sizeof(msg), "UART%lu baudrate set to %lu\r\n",
           (unsigned long)((uint32_t)port + 1U),
           (unsigned long)huart->Init.BaudRate);
  usb_cdc_send_packet(msg, (uint16_t)strlen((char *)msg));
  return status;
}

uint8_t uart_getBaudRate(void){
  uint32_t baudrate = huart2.Init.BaudRate;
  switch (baudrate)
  {
    case 9600U:
      return UART_BAUD_SEL_9600;

    case 19200U:
      return UART_BAUD_SEL_19200;

    case 38400U:
      return UART_BAUD_SEL_38400;

    case 57600U:
      return UART_BAUD_SEL_57600;

    case 115200U:
      return UART_BAUD_SEL_115200;

    case 230400U:
      return UART_BAUD_SEL_230400;

    case 460800U:
      return UART_BAUD_SEL_460800;

    case 921600U:
      return UART_BAUD_SEL_921600;

    default:
      return 0xFFU;
  }
}
