#ifndef __DGUSII_H
#define __DGUSII_H

#ifdef __cplusplus
extern "C" {
#endif

#include "uart.h"
#include <stdint.h>

#define DGUSII_UART_PORT         UART_PORT_USART3
#define DGUSII_FRAME_HEADER_H    0x5AU
#define DGUSII_FRAME_HEADER_L    0xA5U
#define DGUSII_CMD_WRITE_VP      0x82U
#define DGUSII_CMD_READ_VP       0x83U
#define DGUSII_MAX_DATA_LEN      255U
#define DGUSII_RX_BUFFER_SIZE    255U
#define DGUSII_RX_GAP_MS         20U
#define DGUSII_TX_TIMEOUT_MS     100U
#define DGUSII_TX_QUEUE_DEPTH    8U
#define DGUSII_TX_GAP_MS         100U
#define DGUSII_REQ_TIMEOUT_MS    200U

#define DGUSII_REQ_BUSY          0U
#define DGUSII_REQ_DONE          1U
#define DGUSII_REQ_SFAIL         2U
#define DGUSII_REQ_RFAIL         3U

typedef struct
{
    uint8_t data[DGUSII_MAX_DATA_LEN];
    uint16_t len;
} dgusii_frame_t;

typedef void (*dgusii_rx_callback_t)(const dgusii_frame_t *frame, void *user_ctx);

void dgusii_init(dgusii_rx_callback_t rx_cb, void *user_ctx);
void dgusii_taskDelay(void);
void dgusii_poll(void);
uint32_t dgusii_tick_ms(void);
uint8_t dgusii_timeout_expired(uint32_t start_ms, uint32_t timeout_ms);
void dgusii_recover(void);

HAL_StatusTypeDef dgusii_send(const uint8_t *data, uint16_t len);
uint8_t dgusii_request(const uint8_t *tx, uint16_t tx_len, const dgusii_frame_t **rx_frame);

#ifdef __cplusplus
}
#endif

#endif /* __DGUSII_H */
