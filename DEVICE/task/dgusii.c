#include "dgusii.h"
#include "main.h"

#include "uart.h"

#include <string.h>

typedef struct
{
    uint8_t data[DGUSII_MAX_DATA_LEN];
    uint16_t len;
} dgusii_tx_item_t;

typedef struct
{
    dgusii_rx_callback_t rx_cb;
    void *user_ctx;
    dgusii_frame_t last_frame;
    uint32_t rx_seq;
    dgusii_tx_item_t tx_queue[DGUSII_TX_QUEUE_DEPTH];
    uint8_t tx_head;
    uint8_t tx_tail;
    uint8_t tx_count;
    uint8_t rx_buf[DGUSII_RX_BUFFER_SIZE];
    uint16_t rx_len;
    volatile uint32_t rx_gap_ms;
    volatile uint32_t tx_gap_ms;
} dgusii_context_t;

static dgusii_context_t s_dgusii_ctx = {0};
static uint8_t s_dgusii_req_busy = 0U;
static uint8_t s_dgusii_req_ready = 0U;
static uint8_t s_dgusii_req_tx[DGUSII_MAX_DATA_LEN];
static uint16_t s_dgusii_req_tx_len = 0U;
static dgusii_frame_t s_dgusii_req_frame;
static volatile uint32_t s_dgusii_req_timeout_ms = 0U;
static volatile uint32_t s_dgusii_tick_ms = 0U;

static void dgusii_request_reset(void)
{
    s_dgusii_req_busy = 0U;
    s_dgusii_req_ready = 0U;
    s_dgusii_req_tx_len = 0U;
    s_dgusii_req_timeout_ms = 0U;
}

static uint8_t dgusii_request_frame_matches(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (s_dgusii_req_tx_len < 3U) || (len < 3U))
    {
        return 0U;
    }

    /* All current request/response transactions are DGUS read-VP frames.
     * Match both command and VP so a touch event cannot complete a different
     * pending request. */
    if ((s_dgusii_req_tx[0] == DGUSII_CMD_READ_VP) &&
        (data[0] == DGUSII_CMD_READ_VP) &&
        (data[1] == s_dgusii_req_tx[1]) &&
        (data[2] == s_dgusii_req_tx[2]))
    {
        return 1U;
    }

    return 0U;
}

static uint8_t dgusii_deadline_expired(uint32_t deadline_ms)
{
    return ((int32_t)(app_scheduler_millis() - deadline_ms) >= 0) ? 1U : 0U;
}

static void dgusii_reset_rx(void)
{
    s_dgusii_ctx.rx_len = 0U;
    s_dgusii_ctx.rx_gap_ms = 0U;
}

static HAL_StatusTypeDef dgusii_send_now(const uint8_t *data, uint16_t len)
{
    uint8_t frame[3U + DGUSII_MAX_DATA_LEN];
    uint16_t total_len;

    if ((data == NULL) || (len == 0U) || (len > DGUSII_MAX_DATA_LEN))
    {
        return HAL_ERROR;
    }

    frame[0] = DGUSII_FRAME_HEADER_H;
    frame[1] = DGUSII_FRAME_HEADER_L;
    frame[2] = (uint8_t)len;
    memcpy(&frame[3], data, len);

    total_len = (uint16_t)(3U + len);
    return uart_sendData(DGUSII_UART_PORT, frame, total_len, DGUSII_TX_TIMEOUT_MS);
}

static HAL_StatusTypeDef dgusii_enqueue_tx(const uint8_t *data, uint16_t len)
{
    dgusii_tx_item_t *item;

    if ((data == NULL) || (len == 0U))
    {
        return HAL_ERROR;
    }
    if ((len > DGUSII_MAX_DATA_LEN) || (s_dgusii_ctx.tx_count >= DGUSII_TX_QUEUE_DEPTH))
    {
        return HAL_ERROR;
    }

    item = &s_dgusii_ctx.tx_queue[s_dgusii_ctx.tx_tail];
    item->len = len;
    memcpy(item->data, data, len);

    s_dgusii_ctx.tx_tail = (uint8_t)((s_dgusii_ctx.tx_tail + 1U) % DGUSII_TX_QUEUE_DEPTH);
    s_dgusii_ctx.tx_count++;
    return HAL_OK;
}

static void dgusii_try_send_next(void)
{
    dgusii_tx_item_t *item;

    if ((s_dgusii_ctx.tx_count == 0U) || (dgusii_deadline_expired(s_dgusii_ctx.tx_gap_ms) == 0U))
    {
        return;
    }

    item = &s_dgusii_ctx.tx_queue[s_dgusii_ctx.tx_head];
    if (dgusii_send_now(item->data, item->len) != HAL_OK)
    {
        return;
    }

    s_dgusii_ctx.tx_head = (uint8_t)((s_dgusii_ctx.tx_head + 1U) % DGUSII_TX_QUEUE_DEPTH);
    s_dgusii_ctx.tx_count--;
    s_dgusii_ctx.tx_gap_ms = app_scheduler_millis() + DGUSII_TX_GAP_MS;
}

static void dgusii_publish_frame(const uint8_t *data, uint16_t len)
{
    s_dgusii_ctx.last_frame.len = len;

    if ((data != NULL) && (len > 0U))
    {
        memcpy(s_dgusii_ctx.last_frame.data, data, len);
    }
    s_dgusii_ctx.rx_seq++;

    if ((s_dgusii_req_busy != 0U) &&
        (dgusii_request_frame_matches(data, len) != 0U))
    {
        s_dgusii_req_frame = s_dgusii_ctx.last_frame;
        s_dgusii_req_ready = 1U;
        return;
    }

    if (s_dgusii_ctx.rx_cb != NULL)
    {
        s_dgusii_ctx.rx_cb(&s_dgusii_ctx.last_frame, s_dgusii_ctx.user_ctx);
    }
}

static void dgusii_consume_rx(uint16_t used_len)
{
    if ((used_len == 0U) || (used_len > s_dgusii_ctx.rx_len))
    {
        return;
    }

    s_dgusii_ctx.rx_len = (uint16_t)(s_dgusii_ctx.rx_len - used_len);
    if (s_dgusii_ctx.rx_len > 0U)
    {
        memmove(s_dgusii_ctx.rx_buf, &s_dgusii_ctx.rx_buf[used_len], s_dgusii_ctx.rx_len);
    }
}

static void dgusii_parse_rx(void)
{
    uint16_t frame_len;
    uint16_t data_len;

    while (s_dgusii_ctx.rx_len >= 4U)
    {
        if ((s_dgusii_ctx.rx_buf[0] != DGUSII_FRAME_HEADER_H) ||
            (s_dgusii_ctx.rx_buf[1] != DGUSII_FRAME_HEADER_L))
        {
            dgusii_consume_rx(1U);
            continue;
        }

        frame_len = (uint16_t)(3U + s_dgusii_ctx.rx_buf[2]);
        if ((frame_len < 4U) || (frame_len > DGUSII_RX_BUFFER_SIZE))
        {
            dgusii_consume_rx(1U);
            continue;
        }

        if (s_dgusii_ctx.rx_len < frame_len)
        {
            break;
        }

        data_len = (uint16_t)s_dgusii_ctx.rx_buf[2];
        if ((data_len == 0U) || (data_len > DGUSII_MAX_DATA_LEN))
        {
            dgusii_consume_rx(frame_len);
            continue;
        }

        dgusii_publish_frame(&s_dgusii_ctx.rx_buf[3], data_len);
        dgusii_consume_rx(frame_len);
    }
}

void dgusii_init(dgusii_rx_callback_t rx_cb, void *user_ctx)
{
    memset(&s_dgusii_ctx, 0, sizeof(s_dgusii_ctx));
    memset(&s_dgusii_req_frame, 0, sizeof(s_dgusii_req_frame));
    dgusii_request_reset();
    s_dgusii_ctx.rx_cb = rx_cb;
    s_dgusii_ctx.user_ctx = user_ctx;
    uart_startReceiveIT(DGUSII_UART_PORT);
}

void dgusii_taskDelay(void)
{
}

uint32_t dgusii_tick_ms(void)
{
    s_dgusii_tick_ms = app_scheduler_millis();
    return s_dgusii_tick_ms;
}

uint8_t dgusii_timeout_expired(uint32_t start_ms, uint32_t timeout_ms)
{
    return ((uint32_t)(app_scheduler_millis() - start_ms) > timeout_ms) ? 1U : 0U;
}

void dgusii_recover(void)
{
    dgusii_reset_rx();
    s_dgusii_ctx.tx_head = 0U;
    s_dgusii_ctx.tx_tail = 0U;
    s_dgusii_ctx.tx_count = 0U;
    s_dgusii_ctx.tx_gap_ms = 0U;
    dgusii_request_reset();
    uart_startReceiveIT(DGUSII_UART_PORT);
}

void dgusii_poll(void)
{
    uint8_t temp[32];
    uint16_t read_len;

    if ((s_dgusii_ctx.rx_len > 0U) && (dgusii_deadline_expired(s_dgusii_ctx.rx_gap_ms) != 0U))
    {
        dgusii_reset_rx();
    }

    do
    {
        read_len = uart_getData(DGUSII_UART_PORT, temp, (uint16_t)sizeof(temp));
        if (read_len == 0U)
        {
            break;
        }

        if ((uint16_t)(s_dgusii_ctx.rx_len + read_len) > DGUSII_RX_BUFFER_SIZE)
        {
            dgusii_reset_rx();
            if (read_len > DGUSII_RX_BUFFER_SIZE)
            {
                continue;
            }
        }

        memcpy(&s_dgusii_ctx.rx_buf[s_dgusii_ctx.rx_len], temp, read_len);
        s_dgusii_ctx.rx_len = (uint16_t)(s_dgusii_ctx.rx_len + read_len);
        s_dgusii_ctx.rx_gap_ms = app_scheduler_millis() + DGUSII_RX_GAP_MS;
        dgusii_parse_rx();
    } while (read_len > 0U);

    dgusii_try_send_next();
}

HAL_StatusTypeDef dgusii_send(const uint8_t *data, uint16_t len)
{
    return dgusii_enqueue_tx(data, len);
}

uint8_t dgusii_request(const uint8_t *tx, uint16_t tx_len, const dgusii_frame_t **rx_frame)
{
    if (rx_frame != NULL)
    {
        *rx_frame = NULL;
    }

    if ((tx == NULL) || (tx_len < 3U) || (tx_len > DGUSII_MAX_DATA_LEN))
    {
        return DGUSII_REQ_SFAIL;
    }

    if (s_dgusii_req_busy == 0U)
    {
        if (dgusii_send(tx, tx_len) != HAL_OK)
        {
            return DGUSII_REQ_SFAIL;
        }

        memcpy(s_dgusii_req_tx, tx, tx_len);
        s_dgusii_req_tx_len = tx_len;
        s_dgusii_req_busy = 1U;
        s_dgusii_req_ready = 0U;
        s_dgusii_req_timeout_ms = app_scheduler_millis() + DGUSII_REQ_TIMEOUT_MS;
        return DGUSII_REQ_BUSY;
    }

    /* Another state machine owns the single request slot. */
    if ((tx_len != s_dgusii_req_tx_len) ||
        (memcmp(tx, s_dgusii_req_tx, tx_len) != 0))
    {
        return DGUSII_REQ_BUSY;
    }

    if (s_dgusii_req_ready != 0U)
    {
        if (rx_frame != NULL)
        {
            *rx_frame = &s_dgusii_req_frame;
        }
        dgusii_request_reset();
        return DGUSII_REQ_DONE;
    }

    if (dgusii_deadline_expired(s_dgusii_req_timeout_ms) != 0U)
    {
        /* A missing read response must not discard unrelated queued UI writes. */
        dgusii_request_reset();
        return DGUSII_REQ_RFAIL;
    }

    return DGUSII_REQ_BUSY;
}
