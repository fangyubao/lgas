#include "modbus_master.h"
#include "main.h"

#include "rs485.h"
#include "usbd_cdc_if.h"

#include <stdio.h>
#include <string.h>

#define MODBUS_MASTER_MAX_FRAME       256U
#define MODBUS_MASTER_GAP_MS          5U

static volatile uint32_t s_modbus_timeout_tick = 0U;
static volatile uint32_t s_modbus_gap_tick = 0U;
static uint8_t s_modbus_req_buf[MODBUS_MASTER_MAX_FRAME];
static uint8_t s_modbus_rsp_buf[MODBUS_MASTER_MAX_FRAME];
static modbus_master_state_t s_modbus_state = MODBUS_MASTER_IDLE;
static modbus_master_error_t s_modbus_last_error = MODBUS_MASTER_ERR_NONE;

typedef enum
{
    MODBUS_OP_NONE = 0U,
    MODBUS_OP_READ_HOLDING,
    MODBUS_OP_READ_INPUT,
    MODBUS_OP_WRITE_MULTI
} modbus_master_op_t;

typedef struct
{
    modbus_master_op_t op;
    uint8_t slave_addr;
    uint16_t addr;
    uint16_t count;
    uint16_t req_len;
    uint16_t rsp_len;
    uint16_t *read_out;
} modbus_master_ctx_t;

static modbus_master_ctx_t s_modbus_ctx;
static void modbus_default_log_writer(const char *msg);
static modbus_master_log_writer_t s_modbus_log_writer = modbus_default_log_writer;

static uint8_t modbus_deadline_expired(uint32_t deadline_ms)
{
    return ((int32_t)(app_scheduler_millis() - deadline_ms) >= 0) ? 1U : 0U;
}

static void modbus_report_error(modbus_master_error_t err,
                                uint8_t slave_addr,
                                uint8_t func,
                                uint16_t detail)
{
    char buf[96];
    int n;

    s_modbus_last_error = err;
    if ((err == MODBUS_MASTER_ERR_NONE) || (s_modbus_log_writer == NULL))
    {
        return;
    }

    n = snprintf(buf, sizeof(buf), "MB_ERR e=%u st=%u sa=%02X fc=%02X d=%u\r\n",
                 (unsigned)err,
                 (unsigned)s_modbus_state,
                 (unsigned)slave_addr,
                 (unsigned)func,
                 (unsigned)detail);
    if (n > 0)
    {
        s_modbus_log_writer(buf);
    }
}

static void modbus_clear_error(void)
{
    s_modbus_last_error = MODBUS_MASTER_ERR_NONE;
}

static void modbus_default_log_writer(const char *msg)
{
    if (msg == NULL)
    {
        return;
    }
    (void)usb_cdc_send_packet((uint8_t *)msg, (uint16_t)strlen(msg));
}

static HAL_StatusTypeDef modbus_expect_exception_or_addr(uint8_t slave_addr,
                                                         uint8_t func,
                                                         const uint8_t *rsp,
                                                         uint16_t rsp_len);
static HAL_StatusTypeDef modbus_exchange_frame(const uint8_t *req,
                                               uint16_t req_len,
                                               uint8_t *rsp,
                                               uint16_t *rsp_len,
                                               uint32_t timeout_ms);
static HAL_StatusTypeDef modbus_start_read_regs(uint8_t slave_addr,
                                                uint8_t func,
                                                modbus_master_op_t op,
                                                uint16_t reg_addr,
                                                uint16_t reg_count,
                                                uint16_t *out_regs,
                                                uint32_t timeout_ms);
static HAL_StatusTypeDef modbus_validate_read_regs(uint8_t func);
static HAL_StatusTypeDef modbus_wait_current_transaction(void);
static HAL_StatusTypeDef modbus_read_bits(uint8_t slave_addr,
                                          uint8_t func,
                                          uint16_t bit_addr,
                                          uint16_t bit_count,
                                          uint8_t *out_bits,
                                          uint32_t timeout_ms);
static HAL_StatusTypeDef modbus_read_regs(uint8_t slave_addr,
                                          uint8_t func,
                                          uint16_t reg_addr,
                                          uint16_t reg_count,
                                          uint16_t *out_regs,
                                          uint32_t timeout_ms);

static uint16_t modbus_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t j;

    for (i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (j = 0U; j < 8U; j++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

static void modbus_flush_rx(void)
{
    uint8_t dump[32];
    uint16_t read_len;

    do
    {
        read_len = rs485_read(dump, sizeof(dump));
    } while (read_len > 0U);
}

static HAL_StatusTypeDef modbus_start_transaction(const uint8_t *req,
                                                  uint16_t req_len,
                                                  uint32_t timeout_ms)
{
    if ((req == NULL) || (req_len == 0U))
    {
        modbus_report_error(MODBUS_MASTER_ERR_INVALID_PARAM, 0U, 0U, req_len);
        return HAL_ERROR;
    }

    if (s_modbus_state == MODBUS_MASTER_BUSY)
    {
        return HAL_BUSY;
    }

    modbus_flush_rx();
    if (rs485_write(req, req_len, timeout_ms) != HAL_OK)
    {
        s_modbus_state = MODBUS_MASTER_FAIL;
        modbus_report_error(MODBUS_MASTER_ERR_TX_FAIL, req[0], req[1], 0U);
        return HAL_ERROR;
    }

    s_modbus_ctx.req_len = req_len;
    s_modbus_ctx.rsp_len = 0U;
    s_modbus_timeout_tick = app_scheduler_millis() + timeout_ms;
    s_modbus_gap_tick = 0U;
    s_modbus_state = MODBUS_MASTER_BUSY;
    modbus_clear_error();
    return HAL_OK;
}

static HAL_StatusTypeDef modbus_exchange_frame(const uint8_t *req,
                                               uint16_t req_len,
                                               uint8_t *rsp,
                                               uint16_t *rsp_len,
                                               uint32_t timeout_ms)
{
    if ((req == NULL) || (rsp == NULL) || (rsp_len == NULL))
    {
        modbus_report_error(MODBUS_MASTER_ERR_INVALID_PARAM, 0U, 0U, 0U);
        return HAL_ERROR;
    }

    if (s_modbus_state == MODBUS_MASTER_BUSY)
    {
        return HAL_BUSY;
    }

    modbus_flush_rx();
    if (rs485_write(req, req_len, timeout_ms) != HAL_OK)
    {
        modbus_report_error(MODBUS_MASTER_ERR_TX_FAIL, req[0], req[1], 0U);
        return HAL_ERROR;
    }

    *rsp_len = 0U;
    s_modbus_timeout_tick = app_scheduler_millis() + timeout_ms;
    s_modbus_gap_tick = 0U;

    while (modbus_deadline_expired(s_modbus_timeout_tick) == 0U)
    {
        uint16_t remain;
        uint16_t read_len;

        if (*rsp_len >= MODBUS_MASTER_MAX_FRAME)
        {
            modbus_report_error(MODBUS_MASTER_ERR_RX_OVERFLOW, req[0], req[1], *rsp_len);
            return HAL_ERROR;
        }

        remain = (uint16_t)(MODBUS_MASTER_MAX_FRAME - *rsp_len);
        read_len = rs485_read(&rsp[*rsp_len], remain);
        if (read_len > 0U)
        {
            *rsp_len = (uint16_t)(*rsp_len + read_len);
            s_modbus_gap_tick = app_scheduler_millis() + MODBUS_MASTER_GAP_MS;
        }

        if ((*rsp_len > 0U) && (modbus_deadline_expired(s_modbus_gap_tick) != 0U))
        {
            break;
        }
    }

    if (*rsp_len < 5U)
    {
        modbus_report_error(MODBUS_MASTER_ERR_FRAME_SHORT, req[0], req[1], *rsp_len);
        return HAL_ERROR;
    }

    {
        uint16_t crc_calc = modbus_crc16(rsp, (uint16_t)(*rsp_len - 2U));
        uint16_t crc_recv = (uint16_t)rsp[*rsp_len - 2U] | ((uint16_t)rsp[*rsp_len - 1U] << 8);

        if (crc_calc != crc_recv)
        {
            modbus_report_error(MODBUS_MASTER_ERR_CRC, req[0], req[1], *rsp_len);
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef modbus_validate_read_regs(uint8_t func)
{
    uint16_t i;
    uint8_t *rsp = s_modbus_rsp_buf;
    uint16_t rsp_len = s_modbus_ctx.rsp_len;

    if ((s_modbus_ctx.read_out == NULL) || (rsp_len < 5U))
    {
        modbus_report_error(MODBUS_MASTER_ERR_FRAME_SHORT, s_modbus_ctx.slave_addr, func, rsp_len);
        return HAL_ERROR;
    }

    if (modbus_expect_exception_or_addr(s_modbus_ctx.slave_addr, func, rsp, rsp_len) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if ((rsp_len != (uint16_t)(5U + s_modbus_ctx.count * 2U)) ||
        (rsp[1] != func) ||
        (rsp[2] != (uint8_t)(s_modbus_ctx.count * 2U)))
    {
        modbus_report_error(MODBUS_MASTER_ERR_RESPONSE_MISMATCH, s_modbus_ctx.slave_addr, func, rsp_len);
        return HAL_ERROR;
    }

    for (i = 0U; i < s_modbus_ctx.count; i++)
    {
        s_modbus_ctx.read_out[i] = (uint16_t)(((uint16_t)rsp[3U + i * 2U] << 8) | rsp[4U + i * 2U]);
    }

    return HAL_OK;
}

static HAL_StatusTypeDef modbus_validate_write_multi(void)
{
    uint8_t *req = s_modbus_req_buf;
    uint8_t *rsp = s_modbus_rsp_buf;
    uint16_t rsp_len = s_modbus_ctx.rsp_len;

    if (modbus_expect_exception_or_addr(s_modbus_ctx.slave_addr, 0x10U, rsp, rsp_len) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if ((rsp_len != 8U) ||
        (rsp[1] != 0x10U) ||
        (rsp[2] != req[2]) ||
        (rsp[3] != req[3]) ||
        (rsp[4] != req[4]) ||
        (rsp[5] != req[5]))
    {
        modbus_report_error(MODBUS_MASTER_ERR_RESPONSE_MISMATCH, s_modbus_ctx.slave_addr, 0x10U, rsp_len);
        return HAL_ERROR;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef modbus_finalize_transaction(void)
{
    uint16_t crc_calc;
    uint16_t crc_recv;

    if (s_modbus_ctx.rsp_len < 5U)
    {
        modbus_report_error(MODBUS_MASTER_ERR_FRAME_SHORT, s_modbus_ctx.slave_addr, 0U, s_modbus_ctx.rsp_len);
        return HAL_ERROR;
    }

    crc_calc = modbus_crc16(s_modbus_rsp_buf, (uint16_t)(s_modbus_ctx.rsp_len - 2U));
    crc_recv = (uint16_t)s_modbus_rsp_buf[s_modbus_ctx.rsp_len - 2U] |
               ((uint16_t)s_modbus_rsp_buf[s_modbus_ctx.rsp_len - 1U] << 8);
    if (crc_calc != crc_recv)
    {
        modbus_report_error(MODBUS_MASTER_ERR_CRC, s_modbus_ctx.slave_addr, 0U, s_modbus_ctx.rsp_len);
        return HAL_ERROR;
    }

    switch (s_modbus_ctx.op)
    {
    case MODBUS_OP_READ_HOLDING:
        return modbus_validate_read_regs(0x03U);

    case MODBUS_OP_READ_INPUT:
        return modbus_validate_read_regs(0x04U);

    case MODBUS_OP_WRITE_MULTI:
        return modbus_validate_write_multi();

    default:
        modbus_report_error(MODBUS_MASTER_ERR_UNSUPPORTED_OP, s_modbus_ctx.slave_addr, 0U, (uint16_t)s_modbus_ctx.op);
        return HAL_ERROR;
    }
}

/* ------------------------------------------------------------------ */
/* Common Modbus API                                                  */
/* ------------------------------------------------------------------ */

void modbus_master_init(void)
{
    memset(&s_modbus_ctx, 0, sizeof(s_modbus_ctx));
    s_modbus_state = MODBUS_MASTER_IDLE;
    s_modbus_last_error = MODBUS_MASTER_ERR_NONE;
    rs485_init();
}

void modbus_master_clear(void)
{
    memset(&s_modbus_ctx, 0, sizeof(s_modbus_ctx));
    s_modbus_state = MODBUS_MASTER_IDLE;
    s_modbus_timeout_tick = 0U;
    s_modbus_gap_tick = 0U;
    s_modbus_last_error = MODBUS_MASTER_ERR_NONE;
    modbus_flush_rx();
}

modbus_master_state_t modbus_master_get_state(void)
{
    return s_modbus_state;
}

modbus_master_error_t modbus_master_get_last_error(void)
{
    return s_modbus_last_error;
}

void modbus_master_set_log_writer(modbus_master_log_writer_t writer)
{
    s_modbus_log_writer = writer;
}

void modbus_master_taskDelay(void)
{
}

void modbus_master_poll(void)
{
    uint16_t remain;
    uint16_t read_len;

    if (s_modbus_state != MODBUS_MASTER_BUSY)
    {
        return;
    }

    if (s_modbus_ctx.rsp_len >= MODBUS_MASTER_MAX_FRAME)
    {
        s_modbus_state = MODBUS_MASTER_FAIL;
        modbus_report_error(MODBUS_MASTER_ERR_RX_OVERFLOW, s_modbus_ctx.slave_addr, 0U, s_modbus_ctx.rsp_len);
        return;
    }

    remain = (uint16_t)(MODBUS_MASTER_MAX_FRAME - s_modbus_ctx.rsp_len);
    read_len = rs485_read(&s_modbus_rsp_buf[s_modbus_ctx.rsp_len], remain);
    if (read_len > 0U)
    {
        s_modbus_ctx.rsp_len = (uint16_t)(s_modbus_ctx.rsp_len + read_len);
        s_modbus_gap_tick = app_scheduler_millis() + MODBUS_MASTER_GAP_MS;
        return;
    }

    if ((s_modbus_ctx.rsp_len > 0U) && (modbus_deadline_expired(s_modbus_gap_tick) != 0U))
    {
        s_modbus_state = (modbus_finalize_transaction() == HAL_OK) ? MODBUS_MASTER_DONE : MODBUS_MASTER_FAIL;
        return;
    }

    if (modbus_deadline_expired(s_modbus_timeout_tick) != 0U)
    {
        s_modbus_state = MODBUS_MASTER_FAIL;
        modbus_report_error(MODBUS_MASTER_ERR_TIMEOUT, s_modbus_ctx.slave_addr, 0U, (uint16_t)s_modbus_ctx.count);
    }
}

static HAL_StatusTypeDef modbus_start_read_regs(uint8_t slave_addr,
                                                uint8_t func,
                                                modbus_master_op_t op,
                                                uint16_t reg_addr,
                                                uint16_t reg_count,
                                                uint16_t *out_regs,
                                                uint32_t timeout_ms)
{
    uint16_t crc;

    if ((out_regs == NULL) || (reg_count == 0U) || (reg_count > 125U))
    {
        modbus_report_error(MODBUS_MASTER_ERR_INVALID_PARAM, slave_addr, func, reg_count);
        return HAL_ERROR;
    }

    s_modbus_req_buf[0] = slave_addr;
    s_modbus_req_buf[1] = func;
    s_modbus_req_buf[2] = (uint8_t)(reg_addr >> 8);
    s_modbus_req_buf[3] = (uint8_t)(reg_addr & 0xFFU);
    s_modbus_req_buf[4] = (uint8_t)(reg_count >> 8);
    s_modbus_req_buf[5] = (uint8_t)(reg_count & 0xFFU);
    crc = modbus_crc16(s_modbus_req_buf, 6U);
    s_modbus_req_buf[6] = (uint8_t)(crc & 0xFFU);
    s_modbus_req_buf[7] = (uint8_t)(crc >> 8);

    s_modbus_ctx.op = op;
    s_modbus_ctx.slave_addr = slave_addr;
    s_modbus_ctx.addr = reg_addr;
    s_modbus_ctx.count = reg_count;
    s_modbus_ctx.read_out = out_regs;
    return modbus_start_transaction(s_modbus_req_buf, 8U, timeout_ms);
}

static HAL_StatusTypeDef modbus_wait_current_transaction(void)
{
    while (modbus_master_get_state() == MODBUS_MASTER_BUSY)
    {
        modbus_master_poll();
    }

    return (modbus_master_get_state() == MODBUS_MASTER_DONE) ? HAL_OK : HAL_ERROR;
}

/* ------------------------------------------------------------------ */
/* Non-blocking Modbus API                                            */
/* ------------------------------------------------------------------ */

HAL_StatusTypeDef modbus_master_start_read_holding(uint8_t slave_addr,
                                                   uint16_t reg_addr,
                                                   uint16_t reg_count,
                                                   uint16_t *out_regs,
                                                   uint32_t timeout_ms)
{
    return modbus_start_read_regs(slave_addr,
                                  0x03U,
                                  MODBUS_OP_READ_HOLDING,
                                  reg_addr,
                                  reg_count,
                                  out_regs,
                                  timeout_ms);
}

HAL_StatusTypeDef modbus_master_start_read_input(uint8_t slave_addr,
                                                 uint16_t reg_addr,
                                                 uint16_t reg_count,
                                                 uint16_t *out_regs,
                                                 uint32_t timeout_ms)
{
    return modbus_start_read_regs(slave_addr,
                                  0x04U,
                                  MODBUS_OP_READ_INPUT,
                                  reg_addr,
                                  reg_count,
                                  out_regs,
                                  timeout_ms);
}

HAL_StatusTypeDef modbus_master_start_write_multi(uint8_t slave_addr,
                                                  uint16_t reg_addr,
                                                  const uint16_t *values,
                                                  uint16_t reg_count,
                                                  uint32_t timeout_ms)
{
    uint16_t crc;
    uint16_t i;
    uint16_t req_len;

    if ((values == NULL) || (reg_count == 0U) || (reg_count > 123U))
    {
        modbus_report_error(MODBUS_MASTER_ERR_INVALID_PARAM, slave_addr, 0x10U, reg_count);
        return HAL_ERROR;
    }

    s_modbus_req_buf[0] = slave_addr;
    s_modbus_req_buf[1] = 0x10U;
    s_modbus_req_buf[2] = (uint8_t)(reg_addr >> 8);
    s_modbus_req_buf[3] = (uint8_t)(reg_addr & 0xFFU);
    s_modbus_req_buf[4] = (uint8_t)(reg_count >> 8);
    s_modbus_req_buf[5] = (uint8_t)(reg_count & 0xFFU);
    s_modbus_req_buf[6] = (uint8_t)(reg_count * 2U);
    for (i = 0U; i < reg_count; i++)
    {
        s_modbus_req_buf[7U + i * 2U] = (uint8_t)(values[i] >> 8);
        s_modbus_req_buf[8U + i * 2U] = (uint8_t)(values[i] & 0xFFU);
    }

    req_len = (uint16_t)(7U + reg_count * 2U);
    crc = modbus_crc16(s_modbus_req_buf, req_len);
    s_modbus_req_buf[req_len] = (uint8_t)(crc & 0xFFU);
    s_modbus_req_buf[req_len + 1U] = (uint8_t)(crc >> 8);

    s_modbus_ctx.op = MODBUS_OP_WRITE_MULTI;
    s_modbus_ctx.slave_addr = slave_addr;
    s_modbus_ctx.addr = reg_addr;
    s_modbus_ctx.count = reg_count;
    s_modbus_ctx.read_out = NULL;
    return modbus_start_transaction(s_modbus_req_buf, (uint16_t)(req_len + 2U), timeout_ms);
}

/* ------------------------------------------------------------------ */
/* Blocking Modbus API                                                */
/* ------------------------------------------------------------------ */

static HAL_StatusTypeDef modbus_expect_exception_or_addr(uint8_t slave_addr,
                                                         uint8_t func,
                                                         const uint8_t *rsp,
                                                         uint16_t rsp_len)
{
    if ((rsp_len == 5U) && (rsp[0] == slave_addr) && (rsp[1] == (uint8_t)(func | 0x80U)))
    {
        modbus_report_error(MODBUS_MASTER_ERR_EXCEPTION, slave_addr, func, rsp[2]);
        return HAL_ERROR;
    }

    if (rsp[0] != slave_addr)
    {
        modbus_report_error(MODBUS_MASTER_ERR_RESPONSE_MISMATCH, slave_addr, func, rsp[0]);
        return HAL_ERROR;
    }

    return HAL_OK;
}

static void modbus_unpack_bits(const uint8_t *src, uint16_t bit_count, uint8_t *dst)
{
    uint16_t i;

    for (i = 0U; i < bit_count; i++)
    {
        dst[i] = (uint8_t)((src[i / 8U] >> (i % 8U)) & 0x01U);
    }
}

static void modbus_pack_bits(const uint8_t *src, uint16_t bit_count, uint8_t *dst)
{
    uint16_t i;
    uint16_t byte_count = (uint16_t)((bit_count + 7U) / 8U);

    memset(dst, 0, byte_count);
    for (i = 0U; i < bit_count; i++)
    {
        if (src[i] != 0U)
        {
            dst[i / 8U] |= (uint8_t)(1U << (i % 8U));
        }
    }
}

static HAL_StatusTypeDef modbus_read_bits(uint8_t slave_addr,
                                          uint8_t func,
                                          uint16_t bit_addr,
                                          uint16_t bit_count,
                                          uint8_t *out_bits,
                                          uint32_t timeout_ms)
{
    uint8_t *req = s_modbus_req_buf;
    uint8_t *rsp = s_modbus_rsp_buf;
    uint16_t rsp_len;
    uint16_t crc;
    uint8_t byte_count;

    if ((out_bits == NULL) || (bit_count == 0U) || (bit_count > 2000U))
    {
        return HAL_ERROR;
    }

    req[0] = slave_addr;
    req[1] = func;
    req[2] = (uint8_t)(bit_addr >> 8);
    req[3] = (uint8_t)(bit_addr & 0xFFU);
    req[4] = (uint8_t)(bit_count >> 8);
    req[5] = (uint8_t)(bit_count & 0xFFU);
    crc = modbus_crc16(req, 6U);
    req[6] = (uint8_t)(crc & 0xFFU);
    req[7] = (uint8_t)(crc >> 8);

    if (modbus_exchange_frame(req, 8U, rsp, &rsp_len, timeout_ms) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (modbus_expect_exception_or_addr(slave_addr, func, rsp, rsp_len) != HAL_OK)
    {
        return HAL_ERROR;
    }

    byte_count = (uint8_t)((bit_count + 7U) / 8U);
    if ((rsp_len != (uint16_t)(5U + byte_count)) ||
        (rsp[1] != func) ||
        (rsp[2] != byte_count))
    {
        return HAL_ERROR;
    }

    modbus_unpack_bits(&rsp[3], bit_count, out_bits);
    return HAL_OK;
}

static HAL_StatusTypeDef modbus_read_regs(uint8_t slave_addr,
                                          uint8_t func,
                                          uint16_t reg_addr,
                                          uint16_t reg_count,
                                          uint16_t *out_regs,
                                          uint32_t timeout_ms)
{
    uint8_t *req = s_modbus_req_buf;
    uint8_t *rsp = s_modbus_rsp_buf;
    uint16_t rsp_len;
    uint16_t crc;
    uint16_t i;

    if ((out_regs == NULL) || (reg_count == 0U) || (reg_count > 125U))
    {
        return HAL_ERROR;
    }

    req[0] = slave_addr;
    req[1] = func;
    req[2] = (uint8_t)(reg_addr >> 8);
    req[3] = (uint8_t)(reg_addr & 0xFFU);
    req[4] = (uint8_t)(reg_count >> 8);
    req[5] = (uint8_t)(reg_count & 0xFFU);
    crc = modbus_crc16(req, 6U);
    req[6] = (uint8_t)(crc & 0xFFU);
    req[7] = (uint8_t)(crc >> 8);

    if (modbus_exchange_frame(req, 8U, rsp, &rsp_len, timeout_ms) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (modbus_expect_exception_or_addr(slave_addr, func, rsp, rsp_len) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if ((rsp_len != (uint16_t)(5U + reg_count * 2U)) ||
        (rsp[1] != func) ||
        (rsp[2] != (uint8_t)(reg_count * 2U)))
    {
        return HAL_ERROR;
    }

    for (i = 0U; i < reg_count; i++)
    {
        out_regs[i] = (uint16_t)((uint16_t)rsp[3U + i * 2U] << 8) | rsp[4U + i * 2U];
    }

    return HAL_OK;
}

HAL_StatusTypeDef modbus_master_read_coils(uint8_t slave_addr,
                                           uint16_t coil_addr,
                                           uint16_t coil_count,
                                           uint8_t *out_bits,
                                           uint32_t timeout_ms)
{
    return modbus_read_bits(slave_addr, 0x01U, coil_addr, coil_count, out_bits, timeout_ms);
}

HAL_StatusTypeDef modbus_master_read_discrete_inputs(uint8_t slave_addr,
                                                     uint16_t input_addr,
                                                     uint16_t input_count,
                                                     uint8_t *out_bits,
                                                     uint32_t timeout_ms)
{
    return modbus_read_bits(slave_addr, 0x02U, input_addr, input_count, out_bits, timeout_ms);
}

HAL_StatusTypeDef modbus_master_read_holding(uint8_t slave_addr,
                                             uint16_t reg_addr,
                                             uint16_t reg_count,
                                             uint16_t *out_regs,
                                             uint32_t timeout_ms)
{
    HAL_StatusTypeDef status = modbus_master_start_read_holding(slave_addr, reg_addr, reg_count, out_regs, timeout_ms);

    if (status != HAL_OK)
    {
        return status;
    }

    status = modbus_wait_current_transaction();
    modbus_master_clear();
    return status;
}

HAL_StatusTypeDef modbus_master_read_input(uint8_t slave_addr,
                                           uint16_t reg_addr,
                                           uint16_t reg_count,
                                           uint16_t *out_regs,
                                           uint32_t timeout_ms)
{
    HAL_StatusTypeDef status = modbus_master_start_read_input(slave_addr, reg_addr, reg_count, out_regs, timeout_ms);

    if (status != HAL_OK)
    {
        return status;
    }

    status = modbus_wait_current_transaction();
    modbus_master_clear();
    return status;
}

HAL_StatusTypeDef modbus_master_write_single_coil(uint8_t slave_addr,
                                                  uint16_t coil_addr,
                                                  uint8_t value,
                                                  uint32_t timeout_ms)
{
    uint8_t *req = s_modbus_req_buf;
    uint8_t *rsp = s_modbus_rsp_buf;
    uint16_t rsp_len;
    uint16_t crc;
    uint16_t coil_value = (value != 0U) ? 0xFF00U : 0x0000U;

    req[0] = slave_addr;
    req[1] = 0x05U;
    req[2] = (uint8_t)(coil_addr >> 8);
    req[3] = (uint8_t)(coil_addr & 0xFFU);
    req[4] = (uint8_t)(coil_value >> 8);
    req[5] = (uint8_t)(coil_value & 0xFFU);
    crc = modbus_crc16(req, 6U);
    req[6] = (uint8_t)(crc & 0xFFU);
    req[7] = (uint8_t)(crc >> 8);

    if (modbus_exchange_frame(req, 8U, rsp, &rsp_len, timeout_ms) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (modbus_expect_exception_or_addr(slave_addr, 0x05U, rsp, rsp_len) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return ((rsp_len == 8U) && (memcmp(req, rsp, 8U) == 0)) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef modbus_master_write_single(uint8_t slave_addr,
                                             uint16_t reg_addr,
                                             uint16_t value,
                                             uint32_t timeout_ms)
{
    uint8_t *req = s_modbus_req_buf;
    uint8_t *rsp = s_modbus_rsp_buf;
    uint16_t rsp_len;
    uint16_t crc;

    req[0] = slave_addr;
    req[1] = 0x06U;
    req[2] = (uint8_t)(reg_addr >> 8);
    req[3] = (uint8_t)(reg_addr & 0xFFU);
    req[4] = (uint8_t)(value >> 8);
    req[5] = (uint8_t)(value & 0xFFU);
    crc = modbus_crc16(req, 6U);
    req[6] = (uint8_t)(crc & 0xFFU);
    req[7] = (uint8_t)(crc >> 8);

    if (modbus_exchange_frame(req, 8U, rsp, &rsp_len, timeout_ms) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (modbus_expect_exception_or_addr(slave_addr, 0x06U, rsp, rsp_len) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return ((rsp_len == 8U) && (memcmp(req, rsp, 8U) == 0)) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef modbus_master_write_multi_coils(uint8_t slave_addr,
                                                  uint16_t coil_addr,
                                                  const uint8_t *values,
                                                  uint16_t coil_count,
                                                  uint32_t timeout_ms)
{
    uint8_t *req = s_modbus_req_buf;
    uint8_t *rsp = s_modbus_rsp_buf;
    uint16_t rsp_len;
    uint16_t req_len;
    uint16_t crc;
    uint8_t byte_count;

    if ((values == NULL) || (coil_count == 0U) || (coil_count > 1968U))
    {
        return HAL_ERROR;
    }

    byte_count = (uint8_t)((coil_count + 7U) / 8U);
    req[0] = slave_addr;
    req[1] = 0x0FU;
    req[2] = (uint8_t)(coil_addr >> 8);
    req[3] = (uint8_t)(coil_addr & 0xFFU);
    req[4] = (uint8_t)(coil_count >> 8);
    req[5] = (uint8_t)(coil_count & 0xFFU);
    req[6] = byte_count;
    modbus_pack_bits(values, coil_count, &req[7]);

    req_len = (uint16_t)(7U + byte_count);
    crc = modbus_crc16(req, req_len);
    req[req_len] = (uint8_t)(crc & 0xFFU);
    req[req_len + 1U] = (uint8_t)(crc >> 8);
    req_len = (uint16_t)(req_len + 2U);

    if (modbus_exchange_frame(req, req_len, rsp, &rsp_len, timeout_ms) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (modbus_expect_exception_or_addr(slave_addr, 0x0FU, rsp, rsp_len) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if ((rsp_len != 8U) ||
        (rsp[1] != 0x0FU) ||
        (rsp[2] != req[2]) ||
        (rsp[3] != req[3]) ||
        (rsp[4] != req[4]) ||
        (rsp[5] != req[5]))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef modbus_master_write_multi(uint8_t slave_addr,
                                            uint16_t reg_addr,
                                            const uint16_t *values,
                                            uint16_t reg_count,
                                            uint32_t timeout_ms)
{
    HAL_StatusTypeDef status = modbus_master_start_write_multi(slave_addr, reg_addr, values, reg_count, timeout_ms);

    if (status != HAL_OK)
    {
        return status;
    }

    status = modbus_wait_current_transaction();
    modbus_master_clear();
    return status;
}

HAL_StatusTypeDef modbus_master_mask_write_register(uint8_t slave_addr,
                                                    uint16_t reg_addr,
                                                    uint16_t and_mask,
                                                    uint16_t or_mask,
                                                    uint32_t timeout_ms)
{
    uint8_t *req = s_modbus_req_buf;
    uint8_t *rsp = s_modbus_rsp_buf;
    uint16_t rsp_len;
    uint16_t crc;

    req[0] = slave_addr;
    req[1] = 0x16U;
    req[2] = (uint8_t)(reg_addr >> 8);
    req[3] = (uint8_t)(reg_addr & 0xFFU);
    req[4] = (uint8_t)(and_mask >> 8);
    req[5] = (uint8_t)(and_mask & 0xFFU);
    req[6] = (uint8_t)(or_mask >> 8);
    req[7] = (uint8_t)(or_mask & 0xFFU);
    crc = modbus_crc16(req, 8U);
    req[8] = (uint8_t)(crc & 0xFFU);
    req[9] = (uint8_t)(crc >> 8);

    if (modbus_exchange_frame(req, 10U, rsp, &rsp_len, timeout_ms) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (modbus_expect_exception_or_addr(slave_addr, 0x16U, rsp, rsp_len) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return ((rsp_len == 10U) && (memcmp(req, rsp, 10U) == 0)) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef modbus_master_read_write_multi(uint8_t slave_addr,
                                                 uint16_t read_addr,
                                                 uint16_t read_count,
                                                 uint16_t *out_regs,
                                                 uint16_t write_addr,
                                                 const uint16_t *write_values,
                                                 uint16_t write_count,
                                                 uint32_t timeout_ms)
{
    uint8_t *req = s_modbus_req_buf;
    uint8_t *rsp = s_modbus_rsp_buf;
    uint16_t req_len;
    uint16_t rsp_len;
    uint16_t crc;
    uint16_t i;

    if ((out_regs == NULL) || (write_values == NULL) ||
        (read_count == 0U) || (read_count > 125U) ||
        (write_count == 0U) || (write_count > 121U))
    {
        return HAL_ERROR;
    }

    req[0] = slave_addr;
    req[1] = 0x17U;
    req[2] = (uint8_t)(read_addr >> 8);
    req[3] = (uint8_t)(read_addr & 0xFFU);
    req[4] = (uint8_t)(read_count >> 8);
    req[5] = (uint8_t)(read_count & 0xFFU);
    req[6] = (uint8_t)(write_addr >> 8);
    req[7] = (uint8_t)(write_addr & 0xFFU);
    req[8] = (uint8_t)(write_count >> 8);
    req[9] = (uint8_t)(write_count & 0xFFU);
    req[10] = (uint8_t)(write_count * 2U);

    for (i = 0U; i < write_count; i++)
    {
        req[11U + i * 2U] = (uint8_t)(write_values[i] >> 8);
        req[12U + i * 2U] = (uint8_t)(write_values[i] & 0xFFU);
    }

    req_len = (uint16_t)(11U + write_count * 2U);
    crc = modbus_crc16(req, req_len);
    req[req_len] = (uint8_t)(crc & 0xFFU);
    req[req_len + 1U] = (uint8_t)(crc >> 8);
    req_len = (uint16_t)(req_len + 2U);

    if (modbus_exchange_frame(req, req_len, rsp, &rsp_len, timeout_ms) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (modbus_expect_exception_or_addr(slave_addr, 0x17U, rsp, rsp_len) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if ((rsp_len != (uint16_t)(5U + read_count * 2U)) ||
        (rsp[1] != 0x17U) ||
        (rsp[2] != (uint8_t)(read_count * 2U)))
    {
        return HAL_ERROR;
    }

    for (i = 0U; i < read_count; i++)
    {
        out_regs[i] = (uint16_t)((uint16_t)rsp[3U + i * 2U] << 8) | rsp[4U + i * 2U];
    }

    return HAL_OK;
}

HAL_StatusTypeDef modbus_master_report_slave_id(uint8_t slave_addr,
                                                uint8_t *out_buf,
                                                uint16_t out_buf_max,
                                                uint16_t *out_len,
                                                uint32_t timeout_ms)
{
    uint8_t *req = s_modbus_req_buf;
    uint8_t *rsp = s_modbus_rsp_buf;
    uint16_t rsp_len;
    uint16_t crc;

    if ((out_buf == NULL) || (out_len == NULL))
    {
        return HAL_ERROR;
    }

    req[0] = slave_addr;
    req[1] = 0x11U;
    crc = modbus_crc16(req, 2U);
    req[2] = (uint8_t)(crc & 0xFFU);
    req[3] = (uint8_t)(crc >> 8);

    if (modbus_exchange_frame(req, 4U, rsp, &rsp_len, timeout_ms) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (modbus_expect_exception_or_addr(slave_addr, 0x11U, rsp, rsp_len) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if ((rsp_len < 5U) || (rsp[1] != 0x11U) || (rsp[2] != (uint8_t)(rsp_len - 5U)))
    {
        return HAL_ERROR;
    }

    if (rsp[2] > out_buf_max)
    {
        return HAL_ERROR;
    }

    memcpy(out_buf, &rsp[3], rsp[2]);
    *out_len = rsp[2];
    return HAL_OK;
}
