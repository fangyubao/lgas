#include "modbus_master_h2.h"

#include "rs485_h2.h"

#include <string.h>

#define MODBUS_H2_MAX_FRAME 256U
#define MODBUS_H2_GAP_MS    5U

typedef struct
{
    uint8_t slave_addr;
    uint16_t reg_addr;
    uint16_t reg_count;
    uint16_t *out_regs;
    uint16_t rsp_len;
    uint32_t timeout_ms;
    uint32_t start_tick;
    uint32_t last_rx_tick;
} modbus_master_h2_ctx_t;

static uint8_t s_modbus_h2_req_buf[MODBUS_H2_MAX_FRAME];
static uint8_t s_modbus_h2_rsp_buf[MODBUS_H2_MAX_FRAME];
static modbus_master_h2_state_t s_modbus_h2_state = MODBUS_MASTER_H2_IDLE;
static modbus_master_h2_ctx_t s_modbus_h2_ctx;

static uint16_t modbus_h2_crc16(const uint8_t *data, uint16_t len)
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
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static void modbus_h2_flush_rx(void)
{
    uint8_t dump[32];
    while (rs485_h2_read(dump, (uint16_t)sizeof(dump)) > 0U)
    {
    }
}

static HAL_StatusTypeDef modbus_h2_finalize_read_holding(void)
{
    uint16_t i;
    uint16_t crc_calc;
    uint16_t crc_recv;

    if ((s_modbus_h2_ctx.rsp_len < 5U) || (s_modbus_h2_ctx.out_regs == NULL))
    {
        return HAL_ERROR;
    }

    crc_calc = modbus_h2_crc16(s_modbus_h2_rsp_buf, (uint16_t)(s_modbus_h2_ctx.rsp_len - 2U));
    crc_recv = (uint16_t)s_modbus_h2_rsp_buf[s_modbus_h2_ctx.rsp_len - 2U] |
               ((uint16_t)s_modbus_h2_rsp_buf[s_modbus_h2_ctx.rsp_len - 1U] << 8);
    if (crc_calc != crc_recv)
    {
        return HAL_ERROR;
    }

    if ((s_modbus_h2_rsp_buf[0] != s_modbus_h2_ctx.slave_addr) ||
        (s_modbus_h2_rsp_buf[1] != 0x03U) ||
        (s_modbus_h2_rsp_buf[2] != (uint8_t)(s_modbus_h2_ctx.reg_count * 2U)) ||
        (s_modbus_h2_ctx.rsp_len != (uint16_t)(5U + s_modbus_h2_ctx.reg_count * 2U)))
    {
        return HAL_ERROR;
    }

    for (i = 0U; i < s_modbus_h2_ctx.reg_count; i++)
    {
        s_modbus_h2_ctx.out_regs[i] = (uint16_t)(((uint16_t)s_modbus_h2_rsp_buf[3U + i * 2U] << 8) |
                                                   s_modbus_h2_rsp_buf[4U + i * 2U]);
    }
    return HAL_OK;
}

void modbus_master_h2_init(void)
{
    memset(&s_modbus_h2_ctx, 0, sizeof(s_modbus_h2_ctx));
    s_modbus_h2_state = MODBUS_MASTER_H2_IDLE;
    rs485_h2_init();
}

void modbus_master_h2_taskDelay(void)
{
}

void modbus_master_h2_clear(void)
{
    memset(&s_modbus_h2_ctx, 0, sizeof(s_modbus_h2_ctx));
    s_modbus_h2_state = MODBUS_MASTER_H2_IDLE;
    modbus_h2_flush_rx();
}

modbus_master_h2_state_t modbus_master_h2_get_state(void)
{
    return s_modbus_h2_state;
}

void modbus_master_h2_poll(void)
{
    uint16_t read_len;
    uint16_t remain;
    uint32_t now;

    if (s_modbus_h2_state != MODBUS_MASTER_H2_BUSY)
    {
        return;
    }

    now = HAL_GetTick();
    if ((now - s_modbus_h2_ctx.start_tick) >= s_modbus_h2_ctx.timeout_ms)
    {
        s_modbus_h2_state = MODBUS_MASTER_H2_FAIL;
        return;
    }

    if (s_modbus_h2_ctx.rsp_len >= MODBUS_H2_MAX_FRAME)
    {
        s_modbus_h2_state = MODBUS_MASTER_H2_FAIL;
        return;
    }

    remain = (uint16_t)(MODBUS_H2_MAX_FRAME - s_modbus_h2_ctx.rsp_len);
    read_len = rs485_h2_read(&s_modbus_h2_rsp_buf[s_modbus_h2_ctx.rsp_len], remain);
    if (read_len > 0U)
    {
        s_modbus_h2_ctx.rsp_len = (uint16_t)(s_modbus_h2_ctx.rsp_len + read_len);
        s_modbus_h2_ctx.last_rx_tick = now;
        return;
    }

    if ((s_modbus_h2_ctx.rsp_len > 0U) && ((now - s_modbus_h2_ctx.last_rx_tick) >= MODBUS_H2_GAP_MS))
    {
        s_modbus_h2_state = (modbus_h2_finalize_read_holding() == HAL_OK) ? MODBUS_MASTER_H2_DONE : MODBUS_MASTER_H2_FAIL;
    }
}

HAL_StatusTypeDef modbus_master_h2_start_read_holding(uint8_t slave_addr,
                                                      uint16_t reg_addr,
                                                      uint16_t reg_count,
                                                      uint16_t *out_regs,
                                                      uint32_t timeout_ms)
{
    uint16_t crc;

    if ((out_regs == NULL) || (reg_count == 0U) || (reg_count > 125U) || (timeout_ms == 0U))
    {
        return HAL_ERROR;
    }

    if (s_modbus_h2_state == MODBUS_MASTER_H2_BUSY)
    {
        return HAL_BUSY;
    }

    s_modbus_h2_req_buf[0] = slave_addr;
    s_modbus_h2_req_buf[1] = 0x03U;
    s_modbus_h2_req_buf[2] = (uint8_t)(reg_addr >> 8);
    s_modbus_h2_req_buf[3] = (uint8_t)(reg_addr & 0xFFU);
    s_modbus_h2_req_buf[4] = (uint8_t)(reg_count >> 8);
    s_modbus_h2_req_buf[5] = (uint8_t)(reg_count & 0xFFU);
    crc = modbus_h2_crc16(s_modbus_h2_req_buf, 6U);
    s_modbus_h2_req_buf[6] = (uint8_t)(crc & 0xFFU);
    s_modbus_h2_req_buf[7] = (uint8_t)(crc >> 8);

    modbus_h2_flush_rx();
    if (rs485_h2_write(s_modbus_h2_req_buf, 8U, timeout_ms) != HAL_OK)
    {
        s_modbus_h2_state = MODBUS_MASTER_H2_FAIL;
        return HAL_ERROR;
    }

    s_modbus_h2_ctx.slave_addr = slave_addr;
    s_modbus_h2_ctx.reg_addr = reg_addr;
    s_modbus_h2_ctx.reg_count = reg_count;
    s_modbus_h2_ctx.out_regs = out_regs;
    s_modbus_h2_ctx.rsp_len = 0U;
    s_modbus_h2_ctx.timeout_ms = timeout_ms;
    s_modbus_h2_ctx.start_tick = HAL_GetTick();
    s_modbus_h2_ctx.last_rx_tick = s_modbus_h2_ctx.start_tick;
    s_modbus_h2_state = MODBUS_MASTER_H2_BUSY;
    return HAL_OK;
}

HAL_StatusTypeDef modbus_master_h2_read_holding(uint8_t slave_addr,
                                                uint16_t reg_addr,
                                                uint16_t reg_count,
                                                uint16_t *out_regs,
                                                uint32_t timeout_ms)
{
    if (modbus_master_h2_start_read_holding(slave_addr, reg_addr, reg_count, out_regs, timeout_ms) != HAL_OK)
    {
        return HAL_ERROR;
    }

    while (s_modbus_h2_state == MODBUS_MASTER_H2_BUSY)
    {
        modbus_master_h2_poll();
    }
    return (s_modbus_h2_state == MODBUS_MASTER_H2_DONE) ? HAL_OK : HAL_ERROR;
}
