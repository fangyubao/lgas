#include "modbus_slave.h"
#include "main.h"

#include "rs485.h"

#include <string.h>

#define MODBUS_SLAVE_MAX_FRAME        256U
#define MODBUS_SLAVE_GAP_MS           5U

static uint8_t s_slave_addr;
static uint16_t *s_holding_regs;
static uint16_t s_holding_reg_count;
static uint8_t *s_coils;
static uint16_t s_coil_count;
static const uint8_t *s_discrete_inputs;
static uint16_t s_discrete_input_count;
static const uint16_t *s_input_regs;
static uint16_t s_input_reg_count;
static uint8_t s_slave_id[32];
static uint8_t s_slave_id_len;
static uint8_t s_run_status = 0xFFU;
static uint8_t s_rx_buf[MODBUS_SLAVE_MAX_FRAME];
static uint16_t s_rx_len;
static volatile uint32_t s_gap_tick;

static uint8_t modbus_slave_deadline_expired(uint32_t deadline_ms)
{
    return ((int32_t)(app_scheduler_millis() - deadline_ms) >= 0) ? 1U : 0U;
}

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

static void modbus_send_response(const uint8_t *frame, uint16_t len)
{
    (void)rs485_write(frame, len, 1000U);
}

static void modbus_send_exception(uint8_t function, uint8_t exception_code)
{
    uint8_t rsp[5];
    uint16_t crc;

    rsp[0] = s_slave_addr;
    rsp[1] = (uint8_t)(function | 0x80U);
    rsp[2] = exception_code;
    crc = modbus_crc16(rsp, 3U);
    rsp[3] = (uint8_t)(crc & 0xFFU);
    rsp[4] = (uint8_t)(crc >> 8);
    modbus_send_response(rsp, sizeof(rsp));
}

static uint8_t modbus_addr_range_ok(uint16_t addr, uint16_t count, uint16_t max_count)
{
    if ((count == 0U) || (addr >= max_count))
    {
        return 0U;
    }

    if ((uint32_t)addr + (uint32_t)count > (uint32_t)max_count)
    {
        return 0U;
    }

    return 1U;
}

static uint8_t modbus_get_bit(const uint8_t *bits, uint16_t index)
{
    return (uint8_t)(bits[index] != 0U);
}

static void modbus_set_bit(uint8_t *bits, uint16_t index, uint8_t value)
{
    bits[index] = (uint8_t)((value != 0U) ? 1U : 0U);
}

static void modbus_pack_bit_response(uint8_t *rsp,
                                     uint8_t func,
                                     const uint8_t *bits,
                                     uint16_t start_addr,
                                     uint16_t bit_count)
{
    uint16_t i;
    uint8_t byte_count = (uint8_t)((bit_count + 7U) / 8U);
    uint16_t crc;

    rsp[0] = s_slave_addr;
    rsp[1] = func;
    rsp[2] = byte_count;
    memset(&rsp[3], 0, byte_count);

    for (i = 0U; i < bit_count; i++)
    {
        if (modbus_get_bit(bits, (uint16_t)(start_addr + i)) != 0U)
        {
            rsp[3U + i / 8U] |= (uint8_t)(1U << (i % 8U));
        }
    }

    crc = modbus_crc16(rsp, (uint16_t)(3U + byte_count));
    rsp[3U + byte_count] = (uint8_t)(crc & 0xFFU);
    rsp[4U + byte_count] = (uint8_t)(crc >> 8);
    modbus_send_response(rsp, (uint16_t)(5U + byte_count));
}

static void modbus_handle_read_coils(const uint8_t *req, uint16_t req_len)
{
    uint8_t rsp[MODBUS_SLAVE_MAX_FRAME];
    uint16_t addr;
    uint16_t count;

    if ((s_coils == NULL) || (req_len != 8U))
    {
        modbus_send_exception(req[1], 0x03U);
        return;
    }

    addr = (uint16_t)((uint16_t)req[2] << 8) | req[3];
    count = (uint16_t)((uint16_t)req[4] << 8) | req[5];

    if ((count > 2000U) || (modbus_addr_range_ok(addr, count, s_coil_count) == 0U))
    {
        modbus_send_exception(req[1], 0x02U);
        return;
    }

    modbus_pack_bit_response(rsp, 0x01U, s_coils, addr, count);
}

static void modbus_handle_read_discrete_inputs(const uint8_t *req, uint16_t req_len)
{
    uint8_t rsp[MODBUS_SLAVE_MAX_FRAME];
    uint16_t addr;
    uint16_t count;

    if ((s_discrete_inputs == NULL) || (req_len != 8U))
    {
        modbus_send_exception(req[1], 0x03U);
        return;
    }

    addr = (uint16_t)((uint16_t)req[2] << 8) | req[3];
    count = (uint16_t)((uint16_t)req[4] << 8) | req[5];

    if ((count > 2000U) || (modbus_addr_range_ok(addr, count, s_discrete_input_count) == 0U))
    {
        modbus_send_exception(req[1], 0x02U);
        return;
    }

    modbus_pack_bit_response(rsp, 0x02U, s_discrete_inputs, addr, count);
}

static void modbus_handle_read_regs(const uint8_t *req,
                                    uint16_t req_len,
                                    uint8_t func,
                                    const uint16_t *regs,
                                    uint16_t reg_max)
{
    uint8_t rsp[MODBUS_SLAVE_MAX_FRAME];
    uint16_t addr;
    uint16_t count;
    uint16_t crc;
    uint16_t i;

    if ((regs == NULL) || (req_len != 8U))
    {
        modbus_send_exception(req[1], 0x03U);
        return;
    }

    addr = (uint16_t)((uint16_t)req[2] << 8) | req[3];
    count = (uint16_t)((uint16_t)req[4] << 8) | req[5];

    if ((count > 125U) || (modbus_addr_range_ok(addr, count, reg_max) == 0U))
    {
        modbus_send_exception(req[1], 0x02U);
        return;
    }

    rsp[0] = s_slave_addr;
    rsp[1] = func;
    rsp[2] = (uint8_t)(count * 2U);

    for (i = 0U; i < count; i++)
    {
        uint16_t value = regs[addr + i];
        rsp[3U + i * 2U] = (uint8_t)(value >> 8);
        rsp[4U + i * 2U] = (uint8_t)(value & 0xFFU);
    }

    crc = modbus_crc16(rsp, (uint16_t)(3U + count * 2U));
    rsp[3U + count * 2U] = (uint8_t)(crc & 0xFFU);
    rsp[4U + count * 2U] = (uint8_t)(crc >> 8);
    modbus_send_response(rsp, (uint16_t)(5U + count * 2U));
}

static void modbus_handle_write_single_coil(const uint8_t *req, uint16_t req_len)
{
    uint16_t addr;
    uint16_t value;

    if ((s_coils == NULL) || (req_len != 8U))
    {
        modbus_send_exception(req[1], 0x03U);
        return;
    }

    addr = (uint16_t)((uint16_t)req[2] << 8) | req[3];
    value = (uint16_t)((uint16_t)req[4] << 8) | req[5];

    if (modbus_addr_range_ok(addr, 1U, s_coil_count) == 0U)
    {
        modbus_send_exception(req[1], 0x02U);
        return;
    }

    if ((value != 0xFF00U) && (value != 0x0000U))
    {
        modbus_send_exception(req[1], 0x03U);
        return;
    }

    modbus_set_bit(s_coils, addr, (uint8_t)(value == 0xFF00U));
    modbus_send_response(req, req_len);
}

static void modbus_handle_write_single_reg(const uint8_t *req, uint16_t req_len)
{
    uint16_t reg_addr;
    uint16_t value;

    if (req_len != 8U)
    {
        modbus_send_exception(req[1], 0x03U);
        return;
    }

    reg_addr = (uint16_t)((uint16_t)req[2] << 8) | req[3];
    value = (uint16_t)((uint16_t)req[4] << 8) | req[5];

    if ((s_holding_regs == NULL) || (modbus_addr_range_ok(reg_addr, 1U, s_holding_reg_count) == 0U))
    {
        modbus_send_exception(req[1], 0x02U);
        return;
    }

    s_holding_regs[reg_addr] = value;
    modbus_send_response(req, req_len);
}

static void modbus_handle_write_multi_coils(const uint8_t *req, uint16_t req_len)
{
    uint8_t rsp[8];
    uint16_t addr;
    uint16_t count;
    uint8_t byte_count;
    uint16_t crc;
    uint16_t i;

    if ((s_coils == NULL) || (req_len < 10U))
    {
        modbus_send_exception(req[1], 0x03U);
        return;
    }

    addr = (uint16_t)((uint16_t)req[2] << 8) | req[3];
    count = (uint16_t)((uint16_t)req[4] << 8) | req[5];
    byte_count = req[6];

    if ((count == 0U) || (count > 1968U) ||
        (byte_count != (uint8_t)((count + 7U) / 8U)) ||
        (req_len != (uint16_t)(9U + byte_count)))
    {
        modbus_send_exception(req[1], 0x03U);
        return;
    }

    if (modbus_addr_range_ok(addr, count, s_coil_count) == 0U)
    {
        modbus_send_exception(req[1], 0x02U);
        return;
    }

    for (i = 0U; i < count; i++)
    {
        uint8_t value = (uint8_t)((req[7U + i / 8U] >> (i % 8U)) & 0x01U);
        modbus_set_bit(s_coils, (uint16_t)(addr + i), value);
    }

    rsp[0] = s_slave_addr;
    rsp[1] = 0x0FU;
    rsp[2] = req[2];
    rsp[3] = req[3];
    rsp[4] = req[4];
    rsp[5] = req[5];
    crc = modbus_crc16(rsp, 6U);
    rsp[6] = (uint8_t)(crc & 0xFFU);
    rsp[7] = (uint8_t)(crc >> 8);
    modbus_send_response(rsp, sizeof(rsp));
}

static void modbus_handle_write_multi_regs(const uint8_t *req, uint16_t req_len)
{
    uint8_t rsp[8];
    uint16_t reg_addr;
    uint16_t reg_count;
    uint16_t byte_count;
    uint16_t crc;
    uint16_t i;

    if ((s_holding_regs == NULL) || (req_len < 9U))
    {
        modbus_send_exception(req[1], 0x03U);
        return;
    }

    reg_addr = (uint16_t)((uint16_t)req[2] << 8) | req[3];
    reg_count = (uint16_t)((uint16_t)req[4] << 8) | req[5];
    byte_count = req[6];

    if ((reg_count == 0U) || (reg_count > 123U) || (byte_count != reg_count * 2U) || (req_len != (uint16_t)(9U + byte_count)))
    {
        modbus_send_exception(req[1], 0x03U);
        return;
    }

    if (modbus_addr_range_ok(reg_addr, reg_count, s_holding_reg_count) == 0U)
    {
        modbus_send_exception(req[1], 0x02U);
        return;
    }

    for (i = 0U; i < reg_count; i++)
    {
        s_holding_regs[reg_addr + i] = (uint16_t)((uint16_t)req[7U + i * 2U] << 8) | req[8U + i * 2U];
    }

    rsp[0] = s_slave_addr;
    rsp[1] = 0x10U;
    rsp[2] = req[2];
    rsp[3] = req[3];
    rsp[4] = req[4];
    rsp[5] = req[5];
    crc = modbus_crc16(rsp, 6U);
    rsp[6] = (uint8_t)(crc & 0xFFU);
    rsp[7] = (uint8_t)(crc >> 8);
    modbus_send_response(rsp, sizeof(rsp));
}

static void modbus_handle_mask_write_reg(const uint8_t *req, uint16_t req_len)
{
    uint16_t reg_addr;
    uint16_t and_mask;
    uint16_t or_mask;
    uint16_t value;

    if ((s_holding_regs == NULL) || (req_len != 10U))
    {
        modbus_send_exception(req[1], 0x03U);
        return;
    }

    reg_addr = (uint16_t)((uint16_t)req[2] << 8) | req[3];
    and_mask = (uint16_t)((uint16_t)req[4] << 8) | req[5];
    or_mask = (uint16_t)((uint16_t)req[6] << 8) | req[7];

    if (modbus_addr_range_ok(reg_addr, 1U, s_holding_reg_count) == 0U)
    {
        modbus_send_exception(req[1], 0x02U);
        return;
    }

    value = s_holding_regs[reg_addr];
    value = (uint16_t)((value & and_mask) | (or_mask & (uint16_t)(~and_mask)));
    s_holding_regs[reg_addr] = value;
    modbus_send_response(req, req_len);
}

static void modbus_handle_read_write_multi(const uint8_t *req, uint16_t req_len)
{
    uint8_t rsp[MODBUS_SLAVE_MAX_FRAME];
    uint16_t read_addr;
    uint16_t read_count;
    uint16_t write_addr;
    uint16_t write_count;
    uint8_t byte_count;
    uint16_t crc;
    uint16_t i;

    if ((s_holding_regs == NULL) || (req_len < 13U))
    {
        modbus_send_exception(req[1], 0x03U);
        return;
    }

    read_addr = (uint16_t)((uint16_t)req[2] << 8) | req[3];
    read_count = (uint16_t)((uint16_t)req[4] << 8) | req[5];
    write_addr = (uint16_t)((uint16_t)req[6] << 8) | req[7];
    write_count = (uint16_t)((uint16_t)req[8] << 8) | req[9];
    byte_count = req[10];

    if ((read_count == 0U) || (read_count > 125U) ||
        (write_count == 0U) || (write_count > 121U) ||
        (byte_count != (uint8_t)(write_count * 2U)) ||
        (req_len != (uint16_t)(13U + byte_count)))
    {
        modbus_send_exception(req[1], 0x03U);
        return;
    }

    if ((modbus_addr_range_ok(read_addr, read_count, s_holding_reg_count) == 0U) ||
        (modbus_addr_range_ok(write_addr, write_count, s_holding_reg_count) == 0U))
    {
        modbus_send_exception(req[1], 0x02U);
        return;
    }

    for (i = 0U; i < write_count; i++)
    {
        s_holding_regs[write_addr + i] = (uint16_t)((uint16_t)req[11U + i * 2U] << 8) | req[12U + i * 2U];
    }

    rsp[0] = s_slave_addr;
    rsp[1] = 0x17U;
    rsp[2] = (uint8_t)(read_count * 2U);

    for (i = 0U; i < read_count; i++)
    {
        uint16_t value = s_holding_regs[read_addr + i];
        rsp[3U + i * 2U] = (uint8_t)(value >> 8);
        rsp[4U + i * 2U] = (uint8_t)(value & 0xFFU);
    }

    crc = modbus_crc16(rsp, (uint16_t)(3U + read_count * 2U));
    rsp[3U + read_count * 2U] = (uint8_t)(crc & 0xFFU);
    rsp[4U + read_count * 2U] = (uint8_t)(crc >> 8);
    modbus_send_response(rsp, (uint16_t)(5U + read_count * 2U));
}

static void modbus_handle_report_slave_id(void)
{
    uint8_t rsp[MODBUS_SLAVE_MAX_FRAME];
    uint16_t crc;

    rsp[0] = s_slave_addr;
    rsp[1] = 0x11U;
    rsp[2] = (uint8_t)(s_slave_id_len + 1U);
    rsp[3] = s_run_status;
    memcpy(&rsp[4], s_slave_id, s_slave_id_len);
    crc = modbus_crc16(rsp, (uint16_t)(4U + s_slave_id_len));
    rsp[4U + s_slave_id_len] = (uint8_t)(crc & 0xFFU);
    rsp[5U + s_slave_id_len] = (uint8_t)(crc >> 8);
    modbus_send_response(rsp, (uint16_t)(6U + s_slave_id_len));
}

static void modbus_handle_frame(const uint8_t *frame, uint16_t frame_len)
{
    uint16_t crc_calc;
    uint16_t crc_recv;

    if ((frame_len < 4U) || (frame[0] != s_slave_addr))
    {
        return;
    }

    crc_calc = modbus_crc16(frame, (uint16_t)(frame_len - 2U));
    crc_recv = (uint16_t)frame[frame_len - 2U] | ((uint16_t)frame[frame_len - 1U] << 8);

    if (crc_calc != crc_recv)
    {
        return;
    }

    switch (frame[1])
    {
        case 0x01U:
            modbus_handle_read_coils(frame, frame_len);
            break;

        case 0x02U:
            modbus_handle_read_discrete_inputs(frame, frame_len);
            break;

        case 0x03U:
            modbus_handle_read_regs(frame, frame_len, 0x03U, s_holding_regs, s_holding_reg_count);
            break;

        case 0x04U:
            modbus_handle_read_regs(frame, frame_len, 0x04U, s_input_regs, s_input_reg_count);
            break;

        case 0x05U:
            modbus_handle_write_single_coil(frame, frame_len);
            break;

        case 0x06U:
            modbus_handle_write_single_reg(frame, frame_len);
            break;

        case 0x0FU:
            modbus_handle_write_multi_coils(frame, frame_len);
            break;

        case 0x10U:
            modbus_handle_write_multi_regs(frame, frame_len);
            break;

        case 0x11U:
            modbus_handle_report_slave_id();
            break;

        case 0x16U:
            modbus_handle_mask_write_reg(frame, frame_len);
            break;

        case 0x17U:
            modbus_handle_read_write_multi(frame, frame_len);
            break;

        default:
            modbus_send_exception(frame[1], 0x01U);
            break;
    }
}

void modbus_slave_init(uint8_t slave_addr, uint16_t *holding_regs, uint16_t holding_reg_count)
{
    s_slave_addr = slave_addr;
    s_holding_regs = holding_regs;
    s_holding_reg_count = holding_reg_count;
    s_coils = NULL;
    s_coil_count = 0U;
    s_discrete_inputs = NULL;
    s_discrete_input_count = 0U;
    s_input_regs = NULL;
    s_input_reg_count = 0U;
    s_slave_id[0] = slave_addr;
    s_slave_id_len = 1U;
    s_run_status = 0xFFU;
    s_rx_len = 0U;
    s_gap_tick = 0U;

    rs485_init();
}

void modbus_slave_taskDelay(void)
{
}

void modbus_slave_set_coils(uint8_t *coils, uint16_t coil_count)
{
    s_coils = coils;
    s_coil_count = coil_count;
}

void modbus_slave_set_discrete_inputs(const uint8_t *discrete_inputs, uint16_t discrete_input_count)
{
    s_discrete_inputs = discrete_inputs;
    s_discrete_input_count = discrete_input_count;
}

void modbus_slave_set_input_regs(const uint16_t *input_regs, uint16_t input_reg_count)
{
    s_input_regs = input_regs;
    s_input_reg_count = input_reg_count;
}

void modbus_slave_set_slave_id(const uint8_t *slave_id, uint8_t slave_id_len, uint8_t run_status)
{
    if (slave_id == NULL)
    {
        s_slave_id_len = 0U;
    }
    else
    {
        if (slave_id_len > sizeof(s_slave_id))
        {
            slave_id_len = sizeof(s_slave_id);
        }
        memcpy(s_slave_id, slave_id, slave_id_len);
        s_slave_id_len = slave_id_len;
    }

    s_run_status = run_status;
}

void modbus_slave_poll(void)
{
    uint16_t remain;
    uint16_t read_len;

    if ((s_holding_regs == NULL) || (s_holding_reg_count == 0U))
    {
        return;
    }

    remain = (uint16_t)(MODBUS_SLAVE_MAX_FRAME - s_rx_len);
    if (remain == 0U)
    {
        s_rx_len = 0U;
        return;
    }

    read_len = rs485_read(&s_rx_buf[s_rx_len], remain);
    if (read_len > 0U)
    {
        s_rx_len = (uint16_t)(s_rx_len + read_len);
        s_gap_tick = app_scheduler_millis() + MODBUS_SLAVE_GAP_MS;
        return;
    }

    if ((s_rx_len >= 4U) && (modbus_slave_deadline_expired(s_gap_tick) != 0U))
    {
        modbus_handle_frame(s_rx_buf, s_rx_len);
        s_rx_len = 0U;
    }
}
