#include "H2.h"

#include "modbus_master_h2.h"
#include "usb_task.h"
#include "uart.h"

static H2_data_t s_h2_data = {0U, 0.0f, 0U, 0.0f, 0U, 0.0f};
static uint16_t s_read_data[2] = {0U, 0U};

uint8_t H2_Getdata(uint8_t *cmd)
{
    static uint8_t h2_status = IDLE;
    static uint8_t s_h2_cmd = H2_CMD_CONC;
    uint32_t conc_raw;

    if (cmd == NULL)
    {
        h2_status = IDLE;
        modbus_master_h2_clear();
        return H2_RUN_FAIL;
    }

    modbus_master_h2_poll();

    switch (h2_status)
    {
    case IDLE:
        s_h2_cmd = *cmd;
        switch (s_h2_cmd)
        {
        case H2_CMD_TEMP_SENSOR:
            if (modbus_master_h2_start_read_holding(H2_MODBUS_SLAVE_ID, H2_REG_TEMP_SENSOR, 1U, s_read_data, H2_MODBUS_TIMEOUT_MS) != HAL_OK)
            {
                modbus_master_h2_clear();
                 
                return H2_RUN_FAIL;
            }
            break;

        case H2_CMD_TEMP_BOARD:
            if (modbus_master_h2_start_read_holding(H2_MODBUS_SLAVE_ID, H2_REG_TEMP_BOARD, 1U, s_read_data, H2_MODBUS_TIMEOUT_MS) != HAL_OK)
            {
                modbus_master_h2_clear();
                 
                return H2_RUN_FAIL;
            }
            break;

        case H2_CMD_CONC:
        default:
            if (modbus_master_h2_start_read_holding(H2_MODBUS_SLAVE_ID, H2_REG_CONC_H, 2U, s_read_data, H2_MODBUS_TIMEOUT_MS) != HAL_OK)
            {
                modbus_master_h2_clear();
                 
                return H2_RUN_FAIL;
            }
            break;
        }

        h2_status = WAIT;
        return H2_RUN_BUSY;

    case WAIT:
        if (*cmd != s_h2_cmd)
        {
            return H2_RUN_BUSY;
        }
        if (modbus_master_h2_get_state() == MODBUS_MASTER_H2_BUSY)
        {
            return H2_RUN_BUSY;
        }
        if (modbus_master_h2_get_state() != MODBUS_MASTER_H2_DONE)
        {
            h2_status = IDLE;
            modbus_master_h2_clear();
             
            return H2_RUN_FAIL;
        }
        h2_status = UT_SUCCESS;
        modbus_master_h2_clear();
        return H2_RUN_BUSY;

    case UT_SUCCESS:
        if (*cmd != s_h2_cmd)
        {
            return H2_RUN_BUSY;
        }
        h2_status = IDLE;
        if (s_h2_cmd == H2_CMD_TEMP_SENSOR)
        {
            s_h2_data.temp_sensor_raw = s_read_data[0];
            s_h2_data.temp_sensor = ((float)s_h2_data.temp_sensor_raw / 100.0f) - 100.0f;
        }
        else if (s_h2_cmd == H2_CMD_TEMP_BOARD)
        {
            s_h2_data.temp_board_raw = s_read_data[0];
            s_h2_data.temp_board = ((float)s_h2_data.temp_board_raw / 100.0f) - 100.0f;
        }
        else
        {
            conc_raw = (((uint32_t)s_read_data[0]) << 16) | (uint32_t)s_read_data[1];
            s_h2_data.conc_raw = conc_raw;
            s_h2_data.conc = (float)conc_raw;
        }
         
        return H2_RUN_DONE;

    default:
        h2_status = IDLE;
        modbus_master_h2_clear();
         
        return H2_RUN_FAIL;
    }
}

const H2_data_t *H2_getData(void)
{
    return &s_h2_data;
}

HAL_StatusTypeDef H2_readConc(float *h2_conc)
{
    uint16_t regs[2] = {0U, 0U};
    uint32_t raw;

    if (h2_conc == NULL)
    {
        return HAL_ERROR;
    }

    if (modbus_master_h2_read_holding(H2_MODBUS_SLAVE_ID,
                                   H2_REG_CONC_H,
                                   2U,
                                   regs,
                                   H2_MODBUS_TIMEOUT_MS) != HAL_OK)
    {
        return HAL_ERROR;
    }

    raw = (((uint32_t)regs[0]) << 16) | (uint32_t)regs[1];
    s_h2_data.conc_raw = raw;
    s_h2_data.conc = (float)raw;
    *h2_conc = s_h2_data.conc;

    return HAL_OK;
}
