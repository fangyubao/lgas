#include "main.h"
#include "modbus_master.h"
#include "CH4.h"
#include "bus485_task.h"
#include <stddef.h>
#define CH4_INFO_BLOCK_DELAY_MS 100U

typedef enum
{
    CH4_OP_IDLE = 0U,
    CH4_OP_WAIT_IO,
    CH4_OP_DELAY
} ch4_op_state_t;

typedef enum
{
    CH4_ACT_NONE = 0U,
    CH4_ACT_READ_INFO,
    CH4_ACT_CLOSE_PROJECT,
    CH4_ACT_SET_ID
} ch4_action_t;

typedef struct
{
    uint16_t reg;
    uint16_t fail_value;
    uint16_t offset;
    uint8_t is_u8;
} ch4_reg_item_t;

static CH4_t CH4_data ={.id=CH4_MODBUS_SLAVE_ID_DEFAULT};
volatile usb_delay_t ch4_delay = {0U, IDLE, IDLE};
static uint16_t ch4_pending_id;
static ch4_op_state_t s_ch4_state = CH4_OP_IDLE;
static ch4_action_t s_ch4_action = CH4_ACT_NONE;
static uint8_t s_ch4_cmd = 0U;
static uint16_t s_ch4_reg_value = 0U;
static uint16_t s_ch4_write_value = 0U;

static const ch4_reg_item_t s_ch4_info_items[CH4_INFO_COUNT] = {
    {CH4_REG_RUN,       0xFFFFU, (uint16_t)offsetof(CH4_t, run_status), 0U},
    {CH4_REG_PN,        0x00FFU, (uint16_t)offsetof(CH4_t, pointbit),   1U},
    {CH4_REG_UCODE,     0xFFFFU, (uint16_t)offsetof(CH4_t, ucode),      0U},
    {CH4_REG_RANGE,     0x0000U, (uint16_t)offsetof(CH4_t, crang),      0U},
    {CH4_REG_CONCCODE,  0xFFFFU, (uint16_t)offsetof(CH4_t, conc_code),  0U},
    {CH4_REG_AD,        0xFFFFU, (uint16_t)offsetof(CH4_t, CH4da),      0U},
    {CH4_REG_CONCD,     0xFFFFU, (uint16_t)offsetof(CH4_t, conc_m),     0U},
    {CH4_REG_TYPE,      0x0000U, (uint16_t)offsetof(CH4_t, ctype),      0U}
};

static void CH4_storeValue(uint8_t cmd, uint16_t value)
{
    uint8_t *base;
    const ch4_reg_item_t *item;

    if ((uint32_t)cmd >= (uint32_t)CH4_INFO_COUNT)
    {
        return;
    }

    item = &s_ch4_info_items[(uint32_t)cmd];
    base = (uint8_t *)&CH4_data;
    if (item->is_u8 != 0U)
    {
        *(uint8_t *)(void *)(base + item->offset) = (uint8_t)value;
    }
    else
    {
        *(uint16_t *)(void *)(base + item->offset) = value;
    }
}

static void CH4_applyResult(uint8_t cmd, uint16_t value, uint8_t success)
{
    if ((uint32_t)cmd >= (uint32_t)CH4_INFO_COUNT)
    {
        return;
    }

    if (success == 0U)
    {
        value = s_ch4_info_items[(uint32_t)cmd].fail_value;
    }
    CH4_storeValue(cmd, value);
}

static void CH4_reset_op(void)
{
    s_ch4_state = CH4_OP_IDLE;
    s_ch4_action = CH4_ACT_NONE;
    s_ch4_cmd = 0U;
    modbus_master_clear();
}

uint8_t CH4_CloseProject(void)
{
    uint16_t temp=0x55AA;
    if(modbus_master_write_multi(CH4_data.id,0x4fff,&temp,1,CH4_MODBUS_TIMEOUT_MS)!=HAL_OK)
    {
        return 1;
    }
    return 0;
}

uint8_t CH4_setCH4Ucode(void)
{
    uint16_t ppma = (uint16_t)PPM;

    if (modbus_master_write_multi(CH4_data.id, CH4_REG_UCODE, &ppma, 1U, CH4_MODBUS_TIMEOUT_MS) != HAL_OK)
    {
        return 1U;
    }
    CH4_data.ucode = ppma;
    return 0U;
}

CH4_t* CH4_getCH4data(void)
{
    return &CH4_data;
}

uint8_t CH4_exec(uint8_t *cmd, CH4_t *out)
{
    modbus_master_poll();

    if ((cmd == NULL) || (out == NULL))
    {
        return CH4_RUN_FAIL;
    }

    if (cmd[0] == CH4_CMD_SET_ID)
    {
        switch (s_ch4_state)
        {
        case CH4_OP_IDLE:
            if ((cmd[1] == 0U) || (cmd[1] > 247U))
            {
                return CH4_RUN_FAIL;
            }
            s_ch4_write_value = 0x55AAU;
            if (modbus_master_start_write_multi(CH4_data.id, 0x4FFFU, &s_ch4_write_value, 1U, CH4_MODBUS_TIMEOUT_MS) != HAL_OK)
            {
                return CH4_RUN_FAIL;
            }
            ch4_pending_id = cmd[1];
            s_ch4_action = CH4_ACT_CLOSE_PROJECT;
            s_ch4_state = CH4_OP_WAIT_IO;
            return CH4_RUN_BUSY;

        case CH4_OP_WAIT_IO:
            if (modbus_master_get_state() == MODBUS_MASTER_BUSY)
            {
                return CH4_RUN_BUSY;
            }
            if (modbus_master_get_state() == MODBUS_MASTER_FAIL)
            {
                CH4_reset_op();
                return CH4_RUN_FAIL;
            }

            modbus_master_clear();
            if (s_ch4_action == CH4_ACT_CLOSE_PROJECT)
            {
                ndelay_ms(&ch4_delay, 300U, UT_SUCCESS);
                s_ch4_state = CH4_OP_DELAY;
                return CH4_RUN_BUSY;
            }

            if (s_ch4_action == CH4_ACT_SET_ID)
            {
                CH4_data.id = ch4_pending_id;
                CH4_reset_op();
                *out = CH4_data;
                return CH4_RUN_DONE;
            }

            CH4_reset_op();
            return CH4_RUN_FAIL;

        case CH4_OP_DELAY:
            if (ndelay_expired(&ch4_delay) == 0U)
            {
                return CH4_RUN_BUSY;
            }
            ch4_delay.usb_status = IDLE;
            s_ch4_write_value = ch4_pending_id;
            if (modbus_master_start_write_multi(CH4_data.id, CH4_REG_ID, &s_ch4_write_value, 1U, CH4_MODBUS_TIMEOUT_MS) != HAL_OK)
            {
                CH4_reset_op();
                return CH4_RUN_FAIL;
            }
            s_ch4_action = CH4_ACT_SET_ID;
            s_ch4_state = CH4_OP_WAIT_IO;
            return CH4_RUN_BUSY;

        default:
            CH4_reset_op();
            return CH4_RUN_FAIL;
        }
    }

    if ((uint32_t)cmd[0] >= (uint32_t)CH4_INFO_COUNT)
    {
        return CH4_RUN_FAIL;
    }

    switch (s_ch4_state)
    {
    case CH4_OP_IDLE:
        if (modbus_master_start_read_holding(CH4_data.id,
                                             s_ch4_info_items[(uint32_t)cmd[0]].reg,
                                             1U,
                                             &s_ch4_reg_value,
                                             CH4_MODBUS_TIMEOUT_MS) != HAL_OK)
        {
            CH4_applyResult(cmd[0], 0U, 0U);
            *out = CH4_data;
            return CH4_RUN_FAIL;
        }
        s_ch4_cmd = cmd[0];
        s_ch4_action = CH4_ACT_READ_INFO;
        s_ch4_state = CH4_OP_WAIT_IO;
        return CH4_RUN_BUSY;

    case CH4_OP_WAIT_IO:
        if (modbus_master_get_state() == MODBUS_MASTER_BUSY)
        {
            return CH4_RUN_BUSY;
        }
        if ((s_ch4_action != CH4_ACT_READ_INFO) || (s_ch4_cmd != cmd[0]))
        {
            CH4_reset_op();
            *out = CH4_data;
            return CH4_RUN_FAIL;
        }

        CH4_applyResult(cmd[0], s_ch4_reg_value, (uint8_t)(modbus_master_get_state() == MODBUS_MASTER_DONE));
        if (modbus_master_get_state() != MODBUS_MASTER_DONE)
        {
            CH4_reset_op();
            *out = CH4_data;
            return CH4_RUN_FAIL;
        }
        CH4_reset_op();
        *out = CH4_data;
        return CH4_RUN_DONE;

    default:
        CH4_reset_op();
        *out = CH4_data;
        return CH4_RUN_FAIL;
    }
}

  uint8_t CH4_requestConc(void)
  {
      static uint8_t step = 0U;
      static const uint8_t req_point[] = {BUS_MODBUS, BUS_MOD_CH4, CH4_INFO_POINTNUM};
      static const uint8_t req_conc[]  = {BUS_MODBUS, BUS_MOD_CH4, CH4_INFO_CONC};
      uint8_t ret;

      switch (step)
      {
      case 0:
          ret = bus485_request(req_point, (uint8_t)sizeof(req_point));
          if (ret == BUS485_FAIL)
          {
              step = 0U;
              return BUS485_FAIL;
          }
          if (ret == BUS485_DONE)
          {
              step = 1U;
          }
          return BUS485_BUSY;

      case 1:
          ret = bus485_request(req_conc, (uint8_t)sizeof(req_conc));
          if (ret == BUS485_FAIL)
          {
              step = 0U;
              return BUS485_FAIL;
          }
          if (ret == BUS485_DONE)
          {
              step = 0U;
              return BUS485_DONE;
          }
          return BUS485_BUSY;

      default:
          step = 0U;
          return BUS485_FAIL;
      }
  }

  uint8_t CH4_getConc(float *data)
  {
      uint32_t pointb = 1U;
      uint8_t i;
      CH4_t *ch4 = CH4_getCH4data();

      if (data == NULL)
      {
          return 1U;
      }

      for (i = 0U; i < ch4->pointbit; i++)
      {
          pointb *= 10U;
      }

      *data = (pointb == 0U) ? 0.0f : (((float)ch4->conc_m / (float)pointb)*10000);
      return 0U;
  }
uint8_t CH4_ReadInfo(CH4_t *out)
{
    uint8_t i;
    uint8_t cmd[1];
    uint8_t ret;

    if (out == NULL)
    {
        return 1U;
    }

    for (i = 0U; i < (uint8_t)CH4_INFO_COUNT; i++)
    {
        cmd[0] = i;
        do
        {
            ret = CH4_exec(cmd, out);
        } while (ret == CH4_RUN_BUSY);

        if (ret != CH4_RUN_DONE)
        {
            return 1U;
        }
        HAL_Delay(CH4_INFO_BLOCK_DELAY_MS);
    }

    *out = CH4_data;
    return 0U;
}
