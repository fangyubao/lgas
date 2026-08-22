#include "main.h"
#include "HMI_task.h"
#include "solenoid.h"
#include "pump.h"
#include "usbd_cdc_if.h"
#include "stats.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "rtc.h"
#include "dataup.h"
#include "flash_storage.h"
#include "app_scheduler.h"
#include "hmi_text_gbk.h"
typedef void (*hmi_handler_t)(const dgusii_frame_t *frame);
typedef uint8_t (*hmi_poll_handler_t)(void);

typedef struct
{
    uint16_t vp;
    hmi_handler_t handler;
} hmi_route_t;

typedef struct
{
    struct
    {
        volatile uint8_t button_key;
        volatile uint8_t gas_select;
        volatile uint8_t param_input;
        volatile uint8_t gas_button;
        volatile uint8_t cyc_time;
        volatile uint8_t pump;
        volatile uint8_t log;
        volatile uint8_t fituse;
        volatile uint8_t param_save;
    } bits;
} hmi_poll_t;

typedef struct
{
    dgusii_frame_t button_key;
    dgusii_frame_t gas_select;
    dgusii_frame_t param_input;
    dgusii_frame_t gas_button;
    volatile uint32_t _tick;
} hmi_frame_cache_t;

typedef struct {
    uint8_t cartype;
    uint8_t conc_type;
}HMI_t;

typedef struct
{
    uint8_t gas_type;
    float k;
    float b;
} fituse_t;

typedef struct
{
    uint8_t *_danwei;
    uint8_t *_where;
    uint8_t *_id;
} param_save_t;

#define HMI_VP_ANAL_TIME_START     0x1000U
#define HMI_VP_ANAL_TIME_END     0x1700U 
#define HMI_VP_ANAL_CLOSE_SOL   0X2700U
#define HMI_VP_LOG_PRINT        0X2F80U
#define HMI_VP_CLEAN_SOL_KEY     0x1A00U
#define HMI_VP_GAS_SELECT       0x1010U
#define HMI_VP_GAS_READ_GRUP     0x2200U

#define HMI_VP_CONC_UNIT   0x1011U
#define HMI_VP_LOG_ADDRESS 0x1040U
#define HMI_VP_DEV_ID      0x1080U
#define HMI_VP_CLEAR_TIME  0x10C0U
#define HMI_VP_H2_BUTTON   0x1300U
#define HMI_VP_C2H2_BUTTON 0x1400U
#define HMI_VP_CH4_BUTTON  0x1500U
#define HMI_VP_C2H6_BUTTON 0x1600U

#define HMI_VP_FITFUN_USE  0X2C00U

#define HMI_KEY_RETURN        0x00FFU
#define HMI_KEY_PARAM_SAVE    0x02F0U
#define HMI_KEY_CLEAN_CONFIRM 0x0300U
#define HMI_KEY_CALIB_READ    0x0450U
#define HMI_KEY_CALIB_BASE    0x0460U
#define HMI_KEY_CALIB_BUILD   0x0470U
#define HMI_KEY_CALIB_SAVE    0x0480U

#define HMI_DATAUP_ANALY       0X01U
#define HMI_DATAUP_CARIBRATION 0X02U

#define HMI_LOG_VP_BASE               0x3100U
#define HMI_LOG_VP_WORDS_PER_SEG      0x75U
#define HMI_LOG_REQ_TRIG              0x81U
#define HMI_LOG_REQ_SEG_MARK          0x82U
#define HMI_LOG_REQ_END_MARK          0x83U
#define HMI_LOG_PRINT_CTRL_START      0x0001U
#define HMI_LOG_PRINT_CTRL_END        0x0003U
#define HMI_LOG_SESSION_TIMEOUT_MS    5000U
#define HMI_LOG_USB_HEAD_LEN          8U
#define HMI_LOG_USB_TAIL_LEN          6U
#define HMI_LOG_USB_TX_MAX            (HMI_LOG_USB_HEAD_LEN + 240U + HMI_LOG_USB_TAIL_LEN)

typedef enum
{
    HMI_LOG_ST_IDLE = 0,
    HMI_LOG_ST_START,
    HMI_LOG_ST_WAIT_SEG,
    HMI_LOG_ST_SEND_USB,
    HMI_LOG_ST_DONE,
    HMI_LOG_ST_FAIL
} hmi_log_state_t;

typedef struct
{
    hmi_log_state_t st;
    uint8_t seg_buf[240];
    uint8_t tx_buf[HMI_LOG_USB_TX_MAX];
    uint16_t seg_len;
    uint16_t seg_seq;
    uint8_t need_trigger;
    uint8_t seg_ready;
    uint8_t got_end;
    uint32_t last_rx_ms;
} hmi_log_ctx_t;

static hmi_log_ctx_t s_hmi_log = {0};
static uint16_t hmi_get_vp(const dgusii_frame_t *frame);
static uint16_t hmi_get_word(const dgusii_frame_t *frame, uint16_t offset);
static void hmi_copy_frame(dgusii_frame_t *dst, const dgusii_frame_t *src);
static void hmi_debug_send(const uint8_t *buf, uint16_t len);
static void hmi_debug_dump_frame(const char *name, const dgusii_frame_t *frame);

static void hmi_handle_button_key(const dgusii_frame_t *frame);
static void hmi_handle_gas_select(const dgusii_frame_t *frame);
static void hmi_handle_param_input(const dgusii_frame_t *frame);
static void hmi_handle_gas_button(const dgusii_frame_t *frame);
static void hmi_handle_disable_cyctime(const dgusii_frame_t *frame);
static void hmi_handle_enable_cyctime(const dgusii_frame_t *frame);
static void hmi_handle_gas_read(const dgusii_frame_t *frame);
static void hmi_handle_log_print(const dgusii_frame_t *frame);
static void hmi_handle_log_data(const dgusii_frame_t *frame);
static void hmi_handle_close_sol(const dgusii_frame_t *frame);
static void hmi_handle_fitfun_use(const dgusii_frame_t *frame);
static uint8_t _hmi_handle_button_key(void);
static uint8_t _hmi_handle_gas_select(void);
static uint8_t _hmi_handle_param_input(void);
static uint8_t _hmi_handle_gas_button(void);
static uint8_t _hmi_handle_gas_button_Cycletime(void);
static uint8_t _hmi_handle_pump_start(void);
static uint8_t _hmi_handle_log_print(void);
static uint8_t _hmi_handle_fitfun_use(void);
static uint8_t _hmi_handle_param_save(void);
static uint8_t hmi_poll_is_enabled(uint8_t index);
static void hmi_log_dbg(const char *fmt, ...);
static void hmi_log_send_print_ctrl(uint16_t v);
static uint8_t hmi_log_send_segment_usb(void);
void HMI_closeTouch(void);
void HMI_openTouch(void);
void Time_DislayBot(uint8_t en);

/* Keep each pending flag independent of compiler-specific bit-field layout. */
static hmi_poll_t s_hmi_poll = {0};
static hmi_frame_cache_t s_hmi_frame_cache = {0};
/* The product default gas is C2H2.  The HMI may override this after sending
 * a gas-selection frame, but startup must not fall back to H2. */
static HMI_t HMI_param = {.cartype = 0U, .conc_type = sC2H2};

static uint16_t clean_time = DEFAULT_CLEAN_TIME;
static uint8_t anay_init_flag = 0;
static void *temp_parm= NULL;
static uint8_t send_data[20];
static uint8_t s_math_open[4];


static hmi_poll_handler_t const s_hmi_poll_fun[] = {
    _hmi_handle_button_key,
    _hmi_handle_gas_select,
    _hmi_handle_param_input,
    _hmi_handle_gas_button,
    _hmi_handle_gas_button_Cycletime,
    _hmi_handle_pump_start,
    _hmi_handle_log_print,
    _hmi_handle_fitfun_use,
    _hmi_handle_param_save,
};

static const hmi_route_t s_hmi_routes[] = {
    {HMI_VP_CLEAN_SOL_KEY, hmi_handle_button_key},
    {HMI_VP_GAS_SELECT, hmi_handle_gas_select},
    {HMI_VP_CONC_UNIT, hmi_handle_param_input},
    {HMI_VP_LOG_ADDRESS, hmi_handle_param_input},
    {HMI_VP_DEV_ID, hmi_handle_param_input},
    {HMI_VP_CLEAR_TIME, hmi_handle_param_input},

    {HMI_VP_H2_BUTTON, hmi_handle_gas_button},
    {HMI_VP_C2H2_BUTTON, hmi_handle_gas_button},
    {HMI_VP_CH4_BUTTON, hmi_handle_gas_button},
    {HMI_VP_C2H6_BUTTON, hmi_handle_gas_button},

    {HMI_VP_ANAL_TIME_START ,hmi_handle_enable_cyctime},
    {HMI_VP_ANAL_TIME_END ,hmi_handle_disable_cyctime},
    {HMI_VP_GAS_READ_GRUP ,hmi_handle_gas_read},
    {HMI_VP_ANAL_CLOSE_SOL ,hmi_handle_close_sol},
    {HMI_VP_LOG_PRINT, hmi_handle_log_print},
    {HMI_LOG_VP_BASE, hmi_handle_log_data},
    {HMI_VP_FITFUN_USE, hmi_handle_fitfun_use},
};

void dgusii_HMI_task(const dgusii_frame_t *frame, void *user_ctx)
{
    uint16_t i;
    uint16_t vp;

    (void)user_ctx;
#ifdef DEBUG_MODE
    if (frame == NULL)
    {
        return;
    }

    if (s_hmi_log.st == HMI_LOG_ST_IDLE)
    {
        hmi_debug_dump_frame("rx", frame);
    }
#endif 
    if ((frame == NULL) || (frame->len < 3U) )
    {
        return;
    }
    
    vp = hmi_get_vp(frame);
    if ((s_hmi_log.st != HMI_LOG_ST_IDLE) &&
        (frame->data[0] == DGUSII_CMD_READ_VP) &&
        (vp == HMI_LOG_VP_BASE))
    {
        hmi_handle_log_data(frame);
        return;
    }
    Time_DislayBot(TRUE);
    for (i = 0U; i < (uint16_t)(sizeof(s_hmi_routes) / sizeof(s_hmi_routes[0])); i++)
    {
        if ((s_hmi_routes[i].vp == vp) && (s_hmi_routes[i].handler != NULL))
        {
            s_hmi_routes[i].handler(frame);
            return;
        }
    }
}

static uint8_t hmi_poll_is_enabled(uint8_t index)
{
    switch (index)
    {
        case 0U: return s_hmi_poll.bits.button_key;
        case 1U: return s_hmi_poll.bits.gas_select;
        case 2U: return s_hmi_poll.bits.param_input;
        case 3U: return s_hmi_poll.bits.gas_button;
        case 4U: return s_hmi_poll.bits.cyc_time;
        case 5U: return s_hmi_poll.bits.pump;
        case 6U: return s_hmi_poll.bits.log;
        case 7U: return s_hmi_poll.bits.fituse;
        case 8U: return s_hmi_poll.bits.param_save;
        default: return 0U;
    }
}

void HMI_task_poll(void)
{
    static uint8_t i = 0U;

    if (hmi_poll_is_enabled(i) != 0U)
    {
        if (s_hmi_poll_fun[i] != NULL)
        {
            (void)s_hmi_poll_fun[i]();
        }
    }

    i = (uint8_t)((i + 1U) % (sizeof(s_hmi_poll_fun) / sizeof(s_hmi_poll_fun[0])));
}

void HMI_task_debug_dump_cache(void)
{
    hmi_debug_dump_frame("button_key", &s_hmi_frame_cache.button_key);
    hmi_debug_dump_frame("gas_select", &s_hmi_frame_cache.gas_select);
    hmi_debug_dump_frame("param_input", &s_hmi_frame_cache.param_input);
    hmi_debug_dump_frame("gas_button", &s_hmi_frame_cache.gas_button);
}

static uint16_t hmi_get_vp(const dgusii_frame_t *frame)
{
    return (uint16_t)(((uint16_t)frame->data[1] << 8) | (uint16_t)frame->data[2]);
}

static uint16_t hmi_get_word(const dgusii_frame_t *frame, uint16_t offset)
{
    uint16_t index = (uint16_t)(1U + offset);

    if ((frame == NULL) || ((uint16_t)(index + 1U) >= frame->len))
    {
        return 0U;
    }

    return (uint16_t)(((uint16_t)frame->data[index] << 8) | (uint16_t)frame->data[index + 1U]);
}

static void hmi_copy_frame(dgusii_frame_t *dst, const dgusii_frame_t *src)
{
    if ((dst == NULL) || (src == NULL))
    {
        return;
    }

    *dst = *src;
}

static void hmi_debug_send(const uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || (len == 0U))
    {
        return;
    }

    (void)usb_cdc_send_packet(buf, len);
}

static void hmi_debug_dump_frame(const char *name, const dgusii_frame_t *frame)
{
    char line[192];
    int len;
    uint16_t i;

    if ((name == NULL) || (frame == NULL))
    {
        return;
    }

    len = snprintf(line, sizeof(line), "%s: cmd=%02X len=%u data=", name, frame->data[0], frame->len);
    if (len < 0)
    {
        return;
    }

    for (i = 0U; (i < frame->len) && (len < (int)(sizeof(line) - 4U)); i++)
    {
        len += snprintf(&line[len], sizeof(line) - (size_t)len, "%02X ", frame->data[i]);
    }

    if (len < (int)(sizeof(line) - 3U))
    {
        line[len++] = '\r';
        line[len++] = '\n';
        line[len] = '\0';
    }
    else
    {
        line[sizeof(line) - 3U] = '\r';
        line[sizeof(line) - 2U] = '\n';
        line[sizeof(line) - 1U] = '\0';
        len = (int)strlen(line);
    }

}

static void hmi_handle_button_key(const dgusii_frame_t *frame)
{
    enum BOT{CLEAN,SAVE};
    uint8_t status;

    if ((frame == NULL) || (frame->len < 5U))
    {
        usb_cdc_send_packet((uint8_t *)"addr=0X1A00 short\r\n", 19U);
        return;
    }

    if ((frame->data[3] == 0x01U) && (frame->data[4] == 0x03U))
    {
        status = CLEAN;
    }
    else if (((frame->data[3] == 0x01U) && (frame->data[4] == 0x02U)) ||
             (frame->data[3] == 0x81U))
    {
        status = SAVE;
    }
    else
    {
        usb_cdc_send_packet((uint8_t *)"addr=0X1A00 fail\r\n",18);
        return;
    }

    switch (status)
    {
        case CLEAN:
        {
            static uint32_t once_time ;
            HAL_StatusTypeDef pump_status;
            solenoid_set_duty(0);
            HMI_closeTouch();
            hmi_copy_frame(&s_hmi_frame_cache.button_key, frame);
            pump_status = pump_set_strength(PUMP_START_DUTY);
            if (pump_status != HAL_OK)
            {
                usb_cdc_send_packet((uint8_t *)"pump start fail\r\n", 17U);
                HMI_openTouch();
                return;
            }
            once_time = app_scheduler_millis();
            temp_parm = &once_time;
            s_hmi_poll.bits.button_key = 1U;

            break;
        }

        case SAVE:
            s_hmi_poll.bits.param_save = 1U;
            break;

        default:
            break;
    }
}

static void hmi_handle_gas_select(const dgusii_frame_t *frame)
{
    uint8_t buf[20];
    uint8_t gas;

    if ((frame == NULL) || (frame->len < 6U))
    {
        return;
    }

    gas = (uint8_t)(frame->data[5] & 0x0FU);
    if ((gas < sH2) || (gas > sC2H6))
    {
        usb_cdc_send_packet((uint8_t *)"gas select invalid\r\n", 20U);
        return;
    }

    hmi_copy_frame(&s_hmi_frame_cache.gas_select, frame);
    s_hmi_poll.bits.gas_select = 1U;
    HMI_param.conc_type = gas;
    sprintf((char *)buf,"gas select %u\r\n",HMI_param.conc_type);
    usb_cdc_send_packet(buf,strlen((char *)buf));
}

static void hmi_handle_param_input(const dgusii_frame_t *frame)
{
    if ((frame == NULL) || (frame->len < 3U))
    {
        return;
    }

    hmi_copy_frame(&s_hmi_frame_cache.param_input, frame);
    s_hmi_poll.bits.param_input = 1U;
}

static void hmi_handle_gas_button(const dgusii_frame_t *frame)
{
    if ((frame == NULL) || (frame->len < 2U))
    {
        return;
    }

    hmi_copy_frame(&s_hmi_frame_cache.gas_button, frame);
    s_hmi_poll.bits.gas_button = 1U;
}
static void hmi_handle_enable_cyctime(const dgusii_frame_t *frame){
    if ((frame == NULL) || (frame->len < 3U))
    {
        return;
    }
    static uint32_t start_time;
    start_time = app_scheduler_millis();
    temp_parm = &start_time;

    if (frame->len < 6U)
    {
        return;
    }

    if(frame->data[5]==0x02||frame->data[5]==0x04){
        solenoid_set_duty(0);
        pump_set_strength(30);
        s_hmi_poll.bits.pump = 1U;
    }
    if(frame->data[5]==0x02){
        uint8_t clear_info[]= {0X82,0X28,0,0,0,0,0,0,0,0};
        dgusii_send(clear_info,10);
        //Time_DislayBot(TRUE);
        anay_init_flag = 0;
    }else if(frame->data[5]==0x04){
        //HMI_param.cyc_botton = TRUE;
        uint8_t clear_info[]= {0X82,0X29,0,0,0,0,0,0,0,0};
        dgusii_send(clear_info,10);

        solenoid_set_duty(0);
        //Time_DislayBot(TRUE);
        anay_init_flag = 1;
    }else if(frame->data[5]==0x03){
        //Time_DislayBot(TRUE);
        anay_init_flag = 1;
    }
}

static void hmi_handle_disable_cyctime(const dgusii_frame_t *frame){
    if (frame == NULL)
    {
        return;
    }
    //solenoid_set_duty(0);
    //HMI_param.cyc_botton = FALSE;
    //Time_DislayBot(FALSE);
}

static void hmi_handle_gas_read(const dgusii_frame_t *frame){
    if ((frame == NULL) || (frame->len < 8U))
    {
        return;
    }
    if((frame->data[4]!=0x04) ||(frame->data[5]!=0x50))
        return ;
    if ((HMI_param.conc_type < sH2) || (HMI_param.conc_type > sC2H6))
        return;
    HMI_closeTouch();
    dgusii_send(HMI_TEXT_READ_ING_CAB_GBK, HMI_TEXT_READ_ING_CAB_GBK_LEN);

    send_data[0] = 0x82;
    send_data[1] = frame->data[6];
    send_data[2] = frame->data[7];
    HMI_param.cartype = HMI_DATAUP_CARIBRATION;
    dataup_set_enable(TRUE,HMI_param.conc_type);
}

static void hmi_handle_log_print(const dgusii_frame_t *frame){
    uint16_t v;

    if ((frame == NULL) || (frame->len < 6U))
    {
        hmi_log_dbg("[LOG] trigger drop: frame null/short\r\n");
        return;
    }

    if ((frame->data[0] != DGUSII_CMD_READ_VP) || (frame->data[3] < 1U))
    {
        hmi_log_dbg("[LOG] trigger drop: cmd=%02X words=%u\r\n", frame->data[0], frame->data[3]);
        return;
    }

    /* 83 VP_H VP_L WORDS DATA_H DATA_L ... */
    v = (uint16_t)(((uint16_t)frame->data[4] << 8) | (uint16_t)frame->data[5]);
    if (v == HMI_LOG_PRINT_CTRL_START)
    {
        s_hmi_log.need_trigger = 1U;
    }
    else
    {
        hmi_log_dbg("[LOG] trigger non-start key=0x%04X\r\n", v);
    }
    s_hmi_poll.bits.log = TRUE;
}

static void hmi_handle_log_data(const dgusii_frame_t *frame)
{
    uint8_t wc;
    uint16_t nbytes;

    if ((frame == NULL) || (frame->len < 4U) || (frame->data[0] != DGUSII_CMD_READ_VP))
    {
        return;
    }

    if (s_hmi_log.st == HMI_LOG_ST_IDLE)
    {
        return;
    }

    if ((frame->len == 4U) && (frame->data[3] == HMI_LOG_REQ_END_MARK))
    {
        s_hmi_log.got_end = 1U;
        s_hmi_poll.bits.log = TRUE;
        return;
    }
    if ((frame->len == 4U) && (frame->data[3] == HMI_LOG_REQ_SEG_MARK))
    {
        return;
    }

    wc = frame->data[3];
    nbytes = (uint16_t)wc * 2U;
    if ((wc == 0U) || ((uint16_t)(4U + nbytes) > frame->len) || (nbytes > (uint16_t)sizeof(s_hmi_log.seg_buf)))
    {
        hmi_log_dbg("[LOG] rx seg invalid wc=%u len=%u\r\n", wc, frame->len);
        s_hmi_log.st = HMI_LOG_ST_FAIL;
        s_hmi_poll.bits.log = TRUE;
        return;
    }

    if (s_hmi_log.seg_ready != 0U)
    {
        hmi_log_dbg("[LOG] rx overrun, seg not consumed\r\n");
        return;
    }

    memcpy(s_hmi_log.seg_buf, &frame->data[4], nbytes);
    s_hmi_log.seg_len = nbytes;
    s_hmi_log.seg_ready = 1U;
    s_hmi_log.last_rx_ms = dgusii_tick_ms();
    s_hmi_poll.bits.log = TRUE;
}

static void hmi_handle_fitfun_use(const dgusii_frame_t *frame)
{
    s_hmi_poll.bits.fituse = 1U;
}

static void hmi_handle_close_sol(const dgusii_frame_t *frame){
    solenoid_set_duty(0);
}

static void hmi_log_dbg(const char *fmt, ...)
{
    char line[128];
    va_list ap;
    int n;

    if (fmt == NULL)
    {
        return;
    }

    va_start(ap, fmt);
    n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n <= 0)
    {
        return;
    }
    if (n >= (int)sizeof(line))
    {
        n = (int)sizeof(line) - 1;
    }
    (void)usb_cdc_send_packet((const uint8_t *)line, (uint16_t)n);
}

static void hmi_log_send_print_ctrl(uint16_t v)
{
    uint8_t tx[5];

    tx[0] = 0x82U;
    tx[1] = 0x2FU;
    tx[2] = 0x80U;
    tx[3] = (uint8_t)(v >> 8);
    tx[4] = (uint8_t)v;
    (void)dgusii_send(tx, sizeof(tx));
}

static uint8_t hmi_log_send_segment_usb(void)
{
    uint16_t pos = 0U;
    uint16_t len = s_hmi_log.seg_len;
    uint16_t seq = s_hmi_log.seg_seq;

    if (len == 0U)
    {
        return 1U;
    }
    if (len > (uint16_t)sizeof(s_hmi_log.seg_buf))
    {
        return 0U;
    }

    s_hmi_log.tx_buf[pos++] = 0xA5U;
    s_hmi_log.tx_buf[pos++] = 0x5AU;
    s_hmi_log.tx_buf[pos++] = 'L';
    s_hmi_log.tx_buf[pos++] = 'G';
    s_hmi_log.tx_buf[pos++] = (uint8_t)seq;
    s_hmi_log.tx_buf[pos++] = (uint8_t)(seq >> 8);
    s_hmi_log.tx_buf[pos++] = (uint8_t)len;
    s_hmi_log.tx_buf[pos++] = (uint8_t)(len >> 8);

    memcpy(&s_hmi_log.tx_buf[pos], s_hmi_log.seg_buf, len);
    pos = (uint16_t)(pos + len);

    s_hmi_log.tx_buf[pos++] = 0x5AU;
    s_hmi_log.tx_buf[pos++] = 0xA5U;
    s_hmi_log.tx_buf[pos++] = 'G';
    s_hmi_log.tx_buf[pos++] = 'L';
    s_hmi_log.tx_buf[pos++] = (uint8_t)seq;
    s_hmi_log.tx_buf[pos++] = (uint8_t)(seq >> 8);

    if (usb_cdc_send_packet(s_hmi_log.tx_buf, pos) == 0U)
    {
        return 0U;
    }

    s_hmi_log.seg_seq++;
    return 1U;
}

static uint8_t _hmi_handle_fitfun_use(void)
{
    enum GAS_FIT{GAS_TYPE,FIT_K,FIT_B,FINDSH};
    static uint8_t status = GAS_TYPE; 
    static fituse_t fituse_gas = {0};
    const dgusii_frame_t *rx_frame=NULL;
    switch(status){
        case GAS_TYPE:{
            uint8_t fit_data[]= {0X83,0X2B,0,0X02};
            uint8_t ret = dgusii_request(fit_data, sizeof(fit_data), &rx_frame);
            if(ret == DGUSII_REQ_BUSY){
                return 0U;
            }
            if(ret == DGUSII_REQ_DONE)
            {
                if ((rx_frame == NULL) || (rx_frame->len < 5U))
                {
                    break;
                }
                fituse_gas.gas_type = rx_frame->data[4] & 0x0F;
                if ((fituse_gas.gas_type < sH2) || (fituse_gas.gas_type > sC2H6))
                {
                    break;
                }
                status = FIT_K;
            }
        }
        break;
        case FIT_K:{
            uint8_t fit_data[]= {0X83,0X2A,0,0X08};
            uint8_t ret = dgusii_request(fit_data, sizeof(fit_data), &rx_frame);
            if(ret == DGUSII_REQ_BUSY){
                return 0U;
            }
            if(ret == DGUSII_REQ_DONE)
            {
                if ((rx_frame == NULL) || (rx_frame->len < 8U))
                {
                    break;
                }
                {   uint32_t tmp = ((uint32_t)rx_frame->data[4]<<24)|((uint32_t)rx_frame->data[5]<<16)|((uint32_t)rx_frame->data[6]<<8)|(uint32_t)rx_frame->data[7];
                    memcpy(&fituse_gas.k, &tmp, sizeof(fituse_gas.k)); }
                status = FIT_B;
            }
        }
        break;
        case FIT_B:{
            uint8_t fit_data[]= {0X83,0X2A,0x10,0X08};
            uint8_t ret = dgusii_request(fit_data, sizeof(fit_data), &rx_frame);
            if(ret == DGUSII_REQ_BUSY){
                return 0U;
            }
            if(ret == DGUSII_REQ_DONE)
            {
                if ((rx_frame == NULL) || (rx_frame->len < 8U))
                {
                    break;
                }
                {   uint32_t tmp = ((uint32_t)rx_frame->data[4]<<24)|((uint32_t)rx_frame->data[5]<<16)|((uint32_t)rx_frame->data[6]<<8)|(uint32_t)rx_frame->data[7];
                    memcpy(&fituse_gas.b, &tmp, sizeof(fituse_gas.b)); }
                status = FINDSH;
            }
        }
        break;
        case FINDSH:{
            hmi_log_dbg("fitfun use: type=%u K=%f,B=%f\r\n",
                fituse_gas.gas_type,fituse_gas.k,fituse_gas.b);
            int ret = storage_param_write_group(fituse_gas.gas_type, fituse_gas.k, fituse_gas.b);
            if (ret != 0) {
                hmi_log_dbg("fitfun save fail: ret=%d\r\n", ret);
            }
            status = GAS_TYPE;
            s_hmi_poll.bits.fituse = 0U;
        }
        break;
    }
		return 0;
}


static uint8_t _hmi_handle_param_save(void)
{
    enum save_param{SAVE_UINT,FINSH};
    static uint8_t status = SAVE_UINT;
    const dgusii_frame_t *rx_frame=NULL;
    static uint8_t s_param_buf[255];
    static uint16_t s_param_pos;

    switch(status)
    {
        case SAVE_UINT:{
            uint8_t param_data[]= {0X83,0X1A,0,0x81};
            uint8_t ret = dgusii_request(param_data, sizeof(param_data), &rx_frame);
            if(ret == DGUSII_REQ_BUSY){
                return 0U;
            }
            if(ret == DGUSII_REQ_DONE)
            {
                uint16_t dlen;

                if ((rx_frame == NULL) || (rx_frame->len < 4U))
                {
                    break;
                }
                dlen = (uint16_t)(rx_frame->len - 4U);
                if (dlen > sizeof(s_param_buf)) dlen = sizeof(s_param_buf);
                s_param_pos = dlen;
                memcpy(s_param_buf, &rx_frame->data[4], dlen);
                status = FINSH;
            }
        }
        break;

        case FINSH:{
            storage_log_write(s_param_buf, s_param_pos);
            s_hmi_poll.bits.param_save = 0U;
            status = SAVE_UINT;
        }
        break;
    }
		return 0;
}
static uint8_t _hmi_handle_log_print(void){
    uint8_t req[] = {0x83U, 0x31U, 0x00U, HMI_LOG_REQ_TRIG};

    switch (s_hmi_log.st)
    {
        case HMI_LOG_ST_IDLE:
            if (s_hmi_log.need_trigger == 0U)
            {
                s_hmi_poll.bits.log = 0U;
                return 0U;
            }
            s_hmi_log.st = HMI_LOG_ST_START;
            break;

        case HMI_LOG_ST_START:
            s_hmi_log.seg_len = 0U;
            s_hmi_log.seg_seq = 0U;
            s_hmi_log.seg_ready = 0U;
            s_hmi_log.got_end = 0U;
            s_hmi_log.need_trigger = 0U;
            s_hmi_log.last_rx_ms = dgusii_tick_ms();
            if (dgusii_send(req, sizeof(req)) != HAL_OK)
            {
                hmi_log_dbg("[LOG] first req send fail\r\n");
                s_hmi_log.st = HMI_LOG_ST_FAIL;
                return 0U;
            }
            s_hmi_log.st = HMI_LOG_ST_WAIT_SEG;
            return 0U;

        case HMI_LOG_ST_WAIT_SEG:
            if (dgusii_timeout_expired(s_hmi_log.last_rx_ms, HMI_LOG_SESSION_TIMEOUT_MS) != 0U)
            {
                hmi_log_dbg("[LOG] wait timeout\r\n");
                s_hmi_log.st = HMI_LOG_ST_FAIL;
                return 0U;
            }
            if (s_hmi_log.seg_ready != 0U)
            {
                s_hmi_log.st = HMI_LOG_ST_SEND_USB;
                return 0U;
            }
            if (s_hmi_log.got_end != 0U)
            {
                s_hmi_log.st = HMI_LOG_ST_DONE;
            }
            return 0U;

        case HMI_LOG_ST_SEND_USB:
            if ((s_hmi_log.seg_len > 0U) && (hmi_log_send_segment_usb() == 0U))
            {
                hmi_log_dbg("[LOG] usb busy bytes=%u\r\n", s_hmi_log.seg_len);
                if (dgusii_timeout_expired(s_hmi_log.last_rx_ms, HMI_LOG_SESSION_TIMEOUT_MS) != 0U)
                {
                    hmi_log_dbg("[LOG] send timeout, abort\r\n");
                    s_hmi_log.st = HMI_LOG_ST_FAIL;
                }
                return 0U;
            }
            s_hmi_log.seg_len = 0U;
            s_hmi_log.seg_ready = 0U;
            s_hmi_log.last_rx_ms = dgusii_tick_ms();
            if (s_hmi_log.got_end != 0U)
            {
                s_hmi_log.st = HMI_LOG_ST_DONE;
                return 0U;
            }
            if (dgusii_send(req, sizeof(req)) != HAL_OK)
            {
                hmi_log_dbg("[LOG] next req send fail\r\n");
                s_hmi_log.st = HMI_LOG_ST_FAIL;
                return 0U;
            }
            s_hmi_log.st = HMI_LOG_ST_WAIT_SEG;
            return 0U;

        case HMI_LOG_ST_DONE:
            hmi_log_send_print_ctrl(HMI_LOG_PRINT_CTRL_END);
            s_hmi_log.st = HMI_LOG_ST_IDLE;
            s_hmi_log.seg_len = 0U;
            s_hmi_log.seg_seq = 0U;
            s_hmi_log.seg_ready = 0U;
            s_hmi_log.got_end = 0U;
            s_hmi_log.need_trigger = 0U;
            s_hmi_poll.bits.log = 0U;
            return 1U;

        case HMI_LOG_ST_FAIL:
        default:
            hmi_log_dbg("[LOG] state FAIL\r\n");
            dgusii_recover();
            hmi_log_send_print_ctrl(HMI_LOG_PRINT_CTRL_END);
            s_hmi_log.st = HMI_LOG_ST_IDLE;
            s_hmi_log.seg_len = 0U;
            s_hmi_log.seg_seq = 0U;
            s_hmi_log.seg_ready = 0U;
            s_hmi_log.got_end = 0U;
            s_hmi_log.need_trigger = 0U;
            s_hmi_poll.bits.log = 0U;
            return 1U;
    }

    return 0U;
}

static uint8_t _hmi_handle_button_key(void)
{
    if(s_hmi_poll.bits.button_key != 1)
        return 0;
    
    static uint32_t curr_time=0,last_time=0 ;
    enum clean_status {PUMP_STEP,DATA,PUMP_START,PUMP_END};
    static uint8_t c_status = PUMP_STEP;
    static uint8_t pump_duty = 0;
    switch(c_status){
        case PUMP_STEP:{
            curr_time = app_scheduler_millis();
            if(pump_duty <PUMP_RUNING_DUTY-PUMP_START_DUTY)
            {
                if(curr_time-last_time>PUMP_CLEAN_TIME_DEFAULT)
                {
                    last_time = curr_time;
                    pump_duty += 1;
                    pump_set_strength(PUMP_START_DUTY+pump_duty);
                }
                return 0;
            }
            last_time = 0;
            curr_time = 0;
            c_status = DATA ;
        }
        break;
        case DATA:{
            const dgusii_frame_t *rx_frame=NULL;
            uint8_t tx[] = {0x83,0x10,0xC0,0X01};
            uint8_t ret = dgusii_request(tx, sizeof(tx), &rx_frame);
            if (ret == DGUSII_REQ_BUSY)
            {
                return 0U;
            }
            if (ret == DGUSII_REQ_DONE)
            {
                if ((rx_frame == NULL) || (rx_frame->len < 6U))
                {
                    break;
                }
                clean_time = U8_TO_U16(rx_frame->data[rx_frame->len-2],rx_frame->data[rx_frame->len-1]); 
                c_status = PUMP_START;
                curr_time = app_scheduler_millis();
                return 0U;
            }
        break;
        }

        case PUMP_START :{
            if(app_scheduler_millis() - curr_time<clean_time*1000){
                return 0;
            }else{
                last_time = 0;
                curr_time = 0;
                pump_duty = PUMP_RUNING_DUTY;
                c_status = PUMP_END;
            }
            break;
        }

        case PUMP_END:{
            curr_time = app_scheduler_millis();
            if(pump_duty >0)
            {
                if(curr_time-last_time>PUMP_CLEAN_TIME_DEFAULT)
                {
                    last_time = curr_time;
                    pump_duty -= 1;
                    pump_set_strength(pump_duty);
                }
                return 0;
            }
            pump_set_strength(0);
            s_hmi_poll.bits.button_key = 0; 
            pump_duty = 0; 
            HMI_openTouch();
            solenoid_set_duty(100);
            c_status = PUMP_STEP;
            return 0;
        }            
    }
    return 1U;
}

static uint8_t _hmi_handle_pump_start(void){
	if(s_hmi_poll.bits.pump != 1)
			return 0;
	static uint32_t time_diff;
	enum pump_status {PUMP_RUN,PUMP_FIX,PUMP_END};
	static uint8_t p_status = PUMP_RUN;
	switch(p_status){
        case PUMP_RUN:{
            static uint32_t _tick = 0;
            static uint8_t _duty = PUMP_START_DUTY;
            if(app_scheduler_millis() - _tick < PUMP_CLEAN_TIME_DEFAULT){
                return 0;
            }else{
                _tick = app_scheduler_millis();
            }
            pump_set_strength(++_duty);
            if(_duty == PUMP_START_DUTY+3){
                HMI_closeTouch();
            }
            if(_duty >= PUMP_RUNING_DUTY)
            {
                _duty = PUMP_START_DUTY;
                
                p_status = PUMP_FIX;
                time_diff =app_scheduler_millis();
                return 0;
            }
            break;
        }
        case PUMP_FIX:{
            if(app_scheduler_millis() - time_diff<DEFAULT_CLEAN_TIME*1000){
                return 0;
            }else{
                p_status = PUMP_END;
                return 0;
            }
        }
        break;

        case PUMP_END:{
            static uint32_t _tick = 0;
            static uint8_t _duty = PUMP_RUNING_DUTY;
            if(app_scheduler_millis() - _tick < PUMP_CLEAN_TIME_DEFAULT){
                return 0;
            }else{                
                _tick = app_scheduler_millis();
            }
            pump_set_strength(--_duty);
            if(_duty <= 0)
            {
                _duty = PUMP_RUNING_DUTY;
                pump_set_strength(0);
                solenoid_set_duty(100);
                HMI_openTouch();                
                s_hmi_poll.bits.pump = 0U;
                p_status = PUMP_RUN;
                return 0;
            }
        }
        break;

        default:
        break;
	}
	return 0;
}
static uint8_t _hmi_handle_gas_select(void)
{
    s_hmi_poll.bits.gas_select = 0U;
    return 0U;
}

static uint8_t _hmi_handle_param_input(void)
{
    s_hmi_poll.bits.param_input = 0U;
    return 0U;
}

static uint8_t _hmi_handle_gas_button(void)
{   
    enum samp{START,OPEN_MATH,SEND_DATA,CLOSE_MATH};
    static uint8_t tstatus =OPEN_MATH;
    static uint32_t curr_tick ;

    switch (tstatus)
    {
    // case START:
    //     if (s_hmi_frame_cache.gas_button.len < 2U)
    //     {
    //         s_hmi_poll.bits.gas_button = 0U;
    //         return 0U;
    //     }
    //     if((s_hmi_frame_cache.gas_button.data[s_hmi_frame_cache.gas_button.len-1]&0x01)==0){
    //         tstatus = CLOSE_MATH;
    //     }else {
    //         tstatus = OPEN_MATH;
    //     }
    // break;
   case OPEN_MATH:
        dgusii_send(HMI_TEXT_READ_ING_GBK,HMIHMI_READ_ING_LEN);
        send_data[0]= 0x82;
        send_data[1]= 0x30;
        switch (s_hmi_frame_cache.gas_button.data[1])
        {
            case 0x13:{
                send_data[2]= 0x20;
                }
            break;
            
            case 0x14:{
                send_data[2]= 0x60;
                }
            break;

            case 0x15:{
                send_data[2]= 0xA0;
                }
            break;

            case 0x16:{
                send_data[2]= 0xE0;        
                }
            break;
            default:
            break;
        }
        send_data[3] = 0X11;
        memcpy(s_math_open, send_data, 4);  
        dgusii_send(send_data,4);   
        HMI_closeTouch();
        curr_tick = app_scheduler_millis(); 
        tstatus = SEND_DATA;
        break;
    
    case SEND_DATA:
        if(app_scheduler_millis() - curr_tick <100)
            return 0;
        send_data[0]= 0x82;
        send_data[1]=0x11;
        
        switch (s_hmi_frame_cache.gas_button.data[1])
        {
            case 0x13:{
                dataup_set_enable(TRUE,sH2);
                send_data[2]= 0;
                
                }
            break;
            
            case 0x14:{
                dataup_set_enable(TRUE,sC2H2);
                send_data[2]= 0x32;
                
                }
            break;

            case 0x15:{
                dataup_set_enable(TRUE,sCH4);
                send_data[2]= 0x64;
                }
            break;

            case 0x16:{//C2H6
                dataup_set_enable(TRUE,sC2H6);
                send_data[2] = 0x96;
                }
            break;
            default:
            break;
        }
        HMI_param.cartype = HMI_DATAUP_ANALY;
        s_hmi_poll.bits.gas_button = 0;
        tstatus = OPEN_MATH;
    break;  

    // case CLOSE_MATH:
    //     dataup_set_enable(FALSE, HMI_param.conc_type);
    //     if(app_scheduler_millis() - curr_tick <500)
    //         return 0;
    //     send_data[0]= 0x82;
    //     send_data[1]= 0x30;
    //     switch (s_hmi_frame_cache.gas_button.data[1])
    //     {
    //         case 0x13:{
    //             send_data[2]= 0x20;
    //             }
    //         break;
            
    //         case 0x14:{
    //             send_data[2]= 0x60;
    //             }
    //         break;

    //         case 0x15:{
    //             send_data[2]= 0xA0;
    //             }
    //         break;

    //         case 0x16:{
    //             send_data[2]= 0xE0;        
    //             }
    //         break;
    //         default:
    //         break;
    //     }
    //     send_data[3] = 0xFF;
    //     uint8_t clear_info[]= {0X82,0X28,0,0,0,0,0,0,0,0};
    //     dgusii_send(clear_info,10);
    //     dgusii_send(send_data,4);   
    //     app_scheduler_delay_ms(50);
    //     s_hmi_poll.bits.gas_button = 0;
    //     tstatus = START;
    // break;
    default:
        break;
    }

    return 0;
}

static uint8_t _hmi_handle_gas_button_Cycletime(void){
    // if(HMI_param.cyc_botton ==FALSE)
    //     return 0;
    static uint8_t init_num = 0;
    static uint32_t init_tick = 0;
    static uint8_t init_data[4]={0X82,0X30,0X20,0XFF};
    
    if(anay_init_flag == 0){
        if(app_scheduler_millis()-init_tick<80)
            return 0;
        else{
            init_tick = app_scheduler_millis();
        }
        switch (init_num)
        {    
            case 0:
                init_data[2] = 0x20;
                init_num = 1;
                break;
            case 1:
                init_data[2] = 0x60;
                init_num = 2;
                break;
            case 2:
                init_data[2] = 0xA0;               
                init_num = 3;
                break;
            case 3:
                init_data[2] = 0xE0;
                init_num = 0;
                anay_init_flag = 1;
                break;
            }
            dgusii_send(init_data,4);
            return 0;
    }    

    static uint32_t curr_time,last_time;
    curr_time = app_scheduler_millis();
    if((app_scheduler_millis() - last_time)<1000)
        return 0;
    rtc_datetime_t now;
    int len;
    uint8_t usb_buf[32];
    last_time = curr_time;
    usb_buf[0] = 0x82;
    usb_buf[1] = 0x19;
    usb_buf[2] = 0;
    if (rtc_get_datetime(&now) != HAL_OK)
    {
        usb_cdc_send_packet((uint8_t *)"HMI RTC fail\r\n",14);
        return 1;
    }
    len = snprintf((char*)&usb_buf[3], sizeof(usb_buf)-3,
                "%04u-%02u-%02u %02u:%02u:%02u",
                now.year, now.month, now.date,
                now.hour, now.min, now.sec);
    if (len <= 0)
    {
        usb_cdc_send_packet((uint8_t *)"HMI time RTC fail\r\n",19);
        return 1;
    }
    if (dgusii_send(usb_buf, (uint16_t)len + 3U) != HAL_OK)
    {
        return 1U;
    }

    return 0;
}
void HMI_taskDelay(void)
{
}

void HMI_dataup_conc(float conc){
    if(HMI_param.cartype == HMI_DATAUP_ANALY){
        {
            float kb[4][2];
            const uint8_t g = sensordata.last_status;
            if (g >= 1 && g <= 4 && storage_param_read_all(kb) == 0) {
                if (kb[g - 1][0] >= 0.001f || kb[g - 1][0] <= -0.001f)
                    conc = kb[g - 1][0] * conc + kb[g - 1][1];
            }
        }
        dgusii_send(HMI_TEXT_READ_OVER_GBK,HMIHMI_READ_OVER_LEN);
        stats_floatToBytes(conc,&send_data[3]);
        dgusii_send(send_data,7);
        HMI_openTouch();
        Time_DislayBot(TRUE);
        {
            uint8_t log_buf[15];
            rtc_datetime_t now;
            static const char gas_names[][4] = {"H2  ","C2H2","CH4 ","C2H6"};
            const uint8_t g = sensordata.last_status;
            if (rtc_get_datetime(&now) == HAL_OK) {
                log_buf[0] = now.year;
                log_buf[1] = now.month;
                log_buf[2] = now.date;
                log_buf[3] = now.hour;
                log_buf[4] = now.min;
                log_buf[5] = now.sec;
                if (g >= 1 && g <= 4) {
                    memcpy(&log_buf[6], gas_names[g - 1], 4);
                } else {
                    memcpy(&log_buf[6], "UNKN", 4);
                }
                log_buf[10] = s_math_open[2];
                memcpy(&log_buf[11], &conc, 4);
                storage_log_write(log_buf, sizeof(log_buf));
            }
        }
    }else if(HMI_param.cartype == HMI_DATAUP_CARIBRATION){
        dgusii_send(HMI_TEXT_READ_OVER_CAB_GBK, HMI_TEXT_READ_OVER_CAB_GBK_LEN);
        stats_floatToBytes(conc,&send_data[3]);
        dgusii_send(send_data,7);
        HMI_openTouch();
        Time_DislayBot(TRUE);
    }
}

void HMI_dataup_conc_interim(float conc)
{
    if(HMI_param.cartype == HMI_DATAUP_ANALY){
        {
            float kb[4][2];
            const uint8_t g = sensordata.last_status;
            if (g >= 1 && g <= 4 && storage_param_read_all(kb) == 0) {
                if (kb[g - 1][0] >= 0.001f || kb[g - 1][0] <= -0.001f)
                    conc = kb[g - 1][0] * conc + kb[g - 1][1];
            }
        }
        stats_floatToBytes(conc,&send_data[3]);
        dgusii_send(send_data,7);
    }else if(HMI_param.cartype == HMI_DATAUP_CARIBRATION){
        stats_floatToBytes(conc,&send_data[3]);
        dgusii_send(send_data,7);
    }
}

void HMI_dataup_show_failure(void)
{
    if (HMI_param.cartype == HMI_DATAUP_CARIBRATION)
    {
        dgusii_send(HMI_TEXT_READ_FAIL_CAB_GBK, HMI_TEXT_READ_FAIL_CAB_GBK_LEN);
    }
    else
    {
        dgusii_send(HMI_TEXT_READ_FAIL_GBK, HMIHMI_READ_FAIL_LEN);
    }
}

void HMI_dataup_show_timeout(void)
{
    if(HMI_param.cartype == HMI_DATAUP_ANALY){
        dgusii_send(HMI_TEXT_READ_NOSTABLE_GBK, HMIHMI_READ_NOSTABLE_LEN);
    }else if(HMI_param.cartype == HMI_DATAUP_CARIBRATION){
        dgusii_send(HMI_TEXT_READ_FAIL_CAB_GBK, HMI_TEXT_READ_FAIL_CAB_GBK_LEN);
    }
}

void HMI_Powerstatus(uint8_t power)
{
	HAL_GPIO_WritePin(HMI_EN_GPIO_Port, HMI_EN_Pin, power ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void HMI_closeTouch(void)
{
    uint8_t tx_buf[] = {0x82, 0x00, 0xFC, 0x55,0XAA,0X5A,0X5A};
    dgusii_send(tx_buf, sizeof(tx_buf));
}

void HMI_openTouch(void)
{
    uint8_t tx_buf[] = {0x82, 0x00, 0xFC, 0x00,0x00,0x00,0X00};
    dgusii_send(tx_buf, sizeof(tx_buf));
}

uint16_t get_clean_time(void){
    return clean_time;
}
void Time_DislayBot(uint8_t en){
    s_hmi_poll.bits.cyc_time = en; 
}
