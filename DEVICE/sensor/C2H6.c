#include "C2H6.h"
#include "stdio.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
static uint8_t s_c2h6_waiting = 0U;

volatile usb_delay_t c2h6_delay = {0, IDLE, IDLE};
static uint8_t s_tx_payload[60];
static uint8_t s_rx_payload[60];
static C2H6_frame_t s_tx_frame = {C2H6_TX_FRAME_HEAD, 1U, 0U, s_tx_payload, 0U};
static C2H6_frame_t s_rx_frame = {0U, 0U, 0U, s_rx_payload, 0U};

static void C2H6_crc(C2H6_frame_t *frame)
{
    uint8_t crc = 0U;
    uint8_t i;
    uint8_t payload_len;

    if ((frame == NULL) || (frame->len == 0U) || ((frame->len > 1U) && (frame->data == NULL)))
    {
        return;
    }

    payload_len = (uint8_t)(frame->len - 1U);
    crc = (uint8_t)(crc + frame->hand);
    crc = (uint8_t)(crc + frame->len);
    crc = (uint8_t)(crc + frame->opc);
    for (i = 0U; i < payload_len; i++)
    {
        crc = (uint8_t)(crc + frame->data[i]);
    }
    frame->crc = (uint8_t)(0U - crc);
}

static uint8_t C2H6_send_frame(C2H6_frame_t *frame)
{
    uint8_t send_buf[64];
    uint8_t i;
    uint8_t payload_len;

    if ((frame == NULL) || (frame->len == 0U) || (frame->len > 61U) || ((frame->len > 1U) && (frame->data == NULL)))
    {
        return 1U;
    }

    payload_len = (uint8_t)(frame->len - 1U);
    send_buf[0] = frame->hand;
    send_buf[1] = frame->len;
    send_buf[2] = frame->opc;
    for (i = 0U; i < payload_len; i++)
    {
        send_buf[3U + i] = frame->data[i];
    }

    C2H6_crc(frame);
    send_buf[3U + payload_len] = frame->crc;

    return (uart_sendData(UART_PORT_UART4, send_buf, (uint16_t)(frame->len + 3U), CH2_TIMEOUT_MS) == HAL_OK) ? 0U : 1U;
}

static uint8_t C2H6_read_frame(C2H6_frame_t *frame)
{
    uint8_t recv_buf[64];
    uint16_t total_len;
    uint8_t i;
    uint8_t crc_calc = 0U;
    uint8_t payload_len;

    if (frame == NULL)
    {
        return 1U;
    }

    total_len = uart_getData(UART_PORT_UART4, recv_buf, sizeof(recv_buf));
    if (total_len < 4U)
    {
        return 1U;
    }

    frame->hand = recv_buf[0];
    frame->len = recv_buf[1];
    frame->opc = recv_buf[2];

    if ((frame->hand != C2H6_RX_FRAME_HEAD) || (frame->len == 0U))
    {
        return 1U;
    }

    payload_len = (uint8_t)(frame->len - 1U);
    if ((payload_len > sizeof(s_rx_payload)) || ((uint16_t)(frame->len + 3U) > total_len))
    {
        return 1U;
    }

    for (i = 0U; i < payload_len; i++)
    {
        s_rx_payload[i] = recv_buf[3U + i];
    }
    frame->data = s_rx_payload;
    frame->crc = recv_buf[3U + payload_len];

    crc_calc = (uint8_t)(crc_calc + frame->hand);
    crc_calc = (uint8_t)(crc_calc + frame->len);
    crc_calc = (uint8_t)(crc_calc + frame->opc);
    for (i = 0U; i < payload_len; i++)
    {
        crc_calc = (uint8_t)(crc_calc + frame->data[i]);
    }

    if ((uint8_t)(0U - crc_calc) != frame->crc)
    {
        return 1U;
    }

    return 0U;
}

uint8_t* C2H6_SNverify(void)
{
    static C2H6_frame_t C2H6_in={  .hand=C2H6_TX_FRAME_HEAD,
                            .len=0x01,
                            .opc=0x02,
                            };
    C2H6_send_frame(&C2H6_in);
    HAL_Delay(200);
    C2H6_read_frame(&C2H6_in);
    HAL_Delay(50);
    if(C2H6_in.len!=0X14){
        uint8_t buf[48];
        (void)snprintf((char *)buf, sizeof(buf), "C2H6 SN verify failed, len=%u\r\n", C2H6_in.len);
        usb_cdc_send_packet(buf, (uint16_t)strlen((char *)buf));
        return NULL;
    }
    return C2H6_in.data;
}

uint8_t* C2H6_conc(void)
{
    static C2H6_frame_t C2H6_in={  .hand=C2H6_TX_FRAME_HEAD,
                            .len=0x01,
                            .opc=0x03,
                            };
    C2H6_send_frame(&C2H6_in);
    HAL_Delay(50);
    C2H6_read_frame(&C2H6_in);
    return C2H6_in.data;
}

void C2H6_reset(void)
{
    c2h6_delay.usb_status = IDLE;
    s_c2h6_waiting = 0U;
}

uint8_t C2H6_run(uint8_t *c6h6,C2H6_frame_t* data){
    switch(c2h6_delay.usb_status)
    {
    case IDLE:     
        if(s_c2h6_waiting == 0U)
        {
            s_tx_frame.len=c6h6[0];
            s_tx_frame.opc = c6h6[1];
            for(uint8_t i=0;i<s_tx_frame.len-1;i++)
            {
                s_tx_frame.data[i]=c6h6[2U+i];
            }
            if (C2H6_send_frame(&s_tx_frame) != 0U)
            {
                ndelay_ms(&c2h6_delay,50,UT_FAIL);
            }else{
            ndelay_ms(&c2h6_delay,50,IDLE);
            s_c2h6_waiting = 1U;
            }
        }
        else{       
            if (C2H6_read_frame(&s_rx_frame) != 0U)
            {
                ndelay_ms(&c2h6_delay,50,UT_FAIL);
            }
            else if ((s_rx_frame.opc <= C2H6_ZEROCABR) &&(s_rx_frame.len >= 2U))
            {
                ndelay_ms(&c2h6_delay,50,UT_SUCCESS);
            }else{
                ndelay_ms(&c2h6_delay,50,UT_FAIL);
            }      
        }   
        break;
    case DELAY:
        if (ndelay_expired(&c2h6_delay) != 0U)
        {
            c2h6_delay.usb_status = c2h6_delay.next_id;
        }
        break;
    case UT_SUCCESS:
        if (data != NULL)
        {
            *data = s_rx_frame;
        }
        c2h6_delay.usb_status = IDLE;
        s_c2h6_waiting = 0U;
        return 1;
    break;
    case UT_FAIL:
        c2h6_delay.usb_status = IDLE;
        s_c2h6_waiting = 0U;
        return 0;
    break;
    }
    return 2;
}
