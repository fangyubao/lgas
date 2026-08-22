#include "start_task.h"
#include "rs485.h"
#include "CH4.h"
#include "C2H6.h"
#include "C2H2.h"
#include "solenoid.h"
#include "pump.h"
#include "usbd_cdc_if.h"
#include "dataup.h"
#include "stats.h"
#include "H2.h"
#include "main.h"

static uint8_t start_task_ch4_read(float *ch4_conc, float *std_out, uint8_t count);
static void start_base_hw(void);
static uint8_t start_task_c2h6_sn(void);
static uint8_t start_task_CH4_read(void);
static uint8_t start_task_c2h2_read(void);

base_conc base_data = {0};
uint8_t start_task_run(void)
{
    uint8_t fstatus=0;
    start_base_hw();
    #if 0
    uint8_t *c2h6_ctemp;
    if((fstatus=start_task_CH4_read())!=0U)
    {
        char msg[64];
        sprintf(msg, "Failed to read CH4 concentration, status=0x%02X\r\n", fstatus);
        usb_cdc_send_packet((uint8_t *)msg, (   uint16_t)strlen(msg));
        return fstatus;
    }
    HAL_Delay(100);
    
    if (H2_readConc(&base_data.h2.value) != HAL_OK)
    {
        char msg[64];
        sprintf(msg, "Failed to read H2 concentration\r\n");
        usb_cdc_send_packet((uint8_t *)msg, (uint16_t)strlen(msg));
        return 0X50U;
    }
    HAL_Delay(100);
    if(start_task_c2h6_sn()!=0U)
    {
        char msg[64];
        sprintf(msg, "Failed to verify C2H6 SN\r\n");
        usb_cdc_send_packet((uint8_t *)msg, (uint16_t)strlen(msg));
        return 0X20U;
    }
    c2h6_ctemp = C2H6_conc();
    base_data.c2h6.value = (float)((uint16_t)c2h6_ctemp[0]<<8|c2h6_ctemp[1]);
    base_data.c2h6.value = base_data.c2h6.value <1000.0f ? 0.0 :base_data.c2h6.value;
    HAL_Delay(100);
#endif
    for(uint8_t i=0;i<5;i++)
    { 
        HAL_Delay(500);
        if ((fstatus=start_task_c2h2_read())!= 0U)
        {
            
            if(i==4){
                char msg[64];
                sprintf(msg, "Failed to read C2H2 concentration, status=0x%02X\r\n", fstatus);
                usb_cdc_send_packet((uint8_t *)msg, (uint16_t)strlen(msg));
                return fstatus;
            }
        }
        else {
            break;
        }
    }
    HAL_Delay(100);
    HAL_GPIO_WritePin(C2H2_EN_GPIO_Port, C2H2_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(C2H2_EN_GPIO_Port, C2H2_EN_Pin, GPIO_PIN_RESET);
    char msg[128];
    sprintf(msg, "Start task done: CH4=%.2f, C2H6=%.2f, C2H2=%.2f, H2=%.2f\r\n", base_data.ch4.value, base_data.c2h6.value, base_data.c2h2.value, base_data.h2.value);
    usb_cdc_send_packet((uint8_t *)msg, (uint16_t)strlen(msg));
    return 0U;
}
uint8_t start_task_readbase(void){
    solenoid_set_duty(100);
    HAL_Delay(100);
#if 0
    uint8_t fstatus=0;
    uint8_t *c2h6_ctemp;
    if ((fstatus=start_task_ch4_read(&base_data.ch4.value, base_data.ch4.std, 20)) != 0U)
    {
        fstatus = (0x02<<4)|fstatus;
        return fstatus;
    }
    HAL_Delay(100);

    {
        float vals[20];
        uint8_t cnt = 0;
        for (uint8_t i = 0; i < 20; i++)
        {
            float val;
            if (H2_readConc(&val) == HAL_OK)
            {
                vals[cnt++] = val;
            }
            HAL_Delay(100);
        }
        if (cnt > 0)
        {
            base_data.h2.value = stats_getFilteredMean(vals, cnt, 0.9, base_data.h2.std);
        }
        else
        {
            char msg[64];
            sprintf(msg, "Failed to read H2 concentration\r\n");
            usb_cdc_send_packet((uint8_t *)msg, (uint16_t)strlen(msg));
            return 0X50U;
        }
    }
    HAL_Delay(100);

    {
        float vals[20];
        uint8_t cnt = 0;
        for (uint8_t i = 0; i < 20; i++)
        {
            c2h6_ctemp = C2H6_conc();
            float val = (float)((uint16_t)c2h6_ctemp[0]<<8|c2h6_ctemp[1]);
            if (val >= 1000.0f)
            {
                vals[cnt++] = val;
            }
            HAL_Delay(30);
        }
        base_data.c2h6.value = (cnt > 0) ? stats_getFilteredMean(vals, cnt, 0.9, base_data.c2h6.std) : 0.0f;
    }
    HAL_Delay(100);
#endif
#if 0
    {
        float vals[20];
        uint8_t cnt = 0;
        for (uint8_t i = 0; i < 20; i++)
        {
            float val;
            HAL_Delay(100);
            if (C2H2_readinfo(&val) == HAL_OK)
            {
                vals[cnt++] = val;
            }
           
        }
        if (cnt > 0)
        {
            base_data.c2h2.value = stats_getFilteredMean(vals, cnt, 0.9, base_data.c2h2.std);
        }
        else
        {
           char msg[64];
           sprintf(msg, "Failed to read C2H2 concentration\r\n");
           usb_cdc_send_packet((uint8_t *)msg, (uint16_t)strlen(msg));
           return 0X41U;
        }
    }
        #endif
    solenoid_set_duty(0);
    return 0;
}
static uint8_t start_task_ch4_read(float *ch4_conc, float *std_out, uint8_t count)
{
    CH4_t ch4_data;
    float data[20]={0.0};
    uint8_t n = (count > 20) ? 20 : count;
    for(uint8_t i=0;i<n;i++){
        if (CH4_ReadInfo(&ch4_data) != 0U)
            return 1U;
        if(ch4_data.conc_code!=0x06)
            return 2U;
        if(ch4_data.ctype!=0x05)
            return 3U;
        data[i] = (float)ch4_data.conc_m;
        HAL_Delay(10);
    }
    *ch4_conc = stats_getFilteredMean(data,n,0.9,std_out);
    return 0;
}

static uint8_t start_task_c2h6_sn(void){
    uint8_t *temp = C2H6_SNverify();
    if(temp ==NULL)
        return 1;

    else{
        #ifdef DEBUG_MODE
        usb_cdc_send_packet(temp,0x14);
        #endif
        return 0;
    }
}

static void start_base_hw(void){
    for(uint8_t i=0;i<PUMP_RUNING_DUTY-PUMP_START_DUTY;i++){
        pump_set_strength(PUMP_START_DUTY+i);
        HAL_Delay(PUMP_CLEAN_TIME_DEFAULT);
    }
    HAL_Delay(7000);
    pump_set_strength(0);
}

static uint8_t start_task_c2h2_read(void){

    if(C2H2_readinfo(&base_data.c2h2.value)!=HAL_OK){
        char msg[64];
        sprintf(msg, "Failed to read C2H2 info\r\n");
        usb_cdc_send_packet((uint8_t *)msg, (uint16_t)strlen(msg));
        return  0X41U;
    }

    return 0;
}

static uint8_t start_task_CH4_read(void){
    uint8_t fstatus=0;
    if(CH4_CloseProject()!=0) {
        return 0X10U;
	}
	HAL_Delay(50);
    if(CH4_setCH4Ucode()!=0){
        char msg[64];
        sprintf(msg, "Failed to set CH4 ucode\r\n");
        usb_cdc_send_packet((uint8_t *)msg, (uint16_t)strlen(msg));
        return 0X11U;
    }
    HAL_Delay(50);

    if ((fstatus=start_task_ch4_read(&base_data.ch4.value, base_data.ch4.std, 1)) != 0U)
    {
        fstatus = (0x02<<4)|fstatus;
        return fstatus;
    }
    return 0;
}

uint32_t PowerOn_getTime(void)
{
    return app_scheduler_millis();
}
void PowerOn_setTime(uint32_t param)
{
    app_scheduler_set_millis(param);
}
