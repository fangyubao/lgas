#include "bus485_task.h"
#include "CH4.h"
#include "dataup.h"
#include "main.h"
#include "C2H2.h"
#include "stats.h"
#include "C2H6.h"
#include "HMI_task.h"
#include "H2.h"
#include "pump.h"
#include "flash_storage.h"
#include <stdio.h>
#include <math.h>
#include "hmi_text_gbk.h"
#ifdef DEBUG_MODE
#include "usbd_cdc_if.h"
#endif
#define DATAUP_PERIOD_MS 500U
#define COLLECT_TIMEOUT_MS  3*60*1000U
dataup sensordata = {0};
static uint8_t s_dataup_enable = 0U;
static uint32_t s_c2h2_measurement_id = 0U;
uint8_t gas_wait_sec[4]={3,3,3,3};

void dataup_taskDelay(void)
{
}



void dataup_set_enable(uint8_t enable,uint8_t gas)
{
    s_dataup_enable = (enable != 0U) ? 1U : 0U;
    if (s_dataup_enable == 0U)
    {
        sensordata.wait_flag = 0U;
        sensordata.status = sNOGAS;
        return;
    }

    if ((gas < sH2) || (gas > sC2H6))
    {
        s_dataup_enable = 0U;
        sensordata.wait_flag = 0U;
        sensordata.status = sNOGAS;
        return;
    }

    {
        sensordata._tick = 0U;
        sensordata.wait_tick = app_scheduler_millis();
        sensordata.wait_flag = 1U;
        sensordata.wait_sec = gas_wait_sec[gas-1];
        sensordata.status = gas;
        sensordata.collect_start = app_scheduler_millis();
    }
}

uint8_t dataup_get_enable(void)
{
    return s_dataup_enable;
}

uint8_t updata_sensor(void)
{
    if (s_dataup_enable == 0U)
    {
        return 0U;
    }
    if(sensordata.wait_flag == 1U)
    {
        if ((uint32_t)(app_scheduler_millis() - sensordata.wait_tick) < (uint32_t)sensordata.wait_sec * 1000U)
        {
            return 0U;
        }else{
            sensordata.wait_tick = 0U;
            sensordata.wait_flag = 0U;
        }
    }

    if ((sensordata._tick != 0U) && ((int32_t)(app_scheduler_millis() - sensordata._tick) < 0))
    {
        return 0U;
    }
    sensordata._tick = app_scheduler_millis() + DATAUP_PERIOD_MS;

    switch (sensordata.status)
    {
    case sCH4:
        {
            static uint8_t ch4_idx = 0;
            static uint8_t ch4_filled = 0;
            static float ch4_window[10] = {0.0};
            static uint8_t slope_ok = 0;
            static float prev_slope = 0.0f;
            uint8_t ret = CH4_requestConc();

            if (ret == BUS485_FAIL)
            {
                HMI_dataup_show_failure();
                HMI_openTouch();
                s_dataup_enable = 0U;
                return 1U;
            }
            if (ret == BUS485_DONE)
            {
                if (CH4_getConc(&ch4_window[ch4_idx]) != 0U)
                {
                    return 1U;
                }
                ch4_idx += 1;
                if (ch4_idx >= 10) {
                    ch4_idx = 0;
                    ch4_filled = 1;
                }
                if (ch4_filled) {
                    float stddev[2];
                    uint8_t ch4_buf[64];
                    float mean = stats_getFilteredMean(ch4_window, 10, 0.9, stddev);

                    if (!slope_ok) {
                        if (prev_slope * stddev[1] < 0.0f || fabsf(stddev[1]) < 1e-8f)
                            slope_ok = 1;
                        prev_slope = stddev[1];
                    }

                    if (slope_ok && stddev[0] <= 0.001f) {
                        sensordata.CH4 = mean;
                        sensordata.last_status = sCH4;
                        sensordata.status = sUPDATA;
                        slope_ok = 0;
                        prev_slope = 0.0f;
                        sprintf((char *)ch4_buf, "CH4 conc=%f, cv=%f\r\n", sensordata.CH4, stddev[0]);
                        usb_cdc_send_packet(ch4_buf, (uint16_t)strlen((char *)ch4_buf));
                        Time_DislayBot(FALSE);
                        HMI_dataup_conc(sensordata.CH4);
                    } else {
                        HMI_dataup_conc_interim(mean);
                        if ((uint32_t)(app_scheduler_millis() - sensordata.collect_start) >= COLLECT_TIMEOUT_MS) {
                            HMI_dataup_show_timeout();
                            s_dataup_enable = 0U;
                            HMI_openTouch();
                            return 1U;
                        }
                    }
                }
            }
            return 0U;
        }

    case sC2H6:{
        static uint8_t c2h6_idx = 0;
        static uint8_t c2h6_filled = 0;
        static float c2h6_window[10] = {0.0};
        static uint8_t slope_ok = 0;
        static float prev_slope = 0.0f;
        static C2H6_frame_t frame;
        uint8_t c2h6_temp[2]={0x01,0x03};
        uint8_t ret;
        ret = C2H6_run(c2h6_temp, &frame);

        if (ret == 1U){
            if(frame.len<=1){
                HMI_dataup_show_failure();
                HMI_openTouch();
                s_dataup_enable = 0U;
                return 1;
            }
            c2h6_window[c2h6_idx] =(float)((uint16_t)frame.data[0]<<8|frame.data[1]);
            c2h6_idx += 1;
            if (c2h6_idx >= 10) {
                c2h6_idx = 0;
                c2h6_filled = 1;
            }
            if (c2h6_filled){
                float stddev[2];
                uint8_t c2h6_buf[64];
                float mean = stats_getFilteredMean(c2h6_window,10,0.9,stddev);

                if (!slope_ok) {
                    if (prev_slope * stddev[1] < 0.0f || fabsf(stddev[1]) < 1e-8f)
                        slope_ok = 1;
                    prev_slope = stddev[1];
                }

                if (slope_ok && stddev[0] <= 0.001f) {
                    sensordata.C2H6 = mean;
                    sensordata.last_status = sC2H6;
                    sensordata.status = sUPDATA;
                    slope_ok = 0;
                    prev_slope = 0.0f;
                    sprintf((char *)c2h6_buf, "C2H6 conc=%f, cv=%f\r\n", sensordata.C2H6, stddev[0]);
                    usb_cdc_send_packet(c2h6_buf, (uint16_t)strlen((char *)c2h6_buf));
                    Time_DislayBot(FALSE);
                    HMI_dataup_conc(sensordata.C2H6);
                } else {
                    HMI_dataup_conc_interim(mean);
                    if ((uint32_t)(app_scheduler_millis() - sensordata.collect_start) >= COLLECT_TIMEOUT_MS) {
                        HMI_dataup_show_timeout();
                        s_dataup_enable = 0U;
                        HMI_openTouch();
                        return 1U;
                    }
                }
            }
        }
        if (ret == 0U){
            HMI_dataup_show_failure();
            HMI_openTouch();
            s_dataup_enable = 0U;
            sensordata.status = sNOGAS;
            return 1;
        }

        return 0U;
				}
    case sH2:{
        static uint8_t h2_idx = 0;
        static uint8_t h2_filled = 0;
        static float h2_window[10] = {0.0};
        static uint8_t slope_ok = 0;
        static float prev_slope = 0.0f;
        uint8_t req[3] = {BUS_MODBUS, BUS_MOD_H2, H2_CMD_CONC};
        const H2_data_t *h2_data = H2_getData();
        uint8_t ret = bus485_request(req, (uint8_t)sizeof(req));
        if (ret == BUS485_FAIL){
            HMI_dataup_show_failure();
            HMI_openTouch();
            s_dataup_enable = 0U;
            return 1U;
        }
        if (ret == BUS485_DONE){
            float conc = h2_data->conc;
            if (conc < 0.0f)
            {
                return 1U;
            }
            h2_window[h2_idx] = conc;
            h2_idx += 1;
            if (h2_idx >= 10) {
                h2_idx = 0;
                h2_filled = 1;
            }
            if (h2_filled){
                uint8_t h2_buf[64];
                float stddev[2];
                float mean = stats_getFilteredMean(h2_window,10,0.9,stddev);

                if (!slope_ok) {
                    if (prev_slope * stddev[1] < 0.0f || fabsf(stddev[1]) < 1e-8f)
                        slope_ok = 1;
                    prev_slope = stddev[1];
                }

                if (slope_ok && stddev[0] <= 0.001f) {
                    sensordata.H2 = mean;
                    sensordata.last_status = sH2;
                    sensordata.status = sUPDATA;
                    slope_ok = 0;
                    prev_slope = 0.0f;
                    sprintf((char *)h2_buf, "H2 conc=%f, cv=%f\r\n", sensordata.H2, stddev[0]);
                    usb_cdc_send_packet(h2_buf, (uint16_t)strlen((char *)h2_buf));
                    Time_DislayBot(FALSE);
                    HMI_dataup_conc(sensordata.H2);
                } else {
                    HMI_dataup_conc_interim(mean);
                    if ((uint32_t)(app_scheduler_millis() - sensordata.collect_start) >= COLLECT_TIMEOUT_MS) {
                        HMI_dataup_show_timeout();
                        s_dataup_enable = 0U;
                        HMI_openTouch();
                        return 1U;
                    }
                }
            }
        }

        return 0U;
				}
    case sC2H2:{
        static uint8_t c2h2_idx = 0;
        static uint8_t c2h2_filled = 0;
        static float c2h2_window[10] = {0.0};
        static uint8_t slope_ok = 0;
        static float prev_slope = 0.0f;
        static uint32_t c2h2_measurement_seen = 0U;
        uint8_t ret;
        uint8_t req[3]={0};
        C2H2_t* c2h2temp =_C2H2_getdata();

        if (c2h2_measurement_seen != s_c2h2_measurement_id)
        {
            uint8_t i;

            c2h2_measurement_seen = s_c2h2_measurement_id;
            c2h2_idx = 0U;
            c2h2_filled = 0U;
            slope_ok = 0U;
            prev_slope = 0.0f;
            for (i = 0U; i < (uint8_t)(sizeof(c2h2_window) / sizeof(c2h2_window[0])); i++)
            {
                c2h2_window[i] = 0.0f;
            }
        }

        req[0] = BUS_MODBUS;
        req[1] = BUS_MOD_C2H2;
        req[2] = C2H2_CONC;
        ret = bus485_request(req, (uint8_t)sizeof(req));
        if(ret == BUS485_FAIL){
            HMI_dataup_show_failure();
            HMI_openTouch();
            s_dataup_enable = 0U;
            usb_cdc_send_packet((uint8_t *)"C2H2 get conc failed\r\n",24);
            return 1U;
        }
        if (ret == BUS485_DONE)
        {
            c2h2_window[c2h2_idx] = (float)c2h2temp->conc;
            c2h2_idx += 1;
            if (c2h2_idx >= 10) {
                c2h2_idx = 0;
                c2h2_filled = 1;
            }
            if (c2h2_filled){
                uint8_t c2h2_buf[64];
                float stddev[2];
                float mean = stats_getFilteredMean(c2h2_window,10,0.9,stddev);

                if (!slope_ok) {
                    if (prev_slope * stddev[1] < 0.0f || fabsf(stddev[1]) < 1e-8f)
                        slope_ok = 1;
                    prev_slope = stddev[1];
                }

                if (slope_ok && stddev[0] <= 0.001f) {
                    sensordata.C2H2 = mean;
                    sensordata.last_status = sC2H2;
                    sensordata.status = sUPDATA;
                    slope_ok = 0;
                    prev_slope = 0.0f;
                    sprintf((char *)c2h2_buf, "C2H2 conc=%f, cv=%f\r\n", sensordata.C2H2, stddev[0]);
                    usb_cdc_send_packet(c2h2_buf, (uint16_t)strlen((char *)c2h2_buf));
                    Time_DislayBot(FALSE);
                    HMI_dataup_conc(sensordata.C2H2);
                } else {
                    HMI_dataup_conc_interim(mean);
                    if ((uint32_t)(app_scheduler_millis() - sensordata.collect_start) >= COLLECT_TIMEOUT_MS) {
                        HMI_dataup_show_timeout();
                        s_dataup_enable = 0U;
                        HMI_openTouch();
                        return 1U;
                    }
                }
            }
        }
        return 0U;
    }

  case sUPDATA:
  {
      uint8_t req[] = {BUS_UPDATA, BUS_UPDATA_SENSOR, 0U};
      req[2] = sensordata.last_status;
      uint8_t state = bus485_request(req, (uint8_t)sizeof(req));

      if (state == BUS485_DONE)
      {
          sensordata.status = sNOGAS;
          
          s_dataup_enable = 0U;
          return 0U;
      }
      if (state == BUS485_FAIL)
      {
        /* The concentration was already measured and shown successfully.
         * A later record-upload failure must not overwrite that result on
         * the HMI as a measurement failure. */
        sensordata.status = sNOGAS;
        s_dataup_enable = 0U;
        return 1U;
      }
      return 0U;
  }

    case sNOGAS:{
    default:
        sensordata.status = sUPDATA;
        sensordata.last_status = sNOGAS;
        return 1U;
	}
}
}
