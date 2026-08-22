#ifndef __CH4_H
#define __CH4_H
#include "stdint.h"
#include "usb_task.h"
#ifdef __cplusplus
extern "C" {
#endif



#define CH4_MODBUS_SLAVE_ID_DEFAULT        0xFFU
#define CH4_MODBUS_SLAVE_ID                0x49
#define CH4_MODBUS_TIMEOUT_MS              1000U

#define CH4_REG_ID          0x2011U
#define CH4_REG_TYPE        0x2021U
#define CH4_REG_CONCCODE    0x2027U
#define CH4_REG_RANGE       0x202bU
#define CH4_REG_UCODE       0x2030U
#define CH4_REG_PN          0x2031U
#define CH4_REG_WSTATUS     0X6000U
#define CH4_REG_CONCD       0x6001U
#define CH4_REG_AD          0X6002U
#define CH4_REG_RUN         0X6006U

#define CH4_CMD_SET_ID      0x80U
#define CH4_RUN_DONE        0U
#define CH4_RUN_FAIL        1U
#define CH4_RUN_BUSY        2U

#define PPM                 0X02

#define CH4_taskDelay() do { } while (0)

typedef struct {
    uint16_t id;
    uint8_t pointbit;//小数点位�?
    uint16_t CH4da;//模拟�?
    uint16_t conc_code;//气体代码
    uint16_t conc_m;//浓度�?
    uint16_t run_status;//运�?�状�?
    uint16_t waring;//警报
    uint16_t ucode;//单位代码
    uint16_t crang;//范围
    uint16_t ctype;//类型
}CH4_t;

typedef enum
{
    CH4_INFO_RUN = 0,
    CH4_INFO_POINTNUM,
    CH4_INFO_GUCODE,
    CH4_INFO_RANGE,
    CH4_INFO_CONCCODE,
    CH4_INFO_AD,
    CH4_INFO_CONC,
    CH4_INFO_TYPE,
    CH4_INFO_COUNT
} CH4_info_id_t;

uint8_t CH4_exec(uint8_t *cmd, CH4_t *out);
CH4_t* CH4_getCH4data(void);
uint8_t CH4_readConc(float *data);
uint8_t CH4_ReadInfo( CH4_t *out);
uint8_t CH4_setCH4Ucode(void);
uint8_t CH4_CloseProject(void);
uint8_t CH4_requestConc(void);
uint8_t CH4_getConc(float *data);
extern volatile usb_delay_t ch4_delay;
#ifdef __cplusplus
}
#endif

#endif /* __CH4_H */
