#ifndef HMI_TEXT_GBK_H
#define HMI_TEXT_GBK_H

#include <stdint.h>

extern const uint8_t HMI_TEXT_INITING_GBK[];
extern const uint8_t HMI_TEXT_PREHEAT_GBK[];
extern uint8_t HMI_TEXT_INIT_ERR_ID_GBK[];
extern const uint8_t HMI_TEXT_READ_ING_GBK[];
extern const uint8_t HMI_TEXT_READ_OVER_GBK[];
extern const uint8_t HMI_TEXT_READ_FAIL_GBK[];
extern const uint8_t HMI_CLOSE_TOUCH[];
extern const uint8_t HMI_OPEN_TOUCH[];
extern const uint8_t HMI_INTO_MENU[];
extern const uint8_t HMI_SCREEN_LIGHT[];
extern const uint8_t HMI_TEXT_READ_NOSTABLE_GBK[];
extern const uint8_t HMI_TEXT_READ_ING_CAB_GBK[];
extern const uint8_t HMI_TEXT_READ_OVER_CAB_GBK[];
extern const uint8_t HMI_TEXT_READ_FAIL_CAB_GBK[];

#define HMI_TEXT_INITING_GBK_LEN      20U
#define HMI_TEXT_PREHEAT_GBK_LEN      20U
#define HMI_TEXT_INIT_ERR_ID_GBK_LEN  20U

#define HMI_CLOSE_TOUCH_LEN           10U
#define HMI_OPEN_TOUCH_LEN            10U
#define HMI_INTO_MENU_LEN             10U
#define HMIHMI_SCREEN_LIGHT_LEN       8U

#define HMIHMI_READ_ING_LEN           17U
#define HMIHMI_READ_OVER_LEN          17U
#define HMIHMI_READ_FAIL_LEN          17U
#define HMIHMI_READ_NOSTABLE_LEN      17U
#define HMI_TEXT_READ_CAB_GBK_LEN      4U
#define HMI_TEXT_READ_ING_CAB_GBK_LEN  HMI_TEXT_READ_CAB_GBK_LEN
#define HMI_TEXT_READ_OVER_CAB_GBK_LEN HMI_TEXT_READ_CAB_GBK_LEN
#define HMI_TEXT_READ_FAIL_CAB_GBK_LEN HMI_TEXT_READ_CAB_GBK_LEN
#endif /* HMI_TEXT_GBK_H */
