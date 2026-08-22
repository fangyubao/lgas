#include "dgusii.h"
#define DEFAULT_CLEAN_TIME 5U /* seconds */
void dgusii_HMI_task(const dgusii_frame_t *frame, void *user_ctx);
void HMI_task_poll(void);
void HMI_task_debug_dump_cache(void);
void HMI_taskDelay(void);
void HMI_dataup_conc(float conc);
void HMI_dataup_conc_interim(float conc);
void HMI_dataup_show_failure(void);
void HMI_dataup_show_timeout(void);
void HMI_Powerstatus(uint8_t power);
void HMI_closeTouch(void);
void HMI_openTouch(void);
uint16_t get_clean_time(void);
void Time_DislayBot(uint8_t en);
