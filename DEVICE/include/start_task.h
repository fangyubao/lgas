#ifndef __START_TASK_H
#define __START_TASK_H

#include <stdint.h>
typedef struct
{
    float value;
    float std[2];
}base_conc_ch;

typedef struct
{
    base_conc_ch ch4;
    base_conc_ch c2h6;
    base_conc_ch c2h2;
    base_conc_ch h2;
}base_conc;
extern base_conc base_data;
uint8_t start_task_run(void);
uint32_t PowerOn_getTime(void);
void PowerOn_setTime(uint32_t param);
uint8_t start_task_readbase(void);
#endif /* __START_TASK_H */
