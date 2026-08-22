#include "C2H2.h"
#include "usb_task.h"
#include "modbus_master.h"
#include "stats.h"
C2H2_t c2h2_data = {0};
static uint16_t read_data[15];
uint8_t C2H2_Getdata(uint8_t *cmd)
{
    static uint8_t C2H2_status = IDLE;
    if (cmd == NULL)
    {
        C2H2_status = IDLE;
        modbus_master_clear();
        return C2H2_RUN_FAIL;
    }
    modbus_master_poll();
    switch(C2H2_status){
        case IDLE:
            switch(*cmd){
                case C2H2_INFO:
                if(modbus_master_start_read_input(C2H2_MODBUS_SLAVE_ID,0,4,read_data,C2H2_MODBUS_TIMEOUT_MS)!=HAL_OK){
                    modbus_master_clear(); 
                    return C2H2_RUN_FAIL;
                }
                break;
                case C2H2_CONC:
                default:
                if(modbus_master_start_read_input(C2H2_MODBUS_SLAVE_ID,0,1,read_data,C2H2_MODBUS_TIMEOUT_MS)!=HAL_OK){
                    modbus_master_clear(); 
                    return C2H2_RUN_FAIL;
                } 
                break;
            }
            C2H2_status = WAIT; 
            return C2H2_RUN_BUSY;

        case WAIT:
            if (modbus_master_get_state() == MODBUS_MASTER_BUSY)
            {
                return C2H2_RUN_BUSY;
            }
            if (modbus_master_get_state() != MODBUS_MASTER_DONE){
                C2H2_status = IDLE;
                modbus_master_clear(); 
                return C2H2_RUN_FAIL;
            }
            C2H2_status = UT_SUCCESS ;
            modbus_master_clear();    
            return C2H2_RUN_BUSY;

        case UT_SUCCESS:
        C2H2_status = IDLE ;
        switch (*cmd)
        {
        case C2H2_INFO:
            c2h2_data.conc = read_data[0];
            c2h2_data.statusH = read_data[1];
            c2h2_data.statusL = read_data[2];
            c2h2_data.relight = read_data[3];            
            break;
        case C2H2_CONC:
        default:
            c2h2_data.conc = read_data[0];
            break;
        }
            return C2H2_RUN_DONE;
        default:
        modbus_master_clear();
        C2H2_status = IDLE ;
        return C2H2_RUN_FAIL;
    }
}

C2H2_t* _C2H2_getdata(void){
    return &c2h2_data;
}

HAL_StatusTypeDef C2H2_readinfo(float *c2h2_conc)
{
    uint16_t read_C2H2data[15];
    float data[3]={0.0},std[2];
    for(uint8_t i=0;i<3;i++){
        if(modbus_master_read_input(C2H2_MODBUS_SLAVE_ID,0,1,read_C2H2data,C2H2_MODBUS_TIMEOUT_MS)!=HAL_OK)
        {
            return HAL_ERROR;
        }
        data[i] = (float)read_C2H2data[0];
        HAL_Delay(50);
    }
    *c2h2_conc = stats_getFilteredMean(data,3,0.9,std);
    
    return HAL_OK;
}
