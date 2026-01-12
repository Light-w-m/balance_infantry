#include "vofa_task.h"
#include "ins_task.h"
#include "cmsis_os.h"
#include "arm_math.h"

extern INS_t INS;

void VofaDebug_Task(void)
{
    char tx_buff[128];
    uint16_t len;

    while (1)
    {
        /* code */
        len = snprintf(tx_buff,sizeof(tx_buff), "%.4f, %.4f, %.4f\r\n",
                 INS.Pitch, INS.Roll, INS.Yaw);
                 
        HAL_UART_Transmit(&huart7, (uint8_t *)tx_buff, len, 10);

        osDelay(10);
    }
    
}
