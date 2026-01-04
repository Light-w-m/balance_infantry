#include "uartReceive_task.h"
#include "INS_task.h"
#include "cmsis_os.h"
#include "arm_math.h"

extern INS_t INS;

void UartReceive_Task(void)
{
    char tx_buff[128];
    uint16_t len;

    while (1)
    {
        /* code */
        len = snprintf(tx_buff,sizeof(tx_buff), "%.4f, %.4f, %.4f\r\n",
                 INS.Pitch, INS.Roll, INS.Yaw);

        // len = snprintf(tx_buff,sizeof(tx_buff), "%.4f, %.4f, %.4f, %.4f, %.4f, %.4f\r\n",
        //          INS.Gyro[0], INS.Gyro[1], INS.Gyro[2],
        //          INS.Accel[0], INS.Accel[1], INS.Accel[2]);

        HAL_UART_Transmit(&huart7, (uint8_t *)tx_buff, len, 10);

        osDelay(10);
    }
    
}
