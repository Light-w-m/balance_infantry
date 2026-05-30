#include "vofa_task.h"
#include "ins_task.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "chassis_def.h"
#include "kinematics.h"
#include "observe_task.h"

extern INS_t INS;
extern chassis_t chassis_move;
extern Leg_t legR;
extern Leg_t legL;
extern Observe_Data_t observe_data;

void VofaDebug_Task(void)
{
    char tx_buff[128];
    uint16_t len;

    while (1)
    {
        /* code */
        // len = snprintf(tx_buff,sizeof(tx_buff), "%.4f, %.4f\r\n",
        //          observe_data.forward_v, chassis_move.state.v_filter*10);
        // len = snprintf(tx_buff,sizeof(tx_buff), "%.4f, %.4f, %.4f, %.4f\r\n",
        //          INS.Pitch, INS.Roll, INS.Gyro[X_AXIS], INS.Gyro[Y_AXIS]);
        len = snprintf(tx_buff,sizeof(tx_buff), 
                "%.4f, %.4f, %.4f, %.4f,%.4f, %.4f, %.4f, %.4f, %.4f, %.4f\r\n",
                legL.rod.T, legR.rod.T, legL.rod.Tp, legR.rod.Tp,
                legL.joint.T1, legL.joint.T2, legR.joint.T1, legR.joint.T2,
                observe_data.forward_v, chassis_move.state.v_filter*10);
                 
        HAL_UART_Transmit(&huart7, (uint8_t *)tx_buff, len, 10);

        osDelay(10);
    }
}
