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
extern LegState_t legR_state;
extern LegState_t legL_state;
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
                "%.4f, %.4f, %.4f, %.4f,%.4f, %.4f, %.4f, %.4f,%.4f, %.4f, %.4f, %.4f\r\n",
                // legL.rod.T, legR.rod.T, legL.rod.Tp, legR.rod.Tp,
                // legL.joint.T1, legL.joint.T2, legR.joint.T1, legR.joint.T2,
                // observe_data.forward_v, chassis_move.state.v_filter*10,
                // chassis_move.leg_set, legL.rod.L0, legR.rod.L0,
                // INS.Roll, legL.rod.F0, legR.rod.F0
                legR_state.theta, legR_state.phi, legR_state.x_dot, 
                legL_state.theta, legL_state.phi, legL_state.x_dot,
                INS.Roll,
                legR.rod.L0, legR.rod.F0, legL.rod.L0, legL.rod.F0,
                chassis_move.roll_T
                );
                 
        HAL_UART_Transmit(&huart7, (uint8_t *)tx_buff, len, 10);

        osDelay(10);
    }
}
