/**
 * @file Robot.c
 * @brief 用于整个底盘初始化
 * @version 0.1
 * @date 2025-12-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "Robot.h"
#include "Robot_def.h"
#include "INS_task.h"
#include "djimotor.h"
#include "bsp_can.h"
#include "BMI088driver.h"
#include "bsp_dwt.h"

void RobotInit(void)
{
    
    DWT_Init(480); 
    #ifdef INS_OF_HIPNUC
    HIPNUC_Init();
    #endif

    FDCAN1_Config();
}

/**
 * @brief 以1khz的频率运行
 * 
 */
void MotorTask(void)
{
    DJIMotorControl();
}