#ifndef CHASSISR_TASK_H
#define CHASSISR_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "chassis_def.h"
#include "kinematics.h"
#include "kalman_filter.h"
#include "pid.h"
#include "DJI3508.h"
#include "DM8009.h"
#include "INS_task.h"

/**
 * @brief 在rtos中调用的底盘任务--1kHz
 * 
 */
void ChassisR_Task(void);


void ChasssisR_Control(
    chassis_t* chassis, Leg_t* leg, Excessive_t* excessive, INS_t* ins, PidTypeDef* length_pid, float lqr_k[2][6]);


#endif // !CHASSISR_TASK_H
