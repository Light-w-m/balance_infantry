#ifndef CHASSISR_TASK_H
#define CHASSISR_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "fdcan.h"

/**
 * @brief 在rtos中调用的底盘任务--1kHz
 * 
 */
void ChassisR_Task(void);

#endif // !CHASSISR_TASK_H
