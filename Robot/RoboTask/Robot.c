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
#include "INS_task.h"
// #include "uartReceive_task.h"
#include "BMI088driver.h"
#include "bsp_dwt.h"

void Robot_Init(void)
{
    
    DWT_Init(480); 
    while (BMI088_init(&hspi2,2) != BMI088_NO_ERROR)
    {
        /* code */
    }
}