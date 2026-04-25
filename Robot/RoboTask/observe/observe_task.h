#ifndef OBSERVE_TASK_H
#define OBSERVE_TASK_H

#include "stdint.h"
#include "INS_task.h"
#include "main.h"

typedef struct 
{
    /* data */
    float wr, wl;       //驱动轮转子相对大地角速度，这里定义的是顺时针为正（方向待定）
    float vrb,vlb;      //机体b系的速度
    float forward_v;    //前进速度
    float angular_v;    //角速度
    float T;            //速度环力矩
} Observe_Data_t;

extern Observe_Data_t observe_data;

void Observe_Task(void);

#endif