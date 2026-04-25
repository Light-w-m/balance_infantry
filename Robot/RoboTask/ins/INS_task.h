/**
 ******************************************************************************
 * @file    ins_task.h
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "stdint.h"
#include "BMI088driver.h"
#include "string.h"

//温度控制参数
#define DES_TEMP    40.0f
#define KP          100.f
#define KI          50.f
#define KD          10.f
#define MAX_OUT     500

#define INS_TASK_PERIOD 1

#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t  *)(p)))
#define I2(p) (*((int16_t  *)(p)))

static uint16_t U2(uint8_t *p) {uint16_t u; memcpy(&u,p,2); return u;}
static uint32_t U4(uint8_t *p) {uint32_t u; memcpy(&u,p,4); return u;}
static int32_t  I4(uint8_t *p) {int32_t  u; memcpy(&u,p,4); return u;}
static float    R4(uint8_t *p) {float    r; memcpy(&r,p,4); return r;}

typedef struct
{
    float q[4]; // 四元数

    float Gyro[3];  // 角速度--根据安装位置--0：roll, 1：pitch, 2：yaw
    float Accel[3]; // 加速度
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // 位姿
    float Roll;
    float Pitch;
    float Yaw;

    float YawTotal; // 累计的航向角,用于底盘控制
		
	uint8_t ins_flag;
} INS_t;


/**
 * @brief 用于修正安装误差的参数,demo中可无视
 * 
 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;


void HIPNUC_Init(void);
void HIPNUC_Receive(void);
void INS_task(void);

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

#endif


