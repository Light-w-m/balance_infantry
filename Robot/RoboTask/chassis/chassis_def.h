/**
 * @file chassis_def.h
 * @author your name (you@domain.com)
 * @brief 用于包含各种宏定义与全局结构体
 * @version 0.1
 * @date 2025-12-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef CHASSIS_DEF_H
#define CHASSIS_DEF_H

#include "DJI3508.h"
#include "DM8009.h"

// 限幅
#define SATURATE(in, min, max)    \
    do {                          \
        float *_p = (in);         \
        if (*_p < (min))          \
            *_p = (min);          \
        else if (*_p > (max))     \
            *_p = (max);          \
    } while(0)

/**********************physical parameters*******************/
#define GRAVITY 9.81f       // 重力加速度

#define BODY_MASS 0.0f          //  载体重量
#define WHEEL_MASS 0.0f         //  轮重量
#define WHEEL_RADIUS 0.0f       //  轮子半径
#define WHEEL_DISTANCE 0.0f     // 轮子间距

// 支持力阈值，当支持力小于这个值时认为离地
#define TAKE_OFF_FN_THRESHOLD (3.0f)
// 触地状态切换时间阈值，当时间接触或离地时间超过这个值时切换触地状态
#define TOUCH_TOGGLE_THRESHOLD (100)

/***********************pid parameters*******************/
#define LEG_PID_KP  0.0f
#define LEG_PID_KI  0.0f
#define LEG_PID_KD  0.0f
#define LEG_PID_MAX_OUT  90.0f //90ţ
#define LEG_PID_MAX_IOUT 0.0f

#define ROLL_PID_KP 140.0f
#define ROLL_PID_KI 0.0f 
#define ROLL_PID_KD 10.0f
#define ROLL_PID_MAX_OUT  100.0f
#define ROLL_PID_MAX_IOUT 0.0f

#define TP_PID_KP 30.0f
#define TP_PID_KI 0.0f 
#define TP_PID_KD 1.0f
#define TP_PID_MAX_OUT  2.0f
#define TP_PID_MAX_IOUT 0.0f

#define TURN_PID_KP 2.5f
#define TURN_PID_KI 0.0f 
#define TURN_PID_KD 0.3f
#define TURN_PID_MAX_OUT  1.0f//轮毂电机的额定扭矩
#define TURN_PID_MAX_IOUT 0.0f

/**********************offset parameters*******************/
#define X0_OFFSET (0.0f)    // 目标theta偏移量
#define X1_OFFSET (0.0f)    // 目标theta_dot偏移量
#define X2_OFFSET (0.0f)    // 目标x偏移量
#define X3_OFFSET (0.0f)    // 目标x_dot偏移量
#define X4_OFFSET (0.0f)    // 目标phi偏移量
#define X5_OFFSET (0.0f)    // 目标phi_dot偏移量

/**************************structural***********************/
typedef enum 
{
    CHASSIS_OFF,        // 底盘关闭
    CHASSIS_SAFE,       // 底盘无力，所有控制量置0
    CHASSIS_STAND_UP,   // 底盘起立，从倒地状态到站立状态的中间过程
    CHASSIS_FLOATING,   // 底盘悬空状态
    CHASSIS_CRASHING    // 底盘接地状态，进行缓冲
} ChassisMode_e;

typedef struct
{
    Joint_Motor_t joint_motor[4];       // 0，1 为右腿
    DJIMotorInstance *wheel_motor[2];    // 0 为右腿

    ChassisMode_e chassis_mode;

    struct state
    {   // x 和 v的参数
        float v_set;    //期望速度
        float x_set;    //期望位置
        float v_filter; //滤波后的车体速度，单位是m/s
        float x_filter; //滤波后的车体位置，单位是m

    } state;

    float turn_set;     //期望yaw轴弧度
	float roll_set;	    //期望roll轴弧度

    float phi_set;
	float theta_set;
    float leg_set;      //期望腿长
	float last_leg_set;

    float myPithR;
	float myPithGyroR;
	float myPithL;
	float myPithGyroL;
    float total_yaw;
    float roll;

    float turn_T;       //yaw补偿
    float roll_T;       //roll补偿
    float leg_tp;       //防劈叉补偿

    float theta_err;    //两腿夹角误差

    struct flag
    {
        uint8_t start_flag;     //启动标志
        uint8_t right_flag;     //右腿离地检测标志
        uint8_t left_flag;      //左腿离地检测标志
        uint8_t recover_flag;   //倒地自起完成标志
    } flag;

} chassis_t;

// LQR 状态向量
typedef struct 
{
    /* data */
    float theta;
    float theta_dot;
    float x;
    float x_dot;
    float phi;
    float phi_dot;
} LegState_t;

typedef struct 
{
    /* data */
    uint32_t last_time;     // (ms)上一次更新时间
    float duration;         // (ms)任务周期
} Period_t;

#endif // !CHASSIS_DEF_H