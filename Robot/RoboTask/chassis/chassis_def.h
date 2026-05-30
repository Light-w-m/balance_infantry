/**
 * @file chassis_def.h
 * @author Light
 * @brief 用于包含各种宏定义与全局结构体
 * @version 0.1
 * @date 2025-12-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef CHASSIS_DEF_H
#define CHASSIS_DEF_H

#include "stdbool.h"
#include "stdint.h"
#include "djimotor.h"
#include "dmmotor.h"

// 限幅
#define SATURATE(p, min, max)           \
    do {                                \
        if (*(p) < (min))               \
            *(p) = (min);               \
        else if (*(p) > (max))          \
            *(p) = (max);               \
    } while (0)

/**********************control parameters*******************/
//内部参数配置，宏定义判断是否启动

//两种控制模式不可同时进行
#define ControlOperation    //正常控制模式--正解
// #define ControlDebug        //调试模式--逆解

/**********************direction parameters*******************/
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

/**********************physical parameters*******************/
#define GRAVITY 9.791f         // 重力加速度--福建

#define BODY_MASS       5.0f       // 载体重量
#define WHEEL_MASS      0.3f       // 轮重量
#define WHEEL_RADIUS    0.06f       // 轮子半径
#define WHEEL_DISTANCE  0.435f       // 轮子间距

#define OFFSET          0.0f        //关节补偿系数

#define TOR_COEFFICIENT 0.3f        // 电机电流扭矩系数
#define REDUCTION_RATIO 15.7f        // 电机减速比
#define EFFICIENCY      0.1f        // 传动效率 0.075
#define CURRENT_LIMIT   20.0f       // 电机最大允许电流
#define CURRENT_MAPPING 16384.0f  // 电机电流映射关系
#define FINAL_COEFFICIENT 3000 //最终电流映射系数
// #define FINAL_COEFFICIENT (CURRENT_MAPPING/(CURRENT_LIMIT*TOR_COEFFICIENT*REDUCTION_RATIO*EFFICIENCY)) //最终电流映射系数
/**********************chassis parameters*******************/
// 支持力阈值，当支持力小于这个值时认为离地
#define TAKE_OFF_FN_THRESHOLD (3.0f)
// 触地状态切换时间阈值，当时间接触或离地时间超过这个值时切换触地状态
#define TAKE_OFF_THRESHOLD (20)    // 离地判断时间阈值 (ms)
#define TOUCH_GROUND_THRESHOLD (30) // 触地恢复时间阈值 (ms)

#define CHASSIS_TIME 1         //延迟时间

/****************amplitude limiting parameters*************/
// 腿长
#define MAX_LEG_LENGTH 0.32f
#define MIN_LEG_LENGTH 0.15f
#define MAX_LEG_VEL 0.2f    //最大腿长变化速度 m/s
// #define CENTER (MAX_LEG_LENGTH + MIN_LEG_LENGTH)/2
// #define RANGE 0.085         

// 扭矩
#define MAX_TORQUE 8.0f    //关机最大输出扭矩 N·m

// 速度
#define MAX_CHASSIS_VEL 0.35f        //最大底盘速度 m/s

/***********************length parameters****************/
#define LEG1 0.21f
#define LEG2 0.25f
#define LEG3 0.25f
#define LEG4 0.21f

#define LEG_RISE_LENGTH_SET 0.20f   //完成倒地自起时腿长
#define LEG_DEFAULT 0.15f   //默认腿长

/***********************pid parameters*******************/
#define LEG_PID_KP  800.0f
#define LEG_PID_KI  0.0f
#define LEG_PID_KD  16000.0f
#define LEG_PID_MAX_OUT  100.0f //90ţ
#define LEG_PID_MAX_IOUT 0.0f

#define Pitch_ki 0.1f

#define PITCH_V_PID_KP 1.2f
#define PITCH_V_PID_KI 0.0f 
#define PITCH_V_PID_KD 0.5f
#define PITCH_V_PID_MAX_OUT  10.0f
#define PITCH_V_PID_MAX_IOUT 0.0f

#define PITCH_PID_KP 8.0f
#define PITCH_PID_KI 0.0f 
#define PITCH_PID_KD 0.0f
#define PITCH_PID_MAX_OUT  5.0f
#define PITCH_PID_MAX_IOUT 0.0f

#define ROLL_PID_KP 150.0f
#define ROLL_PID_KI 0.0f 
#define ROLL_PID_KD 10.0f
#define ROLL_PID_MAX_OUT  100.0f
#define ROLL_PID_MAX_IOUT 0.0f

#define TP_PID_KP 40.0f
#define TP_PID_KI 0.0f 
#define TP_PID_KD 1.5f
#define TP_PID_MAX_OUT  2.0f
#define TP_PID_MAX_IOUT 0.0f

#define kp_Yaw 0.0f

#define TURN_PID_KP 5.0f
#define TURN_PID_KI 0.0f 
#define TURN_PID_KD 1.2f
#define TURN_PID_MAX_OUT  3.0f//轮毂电机的额定扭矩
#define TURN_PID_MAX_IOUT 0.0f

#define WHEEL_PID_KP 0.1f
#define WHEEL_PID_KI 0.0f
#define WHEEL_PID_KD 0.05f
#define WHEEL_PID_MAX_OUT  2.0f//电机最大允许电流
#define WHEEL_PID_MAX_IOUT 0.0f

/**********************offset parameters*******************/
#define THETA_OFFSET (-0.0f)    //腿角偏移量

#define X0_OFFSET (0.0f)    // 目标theta偏移量
#define X1_OFFSET (0.0f)    // 目标theta_dot偏移量
// #define X2_OFFSET(x) (-0.36f + ((x) - 0.15f) * 0.882352941f)    // 目标x偏移量
// 0.15 -0.38       0.32 -0.23
#define X2_OFFSET (-0.45f)    // 目标x偏移量
// #define X2_OFFSET (-0.0f)    // 目标x偏移量
#define X3_OFFSET (0.0f)    // 目标x_dot偏移量
#define X4_OFFSET (0.0f)    // 目标phi偏移量
#define X5_OFFSET (0.0f)    // 目标phi_dot偏移量

/**********************X_integral parameters*******************/
#define X_INTEGRAL_KI   0.03f    // 位移积分增益
#define X_INTEGRAL_LIMIT 0.15f   // 积分限幅
  
/**********************Step definitions*******************/
#define NORMAL_STEP        0  // 正常状态
#define JUMP_STEP_SQUST    1  // 跳跃状态——蹲下
#define JUMP_STEP_JUMP     2  // 跳跃状态——跳跃
#define JUMP_STEP_RECOVERY 3  // 跳跃状态——收腿

#define MAX_STEP_TIME           5000  // 最大步骤时间

#define NORMAL_STEP_TIME        0  // 正常状态
#define JUMP_STEP_TIME_SQUST    50  // 跳跃状态——蹲下
#define JUMP_STEP_TIME_JUMP     50  // 跳跃状态——跳跃
#define JUMP_STEP_TIME_RECOVERY 200  // 跳跃状态——收腿

/**************************structural***********************/
typedef enum 
{
    CHASSIS_OFF,        // 底盘关闭
    CHASSIS_SAFE,       // 底盘无力，所有控制量置0
    CHASSIS_STAND_UP,   // 底盘起立，从倒地状态到站立状态的中间过程
    CHASSIS_FLOATING,   // 底盘悬空状态
    CHASSIS_CRASHING    // 底盘接地状态，进行缓冲
} ChassisMode_e;

typedef union
{
    struct 
    {
        /* data */
        float x;
        float y;
        float z;
    };
    float data[3];
}xyz_t;

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
        float x_integral; // 位移积分，消除稳态误差
        float x_error;  // 位移误差
    } state;

    struct body
    {
        float x_accel;  // 机体坐标系下x轴加速度
        float y_accel;  // 机体坐标系下y轴加速度
        float z_accel;  // 机体坐标系下z轴加速度

        float gx, gy, gz;  //重力加速度在机体坐标系下的分量，用于消除重力加速度对加速度计的影响

        float pitch;
        float pitch_dot;
        float roll;
        float roll_dot;
        float yaw;
        float yaw_dot;
        float yaw_total; // 累计的航向角,用于底盘控制
    } body;

    struct world
    {
        float x_accel;  // 大地坐标系下x轴加速度
        float y_accel;  // 大地坐标系下y轴加速度
        float z_accel;  // 大地坐标系下z轴加速度
    } world;

    struct reference
    {
        float pitch;
        float pitch_dot;
        float roll;
        float roll_dot;
        float yaw;
        float yaw_dot;
        
        float vx;  // (m/s) x方向速度
        float vy;  // (m/s) y方向速度
        float wz;  // (rad/s) 旋转速度
    } reference;

    struct bias
    {
	    float theta_err;
        float myPitch;  //自适应重心误差补偿
        float error;
    } bias;

    float turn_set;     //期望yaw轴弧度
	float roll_set;	    //期望roll轴弧度

    float phi_set;
    float leg_set;      //期望腿长

    float myPithR;
	float myPithGyroR;
	float myPithL;
	float myPithGyroL;
    // float total_yaw;

    float pitch_T;      //俯仰角补偿
    float turn_T;       //yaw补偿
    float roll_T;       //roll补偿
    float leg_tp;       //防劈叉补偿

    // float theta_err;    //两腿夹角误差

    struct step
    {
        uint32_t jump_step_time;
        int8_t jump_step;
    } step;

    struct flag
    {
        uint8_t start_flag;     //启动标志
        uint8_t right_flag;     //右腿离地检测标志
        uint8_t left_flag;      //左腿离地检测标志
        uint8_t recover_flag;   //倒地自起启动标志
        bool is_take_off;       //离地标志
        uint8_t jump_flag;      //跳跃标志
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
    float us_r;             // 右腿倒立自起时间
    float us_l;             // 左腿倒立自起时间
} Period_t;

/***********************functions***********************/
static inline float WrapToPi(float x)
{
    while (x >  PI) x -= 2.0f * PI;
    while (x < -PI) x += 2.0f * PI;
    return x;
}

// 保证电机逆解走最短路径
static inline float ShortestAngle(float target, float *last_set)
{
    float out = *last_set + WrapToPi(target - *last_set);
    *last_set = out;
    return out;
}

static inline float FixedDirAngle(float target, float *last, float dir)
{
    float delta = target - *last;

    // 去掉 ±π 跳变影响（关键）
    if (delta >  PI) delta -= 2.0f * PI;
    if (delta < -PI) delta += 2.0f * PI;

    // 强制方向（核心）
    if (delta * dir < 0)
        delta += dir * 2.0f * PI;

    float out = *last + delta;
    *last = out;
    return out;
}

#endif // !CHASSIS_DEF_H