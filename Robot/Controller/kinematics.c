/**
 * @file kinematics.c
 * @author Light
 * 
 * @brief 主要用于运动学解算与雅可比矩阵的计算
 * @version 0.1
 * @date 2025-11-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "kinematics.h"
#include "ins_task.h"
#include "chassis_def.h"

// 三项式拟合系数
float a11[4];
float a12[4];
float a13[4];
float a14[4];
float a15[4];
float a16[4];
float a21[4];
float a22[4];
float a23[4];
float a24[4];
float a25[4];
float a26[4];

/**
 * @brief 线性化处理K矩
 * 
 * @param k 反馈矩阵K
 * @param length 腿长
 */
void Calc_LQR_K(float k[2][6], float length, bool flag)
{
    float t1 = length;
    float t2 = length*length;
    float t3 = length*length*length;

    k[0][0] = a11[0]*t3 + a11[1]*t2 + a11[2]*t1 + a11[3];
    k[0][1] = a12[0]*t3 + a12[1]*t2 + a12[2]*t1 + a12[3];
    k[0][2] = a13[0]*t3 + a13[1]*t2 + a13[2]*t1 + a13[3];
    k[0][3] = a14[0]*t3 + a14[1]*t2 + a14[2]*t1 + a14[3];
    k[0][4] = a15[0]*t3 + a15[1]*t2 + a15[2]*t1 + a15[3];
    k[0][5] = a16[0]*t3 + a16[1]*t2 + a16[2]*t1 + a16[3];
    k[1][0] = a21[0]*t3 + a21[1]*t2 + a21[2]*t1 + a21[3];
    k[1][1] = a22[0]*t3 + a22[1]*t2 + a22[2]*t1 + a22[3];
    k[1][2] = a23[0]*t3 + a23[1]*t2 + a23[2]*t1 + a23[3];
    k[1][3] = a24[0]*t3 + a24[1]*t2 + a24[2]*t1 + a24[3];
    k[1][4] = a25[0]*t3 + a25[1]*t2 + a25[2]*t1 + a25[3];
    k[1][5] = a26[0]*t3 + a26[1]*t2 + a26[2]*t1 + a26[3];

    //离地判断
    if (flag)
    {
        /* code */
        k[0][0] = 0;
        k[0][1] = 0;
        k[0][2] = 0;
        k[0][3] = 0;
        k[0][4] = 0;
        k[0][5] = 0;
        k[1][0] = 0;
        k[1][1] = 0;
        k[1][2] = 0;
        k[1][3] = 0;
        k[1][4] = 0;
        k[1][5] = 0;
    }
    
}

float constrainValue(float value, float min, float max);
float safe_sqrt(float x);
float safe_div(float num, float denom);

/**
 * @brief 正运动学解算，求解足端坐标
 * 
 * @param leg 
 * @param excessive 
 * @param ins 
 * @param dt 
 */
void ForwardKinematics(Leg_t* leg,Excessive_t* excessive, INS_t* ins, float dt)
{
    float XD, YD, XB, YB;
    float lBD;
    float A0, B0, C0;

    static float PitchR = 0.0f;
    static float PitchGyroR = 0.0f;
    PitchR = ins->Pitch;
    PitchGyroR = ins->Gyro[0];

    XD = LEG4*arm_cos_f32(leg->joint.Phi4);
    YD = LEG4*arm_sin_f32(leg->joint.Phi4);
    XB = LEG1*arm_cos_f32(leg->joint.Phi1);
    YB = LEG1*arm_sin_f32(leg->joint.Phi1);

    lBD = safe_sqrt((XD - XB)*(XD - XB) + (YD -YB)*(YD - YB));

    A0 = 2*LEG2*(XD - XB);
    B0 = 2*LEG2*(YD - YB);
    C0 = LEG2*LEG2 + lBD * lBD - LEG3*LEG3;

    excessive->Phi2 = 2*atan2f(B0 + safe_sqrt(A0*A0 + B0*B0 - C0*C0), A0 + C0);
    excessive->Phi3 = atan2f(YB - YD + LEG2*arm_sin_f32(excessive->Phi2), XB - XD + LEG2*arm_cos_f32(excessive->Phi2));

    leg->rod.xC = LEG1*arm_cos_f32(leg->joint.Phi1) + LEG2*arm_cos_f32(excessive->Phi2);
    leg->rod.yC = LEG1*arm_sin_f32(leg->joint.Phi1) + LEG2*arm_sin_f32(excessive->Phi2);

    leg->rod.L0 = safe_sqrt(leg->rod.xC*leg->rod.xC + leg->rod.yC*leg->rod.yC);
    leg->rod.phi0 = atan2f(leg->rod.yC, leg->rod.xC);

    //LQR控制器参数
    leg->rod.d_phi0 = (leg->rod.phi0 - leg->rod.last_phi0) / dt;
    
    leg->rod.last_phi0 = leg->rod.phi0;

    //theta
    leg->rod.theta = PI/2 - leg->rod.phi0 - PitchR; //状态量1
    leg->rod.d_theta = - leg->rod.d_phi0 - PitchGyroR; //状态量2
    leg->rod.dd_theta = (leg->rod.d_theta - leg->rod.last_d_theta) / dt;
    leg->rod.last_d_theta = leg->rod.d_theta;

    //L0
    leg->rod.d_L0 = (leg->rod.L0 - leg->rod.last_L0) / dt;
    leg->rod.dd_L0 = (leg->rod.d_L0 - leg->rod.last_d_L0) / dt;

    leg->rod.last_L0 = leg->rod.L0;
    leg->rod.last_d_L0 = leg->rod.d_L0;

}

/**
 * @brief 逆运动学解算，根据足端坐标求解关节角
 * 
 * @param leg 
 * @param excessive 
 */
void InverseKinematics(Leg_t* leg,Excessive_t* excessive)
{
    float A1, B1, C1, D1;
    float A4, B4, C4, D4, E4, F4;
    //限幅
    // Leg->rod.yC = constrainValue(Leg->rod.yC, 0.05, 0.35);

    //过度变量
    A1 = LEG1+leg->rod.xC;
    B1 = LEG1*LEG1 - leg->rod.xC*leg->rod.xC;
    C1 = LEG2*LEG2 - leg->rod.yC*leg->rod.yC;
    D1 = LEG2*LEG2 + leg->rod.yC*leg->rod.yC;

    A4 = LEG3 + LEG4;
    B4 = -leg->rod.xC;
    C4 = LEG3 - LEG4;
    D4 = leg->rod.xC + LEG4;
    E4 = A4*A4 - B4*B4 - leg->rod.yC*leg->rod.yC;
    F4 = B4*B4 - C4*C4 + leg->rod.yC*leg->rod.yC;

    leg->joint.Phi1 = 2*atan2f(2*LEG1*leg->rod.yC+sqrt(2*LEG1*LEG1*D1+2*leg->rod.xC*leg->rod.xC*C1-B1*B1-C1*C1),A1*A1-C1);
    leg->joint.Phi4 = 2*atan2f(2*LEG4*leg->rod.yC-sqrt(E4*F4),D4*D4+leg->rod.yC*leg->rod.yC-LEG3*LEG3);

}

/**
 * @brief 雅可比矩阵求解
 * 
 * @param leg 
 * @param excessive 
 */
void JacobianMatrix(Leg_t* leg,Excessive_t* excessive)
{
    leg->j11 = (LEG1*arm_sin_f32(leg->rod.phi0 - excessive->Phi3)*arm_sin_f32(leg->joint.Phi1 - excessive->Phi2))/arm_sin_f32(excessive->Phi3 - excessive->Phi2);
    leg->j12 = (LEG1*arm_cos_f32(leg->rod.phi0 - excessive->Phi3)*arm_sin_f32(leg->joint.Phi1 - excessive->Phi2))/(leg->rod.L0*arm_sin_f32(excessive->Phi3 - excessive->Phi2));
    leg->j21 = (LEG4*arm_sin_f32(leg->rod.phi0 - excessive->Phi2)*arm_sin_f32(excessive->Phi3 - leg->joint.Phi4))/arm_sin_f32(excessive->Phi3 - excessive->Phi2);
    leg->j22 = (LEG4*arm_cos_f32(leg->rod.phi0 - excessive->Phi2)*arm_sin_f32(excessive->Phi3 - leg->joint.Phi4))/(leg->rod.L0*arm_sin_f32(excessive->Phi3 - excessive->Phi2));

    leg->joint.T1 = leg->j11*leg->rod.F0 + leg->j12*leg->rod.Tp;
    leg->joint.T2 = leg->j21*leg->rod.F0 + leg->j22*leg->rod.Tp;
}


/**
 * @brief 离地检测
 * 
 * @param leg 
 * @param overall 
 */
uint8_t GroundDetect(Leg_t* leg, Period_t* period, INS_t* ins)
{
    float P;                                    //驱动轮对摆杆的力的竖直分量--用于判断离地检测
    float dd_z_M = ins->MotionAccel_n[2];       //机体竖直方向位移--二阶导--加速度
    float dd_z_w;                               //轮子竖直方向位移--二阶导
    
    dd_z_w = dd_z_M
            - leg->rod.dd_L0 * arm_cos_f32(leg->rod.theta)
            + 2 * leg->rod.d_L0 * leg->rod.d_theta * arm_sin_f32(leg->rod.theta)
            + leg->rod.L0 * leg->rod.dd_theta * arm_sin_f32(leg->rod.theta)
            + leg->rod.L0 * leg->rod.d_theta * leg->rod.d_theta * arm_cos_f32(leg->rod.theta);

    P = leg->rod.F * cosf(leg->rod.theta) + leg->rod.Tp * sinf(leg->rod.theta) / leg->rod.L0;
    leg->FN = P + WHEEL_MASS * (dd_z_w + GRAVITY);

    if (leg->FN < TAKE_OFF_FN_THRESHOLD)
    {
        /* code */
        leg->touch_time = 0;
        leg->take_off_time += period->duration;
        return 1;
    }
    else
    {
        leg->touch_time += period->duration;
        leg->take_off_time = 0;
        return 0;
    }
    
}

/**
 * @brief 双腿腿长协调控制，维持腿长目标在范围内，同时尽可能达到两腿目标差值
 * 
 * @param LengthL 
 * @param LengthR 
 * @param diff 
 * @param add 差值补偿
 */
void CoordinateLength(float *LengthL, float *LengthR, float diff, float add)
{
    *LengthL = *LengthL + diff * 0.5f + add;
    *LengthR = *LengthR - diff * 0.5f - add;

    // (保证指向不同的地址)
    float *short_leg = *LengthL < *LengthR ? LengthL : LengthR;
    float *long_leg = *LengthL < *LengthR ? LengthR : LengthL;

    float temp = 0;
    temp = MIN_LEG_LENGTH - *short_leg;
    if (temp > 0)
    {
        /* code */
        *short_leg += temp;
        *long_leg += temp;
    }
    if (*long_leg > MAX_LEG_LENGTH)
    {
        /* code */
        *long_leg = MAX_LEG_LENGTH;
    }    
}

/**
 * @brief 通过当前底盘姿态和目标roll角计算两腿长度期望差值
 * 
 * @param diff 左右腿差值
 * @param real 当前roll角
 * @param target 目标roll角
 * @return float 
 */
float DeviationCalc(float diff, float real, float target)
{
    return WHEEL_DISTANCE * tanf(target) - 
           cosf(real) / cosf(target) * (WHEEL_DISTANCE * tanf(real) - diff);
}

// 限幅
float constrainValue(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// 防止负数输入
float safe_sqrt(float x)
{
    return x >= 0.0f ? sqrtf(x) : 0.0f;
}

// 防止除以零
float safe_div(float num, float denom) 
{
    if (fabs(denom) < 1e-6f) denom = (denom < 0 ? -1e-6f : 1e-6f); // 防止分母0
    return num / denom;
}
