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
#include "math.h"

// #define MPC

// LQR三项式拟合系数
float a11[4] = {-4.013500686481230e+02,	4.448168023846569e+02,	-1.962028941807348e+02,	-11.178950289533221};
float a12[4] = {-3.963142203218219,	5.974341010796577,	-10.345757837820608,	-0.379472085007600};
float a13[4] = {13.625473926660618,	-9.910743828794351,	1.793639106438964,	-3.076186092182440};
float a14[4] = {43.357855170296183,	-38.718990885436121,	12.121443823004514,	-6.734153060555737};
float a15[4] = {-79.227077551723511,	1.287032367105078e+02,	-85.437751505781875,	28.017162235291160};
float a16[4] = {-18.490237588368892,	20.740423352128705,	-9.801029890882928,	2.835686870776124};
float a21[4] = {2.217231851839134e+02,	24.255415629715962,	-1.891884586751662e+02,	1.056601348980694e+02};
float a22[4] = {37.184934500524164,	-33.780935683063007,	7.805940995831365,	2.597931486254918};
float a23[4] = {1.225184080264510e+02,	-95.215592587845904,	18.153038919986450,	2.313543146540224};
float a24[4] = {2.016931230213475e+02,	-1.488737458350358e+02,	23.819766946235021,	5.029722415864938};
float a25[4] = {6.295080995420572e+02,	-7.090475420156093e+02,	3.025497378564039e+02,	19.964644390910113};
float a26[4] = {21.917896839010758,	-30.787189361269718,	16.341582536481724,	1.319006761556430};

#ifdef MPC
// MPC拟合系数
float b11[4];
float b12[4];
float b13[4];
float b14[4];
float b15[4];
float b16[4];
float b21[4];
float b22[4];
float b23[4];
float b24[4];
float b25[4];
float b26[4];
#endif

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

#ifdef MPC
    float u[2][6];  // 待后期使用放入形参中

    u[0][0] = b11[0]*expf(b11[1]*length)+b11[2]*expf(b11[3]*length);
    u[0][1] = b12[0]*expf(b12[1]*length)+b12[2]*expf(b12[3]*length);
    u[0][2] = b13[0]*expf(b13[1]*length)+b13[2]*expf(b13[3]*length);
    u[0][3] = b14[0]*expf(b14[1]*length)+b14[2]*expf(b14[3]*length);
    u[0][4] = b15[0]*expf(b15[1]*length)+b15[2]*expf(b15[3]*length);
    u[0][5] = b16[0]*expf(b16[1]*length)+b16[2]*expf(b16[3]*length);
    u[1][0] = b21[0]*expf(b21[1]*length)+b21[2]*expf(b21[3]*length);
    u[1][1] = b22[0]*expf(b22[1]*length)+b22[2]*expf(b22[3]*length);
    u[1][2] = b23[0]*expf(b23[1]*length)+b23[2]*expf(b23[3]*length);
    u[1][3] = b24[0]*expf(b24[1]*length)+b24[2]*expf(b24[3]*length);
    u[1][4] = b25[0]*expf(b25[1]*length)+b25[2]*expf(b25[3]*length);
    u[1][5] = b26[0]*expf(b26[1]*length)+b26[2]*expf(b26[3]*length);
#endif

    //离地判断
    // if (flag)
    // {
    //     /* code */
    //     k[0][0] = 0;
    //     k[0][1] = 0;
    //     k[0][2] = 0;
    //     k[0][3] = 0;
    //     k[0][4] = 0;
    //     k[0][5] = 0;
    //     k[1][0] = 0;
    //     k[1][1] = 0;
    //     k[1][2] = 0;
    //     k[1][3] = 0;
    //     k[1][4] = 0;
    //     k[1][5] = 0;
    // }
    
}

float safe_sqrt(float x);
float safe_div(float num, float denom);

/**
 * @brief 正运动学解算，求解足端坐标
 * 
 * @param leg 
 * @param excessive 
 * @param ins 
 */
void ForwardKinematics(Leg_t* leg,Excessive_t* excessive)
{
    float XD, YD, XB, YB;
    float lBD;
    float A0, B0, C0;

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
}

/**
 * @brief 逆运动学解算，根据足端坐标求解关节角
 * 
 * @param leg 
 * @param excessive 
 */
void InverseKinematics(chassis_t* chassis, Leg_t* leg)
{
    float A1, B1, C1, D1;
    float A4, B4, C4, D4, E4, F4;

    leg->rod.xC = chassis->leg_set * arm_cos_f32(leg->rod.phi0);
    leg->rod.yC = chassis->leg_set * arm_sin_f32(leg->rod.phi0);


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

    // leg->rod.d_L0 = leg->j11*leg->joint.d_Phi1 + leg->j12*leg->joint.d_Phi4;
    // leg->rod.d_phi0 = leg->j21*leg->joint.d_Phi1 + leg->j22*leg->joint.d_Phi4;
}

/**
 * @brief 更新大地坐标系下的加速度
 * 
 */
void Acceleration_Updata(chassis_t chassis, INS_t* ins)
{
    float ax = ins->Accel[X_AXIS];
    float ay = ins->Accel[Y_AXIS];
    float az = ins->Accel[Z_AXIS];

    float sin_pitch,cos_pitch,sin_roll,cos_roll,sin_yaw,cos_yaw;
    
    sin_roll = arm_sin_f32(ins->Roll);
    cos_roll = arm_cos_f32(ins->Roll);
    sin_pitch = arm_sin_f32(ins->Pitch);
    cos_pitch = arm_cos_f32(ins->Pitch);
    sin_yaw = arm_sin_f32(ins->Yaw);
    cos_yaw = arm_cos_f32(ins->Yaw);

    chassis.body.gx = GRAVITY * sin_pitch;
    chassis.body.gy = -GRAVITY * sin_roll * cos_pitch;
    chassis.body.gz = -GRAVITY * cos_roll * cos_pitch;

    chassis.body.x_accel = ax + chassis.body.gx;
    chassis.body.y_accel = ay + chassis.body.gy;
    chassis.body.z_accel = az + chassis.body.gz;

    // 计算旋转矩阵
    float R[3][3] = 
    {
        {cos_pitch * cos_yaw, sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw, cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw},
        {cos_pitch * sin_yaw, sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw, cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw},
        {-sin_pitch         , sin_roll * cos_pitch                               , cos_roll * cos_pitch                               }
    };

    // 更新大地坐标系下的加速度
    chassis.world.x_accel = R[0][0]*ax + R[0][1]*ay + R[0][2]*az;
    chassis.world.y_accel = R[1][0]*ax + R[1][1]*ay + R[1][2]*az;
    chassis.world.z_accel = R[2][0]*ax + R[2][1]*ay + R[2][2]*az-GRAVITY;

}

/**
 * @brief 离地检测
 * 
 * @param leg 
 * @param overall 
 */
uint8_t GroundDetect(chassis_t* chassis, Leg_t* leg, Period_t* period)
{
    float P;                                    //驱动轮对摆杆的力的竖直分量--用于判断离地检测
    float dd_z_M = chassis->world.z_accel;       //机体竖直方向位移--二阶导--加速度
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
 * @brief 翻滚角补偿，用于单边桥等特殊地形
 * 
 * @param LengthL 实际左腿长度
 * @param LengthR 
 * @param refL 期望左腿长度
 * @param refR 期望右腿长度
 * @param diff 左右腿长度差值
 * @param theta 翻滚角
 */
void RollCompensation(float *LengthL,float *LengthR,float *refL,float *refR,float *diff, float theta)
{
    float BD, AF, ED, EF;
    float BC, FD;

    BD = (*LengthR - *LengthL) * cosf(theta);
    AF = 2 * WHEEL_DISTANCE * sinf(theta);
    ED = (*LengthR - *LengthL) * sinf(theta);
    EF = 2 * WHEEL_DISTANCE * cosf(theta);

    // theta = -theta; //翻转角取反
    BC = BD - AF;
    FD = ED + EF;

    float slope_tan = BC / FD;
    float hight = *LengthR * cosf(theta) + WHEEL_DISTANCE * sinf(theta);
    *refL = hight;
    *refR = hight - 2 * WHEEL_DISTANCE * slope_tan;
    *diff = 2 * WHEEL_DISTANCE * slope_tan;
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
