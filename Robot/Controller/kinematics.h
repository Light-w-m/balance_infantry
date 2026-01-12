#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include "arm_math.h"
#include "stdint.h"
#include "stdbool.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

typedef struct 
{
    /* data */
    struct rod
    {
        float xC, yC;           //足关节坐标

        float L0;               //杆长
        float d_L0;
        float dd_L0;
        float last_L0;
        float last_d_L0;

        float phi0;             //摆杆与机体水平方向夹角--phi0
        float d_phi0;
        float last_phi0;        //上一次C点角度，用于计算角度phi0的变换率d_phi0

        float theta;            //摆杆与垂直方向夹角--theta
        float d_theta;
        float last_d_theta;
        float dd_theta;

        float F0;               //F0为五连杆机构末端沿腿的推力 
        float F;                //F为髋关节产生的沿摆杆的推力 
        float Tp;               //髋关节等效力矩
        float T;                //轮电机等效力矩

    } rod;  //杆参数--一阶倒立摆

    struct joint
    {
        float T1, T2;           //髋关节输出扭矩
        float Phi1, Phi4;

        #ifdef ControlDebug
        // 位控调试时使用
        float Phi1_set, Phi4_set;
        #endif   
    } joint;    //关节参数

    struct wheel
    {
        
    } wheel;

    float j11, j12, j21, j22;   //雅可比矩阵
    float FN;                   //支持力
    bool is_take_off;           // 是否离地
    uint32_t take_off_time;     // 离地计时
    uint32_t touch_time;        // 触地计时

}Leg_t;

typedef struct 
{
    /* data */  
    float Phi2;
    float Phi3;

    // float XD, YD;
    // float XB, YB;
    // float lBD;
    // float A0, B0, C0;

    // float A1, B1, C1, D1;
    // float A4, B4, C4, D4, E4, F4;

} Excessive_t;      //中间过度参数--可删，用局部变量代替


void Calc_LQR_K(float k[2][6], float length, bool flag);
void ForwardKinematics(Leg_t* leg,Excessive_t* excessive, INS_t* ins, float dt);
void InverseKinematics(Leg_t* leg,Excessive_t* excessive);
void JacobianMatrix(Leg_t* leg,Excessive_t* excessive);
uint8_t GroundDetect(Leg_t* leg, Period_t* period, INS_t* ins);
void CoordinateLength(float *LengthL, float *LengthR, float diff, float add);
float DeviationCalc(float diff, float real, float target);

#endif // !__KINEMATICS_H