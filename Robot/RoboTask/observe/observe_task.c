/**
 * @file observe_task.c
 * @author your name (you@domain.com)
 * @brief 用于对机体速度估计
 * @version 0.1
 * @date 2025-12-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "observe_task.h"
#include "kalman_filter.h"
#include "kinematics.h"
#include "chassisR_task.h"
#include "chassis_def.h"
#include "user_lib.h"
#include "cmsis_os.h"

KalmanFilter_t vaEstimateKF;	   // 卡尔曼滤波器结构体

float vaEstimateKF_F[4] = {1.0f, 0.003f, 
                           0.0f, 1.0f};	   // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {1.0f, 0.0f, 
                           0.0f, 1.0f};    // Q矩阵初始值

float vaEstimateKF_R[4] = {200.0f, 0.0f, 
                            0.0f,  200.0f}; 	
														
float vaEstimateKF_K[4];
													 
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// 设置矩阵H为常量

extern INS_t INS;
extern chassis_t chassis_move;
extern Leg_t legR;
extern Leg_t legL;

Observe_Data_t observe_data;
float vel_acc[2]; 
uint32_t OBSERVE_TIME = 3;//任务周期是3ms	


void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// 状态向量2维 没有控制量 测量向量2维
	
		memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));
}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{   	
	 memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
   memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
	
    //卡尔曼滤波器测量值更新
    EstimateKF->MeasuredVector[0] =	vel;//测量速度
    EstimateKF->MeasuredVector[1] = acc;//测量加速度
    		
    //卡尔曼滤波器更新函数
    Kalman_Filter_Update(EstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 2; i++)
    {
      vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}

void Observe_Task(void)
{
  while (INS.ins_flag == 0)
  {
    /* code */
    osDelay(1);
  }
  // static float wr,wl = 0.0f;    //驱动轮转子相对大地角速度，这里定义的是顺时针为正（方向待定）
  // static float vrb,vlb = 0.0f;  //机体b系的速度
  // static float aver_v = 0.0f;
    
  xvEstimateKF_Init(&vaEstimateKF);

  while (1)
  {
    /* code */
    observe_data.wr = -angle_to_radian(chassis_move.wheel_motor[0]->measure.speed_aps)-INS.Gyro[Y_AXIS]-legR.rod.d_phi0;
    observe_data.wl = -angle_to_radian(chassis_move.wheel_motor[1]->measure.speed_aps)+INS.Gyro[Y_AXIS]-legR.rod.d_phi0;

    // 公式意义：角速度 x 半径 + 角速度 x 长度 x cos(角度) + 长度变化量 x sin(角度)
    observe_data.vrb = observe_data.wr*WHEEL_RADIUS + legR.rod.L0*legR.rod.d_theta*arm_cos_f32(legR.rod.theta) + legR.rod.d_L0*arm_sin_f32(legR.rod.theta);
    observe_data.vlb = observe_data.wl*WHEEL_RADIUS + legL.rod.L0*legL.rod.d_theta*arm_cos_f32(legL.rod.theta) + legL.rod.d_L0*arm_sin_f32(legL.rod.theta);

    // 因规定顺时针为正，所以右轮为正，左轮为负，因此前进速度为差值，角速度为和值
    observe_data.forward_v = (observe_data.vrb - observe_data.vlb)/2.0f;
    observe_data.angular_v = (observe_data.vrb + observe_data.vlb)/WHEEL_DISTANCE;
    
    xvEstimateKF_Update(&vaEstimateKF, -chassis_move.body.x_accel, observe_data.forward_v);

    // 原地自转时，v_filter和x_filter应该都为0
    chassis_move.state.v_filter = vel_acc[0];
    chassis_move.state.x_filter = chassis_move.state.x_filter + chassis_move.state.v_filter*((float)OBSERVE_TIME/1000.0f);
    
    osDelay(OBSERVE_TIME);
  }
  
}
