/**
 * @file chassis.c
 * @author your name (you@domain.com)
 * @brief 两腿共同的任务函数
 * @version 0.1
 * @date 2026-01-31
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "chassis_def.h"
#include "kinematics.h"
#include "ins_task.h"
#include "pid.h"

chassis_t chassis_move;
extern Leg_t legR;
extern Leg_t legL;

// PidTypeDef Turn_Pid;    //转向pd
// PidTypeDef Tp_Pid;      //防劈叉补偿pd
// PidTypeDef Roll_Pid;    //横滚角补偿pd

// /**
//  * @brief 补偿初始化：roll补偿、防劈叉补偿、偏航角补偿
//  * 
//  * @param roll 
//  * @param Tp 
//  * @param turn 
//  */
// void Pensation_Init(PidTypeDef *roll,PidTypeDef *Tp,PidTypeDef *turn)
// {
//     const static float roll_pid[3] = {ROLL_PID_KP, ROLL_PID_KI,ROLL_PID_KD};
// 	const static float tp_pid[3] = {TP_PID_KP, TP_PID_KI, TP_PID_KD};
// 	const static float turn_pid[3] = {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};
	
// 	PID_init(roll, PID_POSITION, roll_pid, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT);
// 	PID_init(Tp, PID_POSITION, tp_pid, TP_PID_MAX_OUT,TP_PID_MAX_IOUT);
// 	PID_init(turn, PID_POSITION, turn_pid, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);
// }

// /**
//  * @brief 转向控制
//  * 
//  */
// void Turn_Loop(void)
// {
//     float L_d0 = legR.rod.L0 - legL.rod.L0;
//     float diff = DeviationCalc(L_d0, chassis_move.body.roll, chassis_move.reference.roll);
//     float delta_L0 = 0.0f;

//     CoordinateLength(&legL.rod.L0, &legR.rod.L0, diff, delta_L0);

//     PID_Calc(&Turn_Pid, chassis_move.body.yaw_dot, chassis_move.reference.wz);
//     legL.rod.T += Turn_Pid.out;
//     legR.rod.T -= Turn_Pid.out;
// }