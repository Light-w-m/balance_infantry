/**
 * @file chassisR_task.c
 * @author Light
 * @brief 
 * @version 0.1
 * @date 2025-12-10
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "chassisR_task.h"
#include "chassis_def.h"
#include "kinematics.h"
#include "kalman_filter.h"
#include "pid.h"
#include "djimotor.h"
#include "dmmotor.h"
#include "ins_task.h"

Leg_t legR;
LegState_t legR_state;
Excessive_t excessiveR;
extern chassis_t chassis_move;
// Period_t periods;

extern INS_t INS;

PidTypeDef LegR_pid;    //右腿腿长PID

float LQR_K_R[2][6] = {
    {-8.973116618168689,	-0.673550201762786,	-0.360066358493418,	-0.846214660322890,	8.199359202435174,	0.843488553139154},
    {35.260334676919378,	3.177413507736960,	2.041041348591034,	4.702010005595314,	39.261430584893219,	2.209194672639389}
};
// int8_t TRANSITION_MATRIX[10] = {0};
static float current_prtR = 420;
/**
 * @brief 底盘初始化,包括PID参数初始化，电机初始化
 * 
 * @param chassis 
 * @param pid 
 */
static void ChassisR_Init(chassis_t* chassis, PidTypeDef* length_pid)
{
    // 初始化转移矩阵
    // TRANSITION_MATRIX[NORMAL_STEP] = NORMAL_STEP;
    // TRANSITION_MATRIX[JUMP_STEP_SQUST] = JUMP_STEP_JUMP;
    // TRANSITION_MATRIX[JUMP_STEP_JUMP] = JUMP_STEP_RECOVERY;
    // TRANSITION_MATRIX[JUMP_STEP_RECOVERY] = NORMAL_STEP;

    // 两个轮电机的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_configR = {
        .can_init_config.can_handle = &hcan2,
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = OPEN_LOOP,
            .close_loop_type = OPEN_LOOP,
        },
        .controller_param_init_config = {
        .current_feedforward_ptr = &current_prtR,
        },
        .motor_type = M3508,
    };

    const static float leg_pid_R[3] = {LEG_PID_KP, LEG_PID_KI, LEG_PID_KD}; //P I D

    joint_motor_init(&chassis->joint_motor[0], 3, MIT_MODE);
    joint_motor_init(&chassis->joint_motor[1], 4, MIT_MODE);

    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_configR.can_init_config.tx_id = 2;
    chassis_motor_configR.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis->wheel_motor[0] = DJIMotorInit(&chassis_motor_configR);

    //腿长PID初始化
    PID_init(length_pid, PID_POSITION, leg_pid_R, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);

    //电机使能
    for (uint8_t i = 0; i < 3; i++)
    {
        /* code */
        Enable_Motor_Mode(&hfdcan1, chassis->joint_motor[0].para.id, chassis->joint_motor[0].mode);
        Enable_Motor_Mode(&hfdcan1, chassis->joint_motor[1].para.id, chassis->joint_motor[1].mode);
        DJIMotorEnable(chassis->wheel_motor[0]);
        osDelay(1);
    }
     
}

/**
 * @brief 输入更新
 * 
 * @param chassis 
 * @param ins 
 * @param  
 */
static void ChassisR_Feedback_Update(chassis_t* chassis,Leg_t* leg, INS_t* ins, float dt)
{
    leg->joint.Phi4 = PI/2.0f + chassis->joint_motor[1].para.pos;
    leg->joint.Phi1 = PI/2.0f + chassis->joint_motor[0].para.pos;

    leg->joint.d_Phi4 = chassis->joint_motor[1].para.vel;
    leg->joint.d_Phi1 = chassis->joint_motor[0].para.vel;

    SATURATE(&leg->joint.Phi4, -PI/3.0f, PI/2.0f);
    SATURATE(&leg->joint.Phi1, PI/2.0f, PI*4/3.0f);

    chassis->myPithR = -ins->Pitch;
    chassis->myPithGyroR = -ins->Gyro[Y_AXIS];

    // L0
    leg->rod.d_L0 = leg->j11*leg->joint.d_Phi1 + leg->j12*leg->joint.d_Phi4;
    leg->rod.last_d_L0 = leg->rod.d_L0;
    leg->rod.dd_L0 = (leg->rod.d_L0 - leg->rod.last_d_L0) / dt;

    // phi0
    leg->rod.d_phi0 = leg->j21*leg->joint.d_Phi1 + leg->j22*leg->joint.d_Phi4;

    //theta
    leg->rod.theta = PI/2 - leg->rod.phi0 - chassis->myPithR;
    leg->rod.d_theta = - leg->rod.d_phi0 - chassis->myPithGyroR;
    leg->rod.last_d_theta = leg->rod.d_theta;
    leg->rod.dd_theta = (leg->rod.d_theta - leg->rod.last_d_theta) / dt;

    //倒地自起检测
    if(1)
    {
        // chassis->flag.recover_flag = 0;
    }
}

/**
 * @brief 跳跃控制
 * 
 */
static void JumpR_Loop(chassis_t* chassis, Leg_t* leg, PidTypeDef* pid)
{
    if (chassis->flag.jump_flag == JUMP_STEP_SQUST)
    {
        /* code */

    }
    
}

/**
 * @brief 倒地自起
 *  
 */
void RiseR_Loop(chassis_t* chassis, Leg_t* leg)
{
    // 用关节电机角度判断是否完成自起
    if ((chassis->joint_motor[0].para.pos > 0.0f && chassis->joint_motor[0].para.pos < PI/2.0f) && 
        (chassis->joint_motor[1].para.pos > -PI/2.0f && chassis->joint_motor[1].para.pos < 0.0f))
        chassis->flag.recover_flag = 1;

    float diff1, diff4;
    float target1 = -PI/3.0f;
    float target4 = PI/3.0f;

    if (chassis->joint_motor[1].para.pos < 0 && chassis->joint_motor[0].para.pos < 0)
    {
        diff1 = target1 - chassis->joint_motor[1].para.pos;
        diff4 = target4 - chassis->joint_motor[0].para.pos;
    }
    if (chassis->joint_motor[1].para.pos > 0 && chassis->joint_motor[0].para.pos > 0)
    {
        diff1 = target1 + 2*PI - chassis->joint_motor[1].para.pos;
        diff4 = target4 + 2*PI - chassis->joint_motor[0].para.pos;
    }
    
    if (chassis->flag.recover_flag == 0)
    {
        /* code */
        chassis->leg_set = LEG_RISE_LENGTH_SET;
        Mit_Ctrl(&hcan1, chassis->joint_motor[0].para.id, diff4, 0.0f, 10.0f, 5.0f, 0.0f);
        Mit_Ctrl(&hcan1, chassis->joint_motor[1].para.id, diff1, 0.0f, 10.0f, 5.0f, 0.0f);
    }
}

/**
 * @brief 底盘控制
 * 
 * @param chassis
 * @param ins 
 * @param pid 
 */
#ifdef ControlOperation
static void ChasssisR_Control(
    chassis_t* chassis, Leg_t* leg, Excessive_t* excessive, INS_t* ins, PidTypeDef* length_pid, float lqr_k[2][6])
{
    // chassis->flag.is_take_off = legR.is_take_off || legL.is_take_off;

    ForwardKinematics(leg, excessive); 

    // Calc_LQR_K(lqr_k, leg->rod.L0, chassis->flag.is_take_off);

    // 状态向量更新--方便调试查看
    legR_state.theta     = X0_OFFSET + (leg->rod.theta - 0.0f);
    legR_state.theta_dot = X1_OFFSET + (leg->rod.d_theta - 0.0f);
    legR_state.x         = X2_OFFSET + (chassis->state.x_filter - chassis->state.x_set);
    legR_state.x_dot     = X3_OFFSET + (chassis->state.v_filter - chassis->state.v_set);
    legR_state.phi       = X4_OFFSET + (chassis->myPithR - chassis->phi_set);
    legR_state.phi_dot   = X5_OFFSET + (chassis->myPithGyroR - 0.0f);

    leg->rod.T = (    
                    + lqr_k[0][0] * legR_state.theta
                    + lqr_k[0][1] * legR_state.theta_dot
                    + lqr_k[0][2] * legR_state.x
                    + lqr_k[0][3] * legR_state.x_dot
                    + lqr_k[0][4] * legR_state.phi
                    + lqr_k[0][5] * legR_state.phi_dot
                    );

    leg->rod.Tp = (   
                    + lqr_k[1][0] * legR_state.theta
                    + lqr_k[1][1] * legR_state.theta_dot
                    + lqr_k[1][2] * legR_state.x
                    + lqr_k[1][3] * legR_state.x_dot
                    + lqr_k[1][4] * legR_state.phi
                    + lqr_k[1][5] * legR_state.phi_dot
                    );

    // leg->rod.Tp = leg->rod.Tp + chassis->leg_tp;        //髋关节输出力矩
    // leg->rod.T = leg->rod.T - chassis->turn_T;          //轮毂关节输出力矩

    //输出限幅
    SATURATE(&leg->rod.T, -20.0f, 20.0f);
    
    leg->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(leg->rod.theta) / 2 
                    + PID_Calc(length_pid, leg->rod.L0, chassis->leg_set);

    // leg->is_take_off = chassis->flag.right_flag;

    // if (chassis->flag.recover_flag == 0)
    // {
    //     /* code */
    //     if(leg->is_take_off && leg->touch_time > TOUCH_TOGGLE_THRESHOLD)
    //     {
    //         leg->is_take_off = false;
    //     }
    //     else if (!leg->is_take_off && leg->take_off_time > TOUCH_TOGGLE_THRESHOLD)
    //     {
    //         leg->is_take_off = true;
    //         chassis->state.x_filter = 0.0f;
    //         chassis->state.x_set = chassis->state.x_filter;
    //     }
    // }
    SATURATE(&leg->rod.F0, -500.0f, 500.0f);

    JacobianMatrix(leg, excessive);

    // 髋关节输出限幅
    SATURATE(&leg->joint.T1, -MAX_TORQUE, MAX_TORQUE);
    SATURATE(&leg->joint.T2, -MAX_TORQUE, MAX_TORQUE);
}
#endif

/**
 * @brief 位控调试模式
 * 
 */
#ifdef ControlDebug
// 保证电机逆解走最短路径

static void ChassisR_Debug(chassis_t* chassis, Leg_t* leg, Excessive_t* excessive,PidTypeDef* length_pid)
{
    // PID_Calc(length_pid, leg->rod.L0, chassis->leg_set);
    InverseKinematics(chassis, leg, excessive);

    float phi1 = -PI/2.0f + leg->joint.Phi1;
    float phi4 = -PI/2.0f + leg->joint.Phi4;

    // // 达妙电机，顺时针是正角度，逆时针是负角度
    leg->joint.Phi1_set = ShortestAngle(phi1, &leg->joint.Phi1_set);
    leg->joint.Phi4_set = ShortestAngle(phi4, &leg->joint.Phi4_set);

    //输出角度限幅
    SATURATE(&leg->joint.Phi1_set, 0.0f, PI);
    SATURATE(&leg->joint.Phi4_set, -PI, 0.0f);
}
#endif

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutputR(chassis_t* chassis, Leg_t* leg)
{
    // 功率限制 待添加


    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(chassis->wheel_motor[0], FINAL_COEFFICIENT*leg->rod.T);
    // DJIMotorSetRef(chassis->wheel_motor[0], 500);
}

/**
 * @brief 底盘任务
 * 
 */
void ChassisR_Task(void)
{
    #ifdef ControlOperation
    while(INS.ins_flag == 0) //等待INS初始化完成
    {
        osDelay(1);
    }
    #endif

    ChassisR_Init(&chassis_move, &LegR_pid);
    // Pensation_Init(&Roll_Pid,&Tp_Pid,&Turn_Pid);
    
    while (1)
    {
        #ifdef ControlOperation
        // /* code */
        ChassisR_Feedback_Update(&chassis_move, &legR, &INS, (float)CHASSIS_TIME/1000.0f);
        ChasssisR_Control(&chassis_move, &legR, &excessiveR, &INS, &LegR_pid, LQR_K_R);
        #endif
        #ifdef ControlDebug
        ChassisR_Debug(&chassis_move, &legR, &excessiveR, &LegR_pid);
        #endif
        if (chassis_move.flag.start_flag == 1)
        {
            #ifdef ControlOperation
            /* code */
                /* code */
                Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f, legR.joint.T1);
                osDelay(CHASSIS_TIME);
                Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[1].para.id, 0.0f, 0.0f, 0.0f, 0.0f, legR.joint.T2);
                osDelay(CHASSIS_TIME);
                // 3508控制
                LimitChassisOutputR(&chassis_move, &legR);
            #endif

            #ifdef ControlDebug
            Mit_Ctrl(&hfdcan1, 0x03, legR.joint.Phi4_set, 0.0f, 100.0f, 4.0f, 0.0f);
            osDelay(CHASSIS_TIME);
            Mit_Ctrl(&hfdcan1, 0x04, legR.joint.Phi1_set, 0.0f, 100.0f, 4.0f, 0.0f);
            osDelay(CHASSIS_TIME);
            #endif
        }
        else if (chassis_move.flag.start_flag == 0)
        {
            Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            osDelay(CHASSIS_TIME);
            Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[1].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            osDelay(CHASSIS_TIME);
            // 3508控制
            DJIMotorStop(chassis_move.wheel_motor[0]);
            // DJIMotorSetRef(chassis_move.wheel_motor[0], 300.0f);
        }
        
    }
}
