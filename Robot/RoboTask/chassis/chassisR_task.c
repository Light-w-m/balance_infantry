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
#include "observe_task.h"
#include "pid.h"
#include "djimotor.h"
#include "dmmotor.h"
#include "ins_task.h"

Leg_t legR;
extern Leg_t legL;
LegState_t legR_state;
Excessive_t excessiveR;
extern chassis_t chassis_move;
Period_t periods;

extern INS_t INS;

PidTypeDef LegR_pid;    //右腿腿长PID
PidTypeDef Turn_Pid;    //转向pd
// PidTypeDef Yaw_Pid;     //航向pd
PidTypeDef Tp_Pid;      //防劈叉补偿pd
PidTypeDef Pitch_Pid;    //俯仰角补偿pd
PidTypeDef Pitch_V_Pid;  //俯仰角速度补偿pd
PidTypeDef Roll_Pid;    //横滚角补偿pd

//Q=diag([80 1 600 200 7000 1]);%theta d_theta x d_x phi d_phi
// R=diag([50,2]);              %T Tp
float LQR_K_R[2][6] = {
     {-6.223669982749576,	-0.664224151725734,	-2.728282250312658,	-2.533796701097422,	7.291014074233116,	0.455184197951449},
    {15.665008329804538,	2.102980678432747,	10.672951750369741,	9.484717958425000,	46.594289824326587,	1.042990420163179}
};

static float current_prtR = 0;
/**
 * @brief 底盘初始化,包括PID参数初始化，电机初始化
 * 
 * @param chassis 
 * @param pid 
 */
static void ChassisR_Init(chassis_t* chassis, PidTypeDef* length_pid)
{
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

    joint_motor_init(&chassis->joint_motor[0], 4, MIT_MODE);
    joint_motor_init(&chassis->joint_motor[1], 3, MIT_MODE);

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
static float theta_biasR = 0.041f;
static void ChassisR_Feedback_Update(chassis_t* chassis,Leg_t* leg, INS_t* ins, float dt)
{
    leg->joint.Phi4 = PI/2.0f + chassis->joint_motor[1].para.pos;
    leg->joint.Phi1 = PI/2.0f + chassis->joint_motor[0].para.pos;

    leg->joint.d_Phi4 = chassis->joint_motor[1].para.vel;
    leg->joint.d_Phi1 = chassis->joint_motor[0].para.vel;

    SATURATE(&leg->joint.Phi4, -PI/3.0f, PI/2.0f);
    SATURATE(&leg->joint.Phi1, PI/2.0f, PI*4/3.0f);

    chassis->myPithR = ins->Pitch;
    chassis->myPithGyroR = ins->Gyro[Y_AXIS];
    
    chassis->body.pitch = ins->Pitch;
    chassis->body.pitch_dot = ins->Gyro[Y_AXIS];
    chassis->body.yaw = ins->Yaw;
    chassis->body.yaw_dot = ins->Gyro[Z_AXIS];
    chassis->body.yaw_total = ins->YawTotal;
    chassis->body.roll = ins->Roll;

    // L0
    leg->rod.d_L0 = leg->j11*leg->joint.d_Phi1 + leg->j12*leg->joint.d_Phi4;
    leg->rod.last_d_L0 = leg->rod.d_L0;
    leg->rod.dd_L0 = (leg->rod.d_L0 - leg->rod.last_d_L0) / dt;

    // phi0
    // leg->rod.last_phi0 = leg->rod.phi0;
    // leg->rod.d_phi0 = (leg->rod.phi0 - leg->rod.d_phi0) / dt;
    leg->rod.d_phi0 = leg->j21*leg->joint.d_Phi1 + leg->j22*leg->joint.d_Phi4;

    //theta
    // leg->rod.theta = PI/2 - leg->rod.phi0 - chassis->myPithR - 0.24f;
    leg->rod.theta = PI/2 - leg->rod.phi0 - chassis->myPithR + theta_biasR;
    leg->rod.d_theta = - leg->rod.d_phi0 - chassis->myPithGyroR;
    leg->rod.last_d_theta = leg->rod.d_theta;
    leg->rod.dd_theta = (leg->rod.d_theta - leg->rod.last_d_theta) / dt;

    chassis->bias.theta_err = 0.0f - (leg->rod.theta + legL.rod.theta); 
    //倒地自起检测
    if(1)
    {
        // chassis->flag.recover_flag = 0;
    }
}

/**
 * @brief 补偿初始化：roll补偿、防劈叉补偿、偏航角补偿
 * 
 * @param roll 
 * @param Tp 
 * @param turn 
 */
void Pensation_Init(PidTypeDef *pitch, PidTypeDef *pitch_v, PidTypeDef *roll, PidTypeDef *Tp, PidTypeDef *turn)
{
    const static float pitch_pid[3] = {PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD};
    const static float pitch_v_pid[3] = {PITCH_V_PID_KP, PITCH_V_PID_KI, PITCH_V_PID_KD};
    const static float roll_pid[3] = {ROLL_PID_KP, ROLL_PID_KI,ROLL_PID_KD};
	const static float tp_pid[3] = {TP_PID_KP, TP_PID_KI, TP_PID_KD};
	const static float turn_pid[3] = {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};
	
	PID_init(pitch, PID_POSITION, pitch_pid, PITCH_PID_MAX_OUT, PITCH_PID_MAX_IOUT);
    PID_init(pitch_v, PID_POSITION, pitch_v_pid, PITCH_V_PID_MAX_OUT, PITCH_V_PID_MAX_IOUT);
	PID_init(roll, PID_POSITION, roll_pid, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT);
	PID_init(Tp, PID_POSITION, tp_pid, TP_PID_MAX_OUT,TP_PID_MAX_IOUT);
	PID_init(turn, PID_POSITION, turn_pid, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);
}

/**
 * @brief 跳跃控制
 * 
 */
static void JumpR_Loop(chassis_t* chassis, Leg_t* leg, PidTypeDef* length_pid)
{
    if (chassis->flag.jump_flag == 1)
    {
        if (chassis->step.jump_step == NORMAL_STEP)
        {
            leg->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(leg->rod.theta) / 2 
                    + PID_Calc(length_pid, leg->rod.L0, MIN_LEG_LENGTH);
            if (leg->rod.L0 < 0.18f) chassis->step.jump_step_time++;
            if (chassis->step.jump_step_time > 10)
            {
                chassis->step.jump_step_time = 0;
                chassis->step.jump_step = JUMP_STEP_SQUST;
            }
        }
        else if (chassis->step.jump_step == JUMP_STEP_SQUST)
        {
            leg->rod.F0 = 70.0f;
            if (leg->rod.L0 > 0.22f) chassis->step.jump_step_time++;
            if (chassis->step.jump_step_time > 20)
            {
                chassis->step.jump_step_time = 0;
                chassis->step.jump_step = JUMP_STEP_JUMP;
            }
        }
        else if (chassis->step.jump_step == JUMP_STEP_JUMP)
        {
            leg->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(leg->rod.theta) / 2 
                    + PID_Calc(length_pid, leg->rod.L0, chassis->leg_set);
            if (leg->rod.L0 < 0.20f) chassis->step.jump_step_time++;
            if (chassis->step.jump_step_time > 10)
            {
                chassis->step.jump_step_time = 0;
                chassis->step.jump_step = JUMP_STEP_RECOVERY;
            }
        }
        else 
            leg->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(leg->rod.theta) / 2 
                    + PID_Calc(length_pid, leg->rod.L0, chassis->leg_set);
        if (chassis->step.jump_step == JUMP_STEP_RECOVERY)
        {
            chassis->flag.jump_flag = 0;
            chassis->step.jump_step = NORMAL_STEP;
            chassis->step.jump_step_time = 0;
        }
    }
    else
        leg->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(leg->rod.theta) / 2 
                    + PID_Calc(length_pid, leg->rod.L0, chassis->leg_set);
}

/**
 * @brief 倒地自起
 *  
 */
static void RiseR_Loop(chassis_t* chassis, Leg_t* leg)
{
    // 用关节电机角度判断是否完成自起
    // if ((chassis->joint_motor[0].para.pos > -0.6f && chassis->joint_motor[0].para.pos < 2.3f) && 
    //     (chassis->joint_motor[1].para.pos > -2.5f && chassis->joint_motor[1].para.pos < -1.0f))
    //     leg->rod.recovery_flag = 0; // 倒地自起完成标志
    // else 
    //     leg->rod.recovery_flag = 1; // 倒地自起进行中标志

    float target0 = -0.5f;
    float target1 = -2.2f;

    float diff0 = 0.0f; float diff1 = 0.0f;

    // diff0 = ShortestAngle(target0, &diff0);
    // diff1 = ShortestAngle(target1, &diff1);
    
    if (chassis->flag.recover_flag == 1 && leg->rod.recovery_flag == 1)
    {
        /* code */
        // chassis->leg_set = LEG_RISE_LENGTH_SET;
        // 达妙电机mit位置模式，小到大：逆时针，大到小：顺时针
        Mit_Ctrl(&hcan1, chassis->joint_motor[0].para.id, target0, 0.0f, 15.0f, 5.0f, 0.0f);
        Mit_Ctrl(&hcan1, chassis->joint_motor[1].para.id, target1, 0.0f, 15.0f, 5.0f, 0.0f);
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
    // chassis->flag.is_take_off = chassis->flag.right_flag || chassis->flag.left_flag;

    ForwardKinematics(leg, excessive); 

    // Calc_LQR_K(lqr_k, leg->rod.L0, chassis->flag.is_take_off);

    chassis->turn_T = Turn_Pid.Kp * (chassis->reference.yaw - chassis->body.yaw_total)
                        - Turn_Pid.Kd * chassis->body.yaw_dot;
    // chassis->turn_T = kp_Yaw*(chassis->reference.yaw - chassis->body.yaw) 
    //                     + PID_Calc(&Turn_Pid, chassis->body.yaw_dot, chassis->reference.wz);
    chassis->roll_T = PID_Calc(&Roll_Pid, chassis->body.roll, chassis->reference.roll);
    chassis->reference.pitch_dot = PID_Calc(&Pitch_Pid, chassis->body.pitch, 0.0f);
    chassis->pitch_T = PID_Calc(&Pitch_V_Pid, chassis->body.pitch_dot, chassis->reference.pitch_dot);
    chassis->leg_tp = PID_Calc(&Tp_Pid, chassis->bias.theta_err, 0.0f);

    // 状态向量更新--方便调试查看
    legR_state.theta     = X0_OFFSET + (leg->rod.theta - 0.0f);
    legR_state.theta_dot = X1_OFFSET + (leg->rod.d_theta - 0.0f);
    // legR_state.x         = X2_OFFSET(chassis->leg_set) + chassis->state.x_filter;
    legR_state.x         = X2_OFFSET + chassis->state.x_filter;
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

    leg->rod.Tp = leg->rod.Tp + chassis->leg_tp + (1 - 2*leg->rod.L0)*OFFSET;        //髋关节输出力矩
    leg->rod.T = leg->rod.T - chassis->turn_T + chassis->bias.myPitch;          //轮毂关节输出力矩

    //输出限幅
    SATURATE(&leg->rod.T, -2.0f, 2.0f);
    SATURATE(&leg->rod.Tp, -10.0f, 10.0f);
    
    leg->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(leg->rod.theta) / 2 
                    + PID_Calc(length_pid, leg->rod.L0, chassis->leg_set) + 2.0f;
                    // + chassis->roll_T;

    JumpR_Loop(chassis, leg, length_pid);
    
    // 离地判断
    // chassis->flag.right_flag = GroundDetect(chassis, leg, &periods);
    if(chassis->flag.is_take_off == 1)
    {
        for (uint8_t i = 0; i < 2; i++)
        {
            lqr_k[1][i] = 0.0f;
        }
    }
    
    // 倒地自起判断
    // RiseR_Loop(chassis, leg);

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
    DJIMotorSetRef(chassis->wheel_motor[0], -FINAL_COEFFICIENT*leg->rod.T);
    // DJIMotorSetRef(chassis->wheel_motor[0], 0.0f);
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
    Pensation_Init(&Pitch_Pid,&Pitch_V_Pid,&Roll_Pid,&Tp_Pid,&Turn_Pid);
    
    while (1)
    {
        #ifdef ControlOperation
        // /* code */
        ChassisR_Feedback_Update(&chassis_move, &legR, &INS, (float)CHASSIS_TIME*3/1000.0f);
        ChasssisR_Control(&chassis_move, &legR, &excessiveR, &INS, &LegR_pid, LQR_K_R);
        #endif
        #ifdef ControlDebug
        ChassisR_Debug(&chassis_move, &legR, &excessiveR, &LegR_pid);
        #endif
        if (chassis_move.flag.start_flag == 1)
        {
            #ifdef ControlOperation
                /* code */
                if(chassis_move.flag.recover_flag == 0 && legR.rod.recovery_flag == 0)
                {
                    Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f, legR.joint.T1);
                    Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[1].para.id, 0.0f, 0.0f, 0.0f, 0.0f, legR.joint.T2);
                    // Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    // Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[1].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    // 3508控制
                    LimitChassisOutputR(&chassis_move, &legR);
                    osDelay(CHASSIS_TIME);
                }
            #endif

            #ifdef ControlDebug
            Mit_Ctrl(&hfdcan1, 0x03, legR.joint.Phi4_set, 0.0f, 100.0f, 4.0f, 0.0f);
            Mit_Ctrl(&hfdcan1, 0x04, legR.joint.Phi1_set, 0.0f, 100.0f, 4.0f, 0.0f);
            osDelay(CHASSIS_TIME);
            #endif
        }
        else if (chassis_move.flag.start_flag == 0)
        {
            Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[1].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            // 3508控制
            DJIMotorStop(chassis_move.wheel_motor[0]);
            osDelay(CHASSIS_TIME);
            // DJIMotorSetRef(chassis_move.wheel_motor[0], 300.0f);
        }
        
    }
}
