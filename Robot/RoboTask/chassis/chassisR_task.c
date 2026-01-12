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

// #define CHASSIS_CONTROL_TIME_MS 2
Leg_t legR;
extern Leg_t legL;
LegState_t legR_state;
Excessive_t excessiveR;
chassis_t chassis_move;
Period_t periods;

extern INS_t INS;

PidTypeDef LegR_pid;    //右腿腿长PID
PidTypeDef Tp_Pid;      //防劈叉补偿pd
PidTypeDef Turn_Pid;    //转向pd
PidTypeDef Roll_Pid;    //横滚角补偿pd


float LQR_K[2][6] = {   // 0.15
                    {-10.922637488618983, -0.809732324787194, -0.895806613702953, -1.380810433561345, 5.734138915689102, 0.812075790166094},
                    {18.684145538958852, 1.581462861415775, 3.506607035886385, 5.054126550708173, 8.461555616678476, 0.415774368108967}
                    };
int8_t TRANSITION_MATRIX[10] = {0};

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
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan2,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
            },
            .current_PID = {
                .Kp = 0.5, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        // .motor_type = M3508,
        .motor_type = GM6020,
    };

    const static float leg_pid_R[3] = {LEG_PID_KP, LEG_PID_KI, LEG_PID_KD}; //P I D

    joint_motor_init(&chassis->joint_motor[0], 1, MIT_MODE);
    joint_motor_init(&chassis->joint_motor[1], 2, MIT_MODE);

    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    chassis->wheel_motor[0] = DJIMotorInit(&chassis_motor_config);

    //腿长PID初始化
    PID_init(length_pid, PID_POSITION, leg_pid_R, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);

    //电机使能
    for (uint8_t i = 0; i < 5; i++)
    {
        /* code */
        Enable_Motor_Mode(&hfdcan1, chassis->joint_motor[0].para.id, chassis->joint_motor[0].mode);
        Enable_Motor_Mode(&hfdcan1, chassis->joint_motor[1].para.id, chassis->joint_motor[1].mode);
        DJIMotorEnable(chassis->wheel_motor[0]);
        osDelay(1);
    }
     
}

//补偿初始化：roll补偿、防劈叉补偿、偏航角补偿
void Pensation_Init(PidTypeDef *roll,PidTypeDef *Tp,PidTypeDef *turn)
{
    const static float roll_pid[3] = {ROLL_PID_KP, ROLL_PID_KI,ROLL_PID_KD};
	const static float tp_pid[3] = {TP_PID_KP, TP_PID_KI, TP_PID_KD};
	const static float turn_pid[3] = {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};
	
	PID_init(roll, PID_POSITION, roll_pid, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT);
	PID_init(Tp, PID_POSITION, tp_pid, TP_PID_MAX_OUT,TP_PID_MAX_IOUT);
	PID_init(turn, PID_POSITION, turn_pid, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);
}


/**
 * @brief 输入更新
 * 
 * @param chassis 
 * @param ins 
 * @param  
 */
static void ChassisR_Feedback_Update(chassis_t* chassis,Leg_t* leg, INS_t* ins)
{
    leg->joint.Phi1 = PI/2.0f + chassis->joint_motor[0].para.pos;
    leg->joint.Phi4 = PI/2.0f + chassis->joint_motor[1].para.pos;

    chassis->myPithR = ins->Pitch;
    chassis->myPithGyroR = ins->Gyro[1];

    chassis->total_yaw = ins->YawTotalAngle;
	chassis->roll = ins->Roll;
    chassis->theta_err = 0.0f - (leg->rod.theta + legL.rod.theta);

    //倒地自起检测
    if(ins->Pitch<(PI/6.0f) && ins->Pitch>(-PI/6.0f))
    {
        chassis->flag.recover_flag = 0;
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
 * @brief 底盘控制
 * 
 * @param chassis
 * @param ins 
 * @param pid 
 */
#ifdef ControlOperation
static void ChasssisR_Control(
    chassis_t* chassis, Leg_t* leg, LegState_t* leg_state , Excessive_t* excessive, Period_t* period, INS_t* ins, PidTypeDef* length_pid, float lqr_k[2][6])
{
    chassis->flag.is_take_off = legR.is_take_off || legL.is_take_off;

    ForwardKinematics(leg, excessive, ins, ((float)CHASSIS_TIME)*3.0f/1000); //该任务控制周期是3*0.001秒

    Calc_LQR_K(lqr_k, leg->rod.L0, chassis->flag.is_take_off);

    //yaw轴pid
    chassis->turn_T = Turn_Pid.Kp*(chassis->turn_set-chassis->total_yaw)-Turn_Pid.Kd*ins->Gyro[2];
    //roll轴pid
    chassis->roll_T = Roll_Pid.Kp*(chassis->roll_set-chassis->roll)-Roll_Pid.Kd*ins->Gyro[0];
    //放劈叉pid
    chassis->leg_tp = PID_Calc(&Tp_Pid, chassis->theta_err, 0.0f);

    // 状态向量更新--方便调试查看
    leg_state->theta     = X0_OFFSET + (leg->rod.theta - 0.0f);
    leg_state->theta_dot = X1_OFFSET + (leg->rod.d_theta - 0.0f);
    leg_state->x         = X2_OFFSET + (chassis->state.x_filter - chassis->state.x_set);
    leg_state->x_dot     = X3_OFFSET + (chassis->state.v_filter - chassis->state.v_set);
    leg_state->phi       = X4_OFFSET + (chassis->myPithR - chassis->phi_set);
    leg_state->phi_dot   = X5_OFFSET + (chassis->myPithGyroR - 0.0f);

    leg->rod.T = (    lqr_k[0][0] * leg_state->theta
                    + lqr_k[0][1] * leg_state->theta_dot
                    + lqr_k[0][2] * leg_state->x
                    + lqr_k[0][3] * leg_state->x_dot
                    + lqr_k[0][4] * leg_state->phi
                    + lqr_k[0][5] * leg_state->phi_dot);

    leg->rod.Tp = (   lqr_k[1][0] * leg_state->theta
                    + lqr_k[1][1] * leg_state->theta_dot
                    + lqr_k[1][2] * leg_state->x
                    + lqr_k[1][3] * leg_state->x_dot
                    + lqr_k[1][4] * leg_state->phi
                    + lqr_k[1][5] * leg_state->phi_dot);

    leg->rod.Tp = leg->rod.Tp + chassis->leg_tp;        //髋关节输出力矩
    // leg->rod.T = leg->rod.T - chassis->turn_T;          //轮毂关节输出力矩

    //输出限幅
    SATURATE(&leg->rod.T, -3.0f, 3.0f);

    // chassis->leg_set = 0.0f;
    leg->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(leg->rod.theta) / 2 
                    + PID_Calc(length_pid, leg->rod.L0, chassis->leg_set);

    chassis->flag.right_flag = GroundDetect(leg, period, ins);
    leg->is_take_off = chassis->flag.right_flag;

    if (chassis->flag.recover_flag == 0)
    {
        /* code */
        if(leg->is_take_off && leg->touch_time > TOUCH_TOGGLE_THRESHOLD)
        {
            leg->is_take_off = false;
        }
        else if (!leg->is_take_off && leg->take_off_time > TOUCH_TOGGLE_THRESHOLD)
        {
            leg->is_take_off = true;
            chassis->state.x_filter = 0.0f;
            chassis->state.x_set = chassis->state.x_filter;
        }
    }
    SATURATE(&leg->rod.F0, -0.0f, 0.0f);

    JacobianMatrix(leg, excessive);

    // 髋关节输出限幅
    SATURATE(&leg->joint.T1, -0.0f, 0.0f);
    SATURATE(&leg->joint.T2, -0.0f, 0.0f);
}
#endif

/**
 * @brief 位控调试模式
 * 
 */
#ifdef ControlDebug
static void ChassisR_Debug(chassis_t* chassis, Leg_t* leg, Excessive_t* excessive)
{
    leg->joint.T1 = 0.0f;
    leg->joint.T2 = 0.0f;

    InverseKinematics(leg, excessive);

    leg->joint.Phi1_set = PI/2.0f - leg->joint.Phi1;
    leg->joint.Phi4_set = -PI/2.0f + leg->joint.Phi4;

    //输出角度限幅
    // SATURATE(&leg->joint.Phi1, -0.0f, 0.0f);
    // SATURATE(&leg->joint.Phi4, -0.0f, 0.0f);
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
    DJIMotorSetRef(chassis->wheel_motor[0], leg->rod.T);
}

/**
 * @brief 底盘任务
 * 
 */
void ChassisR_Task(void)
{
    while(INS.ins_flag == 0) //等待INS初始化完成
    {
        osDelay(1);
    }

    ChassisR_Init(&chassis_move, &LegR_pid);
    Pensation_Init(&Roll_Pid,&Tp_Pid,&Turn_Pid);

    while (1)
    {
        #ifdef ControlOperation
        /* code */
        ChassisR_Feedback_Update(&chassis_move, &legR,&INS);
        ChasssisR_Control(&chassis_move, &legR, &legR_state, &excessiveR, &periods, &INS, &LegR_pid, LQR_K);
        #endif

        if (chassis_move.flag.start_flag == 1)
        {
            #ifdef ControlOperation
            /* code */
            Mit_Ctrl(&hfdcan1, 0x01, 0.0f, 0.0f, 0.0f, 0.0f, legR.joint.T1);
            osDelay(CHASSIS_TIME);
            Mit_Ctrl(&hfdcan1, 0x02, 0.0f, 0.0f, 0.0f, 0.0f, legR.joint.T2);
            osDelay(CHASSIS_TIME);
            // 3508控制
            // DJIMotorSetRef(chassis_move.wheel_motor[0], legR.rod.T);
            LimitChassisOutputR(&chassis_move, &legR);
            #endif

            #ifdef ControlDebug
            Mit_Ctrl(&hfdcan1, 0x01, legR.joint.Phi1_set, 0.0f, 10.0f, 4.0f, 0.0f);
            osDelay(CHASSIS_TIME);
            Mit_Ctrl(&hfdcan1, 0x02, legR.joint.Phi4_set, 0.0f, 10.0f, 4.0f, 0.0f);
            osDelay(CHASSIS_TIME);
            #endif
        }
        else if (chassis_move.flag.start_flag == 0)
        {
            Mit_Ctrl(&hfdcan1, 0x01, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            osDelay(CHASSIS_TIME);
            Mit_Ctrl(&hfdcan1, 0x02, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            osDelay(CHASSIS_TIME);
            // 3508控制
            DJIMotorStop(chassis_move.wheel_motor[0]);
            // DJIMotorSetRef(chassis_move.wheel_motor[0], 0.0f);
        }
        
    }
}
