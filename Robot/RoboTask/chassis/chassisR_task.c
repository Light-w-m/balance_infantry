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

// #define CHASSIS_CONTROL_TIME_MS 2
Leg_t legR;
LegState_t legR_state;
Excessive_t excessiveR;
chassis_t chassis_move;

extern INS_t INS;

PidTypeDef LegR_pid;    //右腿腿长PID
PidTypeDef Tp_Pid;      //防劈叉补偿pd
PidTypeDef Turn_Pid;    //转向pd
PidTypeDef Roll_Pid;    //横滚角补偿pd

uint32_t CHASSISR_TIME = 1;  //延迟时间

float LQR_K[2][6];


/**
 * @brief 底盘初始化,包括PID参数初始化，电机初始化
 * 
 * @param chassis 
 * @param pid 
 */
void ChassisR_Init(chassis_t* chassis, PidTypeDef* pid)
{
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
        .motor_type = M3508,
    };

    const static float leg_pid_R[3] = {LEG_PID_KP, LEG_PID_KI, LEG_PID_KD}; //P I D

    joint_motor_init(&chassis->joint_motor[0], 1, MIT_MODE);
    joint_motor_init(&chassis->joint_motor[1], 2, MIT_MODE);

    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    chassis->wheel_motor[0] = DJIMotorInit(&chassis_motor_config);

    //腿长PID初始化
    PID_init(pid, PID_POSITION, leg_pid_R, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);

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
void ChassisR_Feedback_Update(chassis_t* chassis,Leg_t* leg, INS_t* ins, Excessive_t* excessive)
{
    leg->joint.Phi1 = PI/2.0f + chassis->joint_motor[0].para.pos;
    leg->joint.Phi4 = PI/2.0f + chassis->joint_motor[1].para.pos;

    chassis->myPithR = ins->Pitch;
    chassis->myPithGyroR = ins->Gyro[1];

    chassis->total_yaw=ins->YawTotalAngle;
	chassis->roll=ins->Roll;

    //倒地自起检测--待补充
    if(1)
    {

    }
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
        // Dm8009_Fbdata(&chassis_move.joint_motor[0], rx_data, 8);
        // vTaskDelay(CHASSIS_CONTROL_TIME_MS); 
        // Dm8009_Fbdata(&chassis_move.joint_motor[1], rx_data, 8);
        // vTaskDelay(CHASSIS_CONTROL_TIME_MS);
        DJIMotorControl();
        /* code */
        ChassisR_Feedback_Update(&chassis_move, &legR,&INS, &excessiveR);
        ChasssisR_Control(&chassis_move, &legR, &excessiveR, &INS, &LegR_pid, LQR_K);

        if (chassis_move.flag.start_flag == 1)
        {
            /* code */
            Mit_Ctrl(&hfdcan1, 0x01, 0.0f, 0.0f, 0.0f, 0.0f, legR.joint.T1);
            osDelay(CHASSISR_TIME);
            Mit_Ctrl(&hfdcan1, 0x02, 0.0f, 0.0f, 0.0f, 0.0f, legR.joint.T1);
            osDelay(CHASSISR_TIME);
            // 3508控制
            DJIMotorSetRef(chassis_move.wheel_motor[0], legR.rod.T);
        }
        else if (chassis_move.flag.start_flag == 0)
        {
            Mit_Ctrl(&hfdcan1, 0x01, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            osDelay(CHASSISR_TIME);
            Mit_Ctrl(&hfdcan1, 0x02, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            osDelay(CHASSISR_TIME);
            // 3508控制
            DJIMotorSetRef(chassis_move.wheel_motor[0], 0.0f);
        }
        
    }
}

/**
 * @brief 底盘控制
 * 
 * @param chassis
 * @param ins 
 * @param pid 
 */
void ChasssisR_Control(
    chassis_t* chassis, Leg_t* leg, Excessive_t* excessive, INS_t* ins, PidTypeDef* length_pid, float lqr_k[2][6])
{
    ForwardKinematics(leg, excessive, ins, ((float)CHASSISR_TIME)*3.0f/1000); //该任务控制周期是3*0.001秒

    Calc_LQR_K(lqr_k, leg->rod.L0);

    //yaw轴pid
    chassis->turn_T = Turn_Pid.Kp*(chassis->turn_set-chassis->total_yaw)-Turn_Pid.Kd*ins->Gyro[2];
    //roll轴pid
    chassis->roll_T = Roll_Pid.Kp*(chassis->roll_set-chassis->roll)-Roll_Pid.Kd*ins->Gyro[0];
    //放劈叉pid
    chassis->leg_tp = PID_Calc(&Tp_Pid, chassis->theta_err, 0.0f);

    // 状态向量更新--方便调试查看
    legR_state.theta     = X0_OFFSET + (leg->rod.theta - 0.0f);
    legR_state.theta_dot = X1_OFFSET + (leg->rod.d_theta - 0.0f);
    legR_state.x         = X2_OFFSET + (chassis->state.x_filter - chassis->state.x_set);
    legR_state.x_dot     = X3_OFFSET + (chassis->state.v_filter - chassis->state.v_set);
    legR_state.phi       = X4_OFFSET + (chassis->myPithR - chassis->phi_set);
    legR_state.phi_dot   = X5_OFFSET + (chassis->myPithGyroR - 0.0f);

    leg->rod.T = (    lqr_k[0][0] * legR_state.theta
                    + lqr_k[0][1] * legR_state.theta_dot
                    + lqr_k[0][2] * legR_state.x
                    + lqr_k[0][3] * legR_state.x_dot
                    + lqr_k[0][4] * legR_state.phi
                    + lqr_k[0][5] * legR_state.phi_dot);

    leg->rod.Tp = (   lqr_k[1][0] * legR_state.theta
                    + lqr_k[1][1] * legR_state.theta_dot
                    + lqr_k[1][2] * legR_state.x
                    + lqr_k[1][3] * legR_state.x_dot
                    + lqr_k[1][4] * legR_state.phi
                    + lqr_k[1][5] * legR_state.phi_dot);

    leg->rod.Tp = leg->rod.Tp + chassis->leg_tp;        //髋关节输出力矩
    leg->rod.T = leg->rod.T - chassis->turn_T;          //轮毂关节输出力矩

    //输出限幅
    SATURATE(&leg->rod.T, -3.0f, 3.0f);

    chassis->leg_set = 0.0f;

    leg->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(leg->rod.theta) / 2 
                    + PID_Calc(length_pid, leg->rod.L0, chassis->leg_set);

    JacobianMatrix(leg, excessive);
}