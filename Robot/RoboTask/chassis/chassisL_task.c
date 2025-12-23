#include "chassisL_task.h"

Leg_t legL;
LegState_t legL_state;
Excessive_t excessiveL;
extern chassis_t chassis_move;

extern INS_t INS;

PidTypeDef LegL_pid;    //腿长PID
uint32_t CHASSISL_TIME = 1;  //延迟时间

extern float LQR_K[2][6];

/**
 * @brief 底盘初始化,包括PID参数初始化，电机初始化
 * 
 * @param chassis 
 * @param pid 
 */
void ChassisL_Init(chassis_t* chassis, PidTypeDef* pid)
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

    const static float leg_pid_L[3] = {LEG_PID_KP, LEG_PID_KI, LEG_PID_KD}; //P I D

    joint_motor_init(&chassis->joint_motor[2], 3, MIT_MODE);
    joint_motor_init(&chassis->joint_motor[3], 4, MIT_MODE);

    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    chassis->wheel_motor[1] = DJIMotorInit(&chassis_motor_config);

    //腿长PID初始化
    PID_init(pid, PID_POSITION, leg_pid_L, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);

    //电机使能
    for (uint8_t i = 0; i < 5; i++)
    {
        /* code */
        Enable_Motor_Mode(&hfdcan1, chassis->joint_motor[2].para.id, chassis->joint_motor[0].mode);
        Enable_Motor_Mode(&hfdcan1, chassis->joint_motor[3].para.id, chassis->joint_motor[1].mode);
        DJIMotorEnable(chassis->wheel_motor[1]);
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
void ChassisL_Feedback_Update(chassis_t* chassis,Leg_t* leg, INS_t* ins, Excessive_t* excessive)
{
    leg->joint.Phi1 = PI/2.0f + chassis->joint_motor[2].para.pos;
    leg->joint.Phi4 = PI/2.0f + chassis->joint_motor[3].para.pos;

    chassis->myPithL = ins->Pitch;
    chassis->myPithGyroL = ins->Gyro[1];

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
void ChassisL_Task(void)
{
    while(INS.ins_flag == 0) //等待INS初始化完成
    {
        osDelay(1);
    }

    ChassisL_Init(&chassis_move, &LegL_pid);

    while (1)
    {
        DJIMotorControl();
        /* code */
        ChassisL_Feedback_Update(&chassis_move, &legL,&INS, &excessiveL);
        ChasssisL_Control(&chassis_move, &legL, &excessiveL, &INS, &LegL_pid, LQR_K);

        if (chassis_move.flag.start_flag == 1)
        {
            /* code */
            Mit_Ctrl(&hfdcan1, 0x03, 0.0f, 0.0f, 0.0f, 0.0f, legL.joint.T1);
            osDelay(CHASSISL_TIME);
            Mit_Ctrl(&hfdcan1, 0x04, 0.0f, 0.0f, 0.0f, 0.0f, legL.joint.T1);
            osDelay(CHASSISL_TIME);
            // 3508控制
            DJIMotorSetRef(chassis_move.wheel_motor[1], legL.rod.T);
        }
        else if (chassis_move.flag.start_flag == 0)
        {
            Mit_Ctrl(&hfdcan1, 0x03, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            osDelay(CHASSISL_TIME);
            Mit_Ctrl(&hfdcan1, 0x04, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            osDelay(CHASSISL_TIME);
            // 3508控制
            DJIMotorSetRef(chassis_move.wheel_motor[1], 0.0f);
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
void ChasssisL_Control(
    chassis_t* chassis, Leg_t* leg, Excessive_t* excessive, INS_t* ins, PidTypeDef* length_pid, float lqr_k[2][6])
{
    ForwardKinematics(leg, excessive, ins, ((float)CHASSISL_TIME)*3.0f/1000); //该任务控制周期是3*0.001秒

    Calc_LQR_K(lqr_k, leg->rod.L0);


    // 状态向量更新--方便调试查看
    legL_state.theta     = X0_OFFSET + (leg->rod.theta - 0.0f);
    legL_state.theta_dot = X1_OFFSET + (leg->rod.d_theta - 0.0f);
    legL_state.x         = X2_OFFSET + (chassis->state.x_filter - chassis->state.x_set);
    legL_state.x_dot     = X3_OFFSET + (chassis->state.v_filter - chassis->state.v_set);
    legL_state.phi       = X4_OFFSET + (chassis->myPithL - chassis->phi_set);
    legL_state.phi_dot   = X5_OFFSET + (chassis->myPithGyroL - 0.0f);

    leg->rod.T = (    lqr_k[0][0] * legL_state.theta
                    + lqr_k[0][1] * legL_state.theta_dot
                    + lqr_k[0][2] * legL_state.x
                    + lqr_k[0][3] * legL_state.x_dot
                    + lqr_k[0][4] * legL_state.phi
                    + lqr_k[0][5] * legL_state.phi_dot);

    leg->rod.Tp = (   lqr_k[1][0] * legL_state.theta
                    + lqr_k[1][1] * legL_state.theta_dot
                    + lqr_k[1][2] * legL_state.x
                    + lqr_k[1][3] * legL_state.x_dot
                    + lqr_k[1][4] * legL_state.phi
                    + lqr_k[1][5] * legL_state.phi_dot);

    leg->rod.Tp = leg->rod.Tp + chassis->leg_tp;        //髋关节输出力矩
    leg->rod.T = leg->rod.T - chassis->turn_T;          //轮毂关节输出力矩

    //输出限幅
    SATURATE(&leg->rod.T, -3.0f, 3.0f);

    chassis->leg_set = 0.0f;

    leg->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(leg->rod.theta) / 2 
                    + PID_Calc(length_pid, leg->rod.L0, chassis->leg_set);

    JacobianMatrix(leg, excessive);
}
