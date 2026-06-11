#include "chassisL_task.h"
#include "chassis_def.h"
#include "kinematics.h"
#include "kalman_filter.h"
#include "observe_task.h"
#include "pid.h"
#include "djimotor.h"
#include "dmmotor.h"
#include "ins_task.h"

Leg_t legL;
LegState_t legL_state;
Excessive_t excessiveL;
extern chassis_t chassis_move;
extern Period_t periods;

extern INS_t INS;

PidTypeDef LegL_pid;    //腿长PID

float LQR_K_L[2][6] = {
     {-6.223669982749576,	-0.664224151725734,	-2.728282250312658,	-2.533796701097422,	7.291014074233116,	0.455184197951449},
    {15.665008329804538,	2.102980678432747,	10.672951750369741,	9.484717958425000,	46.594289824326587,	1.042990420163179}
};

static float current_prtL = 0;
/**
 * @brief 底盘初始化,包括PID参数初始化，电机初始化
 * 
 * @param chassis 
 * @param pid 
 */
static void ChassisL_Init(chassis_t* chassis, Leg_t* leg, PidTypeDef* pid)
{
    // 两个轮电机的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_configL = {
        .can_init_config.can_handle = &hcan2,
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = OPEN_LOOP,
            .close_loop_type = OPEN_LOOP,

            // .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .controller_param_init_config = {
        .current_feedforward_ptr = &current_prtL,
        },
        .motor_type = M3508,
    };

    const static float leg_pid_L[3] = {LEG_PID_KP, LEG_PID_KI, LEG_PID_KD}; //P I D

    joint_motor_init(&chassis->joint_motor[2], 1, MIT_MODE);
    joint_motor_init(&chassis->joint_motor[3], 2, MIT_MODE);

    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_configL.can_init_config.tx_id = 1;
    chassis_motor_configL.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    chassis->wheel_motor[1] = DJIMotorInit(&chassis_motor_configL);

    //腿长PID初始化
    PID_init(pid, PID_POSITION, leg_pid_L, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);
    for(uint8_t i = 0; i < 2; i++)
    {
        Engineer_PID_Init(&leg->joint.jointAngle[i], Feedforward_PID, JOINT_Angle_KP,JOINT_Angle_KI,JOINT_Angle_KD,JOINT_Angle_KF,JOINT_Angle_I_BAND,JOINT_Angle_DT,JOINT_Angle_MAX_OUT,JOINT_Angle_MAX_IOUT);
        Engineer_PID_Init(&leg->joint.jointSpeed[i], Normal_PID, JOINT_Speed_KP,JOINT_Speed_KI,JOINT_Speed_KD,JOINT_Speed_KF,JOINT_Speed_I_BAND,JOINT_Speed_DT,JOINT_Speed_MAX_OUT,JOINT_Speed_MAX_IOUT);
    }

    //电机使能
    for (uint8_t i = 0; i < 3; i++)
    {
        /* code */
        Enable_Motor_Mode(&hfdcan1, chassis->joint_motor[2].para.id, chassis->joint_motor[2].mode);
        Enable_Motor_Mode(&hfdcan1, chassis->joint_motor[3].para.id, chassis->joint_motor[3].mode);
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
static float theta_biasL = -0.0f;
static void ChassisL_Feedback_Update(chassis_t* chassis,Leg_t* leg, INS_t* ins, float dt)
{
    leg->joint.Phi4 = PI/2.0f + chassis->joint_motor[3].para.pos;
    leg->joint.Phi1 = PI/2.0f + chassis->joint_motor[2].para.pos;

    leg->joint.d_Phi4 = chassis->joint_motor[3].para.vel;
    leg->joint.d_Phi1 = chassis->joint_motor[2].para.vel;

    SATURATE(&leg->joint.Phi4, -PI/3.0f, PI/2.0f);
    SATURATE(&leg->joint.Phi1, PI/2.0f, PI*4/3.0f);

    chassis->myPithL = -ins->Pitch;
    chassis->myPithGyroL = -ins->Gyro[Y_AXIS];

    // L0
    leg->rod.d_L0 = leg->j11*leg->joint.d_Phi1 + leg->j12*leg->joint.d_Phi4;
    leg->rod.last_d_L0 = leg->rod.d_L0;
    leg->rod.dd_L0 = (leg->rod.d_L0 - leg->rod.last_d_L0) / dt;

    // phi0
    // leg->rod.last_phi0 = leg->rod.phi0;
    // leg->rod.d_phi0 = (leg->rod.phi0 - leg->rod.d_phi0) / dt;
    leg->rod.d_phi0 = leg->j21*leg->joint.d_Phi1 + leg->j22*leg->joint.d_Phi4;

    //theta
    // leg->rod.theta = PI/2 - leg->rod.phi0 - chassis->myPithL + 0.20f;
    leg->rod.theta = PI/2 - leg->rod.phi0 - chassis->myPithL + theta_biasL;
    leg->rod.d_theta = - leg->rod.d_phi0 - chassis->myPithGyroL;
    leg->rod.last_d_theta = leg->rod.d_theta;
    leg->rod.dd_theta = (leg->rod.d_theta - leg->rod.last_d_theta) / dt;

    // 自适应重心误差补偿
    chassis->bias.error = ins->Pitch - chassis->bias.myPitch;
    if(fabsf(chassis->bias.error) < 0.02f)  // 1度
    {
        chassis->bias.error = 0.0f;
    }
    if(fabsf(ins->Pitch) < 0.1f) // 6度
    {
        chassis->bias.myPitch += Pitch_ki * chassis->bias.error * dt;
    }
    SATURATE(&chassis->bias.myPitch, -0.05f, 0.05f);

    //倒地自起检测--待补充
    if(1)
    {

    }
}


/**
 * @brief 跳跃控制
 * 
 */
static void JumpL_Loop(chassis_t* chassis, Leg_t* leg, PidTypeDef* length_pid)
{
    if (chassis->flag.jump_flag == 1)
    {
        if (chassis->step.jump_step == NORMAL_STEP)
        {
            leg->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(leg->rod.theta) / 2 
                    + PID_Calc(length_pid, leg->rod.L0, MIN_LEG_LENGTH);
        }
        else if (chassis->step.jump_step == JUMP_STEP_SQUST)
        {
            leg->rod.F0 = 60.0f;
        }
        else if (chassis->step.jump_step == JUMP_STEP_JUMP)
        {
            leg->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(leg->rod.theta) / 2 
                    + PID_Calc(length_pid, leg->rod.L0, MIN_LEG_LENGTH);
        }
        else 
            leg->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(leg->rod.theta) / 2 
                    + PID_Calc(length_pid, leg->rod.L0, chassis->leg_set);
    }
    else
        leg->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(leg->rod.theta) / 2 
                    + PID_Calc(length_pid, leg->rod.L0, chassis->leg_set);  
}

/**
 * @brief 倒地自起
 *  
 */
static void RiseL_Loop(chassis_t* chassis, Leg_t* leg)
{
    leg->rod.L0_set = 0.30f;
    leg->rod.phi0_set = 1.57f;
    InverseKinematics(leg);

    Engineer_PID_Calc(&leg->joint.jointAngle[0], leg->joint.Phi1, leg->joint.Phi1_set);
    Engineer_PID_Calc(&leg->joint.jointSpeed[0], leg->joint.d_Phi1, leg->joint.jointAngle[0].out);
    leg->joint.T1 = leg->joint.jointSpeed[0].out;
    Engineer_PID_Calc(&leg->joint.jointAngle[1], leg->joint.Phi4, leg->joint.Phi4_set);
    Engineer_PID_Calc(&leg->joint.jointSpeed[1], leg->joint.d_Phi4, leg->joint.jointAngle[1].out);
    leg->joint.T2 = leg->joint.jointSpeed[1].out;
}

/**
 * @brief 底盘控制
 * 
 * @param chassis
 * @param ins 
 * @param pid 
 */
#ifdef ControlOperation
static void ChasssisL_Control(
    chassis_t* chassis, Leg_t* leg, Excessive_t* excessive, INS_t* ins, PidTypeDef* length_pid, float lqr_k[2][6])
{
    
    ForwardKinematics(leg, excessive); //该任务控制周期是3*0.001秒
    // chassis->flag.is_take_off = chassis->flag.right_flag && chassis->flag.left_flag;

    Calc_LQR_K(lqr_k, leg->rod.L0, chassis->flag.is_take_off);

    // 状态向量更新--方便调试查看
    legL_state.theta     = -X0_OFFSET + (leg->rod.theta - 0.0f);
    legL_state.theta_dot =  X1_OFFSET + (leg->rod.d_theta - 0.0f);
    // legL_state.x         = -X2_OFFSET(chassis->leg_set) - chassis->state.x_filter;
    legL_state.x         = -X2_OFFSET - (chassis->state.x_filter - chassis->state.x_set);
    legL_state.x_dot     =  X3_OFFSET - (chassis->state.v_filter - chassis->state.v_set);
    legL_state.phi       = -X4_OFFSET + (chassis->myPithL - chassis->phi_set);
    legL_state.phi_dot   =  X5_OFFSET + (chassis->myPithGyroL - 0.0f);

    leg->rod.T = (    
                    + lqr_k[0][0] * legL_state.theta
                    + lqr_k[0][1] * legL_state.theta_dot
                    + lqr_k[0][2] * legL_state.x
                    + lqr_k[0][3] * legL_state.x_dot
                    + lqr_k[0][4] * legL_state.phi
                    + lqr_k[0][5] * legL_state.phi_dot
                    + X_INTEGRAL_KI * chassis->state.x_integral
                    );

    leg->rod.Tp = (  
                    + lqr_k[1][0] * legL_state.theta
                    + lqr_k[1][1] * legL_state.theta_dot
                    + lqr_k[1][2] * legL_state.x
                    + lqr_k[1][3] * legL_state.x_dot
                    + lqr_k[1][4] * legL_state.phi
                    + lqr_k[1][5] * legL_state.phi_dot
                    );

    leg->rod.Tp = leg->rod.Tp  + chassis->leg_tp + (1 - 2*leg->rod.L0)*OFFSET;        //髋关节输出力矩
    leg->rod.T = leg->rod.T - chassis->turn_T + chassis->bias.myPitch;          //轮毂关节输出力矩

    //输出限幅
    SATURATE(&leg->rod.T, -2.0f, 2.0f);
    if (chassis->flag.jump_flag == 1)
        SATURATE(&leg->rod.Tp, -15.0f, 15.0f);    // 髋关节力矩
    else
        SATURATE(&leg->rod.Tp, -10.0f, 10.0f);

    // leg->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(leg->rod.theta) / 2 
    //                 + PID_Calc(length_pid, leg->rod.L0, chassis->leg_set) + 0.0f;
                    // + chassis->roll_T;
                    
    JumpL_Loop(chassis, leg, length_pid);
    if (chassis->flag.jump_flag == 0)
        leg->rod.F0 = leg->rod.F0 - chassis->roll_T + leg->Fn_fa;

    // 离地判断
    // chassis->flag.left_flag = GroundDetect(chassis, leg, &periods);
    GroundDetect(chassis, leg);
    if(chassis->flag.left_flag && leg->touch_time > TOUCH_GROUND_THRESHOLD)
        chassis->flag.left_flag = 0;
    else if(!chassis->flag.left_flag && leg->take_off_time > TAKE_OFF_THRESHOLD)
        chassis->flag.left_flag = 1;
    if(chassis->flag.is_take_off == 1 && leg->take_off_time > 500)
        leg->rod.T = 0.0f;      // 主动抬车时关闭轮毂电机输出
    // if (!chassis->flag.is_take_off && leg->touch_time < 80)
    // {
    //     chassis->state.x_integral = 0.0f;

    //     chassis->state.x_filter *= 0.95f;
    //     chassis->state.v_filter *= 0.8f;

    //     SATURATE(&leg->rod.T, -0.5f, 0.5f);
    // }


    if(chassis->flag.recover_flag == 1);
        // RiseL_Loop(chassis, leg);  
                    
    SATURATE(&leg->rod.F0, -500.0f, 500.0f);

    JacobianMatrix(leg, excessive);
    
    // 髋关节输出限幅
    float torque_limit = (chassis->flag.jump_flag == 1) ? 10.0f : MAX_TORQUE;
    SATURATE(&leg->joint.T1, -torque_limit, torque_limit);
    SATURATE(&leg->joint.T2, -torque_limit, torque_limit);
}
#endif

/**
 * @brief 位控调试模式
 * 
 */
#ifdef ControlDebug
static void ChassisL_Debug(chassis_t* chassis, Leg_t* leg, Excessive_t* excessive,PidTypeDef* length_pid)
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
static void LimitChassisOutputL(chassis_t* chassis, Leg_t* leg)
{
    // 功率限制 待添加


    // 完成功率限制后进行电机参考输入设定


    DJIMotorSetRef(chassis->wheel_motor[1], FINAL_COEFFICIENT*leg->rod.T);
    // DJIMotorSetRef(chassis->wheel_motor[1], 0.0f);
}

/**
 * @brief 底盘任务
 * 
 */
void ChassisL_Task(void)
{
    #ifdef ControlOperation
    while(INS.ins_flag == 0) //等待INS初始化完成
    {
        osDelay(1);
    }
    #endif

    ChassisL_Init(&chassis_move, &legL, &LegL_pid);

    while (1)
    {
        legL.duration = xTaskGetTickCount() - legL.last_time;
        legL.last_time = xTaskGetTickCount();
        /* code */
        #ifdef ControlOperation
        ChassisL_Feedback_Update(&chassis_move, &legL, &INS, (float)CHASSIS_TIME/1000.0f);
        ChasssisL_Control(&chassis_move, &legL, &excessiveL, &INS, &LegL_pid, LQR_K_L);
        #endif
        #ifdef ControlDebug
        ChassisL_Debug(&chassis_move, &legL, &excessiveL, &LegL_pid);
        #endif
        if (chassis_move.flag.start_flag == 1)
        {
            #ifdef ControlOperation
                /* code */
                if(chassis_move.flag.recover_flag == 0 && legL.rod.recovery_flag == 0)
                {
                    // T2 为phi4角，T1为phi1角
                    Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[2].para.id, 0.0f, 0.0f, 0.0f, 0.0f, legL.joint.T1);
                    Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[3].para.id, 0.0f, 0.0f, 0.0f, 0.0f, legL.joint.T2);
                    // Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[2].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    // Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[3].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    // // 3508控制
                    LimitChassisOutputL(&chassis_move, &legL);
                    osDelay(CHASSIS_TIME);
                }
            #endif
            #ifdef ControlDebug
            Mit_Ctrl(&hfdcan1, 0x01, legL.joint.Phi4_set, 0.0f, 100.0f, 4.0f, 0.0f);
            Mit_Ctrl(&hfdcan1, 0x02, legL.joint.Phi1_set, 0.0f, 100.0f, 4.0f, 0.0f);
            osDelay(CHASSIS_TIME);
            #endif
        }
        else if (chassis_move.flag.start_flag == 0)
        {
            Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[2].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[3].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            // 3508控制
            DJIMotorSetRef(chassis_move.wheel_motor[1], 0.0f);
            osDelay(CHASSIS_TIME);
        }
        
    }
}

