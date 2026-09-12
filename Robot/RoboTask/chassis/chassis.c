/**
 * @file chassis.c
 * @author your name (you@domain.com)
 * @brief 
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

#define chassis_all

#ifdef chassis_all

extern INS_t INS;
chassis_t chassis_move;
extern Leg_t legR;
extern Leg_t legL;

Excessive_t excessiveR;
Excessive_t excessiveL;
LegState_t legR_state;
LegState_t legL_state;

PidTypeDef Leg_pid;    //腿长PID
PidTypeDef Turn_Pid;    //转向pd
PidTypeDef Tp_Pid;      //防劈叉补偿pd
PidTypeDef Roll_Pid;    //横滚角补偿pd

float LQR_K_R[2][6] = {0};
float LQR_K_L[2][6] = {0};

static void Jump_Loop(chassis_t* chassis, Leg_t* legR, Leg_t* legL);
static void Rise_Loop(chassis_t* chassis, Leg_t* legR, Leg_t* legL);

/**
 * @brief 底盘初始化,包括PID参数初始化，电机初始化
 * 
 * @param chassis 
 * @param pid 
 */
static void Chassis_Init(chassis_t* chassis, Leg_t* legR, Leg_t* legL)
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
        .current_feedforward_ptr = 0,
        },
        .motor_type = M3508,
    };
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
        .current_feedforward_ptr = 0,
        },
        .motor_type = M3508,
    };

    const static float leg_pid[3] = {LEG_PID_KP, LEG_PID_KI, LEG_PID_KD}; //P I D
    PID_init(&legR->leg_pid, PID_POSITION, leg_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);
    PID_init(&legL->leg_pid, PID_POSITION, leg_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);

    joint_motor_init(&chassis->joint_motor[0], 4, MIT_MODE);
    joint_motor_init(&chassis->joint_motor[1], 3, MIT_MODE);
    joint_motor_init(&chassis->joint_motor[2], 1, MIT_MODE);
    joint_motor_init(&chassis->joint_motor[3], 2, MIT_MODE);

    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_configR.can_init_config.tx_id = 2;
    chassis_motor_configR.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis->wheel_motor[0] = DJIMotorInit(&chassis_motor_configR);
    chassis_motor_configL.can_init_config.tx_id = 1;
    chassis_motor_configL.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    chassis->wheel_motor[1] = DJIMotorInit(&chassis_motor_configL);

    for(uint8_t i = 0; i < 2; i++)
    {
        Engineer_PID_Init(&legR->joint.jointAngle[i], Feedforward_PID, JOINT_Angle_KP,JOINT_Angle_KI,JOINT_Angle_KD,JOINT_Angle_KF,JOINT_Angle_I_BAND,JOINT_Angle_DT,JOINT_Angle_MAX_OUT,JOINT_Angle_MAX_IOUT);
        Engineer_PID_Init(&legR->joint.jointSpeed[i], Normal_PID, JOINT_Speed_KP,JOINT_Speed_KI,JOINT_Speed_KD,JOINT_Speed_KF,JOINT_Speed_I_BAND,JOINT_Speed_DT,JOINT_Speed_MAX_OUT,JOINT_Speed_MAX_IOUT);
        
        Engineer_PID_Init(&legL->joint.jointAngle[i], Feedforward_PID, JOINT_Angle_KP,JOINT_Angle_KI,JOINT_Angle_KD,JOINT_Angle_KF,JOINT_Angle_I_BAND,JOINT_Angle_DT,JOINT_Angle_MAX_OUT,JOINT_Angle_MAX_IOUT);
        Engineer_PID_Init(&legL->joint.jointSpeed[i], Normal_PID, JOINT_Speed_KP,JOINT_Speed_KI,JOINT_Speed_KD,JOINT_Speed_KF,JOINT_Speed_I_BAND,JOINT_Speed_DT,JOINT_Speed_MAX_OUT,JOINT_Speed_MAX_IOUT);
    }

    for (uint8_t i = 0; i < 3; i++)
    {
        /* code */
        Enable_Motor_Mode(&hfdcan1, chassis->joint_motor[0].para.id, chassis->joint_motor[0].mode);
        Enable_Motor_Mode(&hfdcan1, chassis->joint_motor[1].para.id, chassis->joint_motor[1].mode);
        DJIMotorEnable(chassis->wheel_motor[0]);
        
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
static float theta_biasR = 0.0f;
static float theta_biasL = 0.0f;
static void Chassis_Feedback_Update(chassis_t* chassis,Leg_t* legR, Leg_t* legL, INS_t* ins, float dt)
{
    chassis->body.pitch = ins->Pitch;
    chassis->body.pitch_dot = ins->Gyro[Y_AXIS];
    chassis->body.yaw = ins->Yaw;
    chassis->body.yaw_dot = ins->Gyro[Z_AXIS];
    chassis->body.yaw_total = ins->YawTotal;
    chassis->body.roll = ins->Roll;
    
    chassis->myPithR = ins->Pitch;
    chassis->myPithGyroR = ins->Gyro[Y_AXIS];
    chassis->myPithL = -ins->Pitch;
    chassis->myPithGyroL = -ins->Gyro[Y_AXIS];
    
    legR->joint.Phi4 = PI/2.0f + chassis->joint_motor[1].para.pos;
    legR->joint.Phi1 = PI/2.0f + chassis->joint_motor[0].para.pos;
    legR->joint.d_Phi4 = chassis->joint_motor[1].para.vel;
    legR->joint.d_Phi1 = chassis->joint_motor[0].para.vel;

    legL->joint.Phi4 = PI/2.0f + chassis->joint_motor[3].para.pos;
    legL->joint.Phi1 = PI/2.0f + chassis->joint_motor[2].para.pos;
    legL->joint.d_Phi4 = chassis->joint_motor[3].para.vel;
    legL->joint.d_Phi1 = chassis->joint_motor[2].para.vel;

    SATURATE(&legR->joint.Phi4, -PI/3.0f, PI/2.0f);
    SATURATE(&legR->joint.Phi1, PI/2.0f, PI*4/3.0f);
    SATURATE(&legL->joint.Phi4, -PI/3.0f, PI/2.0f);
    SATURATE(&legL->joint.Phi1, PI/2.0f, PI*4/3.0f);
    
    legR->rod.theta = PI/2 - legR->rod.phi0 - chassis->myPithR + theta_biasR;
    legL->rod.theta = PI/2 - legL->rod.phi0 - chassis->myPithL + theta_biasL;
    legR->rod.d_theta = - legR->rod.d_phi0 - chassis->myPithGyroR;
    legL->rod.d_theta = - legL->rod.d_phi0 - chassis->myPithGyroL;

    Leg_t* legs[2] = {legR, legL};
    for (uint8_t i = 0; i < 2; i++)
    {
        // L0
        legs[i]->rod.d_L0 = legs[i]->j11*legs[i]->joint.d_Phi1 + legs[i]->j12*legs[i]->joint.d_Phi4;
        legs[i]->rod.dd_L0 = (legs[i]->rod.d_L0 - legs[i]->rod.last_d_L0) / dt;    
        legs[i]->rod.last_d_L0 = legs[i]->rod.d_L0;

        // phi0
        legs[i]->rod.d_phi0 = legs[i]->j21*legs[i]->joint.d_Phi1 + legs[i]->j22*legs[i]->joint.d_Phi4;

        //theta
        legs[i]->rod.dd_theta = (legs[i]->rod.d_theta - legs[i]->rod.last_d_theta) / dt;
        legs[i]->rod.last_d_theta = legs[i]->rod.d_theta;
    }
    
    chassis->bias.theta_err = 0.0f - (legR->rod.theta + legL->rod.theta); 
    // 自适应重心误差补偿
    chassis->bias.error = ins->Pitch - chassis->bias.myPitch;
    if(fabsf(chassis->bias.error) < 0.02f)  // 1度
        chassis->bias.error = 0.0f;
    if(fabsf(ins->Pitch) < 0.1f) // 6度
        chassis->bias.myPitch += Pitch_ki * chassis->bias.error * dt;
    SATURATE(&chassis->bias.myPitch, -0.05f, 0.05f);
}

/**
 * @brief 底盘控制
 * 
 * @param chassis
 * @param ins 
 * @param pid 
 */
static float LQR_Compute(const float k_row[6], LegState_t *s)
{
    const float v[6] = {s->theta, s->theta_dot, s->x, s->x_dot, s->phi, s->phi_dot};
    float out = 0.0f;
    for (int i = 0; i < 6; i++) {
        out += k_row[i] * v[i];
    }
    return out;

    // 等价为
    // leg->rod.T = (    
    //                 + lqr_k[0][0] * legR_state.theta
    //                 + lqr_k[0][1] * legR_state.theta_dot
    //                 + lqr_k[0][2] * legR_state.x
    //                 + lqr_k[0][3] * legR_state.x_dot
    //                 + lqr_k[0][4] * legR_state.phi
    //                 + lqr_k[0][5] * legR_state.phi_dot
    //                 + X_INTEGRAL_KI * chassis->state.x_integral
    //                 );
    // leg->rod.Tp = (   
    //                 + lqr_k[1][0] * legR_state.theta
    //                 + lqr_k[1][1] * legR_state.theta_dot
    //                 + lqr_k[1][2] * legR_state.x
    //                 + lqr_k[1][3] * legR_state.x_dot
    //                 + lqr_k[1][4] * legR_state.phi
    //                 + lqr_k[1][5] * legR_state.phi_dot
    //                 );
}

static void Chasssis_Control(
    chassis_t* chassis, Leg_t* legR, Leg_t* legL, Excessive_t* excessiveR, Excessive_t* excessiveL,
    INS_t* ins, float lqr_k_R[2][6], float lqr_k_L[2][6])
{
    chassis->flag.is_take_off = chassis->flag.right_flag || chassis->flag.left_flag;

    ForwardKinematics(legR, excessiveR); 
    ForwardKinematics(legL, excessiveL);

    Calc_LQR_K(lqr_k_R, legR->rod.L0, chassis->flag.is_take_off);
    Calc_LQR_K(lqr_k_L, legL->rod.L0, chassis->flag.is_take_off);

    if (fabsf(chassis->reference.yaw_dot) > 0.01f) {
        // 小陀螺：角速度闭环
        chassis->turn_T = PID_Calc(&Turn_Pid, chassis->body.yaw_dot, chassis->reference.yaw_dot);
    } else {
        chassis->turn_T = Turn_Pid.Kp * (chassis->reference.yaw - chassis->body.yaw_total)
                        - Turn_Pid.Kd * chassis->body.yaw_dot;
    }
    SATURATE(&chassis->turn_T, -TURN_PID_MAX_OUT, TURN_PID_MAX_OUT);

    chassis->roll_T = PID_Calc(&Roll_Pid, chassis->body.roll, chassis->reference.roll);
    chassis->leg_tp = PID_Calc(&Tp_Pid, chassis->bias.theta_err, 0.0f);

    // 状态向量更新--方便调试查看
    legR_state.theta     = X0_OFFSET + (legR->rod.theta - 0.0f);
    legR_state.theta_dot = X1_OFFSET + (legR->rod.d_theta - 0.0f);
    // legR_state.x         = X2_OFFSET(chassis->leg_set) + chassis->state.x_filter;
    legR_state.x         = X2_OFFSET + (chassis->state.x_filter - chassis->state.x_set);
    legR_state.x_dot     = X3_OFFSET + (chassis->state.v_filter - chassis->state.v_set);
    legR_state.phi       = X4_OFFSET + (chassis->myPithR + chassis->phi_set);
    legR_state.phi_dot   = X5_OFFSET + (chassis->myPithGyroR - 0.0f);

    legL_state.theta     = -X0_OFFSET + (legL->rod.theta - 0.0f);
    legL_state.theta_dot =  X1_OFFSET + (legL->rod.d_theta - 0.0f);
    // legL_state.x         = -X2_OFFSET(chassis->leg_set) - chassis->state.x_filter;
    legL_state.x         = -X2_OFFSET - (chassis->state.x_filter - chassis->state.x_set);
    legL_state.x_dot     =  X3_OFFSET - (chassis->state.v_filter - chassis->state.v_set);
    legL_state.phi       = -X4_OFFSET + (chassis->myPithL - chassis->phi_set);
    legL_state.phi_dot   =  X5_OFFSET + (chassis->myPithGyroL - 0.0f);
    
    chassis->state.x_error = chassis->state.x_filter - chassis->state.x_set;
    chassis->state.x_integral += chassis->state.x_error * ((float)CHASSIS_TIME/1000.0f);
    SATURATE(&chassis->state.x_integral, -X_INTEGRAL_LIMIT, X_INTEGRAL_LIMIT);

    {
        Leg_t* legs[2] = {legR, legL};
        LegState_t* states[2] = {&legR_state, &legL_state};
        float (*lqr_sets[2])[6] = {lqr_k_R, lqr_k_L};

        for (int i = 0; i < 2; i++) {
            Leg_t* leg = legs[i];
            LegState_t* st = states[i];
            float (*k)[6] = lqr_sets[i];

            leg->rod.T  = LQR_Compute(k[0], st) + X_INTEGRAL_KI * chassis->state.x_integral;
            leg->rod.Tp = LQR_Compute(k[1], st);

            leg->rod.Tp = leg->rod.Tp + chassis->leg_tp + (1 - 2 * leg->rod.L0) * OFFSET;
            leg->rod.T  = leg->rod.T - chassis->turn_T + chassis->bias.myPitch;

            SATURATE(&leg->rod.T, -2.0f, 2.0f);
            if (chassis->flag.jump_flag == 1)
                SATURATE(&leg->rod.Tp, -15.0f, 15.0f);
            else
                SATURATE(&leg->rod.Tp, -10.0f, 10.0f);
        }
    }

    if (chassis->flag.jump_flag == 1)
        Jump_Loop(chassis, legR, legL);
    else
    {
        legR->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(legR->rod.theta) / 2 
                    + PID_Calc(&legR->leg_pid, legR->rod.L0, chassis->leg_set) + 2.0f;
        legL->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(legL->rod.theta) / 2 
                    + PID_Calc(&legL->leg_pid, legL->rod.L0, chassis->leg_set) + 2.0f;
    }
    HighSpeedTurn(chassis,legR->rod.L0, legL->rod.L0, &legR->Fn_fa, &legL->Fn_fa);
    if ((chassis->flag.jump_flag == 0) || (chassis->flag.right_flag == 0))
        legR->rod.F0 = legR->rod.F0 + chassis->roll_T + legR->Fn_fa;
    if ((chassis->flag.jump_flag == 0) || (chassis->flag.left_flag == 0))
        legL->rod.F0 = legL->rod.F0 - chassis->roll_T + legL->Fn_fa;
    
    SATURATE(&legR->rod.F0, 0.0f, 100.0f);
    SATURATE(&legL->rod.F0, 0.0f, 100.0f);

    {
        Leg_t* legs[2] = {legR, legL};
        uint8_t* flags[2] = {&chassis->flag.right_flag, &chassis->flag.left_flag};
        for (int i = 0; i < 2; i++) {
            Leg_t* lg = legs[i];
            uint8_t* fg = flags[i];
            GroundDetect(chassis, lg);
            if (*fg && lg->touch_time > TOUCH_GROUND_THRESHOLD)
                *fg = 0;
            else if (!*fg && lg->take_off_time > TAKE_OFF_THRESHOLD)
                *fg = 1;
            if (chassis->flag.is_take_off == 1 && lg->take_off_time > 500)
                lg->rod.T = 0.0f; // 主动抬车时关闭轮毂电机输出
        }
    }
    
    JacobianMatrix(legR, excessiveR);
    JacobianMatrix(legL, excessiveL);

    // 髋关节输出限幅
    float torque_limit = (chassis->flag.jump_flag == 1) ? 10.0f : MAX_TORQUE;
    SATURATE(&legR->joint.T1, -torque_limit, torque_limit);
    SATURATE(&legR->joint.T2, -torque_limit, torque_limit);
    SATURATE(&legL->joint.T1, -torque_limit, torque_limit);
    SATURATE(&legL->joint.T2, -torque_limit, torque_limit);
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput(chassis_t* chassis, Leg_t* legR, Leg_t* legL)
{
    // 功率限制 待添加


    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(chassis->wheel_motor[0], -FINAL_COEFFICIENT*legR->rod.T);
    DJIMotorSetRef(chassis->wheel_motor[1], FINAL_COEFFICIENT*legL->rod.T);
    // DJIMotorSetRef(chassis->wheel_motor[0], 0.0f);
    // DJIMotorSetRef(chassis->wheel_motor[1], 0.0f);
}


/**
 * @brief 底盘任务
 * 
 */
void Chassis_Task(void)
{
    while(INS.ins_flag == 0) //等待INS初始化完成
    {
        osDelay(1);
    }

    Chassis_Init(&chassis_move, &legR, &legL);

    while (1)
    {
        // /* code */
        Chassis_Feedback_Update(&chassis_move, &legR, &legL, &INS, (float)CHASSIS_TIME/1000.0f);
        Chasssis_Control(&chassis_move, &legR, &legL, &excessiveR, &excessiveL, &INS, LQR_K_R, LQR_K_L);        
        if (chassis_move.flag.start_flag == 1)
        {
            if(chassis_move.flag.recover_flag == 0)
            {
                Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f, legR.joint.T1);
                Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[1].para.id, 0.0f, 0.0f, 0.0f, 0.0f, legR.joint.T2);
                Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[2].para.id, 0.0f, 0.0f, 0.0f, 0.0f, legL.joint.T1);
                Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[3].para.id, 0.0f, 0.0f, 0.0f, 0.0f, legL.joint.T2);
                // Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                // Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[1].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                // Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[2].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                // Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[3].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                // // 3508控制
                LimitChassisOutput(&chassis_move, &legR, &legL);
                osDelay(CHASSIS_TIME);
            }
        }
        else if (chassis_move.flag.start_flag == 0)
        {
            Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[1].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[2].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            Mit_Ctrl(&hfdcan1, chassis_move.joint_motor[3].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            // 3508控制
            DJIMotorSetRef(chassis_move.wheel_motor[0], 0.0f);
            DJIMotorSetRef(chassis_move.wheel_motor[1], 0.0f);

            osDelay(CHASSIS_TIME);
        }
    }
}

/**
 * @brief 跳跃控制
 * 
 */
static void Jump_Loop(chassis_t* chassis, Leg_t* legR, Leg_t* legL)
{
    if (chassis->step.jump_step == NORMAL_STEP)
    {
        // 先收腿
        legR->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(legR->rod.theta) / 2 
                + PID_Calc(&legR->leg_pid, legR->rod.L0, MIN_LEG_LENGTH);
        legL->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(legL->rod.theta) / 2 
                + PID_Calc(&legL->leg_pid, legL->rod.L0, MIN_LEG_LENGTH);
        if ((legR->rod.L0 + legL->rod.L0) / 2 < 0.18f) chassis->step.jump_step_time++;
        if (chassis->step.jump_step_time > 100)
        {
            chassis->step.jump_step_time = 0;
            chassis->step.jump_step = JUMP_STEP_SQUST;
        }
    }
    else if (chassis->step.jump_step == JUMP_STEP_SQUST)
    {
        // 给一个大F0起跳
        legR->rod.F0 = 60.0f;
        legL->rod.F0 = 60.0f;
        if ((legR->rod.L0 + legL->rod.L0) / 2 > 0.22f) chassis->step.jump_step_time++;
        if (chassis->step.jump_step_time > 40)
        {
            chassis->step.jump_step_time = 0;
            chassis->step.jump_step = JUMP_STEP_JUMP;
        }
    }
    else if (chassis->step.jump_step == JUMP_STEP_JUMP)
    {
        // 跳跃阶段快速伸腿
        legR->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(legR->rod.theta) / 2 
                + PID_Calc(&legR->leg_pid, legR->rod.L0, MIN_LEG_LENGTH);
        legL->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(legL->rod.theta) / 2 
                + PID_Calc(&legL->leg_pid, legL->rod.L0, MIN_LEG_LENGTH);
        if ((legR->rod.L0 + legL->rod.L0) / 2 < 0.20f) chassis->step.jump_step_time++;
        if (chassis->step.jump_step_time > 20)
        {
            chassis->step.jump_step_time = 0;
            chassis->step.jump_step = JUMP_STEP_RECOVERY;
        }
    }
    else 
        legR->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(legR->rod.theta) / 2 
                + PID_Calc(&legR->leg_pid, legR->rod.L0, chassis->leg_set);
        legL->rod.F0 = BODY_MASS * GRAVITY / arm_cos_f32(legL->rod.theta) / 2 
                + PID_Calc(&legL->leg_pid, legL->rod.L0, chassis->leg_set);
    if (chassis->step.jump_step == JUMP_STEP_RECOVERY)
    {
        chassis->flag.jump_flag = 0;
        chassis->step.jump_step = NORMAL_STEP;
        chassis->step.jump_step_time = 0;
    }
}

/**
 * @brief 倒地自起
 *  
 */
static void Rise_Loop(chassis_t* chassis, Leg_t* legR, Leg_t* legL)
{
    legR->rod.L0_set = 0.30f;
    legR->rod.phi0_set = 1.57f;
    legL->rod.L0_set = 0.30f;
    legL->rod.phi0_set = 1.57f;

    InverseKinematics(legR);
    InverseKinematics(legL);

    Leg_t* legs[2] = {legR, legL};
    for (uint8_t i = 0; i < 2; i++)
    {
        /* code */
        Engineer_PID_Calc(&legs[i]->joint.jointAngle[0], legs[i]->joint.Phi1, legs[i]->joint.Phi1_set);
        Engineer_PID_Calc(&legs[i]->joint.jointSpeed[0], legs[i]->joint.d_Phi1, legs[i]->joint.jointAngle[0].out);
        legs[i]->joint.T1 = legs[i]->joint.jointSpeed[0].out;
        Engineer_PID_Calc(&legs[i]->joint.jointAngle[1], legs[i]->joint.Phi4, legs[i]->joint.Phi4_set);
        Engineer_PID_Calc(&legs[i]->joint.jointSpeed[1], legs[i]->joint.d_Phi4, legs[i]->joint.jointAngle[1].out);
        legs[i]->joint.T2 = legs[i]->joint.jointSpeed[1].out;
    }
}

#endif