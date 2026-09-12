#include "remote_task.h"
#include "remote_control.h"
#include "chassis_def.h"
#include "ins_task.h"
#include "daemon.h"
#include "kinematics.h"
#include "user_lib.h"

extern chassis_t chassis_move;
extern INS_t INS;
extern Leg_t legR;
extern Leg_t legL;

static RC_ctrl_t *rc_data;                  // 遥控器数据,初始化时返回
// static ramp_function_source_t leg_ramp;     // 腿长斜波
static uint8_t last_switch_left;            // 记录上一次拨杆状态,用于沿触发

static uint8_t RCSwitchIsValid(uint8_t sw)
{
    return sw == RC_SW_UP || sw == RC_SW_MID || sw == RC_SW_DOWN;
}

static uint8_t RemoteControlDataIsValid(const RC_ctrl_t *rc)
{
    return rc != NULL &&
           RCSwitchIsValid(rc->rc.switch_left) &&
           RCSwitchIsValid(rc->rc.switch_right) &&
           abs(rc->rc.rocker_l_) <= 660 &&
           abs(rc->rc.rocker_l1) <= 660 &&
           abs(rc->rc.rocker_r_) <= 660 &&
           abs(rc->rc.rocker_r1) <= 660 &&
           abs(rc->rc.dial) <= 660;
}

void RobotCMDInit(void)
{
    rc_data = RemoteControlInit(&huart5);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    last_switch_left = rc_data[TEMP].rc.switch_left;
    
    chassis_move.leg_set = LEG_DEFAULT;   // 初始腿长
    chassis_move.flag.start_flag = 0;
    // ramp_init(&leg_ramp, 0.005f, MAX_LEG_VEL, -MAX_LEG_VEL);
}

/**
 * @brief 机器人控制
 * 
 */
static void RemoteControlSet(chassis_t* chassis)
{
    #ifdef ControlOperation 
    while (INS.ins_flag == 0) //等待INS初始化完成
    {
        /* code */
        osDelay(1);
    }

    chassis->phi_set = 0.014f;
    chassis->reference.roll = 0.0f;
    
    // int16_t rc = rc_data[TEMP].rc.rocker_r_;
    if (abs(rc_data[TEMP].rc.rocker_r_) < 20) rc_data[TEMP].rc.rocker_r_ = 0;
    if (abs(rc_data[TEMP].rc.rocker_l_) < 20) rc_data[TEMP].rc.rocker_l_ = 0;
    if (abs(rc_data[TEMP].rc.rocker_r1) < 20) rc_data[TEMP].rc.rocker_r1 = 0;
    if (abs(rc_data[TEMP].rc.rocker_l1) < 20) rc_data[TEMP].rc.rocker_l1 = 0;

    float leg_vel = MAX_LEG_VEL * (float)(rc_data[TEMP].rc.rocker_l1) / 660.0f;               // 遥控器最大值660
    float target_vel = MAX_CHASSIS_VEL * (float)(rc_data[TEMP].rc.rocker_r1) / 660.0f;

    if (chassis->flag.start_flag == 1)
    {
        /* code */

        // 斜波控制腿长
        // float leg_vel = ramp_calc(&leg_ramp, vel);
        // chassis->leg_set += leg_vel * ((float)CHASSIS_TIME) / 1000.0f; 
        //   积分控制腿长
        chassis->leg_set += leg_vel * ((float)CHASSIS_TIME) / 1000.0f;   // 控制周期单位ms
        SATURATE(&chassis->leg_set, MIN_LEG_LENGTH, MAX_LEG_LENGTH);

        slope_following(&target_vel, &chassis->state.v_set, 0.005f);
        // chassis->state.x_set += chassis->state.v_set * ((float)CHASSIS_TIME) / 1000.0f;
        if(fabsf(chassis_move.state.v_set) > 0.02f)
            chassis_move.state.x_set = chassis_move.state.x_filter;

        // yaw轴+小陀螺
        if (rc_data[TEMP].rc.switch_right == 1){
            chassis->reference.yaw_dot = 6.0f; 
            chassis->reference.yaw = INS.YawTotal; // 切换到陀螺控制时,将yaw参考值切换到当前航向角,避免陀螺控制突变
        }
        else{
            chassis->reference.yaw_dot = 0.0f;
            chassis->reference.yaw += -(float)(rc_data[TEMP].rc.rocker_l_) / 660.0f * 0.003f; 
        }
        
        // if(rc_data[TEMP].rc.switch_left == 1)
        //     chassis->flag.recover_flag = 1; // 倒地自起标志
        // else
        //     chassis->flag.recover_flag = 0; // 倒地自起完成
        
        // 仅在拨杆切到2的瞬间触发一次跳跃,后续清零由底盘任务负责
        if (rc_data[TEMP].rc.switch_left == 2 
            && last_switch_left != 2
            && chassis->flag.jump_flag == 0)   // 避免重复触发跳跃
        {
            chassis->flag.jump_flag = 1;
            chassis->reference.yaw = INS.YawTotal; // 跳跃时将yaw参考值切换到当前航向角,避免跳跃控制突变
            chassis->reference.yaw_dot = 0.0f; 
        }
        last_switch_left = rc_data[TEMP].rc.switch_left;
    }
    else if (chassis->flag.start_flag == 0)
    {
        chassis->state.x_set = chassis->state.x_filter;   // 离线时将位置参考切换到当前位置,避免重启时位置控制突变
        chassis->state.x_integral = 0.0f;  // 停止时清零积分
        chassis->reference.yaw = INS.YawTotal; // 离线时将yaw
    }
    #endif
    
    #ifdef ControlDebug
    LengthControl(&chassis_move);
    
    // legL.rod.phi0 = PI/2.0f + 0.05 * (float)rc_data[TEMP].rc.rocker_r_ * PI/180.0f;
    legL.rod.phi0 = PI/2.0f;
    #endif
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    static uint8_t daemon_div = 0;
    if (++daemon_div >= 10u)   // 1ms任务分频到10ms
    {
        daemon_div = 0;
        DaemonTask();
    }
}

void Remote_Task(void)
{
    EmergencyHandler();

    static uint8_t lost_cnt = 0;
    static uint8_t rc_ready = 0;
    const uint8_t LOST_CONFIRM_TICKS = 3;   // 连续3次离线才真丢控
    uint8_t rc_online = RemoteControlIsOnline() && RemoteControlDataIsValid(&rc_data[TEMP]);

    if(rc_online)
    {
        lost_cnt = 0;
        if (rc_ready == 0)
        {
            last_switch_left = rc_data[TEMP].rc.switch_left;
            rc_ready = 1;
        }
        chassis_move.flag.start_flag = 1;   // 在线立即恢复
    }
    else
    {
        rc_ready = 0;
        if (lost_cnt < 255) lost_cnt++;
        if (lost_cnt >= LOST_CONFIRM_TICKS)
            chassis_move.flag.start_flag = 0;
    }

    if(rc_data[TEMP].rc.switch_right == 2)
    {
        chassis_move.flag.start_flag = 0;
        rc_ready = 0;
    }

    RemoteControlSet(&chassis_move);
}
