#include "remote_task.h"
#include "remote_control.h"
#include "chassis_def.h"
#include "ins_task.h"
#include "kinematics.h"
#include "user_lib.h"

extern chassis_t chassis_move;
extern INS_t INS;
extern Leg_t legR;
extern Leg_t legL;

static RC_ctrl_t *rc_data;                  // 遥控器数据,初始化时返回
static ramp_function_source_t leg_ramp;     // 腿长斜波

void RobotCMDInit(void)
{
    rc_data = RemoteControlInit(&huart5);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    
    chassis_move.leg_set = LEG_DEFAULT;   // 初始腿长
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
    
    // int16_t rc = rc_data[TEMP].rc.rocker_r_;
    // if (abs(rc) < 20) rc = 0;
    // float leg_vel = MAX_LEG_VEL * (float)(rc_data[TEMP].rc.rocker_r_) / 660.0f;               // 遥控器最大值660
    float target_vel = MAX_CHASSIS_VEL * (float)(rc_data[TEMP].rc.rocker_r1) / 660.0f;
    chassis->reference.wz = 0;

    if (chassis->flag.start_flag == 1)
    {
        /* code */

        // 斜波控制腿长
        // float leg_vel = ramp_calc(&leg_ramp, vel);
        // chassis->leg_set += leg_vel * ((float)CHASSIS_TIME) / 1000.0f; 
        
        //   积分控制腿长
        // chassis->leg_set += leg_vel * ((float)CHASSIS_TIME) / 500.0f;   // 控制周期单位ms
        SATURATE(&chassis->leg_set, MIN_LEG_LENGTH, MAX_LEG_LENGTH);

        slope_following(&target_vel, &chassis->state.v_set, 0.005f);
        // chassis->state.x_set += chassis->state.v_set * ((float)CHASSIS_TIME) / 500.0f;
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

}

void Remote_Task(void)
{
    chassis_move.flag.start_flag = 1;
    RemoteControlSet(&chassis_move);
}