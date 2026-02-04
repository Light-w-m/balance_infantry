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
 * @brief 腿长控制
 *        用遥控器积分得到目标腿长，防止突变
 * 
 */
static void LengthControl(chassis_t* chassis)
{
    int16_t rc = rc_data[TEMP].rc.rocker_l_;
    if (abs(rc) < 20) rc = 0;
    float vel = MAX_LEG_VEL * (float)rc / 660.0f;               // 遥控器最大值660

    // 斜波控制腿长
    // float leg_vel = ramp_calc(&leg_ramp, vel);
    // chassis->leg_set += leg_vel * ((float)CHASSIS_TIME) / 1000.0f; 
    
    
    //   积分控制腿长
    chassis->leg_set += vel * ((float)CHASSIS_TIME) / 500.0f;   // 控制周期单位ms
    SATURATE(&chassis->leg_set, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
}

/**
 * @brief 机器人控制
 * 
 */
static void RemoteControlSet(void)
{
    #ifdef ControlOperation 

    LengthControl(&chassis_move);

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
    RemoteControlSet();
}