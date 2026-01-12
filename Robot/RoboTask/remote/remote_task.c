#include "remote_task.h"
#include "remote_control.h"
#include "chassis_def.h"

extern chassis_t chassis_move;
static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回

void RobotCMDInit(void)
{
    rc_data = RemoteControlInit(&huart5);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
}

/**
 * @brief 机器人控制
 * 
 */
static void RemoteControlSet(void)
{

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
    RemoteControlSet();
}