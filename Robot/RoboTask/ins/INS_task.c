/**
  *********************************************************************
  * @file      ins_task.c/h
  * @brief     该任务是用mahony方法获取机体姿态，同时获取机体在绝对坐标系下的运动加速度
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "INS_task.h"
#include "QuaternionEKF.h"
#include "bsp_dwt.h"
#include "mahony_filter.h"
#include "gpio.h"
#include "pid.h"
#include "spi.h"
#include "tim.h"
#include "cmsis_os.h"

INS_t INS;
extern IMU_Data_t BMI088;
PidTypeDef imu_temp_pid;

struct MAHONY_FILTER_t mahony;
Axis3f Gyro,Accel;
float gravity[3] = {0, 0, 9.81f};

uint32_t INS_DWT_Count = 0;
float ins_dt = 0.0f;
float ins_time;
int stop_time;

// extern osSemaphoreId imuBinarySem01Handle;
float temp;
uint8_t forceStop = 0;

void INS_Init(void)
{ 
	mahony_init(&mahony,1.0f,0.0f,0.001f);
   	INS.AccelLPF = 0.0089f;
}

void INS_task(void)
{
	 INS_Init();
	 
	 while(1)
	 {  
		ins_dt = DWT_GetDeltaT(&INS_DWT_Count);
    
		mahony.dt = ins_dt;

		BMI088_Read(&BMI088);

		INS.Accel[X_AXIS] = BMI088.Accel[X_AXIS];
		INS.Accel[Y_AXIS] = BMI088.Accel[Y_AXIS];
		INS.Accel[Z_AXIS] = BMI088.Accel[Z_AXIS];

		Accel.x=BMI088.Accel[0];
		Accel.y=BMI088.Accel[1];
		Accel.z=BMI088.Accel[2];

		INS.Gyro[X_AXIS] = BMI088.Gyro[X_AXIS];
		INS.Gyro[Y_AXIS] = BMI088.Gyro[Y_AXIS];
		INS.Gyro[Z_AXIS] = BMI088.Gyro[Z_AXIS];

		Gyro.x=BMI088.Gyro[0];
		Gyro.y=BMI088.Gyro[1];
		Gyro.z=BMI088.Gyro[2];

		//核心函数，EKF更新四元数
		// IMU_QuaternionEKF_Update(INS.Gyro[X_AXIS], INS.Gyro[Y_AXIS], INS.Gyro[Z_AXIS], INS.Accel[X_AXIS], INS.Accel[Y_AXIS], INS.Accel[Z_AXIS], mahony.dt);

		mahony_input(&mahony,Gyro,Accel);
		mahony_update(&mahony);
		mahony_output(&mahony);
		RotationMatrix_update(&mahony);
					
		INS.q[0]=mahony.q0;
		INS.q[1]=mahony.q1;
		INS.q[2]=mahony.q2;
		INS.q[3]=mahony.q3;
		
		// 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
		float gravity_b[3];
		EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
		for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
		{
			INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * ins_dt / (INS.AccelLPF + ins_dt) 
															+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + ins_dt); 
	//			INS.MotionAccel_b[i] = (INS.Accel[i] ) * dt / (INS.AccelLPF + dt) 
	//														+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);			
		}
		BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n
		
		//死区处理
		if(fabsf(INS.MotionAccel_n[0])<0.02f)
		{
			INS.MotionAccel_n[0]=0.0f;	//x轴
		}
		if(fabsf(INS.MotionAccel_n[1])<0.02f)
		{
			INS.MotionAccel_n[1]=0.0f;	//y轴
		}
		if(fabsf(INS.MotionAccel_n[2])<0.04f)
		{
			INS.MotionAccel_n[2]=0.0f;//z轴
		}
		
		if(ins_time>3000.0f)
		{
			INS.v_n=INS.v_n+INS.MotionAccel_n[1]*0.001f;
		  	INS.x_n=INS.x_n+INS.v_n*0.001f;
			INS.ins_flag=1;//四元数基本收敛，加速度也基本收敛，可以开始底盘任务
			// 获取最终数据
			INS.Pitch=mahony.roll*180.0f/PI;
			INS.Roll=mahony.pitch*180.0f/PI;
			INS.Yaw=mahony.yaw*180.0f/PI;
		
			//INS.YawTotalAngle=INS.YawTotalAngle+INS.Gyro[2]*0.001f;
			
			if (INS.Yaw - INS.YawAngleLast > 3.1415926f)
			{
					INS.YawRoundCount--;
			}
			else if (INS.Yaw - INS.YawAngleLast < -3.1415926f)
			{
					INS.YawRoundCount++;
			}
			INS.YawTotalAngle = (6.283f* INS.YawRoundCount + INS.Yaw) * 180.0f / PI;
			INS.YawAngleLast = INS.Yaw*180.0f/PI;
		}
		else
		{
		ins_time++;
		}
			
		osDelay(1);
	}
} 

/**
************************************************************************
* @brief:      	IMU_TempCtrlTask(void const * argument)
* @param:       argument - 任务参数
* @retval:     	void
* @details:    	IMU温度控制任务函数
************************************************************************
**/
// void IMU_TempCtrlTask(void const * argument)
// {
// 	osDelay(500);
//     HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
// 	while(BMI088_init(&hspi2,2))
// 	{

// 	}

// 	PID_init(&imu_temp_pid, PID_DELTA, (float[]){KP,KI,KD}, MAX_OUT, 0.0f);

// 	for(;;)
// 	{
// 		osSemaphoreWait(imuBinarySem01Handle, osWaitForever);

// 		BMI088_Read(&BMI088);
// 		temp = BMI088.Temperature;

// 		float out = PID_Calc(&imu_temp_pid, temp, DES_TEMP);

// 		if (out > MAX_OUT) out = MAX_OUT;
//         if (out < 0) out = 0.f;
        
//         if (forceStop == 1)
//         {
//             out = 0.0f;
//         }

// 		htim3.Instance->CCR4 = (uint16_t)out;
// 	}
// }


/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}




