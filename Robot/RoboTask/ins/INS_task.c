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
	
#include "ins_task.h"
#include "Robot_def.h"
#include "hipnuc_dec.h"
#include "controller.h"
#include "QuaternionEKF.h"
#include "bsp_dwt.h"
#include "user_lib.h"
#include "mahony_filter.h"
#include "gpio.h"
#include "pid.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "stdint.h"
#include "cmsis_os.h"

INS_t INS;

struct MAHONY_FILTER_t mahony;
Axis3f Gyro,Accel;
float gravity[3] = {0, 0, 9.81f};

uint32_t INS_DWT_Count = 0;
float ins_dt = 0.0f;
float ins_time;

/**
 * @brief BMI088控制
 * 
 */
#ifdef INS_OF_BMI088

extern IMU_Data_t BMI088;
static PIDInstance TempCtrl = {0};
static float RefTemp = 40; // 恒温设定温度

static void IMUPWMSet(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwm);
}

/**
 * @brief 温度控制
 *
 */
static void IMU_Temperature_Ctrl(void)
{
    PIDCalculate(&TempCtrl, BMI088.Temperature, RefTemp);
    IMUPWMSet(float_constrain(float_rounding(TempCtrl.Output), 0, UINT32_MAX));
}

// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
static void InitQuaternion(float *init_q4)
{
    float acc_init[3] = {0};
    float gravity_norm[3] = {0, 0, 1}; // 导航系重力加速度矢量,归一化后为(0,0,1)
    float axis_rot[3] = {0};           // 旋转轴
    // 读取100次加速度计数据,取平均值作为初始值
    for (uint8_t i = 0; i < 100; ++i)
    {
        BMI088_Read(&BMI088);
        acc_init[X_AXIS] += BMI088.Accel[X_AXIS];
        acc_init[Y_AXIS] += BMI088.Accel[Y_AXIS];
        acc_init[Z_AXIS] += BMI088.Accel[Z_AXIS];
        DWT_Delay(0.001);
    }
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] /= 100;
    Norm3d(acc_init);
    // 计算原始加速度矢量和导航系重力加速度矢量的夹角
    float angle = acosf(Dot3d(acc_init, gravity_norm));
    Cross3d(acc_init, gravity_norm, axis_rot);
    Norm3d(axis_rot);
    init_q4[0] = cosf(angle / 2.0f);
    for (uint8_t i = 0; i < 2; ++i)
        init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f); // 轴角公式,第三轴为0(没有z轴分量)
}

void INS_Init(void)
{ 
	while (BMI088_init(&hspi2,2) != BMI088_NO_ERROR)
    {
        /* code */
    }
	mahony_init(&mahony,1.0f,0.0f,0.001f);
   	INS.AccelLPF = 0.0089f;

	float init_quaternion[4] = {0};
    InitQuaternion(init_quaternion);
    IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);
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
		IMU_QuaternionEKF_Update(INS.Gyro[X_AXIS], INS.Gyro[Y_AXIS], INS.Gyro[Z_AXIS], INS.Accel[X_AXIS], INS.Accel[Y_AXIS], INS.Accel[Z_AXIS], mahony.dt);

		mahony_input(&mahony,Gyro,Accel);
		mahony_update(&mahony);
		mahony_output(&mahony);
		RotationMatrix_update(&mahony);
					
		INS.q[0]=mahony.q0;
		INS.q[1]=mahony.q1;
		INS.q[2]=mahony.q2;
		INS.q[3]=mahony.q3;

		memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));
		
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
		
		if(ins_time>2000.0f)
		{
			INS.v_n=INS.v_n+INS.MotionAccel_n[1]*0.001f;
		  	INS.x_n=INS.x_n+INS.v_n*0.001f;
			INS.ins_flag=1;//四元数基本收敛，加速度也基本收敛，可以开始底盘任务
			// 获取最终数据
			INS.Pitch=mahony.roll*180.0f/PI;
			INS.Roll=mahony.pitch*180.0f/PI;
			INS.Yaw=mahony.yaw*180.0f/PI;
			// INS.Pitch=mahony.roll;
			// INS.Roll=mahony.pitch;
			// INS.Yaw=mahony.yaw;
		
			INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
		}
		else
		{
		ins_time++;
		}
			
		osDelay(1);
		// IMU_Temperature_Ctrl();
	}
} 
#endif

/**
 * @brief hi05控制
 * 
 */
#ifdef INS_OF_HIPNUC

hipnuc_raw_t HIPNUC;

uint8_t uart_rx_buf[1024];
uint16_t uart_rx_index = 0;
uint8_t new_data_flag = 0;
uint8_t rx_byte;

static void AcquireData(void);

// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
static void InitQuaternion(float *init_q4)
{
    float acc_init[3] = {0};
    float gravity_norm[3] = {0, 0, 1}; // 导航系重力加速度矢量,归一化后为(0,0,1)
    float axis_rot[3] = {0};           // 旋转轴
    // 读取100次加速度计数据,取平均值作为初始值
    for (uint8_t i = 0; i < 100; ++i)
    {
		AcquireData();
        acc_init[X_AXIS] += HIPNUC.hi91.acc[X_AXIS];
        acc_init[Y_AXIS] += HIPNUC.hi91.acc[Y_AXIS];
        acc_init[Z_AXIS] += HIPNUC.hi91.acc[Z_AXIS];
        DWT_Delay(0.001);
    }
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] /= 100;
    Norm3d(acc_init);
    // 计算原始加速度矢量和导航系重力加速度矢量的夹角
    float angle = acosf(Dot3d(acc_init, gravity_norm));
    Cross3d(acc_init, gravity_norm, axis_rot);
    Norm3d(axis_rot);
    init_q4[0] = cosf(angle / 2.0f);
    for (uint8_t i = 0; i < 2; ++i)
        init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f); // 轴角公式,第三轴为0(没有z轴分量)
}

void HIPNUC_Init(void)
{
	memset(&HIPNUC, 0, sizeof(hipnuc_raw_t));
	new_data_flag = 0;

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

static void INS_Init(void)
{
	mahony_init(&mahony,1.0f,0.0f,0.001f);
   	INS.AccelLPF = 0.0089f;

	float init_quaternion[4] = {0};
	InitQuaternion(init_quaternion);
    IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);
}

static void AcquireData(void)
{
  char log_buf[1024];
  
  if (new_data_flag)
  {
    /* code */
    for (uint16_t i = 0; i < uart_rx_index; i++)
    {
      /* code */
      if (HipnucInput(uart_rx_buf[i], &HIPNUC))
      {
        /* code */
        HipnucDumpPacket(&HIPNUC, log_buf, sizeof(log_buf));
      }
    }
    new_data_flag = 0;
    uart_rx_index = 0;
  }
  
}

void INS_task(void)
{
	INS_Init();

	while (1)
	{
		/* code */
		ins_dt = DWT_GetDeltaT(&INS_DWT_Count);
    
		mahony.dt = ins_dt;

		AcquireData();

		INS.Accel[X_AXIS] = HIPNUC.hi91.acc[X_AXIS];
		INS.Accel[Y_AXIS] = HIPNUC.hi91.acc[Y_AXIS];
		INS.Accel[Z_AXIS] = HIPNUC.hi91.acc[Z_AXIS];

		Accel.x = HIPNUC.hi91.acc[X_AXIS];
		Accel.y = HIPNUC.hi91.acc[Y_AXIS];
		Accel.z = HIPNUC.hi91.acc[Z_AXIS];

		INS.Gyro[X_AXIS] = HIPNUC.hi91.gyro[X_AXIS];
		INS.Gyro[Y_AXIS] = HIPNUC.hi91.gyro[Y_AXIS];
		INS.Gyro[Z_AXIS] = HIPNUC.hi91.gyro[Z_AXIS];

		Gyro.x = HIPNUC.hi91.gyro[X_AXIS];
		Gyro.y = HIPNUC.hi91.gyro[Y_AXIS];
		Gyro.z = HIPNUC.hi91.gyro[Z_AXIS];

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

		if(ins_time>2000.0f)
		{
			INS.v_n = INS.v_n+INS.MotionAccel_n[1]*0.001f;
		  	INS.x_n = INS.x_n+INS.v_n*0.001f;
			INS.ins_flag = 1;//四元数基本收敛，加速度也基本收敛，可以开始底盘任务
			// 获取最终数据
			// INS.Pitch = mahony.roll*180.0f/PI;
			// INS.Roll = mahony.pitch*180.0f/PI;
			// INS.Yaw = mahony.yaw*180.0f/PI;
		
			INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
		}
		else
		{
		ins_time++;
		}

		INS.Pitch = HIPNUC.hi91.pitch;
		INS.Roll = HIPNUC.hi91.roll;
		INS.Yaw = HIPNUC.hi91.yaw;
			
		osDelay(1);
	}
	
}

//此回调有一缺陷，需在中断中检测空闲，待优化
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    /* code */
    if (uart_rx_index < 1024)
    {
      /* code */
      uart_rx_buf[uart_rx_index++] = rx_byte;
    }
  }
  else
  {
    /* code */
    uart_rx_index = 0;
  }
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

#endif

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




