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
#include "chassis_def.h"
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
#include "bsp_usart.h"
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

static float last_yaw = 0.0f;
static float yaw_round_count = 0.0f;

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
		// 	INS.v_n=INS.v_n+INS.MotionAccel_n[1]*0.001f;
		//   INS.x_n=INS.x_n+INS.v_n*0.001f;
			// 获取最终数据
			// INS.Pitch=mahony.roll*180.0f/PI;
			// INS.Roll=mahony.pitch*180.0f/PI;
			// INS.Yaw=mahony.yaw*180.0f/PI;
			INS.Roll=mahony.pitch;
			INS.Pitch=mahony.roll;
			INS.Yaw=mahony.yaw;

            if(INS.Yaw - last_yaw > PI) yaw_round_count--;
            else if(INS.Yaw - last_yaw < -PI) yaw_round_count++;
            last_yaw = INS.Yaw;
            INS.YawTotal = INS.Yaw + yaw_round_count * 2.0f * PI;
      
			INS.ins_flag=1;//四元数基本收敛，加速度也基本收敛，可以开始底盘任务
		
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
// #ifdef INS_OF_HIPNUC
// hipnuc_raw_t hipnuc_data;
// static uint8_t rx_byte;
// static uint8_t imu_rx_byte;
// static uint16_t imu_rx_cnt = 0;
// void HIPNUC_Init(void)
// {
//   HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
// }
// void hi05()
// {
//     if (hipnuc_data.buf[0]==0x5A && hipnuc_data.buf[1]==0xA5&& hipnuc_data.buf[6]==0x91)
//     {
//         /* code */
//         hipnuc_data.hi91.main_status = U2(&hipnuc_data.buf[1+6]);
//         hipnuc_data.hi91.temperature = I1(&hipnuc_data.buf[3+6]);
//         hipnuc_data.hi91.pressure = R4(&hipnuc_data.buf[4+6]);
//         hipnuc_data.hi91.timestamp = U4(&hipnuc_data.buf[8+6]);
//         hipnuc_data.hi91.acc[X_AXIS] = R4(&hipnuc_data.buf[12+6]) * GRAVITY;
//         hipnuc_data.hi91.acc[Y_AXIS] = R4(&hipnuc_data.buf[16+6]) * GRAVITY;
//         hipnuc_data.hi91.acc[Z_AXIS] = R4(&hipnuc_data.buf[20+6]) * GRAVITY;
//         hipnuc_data.hi91.gyro[X_AXIS] = R4(&hipnuc_data.buf[24+6]);
//         hipnuc_data.hi91.gyro[Y_AXIS] = R4(&hipnuc_data.buf[28+6]);
//         hipnuc_data.hi91.gyro[Z_AXIS] = R4(&hipnuc_data.buf[32+6]);
//         hipnuc_data.hi91.mag[0] = R4(&hipnuc_data.buf[36+6]);
//         hipnuc_data.hi91.mag[1] = R4(&hipnuc_data.buf[40+6]);
//         hipnuc_data.hi91.mag[2] = R4(&hipnuc_data.buf[44+6]);
//         hipnuc_data.hi91.roll = R4(&hipnuc_data.buf[48+6]);
//         hipnuc_data.hi91.pitch = R4(&hipnuc_data.buf[52+6]);
//         hipnuc_data.hi91.yaw = R4(&hipnuc_data.buf[56+6]);
//         hipnuc_data.hi91.quaternion[0] = R4(&hipnuc_data.buf[60+6]);
//         hipnuc_data.hi91.quaternion[1] = R4(&hipnuc_data.buf[64+6]);
//         hipnuc_data.hi91.quaternion[2] = R4(&hipnuc_data.buf[68+6]);
//         hipnuc_data.hi91.quaternion[3] = R4(&hipnuc_data.buf[72+6]);
//     }
// }
// void INS_task(void)
// {
//   // HIPNUC_Init();
//   while (1)
//   {
//     /* code */
//     // Process_HIPNUC_Data();
//     // INS.Accel[X_AXIS] = -hipnuc_data.hi91.acc[X_AXIS];
//     INS.ins_flag = 1;
//     osDelay(1);
//   }
// }
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//   if(huart->Instance == USART1)
//   {
//     hipnuc_data.buf[imu_rx_cnt++] = imu_rx_byte;
//     if (imu_rx_cnt >= 2)
//     {
//       if (hipnuc_data.buf[0] != 0x5A || hipnuc_data.buf[1] != 0xA5)
//       {
//         hipnuc_data.buf[0] = hipnuc_data.buf[1];
//         imu_rx_cnt = 1;  // 丢弃错位
//       }
//     }
//     if (imu_rx_cnt >= HIPNUC_MIN_LENGTH)
//     {
//       if(hipnuc_data.buf[6] == 0x91)
//         hi05();  
//       imu_rx_cnt = 0;  
//     }
//     if(imu_rx_cnt >= 100)
//       imu_rx_cnt = 0; 
//   }
//   HAL_UART_Receive_IT(&huart1, &imu_rx_byte, 1);
// }
// #endif
#ifdef INS_OF_HIPNUC

hipnuc_raw_t hipnuc_data;
static USARTInstance *hipnuc_usart_instance = NULL;

static uint8_t hipnuc_frame_buf[HIPNUC_MIN_LENGTH];
static volatile uint8_t hipnuc_rx_flag = 0;
static volatile uint16_t hipnuc_frame_len = 0;

static void HIPNUC_ParseFrame(const uint8_t *buf, uint16_t len)
{
    if (len < HIPNUC_MIN_LENGTH) return;
    if (buf[0] != 0x5A || buf[1] != 0xA5) return;
    if (buf[6] != 0x91) return;

    memcpy(hipnuc_data.buf, buf, len);
    hipnuc_data.len = len;
    hipnuc_data.nbyte = 0;

    hipnuc_data.hi91.main_status = U2((uint8_t *)&hipnuc_data.buf[1 + 6]);
    hipnuc_data.hi91.temperature = I1((uint8_t *)&hipnuc_data.buf[3 + 6]);
    hipnuc_data.hi91.pressure = R4((uint8_t *)&hipnuc_data.buf[4 + 6]);
    hipnuc_data.hi91.timestamp = U4((uint8_t *)&hipnuc_data.buf[8 + 6]);

    hipnuc_data.hi91.acc[X_AXIS] = R4((uint8_t *)&hipnuc_data.buf[12 + 6]);
    hipnuc_data.hi91.acc[Y_AXIS] = R4((uint8_t *)&hipnuc_data.buf[16 + 6]);
    hipnuc_data.hi91.acc[Z_AXIS] = R4((uint8_t *)&hipnuc_data.buf[20 + 6]);

    hipnuc_data.hi91.gyro[X_AXIS] = R4((uint8_t *)&hipnuc_data.buf[24 + 6]);
    hipnuc_data.hi91.gyro[Y_AXIS] = R4((uint8_t *)&hipnuc_data.buf[28 + 6]);
    hipnuc_data.hi91.gyro[Z_AXIS] = R4((uint8_t *)&hipnuc_data.buf[32 + 6]);

    hipnuc_data.hi91.mag[0] = R4((uint8_t *)&hipnuc_data.buf[36 + 6]);
    hipnuc_data.hi91.mag[1] = R4((uint8_t *)&hipnuc_data.buf[40 + 6]);
    hipnuc_data.hi91.mag[2] = R4((uint8_t *)&hipnuc_data.buf[44 + 6]);

    hipnuc_data.hi91.roll = R4((uint8_t *)&hipnuc_data.buf[48 + 6]);
    hipnuc_data.hi91.pitch = R4((uint8_t *)&hipnuc_data.buf[52 + 6]);
    hipnuc_data.hi91.yaw = R4((uint8_t *)&hipnuc_data.buf[56 + 6]);

    hipnuc_data.hi91.quaternion[0] = R4((uint8_t *)&hipnuc_data.buf[60 + 6]);
    hipnuc_data.hi91.quaternion[1] = R4((uint8_t *)&hipnuc_data.buf[64 + 6]);
    hipnuc_data.hi91.quaternion[2] = R4((uint8_t *)&hipnuc_data.buf[68 + 6]);
    hipnuc_data.hi91.quaternion[3] = R4((uint8_t *)&hipnuc_data.buf[72 + 6]);
}

static void HIPNUC_RxCallback(void)
{
    memcpy(hipnuc_frame_buf, hipnuc_usart_instance->recv_buff, HIPNUC_MIN_LENGTH);
    hipnuc_frame_len = HIPNUC_MIN_LENGTH;
    hipnuc_rx_flag = 1;
}

void HIPNUC_Init(void)
{
    USART_Init_Config_s conf;
    conf.recv_buff_size = HIPNUC_MIN_LENGTH;
    conf.usart_handle = &huart10;
    conf.module_callback = HIPNUC_RxCallback;
    hipnuc_usart_instance = USARTRegister(&conf);
}

void INS_task(void)
{
    osDelay(3000); // 等待串口服务启动
    
    while (1)
    {
        if (hipnuc_rx_flag)
        {
            uint8_t local_buf[HIPNUC_MIN_LENGTH];
            uint16_t local_len;

            taskENTER_CRITICAL();
            local_len = hipnuc_frame_len;
            memcpy(local_buf, hipnuc_frame_buf, local_len);
            hipnuc_rx_flag = 0;
            taskEXIT_CRITICAL();

            HIPNUC_ParseFrame(local_buf, local_len);

            INS.Accel[X_AXIS] = hipnuc_data.hi91.acc[X_AXIS] * GRAVITY;
            INS.Accel[Y_AXIS] = hipnuc_data.hi91.acc[Y_AXIS] * GRAVITY;
            INS.Accel[Z_AXIS] = hipnuc_data.hi91.acc[Z_AXIS] * GRAVITY;

            INS.Gyro[X_AXIS] = hipnuc_data.hi91.gyro[X_AXIS] * PI / 180.0f;
            INS.Gyro[Y_AXIS] = hipnuc_data.hi91.gyro[Y_AXIS] * PI / 180.0f;
            INS.Gyro[Z_AXIS] = hipnuc_data.hi91.gyro[Z_AXIS] * PI / 180.0f;

            INS.q[0] = hipnuc_data.hi91.quaternion[0];
            INS.q[1] = hipnuc_data.hi91.quaternion[1];
            INS.q[2] = hipnuc_data.hi91.quaternion[2];
            INS.q[3] = hipnuc_data.hi91.quaternion[3];

            INS.Roll = hipnuc_data.hi91.pitch * PI / 180.0f;
            INS.Pitch = hipnuc_data.hi91.roll * PI / 180.0f;
            INS.Yaw = hipnuc_data.hi91.yaw * PI / 180.0f;

            if(INS.Yaw - last_yaw > PI) yaw_round_count--;
            else if(INS.Yaw - last_yaw < -PI) yaw_round_count++;
            last_yaw = INS.Yaw;
            INS.YawTotal = INS.Yaw + yaw_round_count * 2.0f * PI;

            INS.ins_flag = 1;
        }

        osDelay(1);
    }
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




