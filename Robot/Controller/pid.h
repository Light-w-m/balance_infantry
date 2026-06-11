#ifndef PID_H
#define PID_H

#include "stdint.h"
#include "string.h"
#include "user_lib.h"

#ifndef PI
#define PI 3.14159265358979f
#endif

#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

enum Engineer_PID_MODE
{
    Feedforward_PID = 0,
    Normal_PID
};

typedef struct
{
    uint8_t mode;

    float Kp;
    float Ki;
    float Kd;

    float max_out;  
    float max_iout; 

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  
    float error[3]; 

} PidTypeDef;

typedef struct
{
    uint8_t mode;

    float Kp;
    float Ki;
    float Kd;
	float Kf;
	
	float I_Band;
	
    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set;
    float fdb;
	float last_set;

    float out;
    float Pout;
    float Iout;
    float Dout;
	float Fout;

	float error;
	float pre_error;
	float pre_fdb;
	float dt;       //控制周期
} FeedforwardPidTypeDef;

extern void PID_init(PidTypeDef *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);
extern float PID_Calc(PidTypeDef *pid, float ref, float set);
extern void PID_clear(PidTypeDef *pid);
extern void Engineer_PID_Init(FeedforwardPidTypeDef *pid, uint8_t mode, float Kp, float Ki, float Kd, float Kf, float I_Band, float dt, float max_out, float max_iout);
extern float Engineer_PID_Calc(FeedforwardPidTypeDef *pid, float ref, float set);
#endif