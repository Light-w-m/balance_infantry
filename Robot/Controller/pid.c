#include "pid.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void PID_init(PidTypeDef *pid, uint8_t mode, const float PID[3], float max_out, float max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

float PID_Calc(PidTypeDef *pid, float ref, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

void Engineer_PID_Init(FeedforwardPidTypeDef *pid, uint8_t mode, float Kp, float Ki, float Kd, float Kf, float I_Band, float dt, float max_out, float max_iout)
{
    if (pid == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Kf = Kf;
	pid->I_Band = I_Band;
	pid->dt = dt;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
	pid->error = pid->pre_error = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

float Engineer_PID_Calc(FeedforwardPidTypeDef *pid, float ref, float set)
{
	if (pid == NULL)
	{
		return 0.0f;
	}
	//数据更新
	pid->pre_error = pid->error;
	pid->set = set;
	pid->fdb = ref;
    if(pid->mode == Feedforward_PID)
    {
        pid->error = rad_format(set - ref);
        //比例
        pid->Pout = pid->Kp * pid->error;
        //积分
        if(fabs(pid->error)<pid->I_Band)//积分分离，避免过早引入积分
            pid->Iout += pid -> Ki * (pid->error+pid->pre_error)/2 *pid->dt;//梯形积分比直接积分更精确更顺滑
        else
            pid->Iout=0;
        LimitMax(pid->Iout, pid->max_iout); //积分限幅
        //微分
        pid->Dout = -pid->Kd * ((pid->fdb-pid->pre_fdb)/pid->dt);
        //前馈
        pid->Fout = pid->Kf*(pid->set-pid->last_set);
        //输出
        pid->out = pid->Pout + pid->Iout + pid->Dout+pid->Fout;
        LimitMax(pid->out, pid->max_out);
        //数据更新
        pid->last_set=pid->set;
        pid->pre_error=pid->error;
        pid->pre_fdb=pid->fdb;
    }
    else if(pid->mode == Normal_PID)
    {
        pid->error = set - ref;
        //比例
        pid->Pout = pid->Kp * pid->error;
        //积分
        if(fabs(pid->error)<pid->I_Band)//积分分离，避免过早引入积分
        {
            pid->Iout += pid -> Ki * (pid->error+pid->pre_error)/2 *pid->dt;//梯形积分比直接积分更精确更顺滑
        }
        else
        {
            pid->Iout=0;
        }
        LimitMax(pid->Iout, pid->max_iout); //积分限幅
        //微分
        pid->Dout = -pid->Kd * ((pid->fdb-pid->pre_fdb)/pid->dt);//应该加个低通滤波，勉强先用了
        //输出
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
        //数据更新
        pid->pre_error=pid->error;
        pid->pre_fdb=pid->fdb;
    }
    return pid->out;
}
