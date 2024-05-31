
#include "stm32f10x.h"                  // Device header

const float midvalue = 0;//6.5

typedef struct pid{
	float Kp;
	float Ki;
	float Kd;
	float error;
	float last_error;
	float error_sum;
	float error_difference;
	float filt_velocity;
	float last_filt_velocity;
	float velocity_sum;
}PID;

PID Vertical;

void Vertical_PID_Init()
{
	Vertical.Kp=-600;
	Vertical.Ki=0;
	Vertical.Kd=-500;
}

/*
void I_amplitude_limiting(int number)
{
	if(Error_sum>number)
	{
		Error_sum=number;
	}
}
*/

/*
//PID控制系统:P、I、D共同作用
//measure,calcu:系统的输入
int PID_control(float measure,float calcu)
{
	Error = calcu - measure;//误差值

	Error_sum += Error;//误差累加
	I_amplitude_limiting(13800);//限幅函数

	Error_difference = Error- Error_old;//误差变化率(近似于对时间的微分)
	
	Error_old = Error;

	return(Kp*Error+Kd*Error_difference+Ki*Error_sum);//PID控制器响应结果
}
*/

float Vertical_PID(float Pitch)
{
	Vertical.error = Pitch-midvalue;//误差值

	Vertical.error_sum += Vertical.error;//误差累加

	Vertical.error_difference = Vertical.error - Vertical.last_error;//误差变化率(近似于对时间的微分)
	
	Vertical.last_error = Vertical.error;

	return Vertical.Kp*Vertical.error + Vertical.Ki*Vertical.error_sum + Vertical.Kd*Vertical.error_difference;//PID控制器响应结果  
}