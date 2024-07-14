#include "stm32f10x.h"                  // Device header
#include "PID.h"

const float midvalue = 0;//6.5
float filt_velocity=0;
float last_filt_velocity=0;
typedef struct 
{
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
PID Velocity;
PID Turn;

void Vertical_PID_Init()
{
	//Vertical.Kp=-600;//1000*0.6 //780
	//Vertical.Ki=0;
	//Vertical.Kd=-7200;//12000*0.6 //3200
	Vertical.Kp=-900*1.1*0.6;//1000*0.6 //780
	Vertical.Ki=0;
	Vertical.Kd=-7500*1.8*0.6;//12000*0.6 //3200
}

void Velociy_PID_Init()
{
	//Velocity.Kp=240;
	//Velocity.Ki=1.2;
	//Velocity.Kd=0;
	Velocity.Kp=240;
	Velocity.Ki=1.2;
	Velocity.Kd=0;	
}

void Turn_PID_Init()
{
	//Turn.Kp=-60;
	//Turn.Ki=0;
	//Turn.Kd=-10;
	Turn.Kp=-80;
	Turn.Ki=0;
	Turn.Kd=0;
}


void I_amplitude_limiting(float number,float *Error_sum)
{
	if(*Error_sum > number)
	{
		*Error_sum = number;
	}
	
	if(*Error_sum <- number)
	{
		*Error_sum = -number;
	}
}


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

float Velocity_PID(float velocity,float velocity_calcu)
{
	float a = 0.3;
	Velocity.error = velocity - velocity_calcu;//误差值
	filt_velocity = a*Velocity.error+(1-a)*last_filt_velocity;
	Velocity.error_sum += filt_velocity ;//误差累加
	I_amplitude_limiting(3000,&Velocity.error_sum);
	last_filt_velocity = filt_velocity;
	
	return Velocity.Kp*filt_velocity + Velocity.Ki*Velocity.error_sum;//PID控制器响应结果  
}

float Turn_PID(float yaw,float yaw_calcu)
{
	Turn.error= yaw - yaw_calcu;
	Turn.error_difference=Turn.error-Turn.last_error;
	Turn.last_error = Turn.error;
	return Turn.Kp*Turn.error+Turn.Kd*Turn.error_difference;
}

