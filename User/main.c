#include <stm32f10x.h> 			//Device header
#include <Delay.h>
#include <OLED.h>
#include <Encoder.h>
#include <Timer.h>
#include <Motor.h>
#include <Key.h>
#include <PWM.H>
#include <MOTOR.H>
#include <Serial.H>
#include <stdio.h>

//定时器定时速度过慢？？？
//此工程运行正常

float PWM1,PWM2;
float PWM_F=0;
float PWM_ALL;
float Speed;
float measure;
float calcu;

unsigned int number;
unsigned char KeyNum;

float Kp=1.80,Ki=0.18,Kd=0;
float Error;
float Error_old;
float Error_difference;
long int Error_sum;

void I_amplitude_limiting(int number)
{
	if(Error_sum>number)
	{
		Error_sum=number;
	}
}

void PWM_amplitude_limiting(float a)
{
	if(PWM_ALL >=5000 )
	{
		PWM_ALL = 5000;
	}
	else if(PWM_ALL<= - 5000)
	{
		PWM_ALL = -5000;
	}
}
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


int main(void)
{
	OLED_Init();
	OLED_ShowString(1,1,"Speed:");
	OLED_ShowString(3,1,"PWM1:");
	OLED_ShowString(4,1,"PWM2:");
	USART3_Init();
	Encoder_Init();
	Motor_PWM_Init();
	TIM1_Init();
	
	PWM1=1000;PWM2=-1000;
	Motor_SpeedSet((int16_t)PWM1,(int16_t)PWM2);
	
	while(1)
	{
		OLED_ShowNum(2,1,number,5);
		OLED_ShowNum(1,7,Speed,5);
		OLED_ShowSignedNum(3,6,(int)PWM1,5);
		OLED_ShowSignedNum(4,6,(int)PWM2,5);
		KeyNum=Key_GetNum();
	}
}

void TIM1_UP_IRQHandler(void)   //TIM1中断
{
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) //检查指定的TIM1中断发生与否:TIM1 中断源 
	{
		printf("%f,%f,%f\n",Speed,calcu,Error_difference);
		Speed=Encoder_GetSpeed();
		measure=Speed;
		number++;
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM1 中断源 	
	}
}

