#include "stm32f10x.h"                  // Device header
#include <PWM.h>
#include <timer.h>
#include <motor.h>
#include <PID.h>

/*宏定义，方便替换引脚*/
#define	LOGIC_A_1 GPIO_Pin_14
#define	LOGIC_A_2 GPIO_Pin_15

#define	LOGIC_B_1 GPIO_Pin_13
#define	LOGIC_B_2 GPIO_Pin_12

#define PWM_A GPIO_Pin_2
#define PWM_B GPIO_Pin_3

//分辨率：0.1ms
//
void Motor_PWM_Init(void)
{	
	Motor_GPIO_Init();
	TIM2_PWM_Init(10000-1,72-1);
}

uint32_t num_counter_pwm=0;

void Motor_SpeedSet_Vertical(int16_t Speed)
{
	if(Speed>0)
	{
		GPIO_SetBits(GPIOB, LOGIC_A_1 | LOGIC_B_1);
		GPIO_ResetBits(GPIOB, LOGIC_A_2 | LOGIC_B_2);	
		TIM_SetCompare3(TIM2,Speed);
		TIM_SetCompare4(TIM2,Speed);			
	}
	else
	{
		GPIO_SetBits(GPIOB, LOGIC_A_2|LOGIC_B_2);	
		GPIO_ResetBits(GPIOB, LOGIC_A_1|LOGIC_B_1);
		TIM_SetCompare3(TIM2,-Speed); 
		TIM_SetCompare4(TIM2,-Speed);    	
	}
	num_counter_pwm++;
}

void Motor_PWMSet(int16_t PWM1,int16_t PWM2)
{
	if(PWM1>0)
	{
		GPIO_SetBits(GPIOB, LOGIC_A_1);
		GPIO_ResetBits(GPIOB, LOGIC_A_2);
		TIM_SetCompare3(TIM2,PWM1);	
	}
	else if(PWM1 <= 0 )
	{
		GPIO_SetBits(GPIOB, LOGIC_A_2);
		GPIO_ResetBits(GPIOB, LOGIC_A_1);
		TIM_SetCompare3(TIM2,-PWM1); 
	}
	
	if(PWM2>0)
	{
		GPIO_ResetBits(GPIOB, LOGIC_B_1);
		GPIO_ResetBits(GPIOB, LOGIC_B_2);
		TIM_SetCompare4(TIM2,PWM2);			
	}
	else if(PWM2 <= 0 )
	{
		GPIO_ResetBits(GPIOB, LOGIC_B_2);
		GPIO_ResetBits(GPIOB, LOGIC_B_1);
		TIM_SetCompare4(TIM2,-PWM2);    	
	}
}

void Motor_GPIO_Init()
{	
	GPIO_InitTypeDef GPIO_InitStructure; //结构体类型(已经定义好的） 结构体变量名 ->结构体变量的定义
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	//PWM口初始化  //mark:重复初始化结构体会覆盖掉，这里PWM就被逻辑口覆盖掉了，逻辑口改为GPIOB
	GPIO_InitStructure.GPIO_Pin = PWM_A | PWM_B; //TIM1_CH1  TIM1_CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA口
	
	//正反转控制口初始化1
	GPIO_InitStructure.GPIO_Pin = LOGIC_A_1|LOGIC_A_2|LOGIC_B_1|LOGIC_B_2; //右正反转
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB口
	
	GPIO_SetBits(GPIOB, LOGIC_A_1|LOGIC_A_2|LOGIC_B_1|LOGIC_B_2);
}
