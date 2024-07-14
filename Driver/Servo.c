#include "stm32f10x.h"                  // Device header
#include <PWM.h>
#include <timer.h>
#include <Servo.h>				//舵机

/*宏定义，方便替换引脚*/
#define PWM_A GPIO_Pin_2
#define PWM_B GPIO_Pin_3

void Servo_GPIO_Init()
{	
	GPIO_InitTypeDef GPIO_InitStructure; //结构体类型(已经定义好的） 结构体变量名 ->结构体变量的定义
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	//PWM口初始化  //mark:重复初始化结构体会覆盖掉，这里PWM就被逻辑口覆盖掉了，逻辑口改为GPIOB
	GPIO_InitStructure.GPIO_Pin = PWM_A | PWM_B; //TIM1_CH1  TIM1_CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA口
	
}

//PWM方波周期设置为20ms,舵机才能正常工作
void Servo_PWM_Init(void)
{	
	Servo_GPIO_Init();
	TIM2_PWM_Init(10000-1,72-1);
}

/*X维舵机控制*/
void Servo_SetAngle_Y(float Angle)
{
	TIM_SetCompare3(TIM2,(unsigned int)(Angle / 180 * 1800 + 600));//500偏移
}

/*Y维舵机控制*/
void Servo_SetAngle_X(float Angle)
{
	TIM_SetCompare4(TIM2,(unsigned int)(Angle / 180 * 1800 + 600));//500偏移
}

uint32_t num_counter_pwm=0;


