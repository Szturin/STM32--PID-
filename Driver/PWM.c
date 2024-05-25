#include "stm32f10x.h"                  // Device header

#define PWM_A GPIO_Pin_2
#define PWM_B GPIO_Pin_3

//TIM1的PWM模式初始化（TIM1_CH1--PA8  TIM1_CH4--PA11）
//arr：自动重装值
//psc：时钟预分频数
//TIM1的PWM模式初始化（TIM1_CH1--PA8  TIM1_CH4--PA11）
//arr：自动重装值
//psc：时钟预分频数

void TIM2_PWM_Init(u16 arr,u16 psc)
{  
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 

   //初始化TIM2
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM Channel 1-4 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	
	
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 OC3
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 OC4
 
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM1_CH3上的预装载寄存器
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM1_CH4上的预装载寄存器
 	TIM_ARRPreloadConfig(TIM2, ENABLE); //使能TIMx在ARR上的预装载寄存器

	TIM_Cmd(TIM2, ENABLE);  //使能TIM1
	
	//TIM_CtrlPWMOutputs(TIM2,ENABLE);        //MOE 主输出使能,高级定时器必须开启这个
}

