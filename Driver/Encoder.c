#include "stm32f10x.h"                  // Device header

void Encoder_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//开启TIM3时钟	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure; //结构体类型(已经定义好的） 结构体变量名 ->结构体变量的定义
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;//上拉输入
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	/*时基单元初始化配置*/
	TIM_InternalClockConfig(TIM3);//定时器上电后默认使用内部时钟,此语句也可以不写
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision =  TIM_CKD_DIV1 ;//1分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStructure.TIM_Prescaler= 1-1;//PSC 不分频
	TIM_TimeBaseInitStructure.TIM_Period= 65536 - 1;//ARR 
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//重复计数器，高级定时器钟存在，这里不需要用
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//时基单元初始化
	
	/*输入捕获单元配置*/
	/**通道1配置**/
	TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);//给结构体赋初始值，防止出现不确定的状态
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;//输入捕获通道选择
	TIM_ICInitStructure.TIM_ICFilter=0XF;//输入捕获滤波器
	//TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;//极性，上升沿触发  TIM_EncoderInterfaceConfig重复配置了相同的寄存器
	TIM_ICInit(TIM3,&TIM_ICInitStructure);
    /**通道2配置**/ //注意：这里不需要再定义新的结构体。因为上一次的值已经写入硬件寄存器
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;//输入捕获通道选择
	TIM_ICInitStructure.TIM_ICFilter=0X6;//输入捕获滤波器
	//TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;//极性，上升沿触发
	TIM_ICInit(TIM3,&TIM_ICInitStructure);

    /*配置编码器接口*/
    TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);//定时器3 ，T1T2同时使用，极性相同(上升沿/下降沿两个都是有效的)，即为正相
    //若要反相，那么一个配置位上升沿，一个配置为下降沿有效
	TIM_Cmd(TIM3,ENABLE);//启动定时器
	

}

void Encoder1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef TIM_ICInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//开启时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;//初始化GPIO--PA6、PA7
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_6 |GPIO_Pin_7;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);//初始化定时器。
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period=65535;
	TIM_TimeBaseInitStruct.TIM_Prescaler=0;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStruct);
	
	TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);//配置编码器模式
	
	TIM_ICStructInit(&TIM_ICInitStruct);//初始化输入捕获
	TIM_ICInitStruct.TIM_ICFilter=10;
	TIM_ICInit(TIM3,&TIM_ICInitStruct);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);//配置溢出更新中断标志位
	
	TIM_SetCounter(TIM3,0);//清零定时器计数值
	
	TIM_Cmd(TIM3,ENABLE);//开启定时器	
}


void Encoder2_Init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
/* Encoder unit connected to TIM3, 4X mode */    
  GPIO_InitTypeDef GPIO_InitStructure;
  //NVIC_InitTypeDef NVIC_InitStructure;
  
  /* TIM3 clock source enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  /* Enable GPIOA, clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  
  GPIO_StructInit(&GPIO_InitStructure);
  /* Configure PA.06,07 as encoder input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Enable the TIM3 Update Interrupt */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    //优先级组别
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
   
  /* Timer configuration in Encoder mode */
  TIM_DeInit(TIM4);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;  
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  //设置时钟分频系数：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式 
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
 
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge, TIM_ICPolarity_BothEdge); //TIM_ICPolarity_Rising上升沿捕获
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 6; //无滤波器
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
  
 // Clear all pending interrupts
 
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);	  //使能中断
  
  //Reset counter
  //TIM4->CNT = 0;
  
//  ENC_Clear_Speed_Buffer();
  
  TIM_Cmd(TIM4, ENABLE);  	   //使能定时器3
}

int16_t Encoder1_GetCounter(void)//int16_t 表示有符号数，能把unint16_相比最高位变为了符号位,这里利用到了补码的特性
{
	int16_t Temp;
	Temp = (short)TIM_GetCounter(TIM3);
	TIM_SetCounter(TIM3,0);
	return Temp;
}

int16_t Encoder2_GetCounter(void)//int16_t 表示有符号数，能把unint16_相比最高位变为了符号位,这里利用到了补码的特性
{
	int16_t Temp;
	Temp = (short)TIM_GetCounter(TIM4);
	TIM_SetCounter(TIM4,0);
	return Temp;
}

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=0)
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	}
}