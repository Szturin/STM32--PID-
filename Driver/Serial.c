#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
#include <OLED.H>
#include <Servo.h>

uint8_t Serial_RxPacket[100];
uint8_t Serial_RxFlag;
uint8_t RxState = 0;
uint8_t pRxState;//表示当前接收的是第几个变量
uint8_t num_c;

uint8_t RxState_Flag;
uint8_t Sign_Flag;

void Serial_Init(void)
{
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1`,ENABLE);		//开启USART1的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);		//开启GPIOA的时钟
	
	/*配置PA9为复用推挽输出，供USART1的TX使用*/
	GPIO_InitTypeDef GPIO_InitStructure; //结构体类型(已经定义好的） 结构体变量名 ->结构体变量的定义
	
	/*串口发送引脚部分*/
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_Init(GPIOA,&GPIO_InitStructure);	

	/*串口接收引脚部分*/
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;//上拉输入模式
	GPIO_Init(GPIOA,&GPIO_InitStructure);	
	
	/*USART结构体初始化*/
	USART_InitTypeDef USART_InitStructrue;
	USART_InitStructrue.USART_BaudRate=9600;//直接写入设定的波特率，函数内部会自动算好9600对应的寄存器配置
	USART_InitStructrue.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//流控制关
	USART_InitStructrue.USART_Mode=USART_Mode_Rx | USART_Mode_Tx;//串口发送+接收
	USART_InitStructrue.USART_Parity=USART_Parity_No;//无校验位
	USART_InitStructrue.USART_StopBits=USART_StopBits_1;//1位停止位
	USART_InitStructrue.USART_WordLength=8;//字长8位
	USART_Init(USART1,&USART_InitStructrue);
	
	/*中断配置*/
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//开启RXNE的标志位到NVIC输出
	
	/*NVIC配置*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//NVIC分组
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART1,ENABLE);
}


void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART3,Byte);//写入DR寄存器
	while( USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);//等待清空标志位为RESET,防止数据覆盖
	//PS:这个标志位不需要手动清零，当检测到为RESET会触发中断，自动清零
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for(i=0;i<Length;i++)
	{
		Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char * String)
{
	uint8_t i;
	for(i=0;String[i] != '\0';i++)
	{
		Serial_SendByte(String[i]);
	}
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while(Y--)
	{
		Result *=X;
	}
	return Result;
}

void Serial_SendNumber(uint32_t Number,uint8_t Length)
{
	uint8_t i;
	for(i=0;i<Length;i++)
	{
		Serial_SendByte(Number/Serial_Pow(10,Length-i-1)%10 + '0');
	}
}


int fputc(int ch,FILE * f)
{
	Serial_SendByte(ch);
	return ch;
}

// 可变参数
void Serial_Printf(char * format,...)
{
	char String[100];
	va_list arg;//参数列表变量
	va_start(arg,format);
	vsprintf(String,format,arg);
	va_end(arg);//释放参数列表
	Serial_SendString(String);
}

//尝试自己配置一遍
void USART3_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;	//定义串口初始化结构体
	
	/* config USART3 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 , ENABLE);
		/* config GPIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB	, ENABLE);
	/* USART3 GPIO config */
	/* Configure USART3 Tx (PB.10) as alternate function push-pull 推拉输出模式*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);    
	/* Configure USART3 Rx (PB.11) as input floating 浮点输入模式*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
  	
	
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* USART3 mode config */
	USART_InitStructure.USART_BaudRate = 19200;//波特率19200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No ;//无校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//禁用RTSCTS硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//使能发送接收
  

	USART_Init(USART3, &USART_InitStructure); 
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//使能接收中断
	
	USART_Cmd(USART3, ENABLE);
}

void USART3_IRQHandler(void)          	
{
	if(USART_GetITStatus(USART3,USART_IT_RXNE)==SET)
	{
		uint8_t RxData = USART_ReceiveData(USART3);
		num_c++;
		//数据帧格式
		if(RxState == 0)//第一个包头
		{
			if(RxData == '@' && Serial_RxFlag == 0)
			{
				RxState=1;
			}
		}
		else if(RxState == 0 )
		{
			if(RxData == 'x' )//第二个包头
			{
				RxState=2;
				RxState_Flag=1;
			}
			else if(RxData == 'y')
			{
				RxState=2;
				RxState_Flag=2;
			}		
		}
		else if(RxState == 2)
		{
			if(RxData == '-')//第三个包头
			{
				RxState=3;
				Sign_Flag=1;
			}
			else if(RxData == '+')
			{
				RxState=3;
				Sign_Flag=2;
			}				
		}
		else if(RxState==3)
		{
			if(RxData == '\r')//第一个包尾
			{
				RxState=4;
			}
			else//连续接收数据
			{
				Serial_RxPacket[pRxState]=RxData;
				pRxState++;	
			}
		}
		else if(RxState==4)//第二个包尾
		{
			if (RxData=='\n')
			{
				RxState = 0;
				Serial_RxFlag=1;
				Serial_RxPacket[pRxState]='\0';//字符串结束时加上\0,表示字符串的结束
				printf("RxState:%d\r\n",(unsigned int)RxState);
			}
		}
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
	}
} 

