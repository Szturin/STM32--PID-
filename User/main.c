#include <stm32f10x.h> 			//Device header
#include <stdio.h>				//标准库
#include <Delay.h>				//Delay函数库
#include <OLED.h>				//OLED库
#include <Encoder.h>			//编码器测速函数库
#include <Timer.h>				//定时器
#include <Motor.h>				//电机（霍尔电机）
#include <Key.h>				//按键
#include <PWM.H>				//PWM
#include <Serial.H>				//串口
#include <MPU6050.H>			//陀螺仪模块(MPU6050)
#include <inv_mpu.h>			//DMP库(MPU6050)
#include <PID.h>				//PID算法
#include <I2C_.h>				//I2C(MPU6050)
#include <usart.h>				//串口（平衡车）

//定时器定时速度过慢？？？
//此工程运行正常

float PWM_ALL;//总PWM

int16_t speed,speed2;
float Velociy_meause;

float Velocity_calcu=0;
float Yaw_setting=0;

float PWM_Vertical;//直立环PWM分量
float PWM_Velociy;
float PWM_1;
float PWM_2;
float PWM_0;

float Speed;//速度
unsigned char Motor_Flag=1;//电机运转标志位

unsigned char KeyNum;//键值

extern float Pitch,Roll,Yaw;//航向角,小车直立环使用Roll即绕z轴的横滚角 

extern uint32_t num_conuter;//函数调用次数测试变量(DMP数据更新)
extern uint32_t num_counter_pwm;//函数调用次数测试变量(PWM数据更新)
extern struct PID Velociy;
//OLED波形显示结构体类型定义
typedef struct 
{
	int16_t t;//OLED波形显示计时变量（横轴）
	int16_t t_temp;//OLED波形显示计时变量暂存值
	int16_t Roll_temp;//Roll角暂存值
	int16_t Roll;
	int16_t Wave_Timer;//Oled折线图波形单位间隔时间
}OLED_Wave_Data;

OLED_Wave_Data Roll_Wave;//定义OLED_Wave_Data类型的Roll波形结构体

void Roll_Wave_Init()//结构体赋初值
{
	Roll_Wave.Roll = (int16_t)Roll;
}


int main(void)
{
	SystemInit();//STM32系统初始化
	OLED_Init();////OLED初始化
	Motor_PWM_Init();//电机PWM输出初始化
	Encoder1_Init();
	Encoder2_Init();//编码器也要配置中断
	TIM1_Init();//定时器1初始化（用于定时测量姿态/速度、PID更新）
	USART1_Config();//USART1初始化配置，DMA后续需利用USART1外设发送初始化数据（仅用于初始化）
	USART3_Init();//USART3初始化配置，蓝牙串口通信
	i2cInit();//IIC初始化 用于挂靠在总线上的设备使用(MPU6050)
	Delay_ms(10); //延时10ms
	MPU6050_DMP_Init();//MPU6050 DMP库初始化
	Vertical_PID_Init();//PID直立环初值初始化
	Velociy_PID_Init();
	Roll_Wave_Init();
	Turn_PID_Init();
	while(1)
	{
		//printf("%f\r\n",Roll);//串口发送ROLL值
		
		//OLED_DrawWave(&Roll_Wave.t,&Roll_Wave.Roll,&Roll_Wave.t_temp,&Roll_Wave.Roll_temp,32,126,126);//绘制波形
		//printf("speed1:%d speed2:%d\r\n",speed,speed2);
		//printf("suduhuan:%f\r\n",PWM_Velociy);
		OLED_ShowSignedNum(0,0,Velocity_calcu,5,OLED_6X8);
		OLED_ShowSignedNum(8,8,Yaw_setting,5,OLED_6X8);
		//OLED_ShowSignedNum(0,8,speed2,5,OLED_6X8);
		OLED_Update();
		if(Roll>= 60 | Roll <=-60)//电机保护,倾斜角大于60度关闭电机，防止电机堵转
		{		
			Motor_SpeedSet_Vertical(0);
			Motor_Flag=0;
			//I_amplitude_limiting(0,&Velociy);
		}	
	}
	
}

void TIM1_UP_IRQHandler(void)   //TIM1中断
{
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) //检查指定的TIM1中断发生与否:TIM1 中断源 
	{
		MPU6050_Pose();//DMP数据解算，获得Pitch,Roll,Yaw值
		Roll_Wave.Roll=Roll;
		if(Motor_Flag)
		{		
			speed = Encoder1_GetCounter();
			speed2 = Encoder2_GetCounter();
			
			Velociy_meause = (speed+speed2)/2.0;
			
			PWM_0=Vertical_PID(Roll)+Velocity_PID(Velociy_meause,Velocity_calcu);//直立环+速度环PWM;
			
			PWM_1=PWM_0+Turn_PID(Yaw,Yaw_setting);
			PWM_2=PWM_0-Turn_PID(Yaw,Yaw_setting);
			
			Motor_PWMSet((int16_t)PWM_1,(int16_t)PWM_2);		
		}
		
		OLED_DrawWave_Cycle(&Roll_Wave.Wave_Timer,&Roll_Wave.t,4);//OLED波形绘制时钟使能
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM1 中断源 	
	}
}

