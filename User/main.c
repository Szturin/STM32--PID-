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
#include <MPU6050.H>
#include <inv_mpu.h>
#include <MPU6050_DMP.h>
#include <PID.h>
#include <MyIIC.h>
#include <I2C_.h>
#include <usart.h>

//定时器定时速度过慢？？？
//此工程运行正常

float PWM1,PWM2;
float PWM_F=0;
float PWM_ALL;

float PWM_V;//直立环PWM
float Speed;
float measure;
float calcu;
unsigned char Motor_Flag=0;
unsigned int number;
unsigned char KeyNum;

extern float Pitch,Roll,Yaw;
int16_t gx,gy,gz,ax,ay,az;
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az);

unsigned char PWM_Flag=0;

extern uint32_t num_conuter;
extern uint32_t num_counter_pwm;


int main(void)
{
	//int result;
	SystemInit();
	OLED_Init();
	LED_GPIO_Config();
	Motor_PWM_Init();
	TIM1_Init();
	USART1_Config();						//串口1初始化 上位机
	USART3_Init();		
	//MPU6050_Init();
	//result = mpu_dmp_init();
	i2cInit();							   //IIC初始化 用于挂靠在总线上的设备使用
	Delay_ms(10);						   //延时10ms
	MPU6050_DMP_Init();	
	
	Vertical_PID_Init();
	OLED_ShowString(2,1,"P:");
	OLED_ShowString(3,1,"R:");
	OLED_ShowString(4,1,"Y:");
	OLED_ShowString(1,1,"Result:");
	//USART3_Init();
	//Encoder_Init();
	while(1)
	{
		printf("%f\r\n",Roll);
		//mpu_dmp_get_data(&Pitch,&Roll,&Yaw);
		//OLED_ShowSignedNum(1,6,result,2);
		//OLED_ShowSignedNum(2,4,Pitch,3);
		//OLED_ShowSignedNum(3,4,Roll,3);
		//OLED_ShowSignedNum(4,4,Yaw,3);	
		if(Roll>= 30 | Roll <=-30)
		{		
			Motor_SpeedSet_Vertical(0);
			Motor_Flag=1;
		}	
	}
	
}

void TIM1_UP_IRQHandler(void)   //TIM1中断
{
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) //检查指定的TIM1中断发生与否:TIM1 中断源 
	{
		MPU6050_Pose();
		
		if(Motor_Flag==0)
		{
			PWM_V=Vertical_PID(Roll);	
			Motor_SpeedSet_Vertical((int16_t)PWM_V);
		}
		//printf("角度：%f\r\n",Roll);
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM1 中断源 	
	}
}

