#include <stm32f10x.h> 			//Device header
#include <stdio.h>				//标准库
#include <Delay.h>				//Delay函数库
#include <OLED.h>				//OLED库
#include <Encoder.h>			//编码器测速函数库
#include <Timer.h>				//定时器
#include <Servo.h>				//舵机
#include <Key.h>				//按键
#include <PWM.H>				//PWM
#include <Serial.H>				//串口
#include <PID.h>				//“位置式”PID

float Angle_X=90;		//X轴转角
float Angle_Y=20;		//Y轴转角

float Angle_X_err;		//X轴误差值
float Angle_Y_err;		//Y轴误差值

int main(void)
{
	Servo_PWM_Init();//电机初始化
	USART3_Init();//串口3初始化
	OLED_Init();
	while(1)
	{
		/*若中断进入速率过快，此程序建议放入中断函数中，否则容易被频繁打断*/
		if(Serial_RxFlag)//判断数据帧是否完成
		{	
			Angle_X_err=0;//清空误差值
			Angle_Y_err=0;//清空误差值	
			switch(RxData_type)//判断数据帧的数据类型
			{
				case 1:
					Angle_X_err = Serial_RxPacket[0];
					break;
				case 2:
					Angle_X_err = -Serial_RxPacket[0];
					break;
				case 3:
					Angle_Y_err = Serial_RxPacket[0];
					break;
				case 4:
					Angle_Y_err = -Serial_RxPacket[0];
					break;
			}
			Serial_RxFlag=0;//清除帧标志位		

			laser_Pose_control_X(Angle_X_err,0);//更新函数数据，数据类型定义存在bug，调用此函数清除缓存值
			Angle_X += laser_Pose_control_X(Angle_X_err,0);//增量赋值
			
			laser_Pose_control_Y(Angle_Y_err,0);//更新函数数据
			Angle_Y += laser_Pose_control_Y(Angle_Y_err,0);//增量赋值		
			
			//二维舵机控制
			Servo_SetAngle_X(Angle_X);
			Servo_SetAngle_Y(Angle_Y);
		}			

		//调试使用，OLED函数较为耗时，建议注释
		OLED_ShowSignedNum(16,16,Angle_X_err,5,8);
		OLED_Update();
	}
}

/*
void TIM1_UP_IRQHandler(void)   //TIM1中断
{
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) //检查指定的TIM1中断发生与否:TIM1 中断源 
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM1 中断源 	
	}
}
*/
