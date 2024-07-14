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

float Angle_X=90;		//X轴转角
float Angle_Y=30;		//Y轴转角
float Angle_Out_Inc_x;	//X轴转角
float Angle_Out_Inc_y;  //Y轴转角

uint8_t counter;		
float Angle_X_err;
float Angle_Y_err;

float laser_Pose_control(float Expet ,float Measure)
{
    static float Kp = 0.02;
    static float Ki = 0.01;
    static float Kd = 0.01;
    //误差，上一次误差，上上一次误差
    static float ERR,Last_ERR,LastLast_ERR;
    float Out_Inc;
    //误差更新
    ERR = Expet - Measure;
    //计算

    Out_Inc = Kp*(ERR - Last_ERR ) + Ki*Last_ERR + Kd*(ERR - 2*Last_ERR + LastLast_ERR);

    //误差更新

    Last_ERR = ERR;

    LastLast_ERR = Last_ERR;

    return Out_Inc;
}

int Serial_pow(unsigned int x,unsigned int y)
{
	unsigned char i=0;
	int result;
	
	if(y == 0)
	{
		result =1;
	}
	else
	{
		for(i=1;i<y;i++)
		{
			result*=x;
		}
	}
	return result;
}//mark

int main(void)
{
	Servo_PWM_Init();//电机初始化
	USART3_Init();//串口3初始化
	OLED_Init();
	
	while(1)
	{
		counter++;
		//Servo_SetAngle_Y(Serial_RxPacket[0]);
		//print发送的是ascii码,6 --> 0x36

		if(Serial_RxFlag)
		{	
			Angle_X_err=0;
			unsigned char i;
			for(i=0; i<pRxState;i++)
			{	
				if(RxState_Flag==1)//X轴数据据
				{
					if(Sign_Flag==1)//负数
					{						
						Angle_X_err -= (Serial_RxPacket[i]-0x30)*Serial_pow(10,i);
					}
					else if(Sign_Flag==2)//正数
					{
						Angle_X_err += (Serial_RxPacket[i]-0x30)*Serial_pow(10,i);
					}
				}
				else if(RxState_Flag==2)//y轴数据
				{
					if(Sign_Flag==1)//负数
					{						
						Angle_Y_err -= (Serial_RxPacket[i]-0x30)*Serial_pow(10,i);
					}
					else if(Sign_Flag==2)//正数
					{
						Angle_Y_err += (Serial_RxPacket[i]-0x30)*Serial_pow(10,i);
					}					
				}
				
				OLED_ShowNum(0,0+i*16,Serial_RxPacket[i]-0x30,1,8);
				OLED_Update();			
			}
			pRxState=0;//MARK:忘记清零
			Serial_RxFlag=0;
		}			
		
		Angle_Out_Inc_x = laser_Pose_control(Angle_X_err,0);
		Angle_X += Angle_Out_Inc_x;
		Angle_Out_Inc_y = laser_Pose_control(Angle_Y_err,0);
		Angle_Y += Angle_Out_Inc_y;		
		
		Servo_SetAngle_X(Angle_X);
		Servo_SetAngle_Y(Angle_Y);
		
		OLED_ShowSignedNum(16,16,Angle_Out_Inc_x,5,8);
		OLED_Update();
		//printf("666\r\n");
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
