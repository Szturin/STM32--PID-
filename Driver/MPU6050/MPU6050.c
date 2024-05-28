#include "stm32f10x.h"                  // Device header
#include "MyIIC.h"
#include "MPU6050_REG.h"
#include "MPU6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "math.h"
#include "delay.h"
#define MPU6050_ADDRESS 	0x68


void MPU6050_WriteReg_Byte(uint8_t RegAddress,uint8_t Data)
{
	MPU6050_IIC_Start();
	MPU6050_IIC_SendByte(MPU6050_ADDRESS<<1|0);
	MPU6050_IIC_ReceiveAck();//可以加判断，确保时序的正确
	MPU6050_IIC_SendByte(RegAddress);//指定要写入的寄存器
	MPU6050_IIC_ReceiveAck();
	MPU6050_IIC_SendByte(Data);
	MPU6050_IIC_ReceiveAck();
	MPU6050_IIC_Stop();
}

uint8_t MPU6050_ReadReg_Byte(uint8_t RegAddress)
{
	uint8_t Data;
	MPU6050_IIC_Start();
	MPU6050_IIC_SendByte(MPU6050_ADDRESS<<1|0);
	MPU6050_IIC_ReceiveAck();//可以加判断，确保时序的正确
	MPU6050_IIC_SendByte(RegAddress);//指定要写入的寄存器
	MPU6050_IIC_ReceiveAck();
	
	MPU6050_IIC_Start();
	MPU6050_IIC_SendByte(MPU6050_ADDRESS<<1|0x01);//读写位改为1
	MPU6050_IIC_ReceiveAck();
	
	Data = MPU6050_IIC_ReceiveByte();
	MPU6050_IIC_SendAck(1);//不给从机应答，表示不再接收数据
	MPU6050_IIC_Stop();
	
	return Data;
}

uint8_t MPU6050_WriteReg_Str(uint8_t DevAddress, uint8_t RegAddress,uint8_t len,uint8_t *Data)
{
	uint8_t i;
	MPU6050_IIC_Start();
	MPU6050_IIC_SendByte((DevAddress<<1)|0);
	MPU6050_IIC_ReceiveAck();//可以加判断，确保时序的正确
	MPU6050_IIC_SendByte(RegAddress);//指定要写入的寄存器
	MPU6050_IIC_ReceiveAck();
	/*
		for(i=0;i<len;i++)
		{
			MPU6050_IIC_SendByte((Data[i]));//逻辑出错,对比异同？
			MPU6050_IIC_ReceiveAck();
		}
	*/
	while(len--)
	{
		MPU6050_IIC_SendByte(*(Data++));
		MPU6050_IIC_ReceiveAck();
	}
	MPU6050_IIC_Stop();
	return 0;
}

uint8_t MPU6050_ReadReg_Str(uint8_t DevAddress, uint8_t RegAddress,uint8_t len,uint8_t *Data)
{
	MPU6050_IIC_Start();
	MPU6050_IIC_SendByte((DevAddress<<1)|0);
	MPU6050_IIC_ReceiveAck();//可以加判断，确保时序的正确
	MPU6050_IIC_SendByte(RegAddress);//指定要写入的寄存器
	MPU6050_IIC_ReceiveAck();
	
	MPU6050_IIC_Start();
	MPU6050_IIC_SendByte((DevAddress<<1)|1);//读写位改为1
	MPU6050_IIC_ReceiveAck();
	
	while(len)
	{
		*Data++=MPU6050_IIC_ReceiveByte();
		if(len==1)
		{
			
			MPU6050_IIC_SendAck(1);//mark:应答写错
		}
		else
		{
			MPU6050_IIC_SendAck(0);
		}
		len--;
	}
	MPU6050_IIC_Stop();
	return 0;
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	MPU6050_WriteReg_Byte(MPU6050_CONFIG,data);//设置数字低通滤波器  
}

//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Rate(u16 rate)
{
	unsigned char data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	MPU6050_WriteReg_Byte(MPU6050_SMPLRT_DIV,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}


void MPU6050_Init(void)
{
	MPU6050_IIC_Init();
	MPU6050_WriteReg_Byte(0x6B,0x80);//复位
	delay_ms(500);              //等待复位完成
	MPU6050_WriteReg_Byte(0x6B,0x00);//取消睡眠
	delay_ms(10);
	MPU6050_WriteReg_Byte(0x1B,0x00);//设置陀螺仪量程为500°/s
	delay_ms(10);
	MPU6050_WriteReg_Byte(0x1C,0x00);//设置加速度计量程2G
	delay_ms(10);
	MPU_Set_Rate(50);
	delay_ms(10);
	MPU6050_WriteReg_Byte(0x6B,0x02);//使用陀螺仪的晶振作为时钟
	delay_ms(10);
	MPU6050_WriteReg_Byte(0x6C,0x00);
	delay_ms(10);
}


uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg_Byte(MPU6050_WHO_AM_I);
}
//使用指针，实现函数多返回值的操作
void MPU6050_GetData(int16_t *AccX,int16_t *AccY,int16_t *AccZ,
						int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ)
{
	uint8_t DataH ,DataL;
	
	DataH=MPU6050_ReadReg_Byte(MPU6050_ACCEL_XOUT_H);
	DataL=MPU6050_ReadReg_Byte(MPU6050_ACCEL_XOUT_L);
	*AccX = (DataH<<8) | DataL;
	
	DataH=MPU6050_ReadReg_Byte(MPU6050_ACCEL_YOUT_H);
	DataL=MPU6050_ReadReg_Byte(MPU6050_ACCEL_YOUT_L);
	*AccY = (DataH<<8) | DataL;

	DataH=MPU6050_ReadReg_Byte(MPU6050_ACCEL_ZOUT_H);
	DataL=MPU6050_ReadReg_Byte(MPU6050_ACCEL_ZOUT_L);
	*AccZ = (DataH<<8) | DataL;
	
	DataH=MPU6050_ReadReg_Byte(MPU6050_GYRO_XOUT_H);
	DataL=MPU6050_ReadReg_Byte(MPU6050_GYRO_XOUT_L);
	*GyroX = (DataH<<8) | DataL;
	
	DataH=MPU6050_ReadReg_Byte(MPU6050_GYRO_YOUT_H);
	DataL=MPU6050_ReadReg_Byte(MPU6050_GYRO_YOUT_L);
	*GyroY = (DataH<<8) | DataL;
	
	DataH=MPU6050_ReadReg_Byte(MPU6050_GYRO_ZOUT_H);
	DataL=MPU6050_ReadReg_Byte(MPU6050_GYRO_ZOUT_L);
	*GyroZ = (DataH<<8) | DataL;
}

void MPU6050_GetData_T(int16_t *AccX,int16_t *AccY,int16_t *AccZ,
						int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ)
{
	uint8_t DataH ,DataL;
	
	MPU6050_ReadReg_Str(MPU6050_ADDRESS,MPU6050_ACCEL_XOUT_H,1,&DataH);
	MPU6050_ReadReg_Str(MPU6050_ADDRESS,MPU6050_ACCEL_XOUT_L,1,&DataL);
	*AccX = (DataH<<8) | DataL;
	
	MPU6050_ReadReg_Str(MPU6050_ADDRESS,MPU6050_ACCEL_YOUT_H,1,&DataH);
	MPU6050_ReadReg_Str(MPU6050_ADDRESS,MPU6050_ACCEL_YOUT_L,1,&DataL);
	*AccY = (DataH<<8) | DataL;

	MPU6050_ReadReg_Str(MPU6050_ADDRESS,MPU6050_ACCEL_ZOUT_H,1,&DataH);
	MPU6050_ReadReg_Str(MPU6050_ADDRESS,MPU6050_ACCEL_ZOUT_L,1,&DataL);
	*AccZ = (DataH<<8) | DataL;
	
	MPU6050_ReadReg_Str(MPU6050_ADDRESS,MPU6050_GYRO_XOUT_H,1,&DataH);
	MPU6050_ReadReg_Str(MPU6050_ADDRESS,MPU6050_GYRO_XOUT_L,1,&DataL);
	*GyroX = (DataH<<8) | DataL;
	
	MPU6050_ReadReg_Str(MPU6050_ADDRESS,MPU6050_GYRO_YOUT_H,1,&DataH);
	MPU6050_ReadReg_Str(MPU6050_ADDRESS,MPU6050_GYRO_YOUT_L,1,&DataL);
	*GyroY = (DataH<<8) | DataL;
	
	MPU6050_ReadReg_Str(MPU6050_ADDRESS,MPU6050_GYRO_ZOUT_H,1,&DataH);
	MPU6050_ReadReg_Str(MPU6050_ADDRESS,MPU6050_GYRO_ZOUT_L,1,&DataL);
	*GyroZ = (DataH<<8) | DataL;
}

