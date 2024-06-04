#include "stm32f10x.h"                  // Device header
#include "MyIIC.h"
#include "MPU6050_REG.h"
#include "MPU6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "math.h"
#include "delay.h"
#include "usart.h"
#define MPU6050_ADDRESS 	0x68

static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
float Pitch,Roll,Yaw;
unsigned long sensor_timestamp;
short gyro[3], accel[3], sensors;
unsigned char more;
long quat[4];

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
	MPU6050_WriteReg_Byte(PWR_MGMT_1,0x00);//解除休眠状态
	delay_ms(100);
	MPU6050_WriteReg_Byte(SMPLRT_DIV,0x07);
	delay_ms(50);
	MPU6050_WriteReg_Byte(CONFIG,0x06);
	delay_ms(50);
	MPU6050_WriteReg_Byte(GYRO_CONFIG,0x00);
	delay_ms(50);
	MPU6050_WriteReg_Byte(ACCEL_CONFIG,0x00);
	delay_ms(50); 	
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

void MPU6050_DMP_Init(void)
{
	int result=0;
	result=mpu_init();
	if(!result)
	{	 		 
	
		PrintChar("mpu initialization complete......\n ");		//mpu initialization complete	 	  

		if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))		//mpu_set_sensor
			PrintChar("mpu_set_sensor complete ......\n");
		else
			PrintChar("mpu_set_sensor come across error ......\n");

		if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))	//mpu_configure_fifo
			PrintChar("mpu_configure_fifo complete ......\n");
		else
			PrintChar("mpu_configure_fifo come across error ......\n");

		if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))	   	  		//mpu_set_sample_rate
		 PrintChar("mpu_set_sample_rate complete ......\n");
		else
		 	PrintChar("mpu_set_sample_rate error ......\n");

		if(!dmp_load_motion_driver_firmware())   	  			//dmp_load_motion_driver_firmvare
			PrintChar("dmp_load_motion_driver_firmware complete ......\n");
		else
			PrintChar("dmp_load_motion_driver_firmware come across error ......\n");

		if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) 	  //dmp_set_orientation
		 	PrintChar("dmp_set_orientation complete ......\n");
		else
		 	PrintChar("dmp_set_orientation come across error ......\n");

		if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
		    DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
		    DMP_FEATURE_GYRO_CAL))		   	 					 //dmp_enable_feature
		 	PrintChar("dmp_enable_feature complete ......\n");
		else
		 	PrintChar("dmp_enable_feature come across error ......\n");

		if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))   	 			 //dmp_set_fifo_rate
		 	PrintChar("dmp_set_fifo_rate complete ......\n");
		else
		 	PrintChar("dmp_set_fifo_rate come across error ......\n");

		run_self_test();		//自检

		if(!mpu_set_dmp_state(1))
		 	PrintChar("mpu_set_dmp_state complete ......\n");
		else
		 	PrintChar("mpu_set_dmp_state come across error ......\n");
	}
	else												 //MPU6050状态指示灯 STM32核心板 PC13 绿色灯亮起为不正常
	 {
	 GPIO_ResetBits(GPIOC, GPIO_Pin_13);				//MPU6050状态指示灯 STM32核心板 PC13 绿色灯亮起为不正常
	 while(1);
	 }
	 
}


void MPU6050_Pose(void)
{
	
	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more);	 
	/* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
	 * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
	**/
	/*if (sensors & INV_XYZ_GYRO )
	send_packet(PACKET_TYPE_GYRO, gyro);
	if (sensors & INV_XYZ_ACCEL)
	send_packet(PACKET_TYPE_ACCEL, accel); */
	/* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
	 * The orientation is set by the scalar passed to dmp_set_orientation during initialization. 
	**/
	
	
	if(sensors & INV_WXYZ_QUAT )
	{
		q0 = quat[0] / q30;	
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30;

		Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
		Roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)*57.3;	// roll
		Yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
    
		
	}
}

/*
 * 函数名：LED_GPIO_Config
 * 描述  ：配置LED用到的I/O口
 * 输入  ：无
 * 输出  ：无
 */
void LED_GPIO_Config(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*先开启GPIOB和AFIO的外设时钟*/
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC| RCC_APB2Periph_AFIO,ENABLE);

	/*改变指定管脚的映射 GPIO_Remap_SWJ_JTAGDisable ，JTAG-DP 禁用 + SW-DP 使能*/
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE); 

	/*选择要控制的GPIOB引脚*/															   
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 |GPIO_Pin_14;
	
	/*设置引脚速率为50MHz */   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 	

	/*设置引脚模式为通用推挽输出*/
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*调用库函数，初始化GPIOB*/
  	GPIO_Init(GPIOC, &GPIO_InitStructure);			  

	/* 打开所有led灯	*/
	GPIO_SetBits(GPIOC, GPIO_Pin_13|GPIO_Pin_14); 
//	GPIO_ResetBits(GPIOC, GPIO_Pin_13|GPIO_Pin_14); 
}

