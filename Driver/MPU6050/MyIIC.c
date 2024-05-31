#include "stm32f10x.h"                  // Device header
#include <delay.h>

#define SDA_PORT	GPIOB
#define SDA_PIN 	GPIO_Pin_9
#define SCL_PORT	GPIOB
#define SCL_PIN		GPIO_Pin_8

static void I2C_delay(void)
{
    volatile int i = 7;
    while (i)
        i--;
}

void MPU6050_IIC_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB,SCL_PIN,(BitAction)BitValue);//BitAction是枚举变量类型,包括RESET和SET两种变量
	I2C_delay();
}	

void MPU6050_IIC_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB,SDA_PIN,(BitAction)BitValue);//BitAction是枚举变量类型,包括RESET和SET两种变量
	I2C_delay();
}

uint8_t MPU6050_IIC_R_SDA(void)
{
	uint8_t BitValue;
	BitValue = GPIO_ReadInputDataBit(GPIOB,SDA_PIN);
	I2C_delay();
	return BitValue;
}

void MPU6050_IIC_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure; //结构体类型(已经定义好的） 结构体变量名 ->结构体变量的定义
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin=SDA_PIN|SCL_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;//开漏输出模式
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB,SDA_PIN|SCL_PIN);//初始化默认低电平输出，所以要置高电平	
}

void MPU6050_IIC_Start(void)
{
	MPU6050_IIC_W_SDA(1);//MARK:先把SDA拉高，因为SDA的先前状态无法确定，一旦在不确定的情况下拉高SCL会导致出现起始条件的错误判断!!!
	MPU6050_IIC_W_SCL(1);
	MPU6050_IIC_W_SDA(0);
	MPU6050_IIC_W_SCL(0);
}

void MPU6050_IIC_Stop(void)//仅有停止单元SDA以高电平结束
{
	MPU6050_IIC_W_SDA(0);//与起始信号同理，SDA先前状态无法确定
	MPU6050_IIC_W_SCL(1);
	MPU6050_IIC_W_SDA(1);	
}

void MPU6050_IIC_SendByte(uint8_t Byte)
{
	uint8_t i;
	for(i=0;i<8;i++)
	{
		MPU6050_IIC_W_SDA(Byte & (0x80 >> i));//此函数传参后参数效果实际上“非0即1”
		MPU6050_IIC_W_SCL(1);
		MPU6050_IIC_W_SCL(0);	
	}
}

uint8_t MPU6050_IIC_ReceiveByte(void)
{
	uint8_t Byte = 0x00;
	uint8_t i;
	MPU6050_IIC_W_SDA(1);//主机释放SDA
	for(i = 0; i<8; i++)
	{
		MPU6050_IIC_W_SCL(1);//主机释放SCL
		if(MPU6050_IIC_R_SDA() == 1){Byte |= (0x80>>i);}
		MPU6050_IIC_W_SCL(0);		
	}
	return Byte;
}

void MPU6050_IIC_SendAck(uint8_t AckBit)
{

	MPU6050_IIC_W_SDA(AckBit);//此函数传参后参数效果实际上“非0即1”
	MPU6050_IIC_W_SCL(1);
	MPU6050_IIC_W_SCL(0);	
}

uint8_t MPU6050_IIC_ReceiveAck(void)
{
	uint8_t AckBit;
	MPU6050_IIC_W_SDA(1);//主机释放SDA,SDA立即转交从机控制
	MPU6050_IIC_W_SCL(1);//主机释放SCL,防止干扰从机
	AckBit = MPU6050_IIC_R_SDA();//思考：为什么前面SDA设置1，这里读不会一定为1？	
	MPU6050_IIC_W_SCL(0);
	return AckBit;
}
