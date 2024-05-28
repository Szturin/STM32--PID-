#include "stm32f10x.h"                  // Device header
#include "Delay.h"
//起始 终止 发送一个字节 接受一个字节 发送应答 接收应答
#define SDA_PORT	GPIOB
#define SDA_PIN 	GPIO_Pin_9
#define SCL_PORT	GPIOB
#define SCL_PIN		GPIO_Pin_8
struct MPU6050{
	short int accx;
	short int accy;
	short int accz;
	short int gyrox;
	short int gyroy;
	short int gyroz;
	short int temp;
};

void MPU6050_I2C_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure; //结构体类型(已经定义好的） 结构体变量名 ->结构体变量的定义
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin=SDA_PIN|SCL_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;//开漏输出模式
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB,SDA_PIN|SCL_PIN);//初始化默认低电平输出，所以要置高电平	
}

void MPU6050_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, SCL_PIN, (BitAction)BitValue);

}

void MPU6050_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, SDA_PIN, (BitAction)BitValue);
	
}

uint8_t MPU6050_R_SDA(void)
{
	uint8_t Flag;
	Flag = GPIO_ReadInputDataBit(GPIOB, SDA_PIN);
	return Flag;
}
	
void MPU6050_I2C_S(void)
{
	MPU6050_W_SDA(1);
	MPU6050_W_SCL(1);
	
	MPU6050_W_SDA(0);
	MPU6050_W_SCL(0);
}

void MPU6050_I2C_P(void)
{
	MPU6050_W_SDA(0);
	
	MPU6050_W_SCL(1);
	MPU6050_W_SDA(1);
}

void MPU6050_I2C_SendByte(uint8_t byte)
{
	for(uint8_t i=0;i<8;i++)
	{
		MPU6050_W_SDA(byte & (0x80>>i));
		MPU6050_W_SCL(1);
		MPU6050_W_SCL(0);
	}
}

uint8_t MPU6050_I2C_ReadByte(void)
{
	uint8_t byte=0x00;
	MPU6050_W_SDA(1);
	for(uint8_t i=0;i<8;i++)
	{
		MPU6050_W_SCL(1);
		if(MPU6050_R_SDA()==1) byte |= 0x80>>i;
		MPU6050_W_SCL(0);
	}
	return byte;
}

void MPU6050_I2C_SendAck(uint8_t Ack)
{
	MPU6050_W_SDA(Ack);
	MPU6050_W_SCL(1);
	MPU6050_W_SCL(0);
}

uint8_t MPU6050_I2C_ReadAck(void)
{
	uint8_t Ack;
	MPU6050_W_SDA(1);

	MPU6050_W_SCL(1);
	Ack = MPU6050_R_SDA();
	MPU6050_W_SCL(0);
	return Ack;
}

void MPU6050_WriteReg(uint8_t Reg, uint8_t data)
{
	MPU6050_I2C_S();
	MPU6050_I2C_SendByte(0XD0);
	MPU6050_I2C_ReadAck();
	MPU6050_I2C_SendByte(Reg);
	MPU6050_I2C_ReadAck();
	MPU6050_I2C_SendByte(data);
	MPU6050_I2C_ReadAck();
	MPU6050_I2C_P();
}

uint8_t MPU6050_ReadReg(uint8_t reg)
{
	uint8_t Data;
	MPU6050_I2C_S();
	MPU6050_I2C_SendByte(0XD0);
	MPU6050_I2C_ReadAck();
	MPU6050_I2C_SendByte(reg);
	MPU6050_I2C_ReadAck();
	MPU6050_I2C_S();
	MPU6050_I2C_SendByte(0XD1);
	MPU6050_I2C_ReadAck();
	Data = MPU6050_I2C_ReadByte();
	MPU6050_I2C_SendAck(1);
	MPU6050_I2C_P();
	
	return Data;
}

uint8_t MPU6050_Burst_Write(uint8_t Addr, uint8_t reg, uint8_t len, uint8_t const *buf)
{
	
	MPU6050_I2C_S();
	MPU6050_I2C_SendByte((Addr<<1)|0);
	MPU6050_I2C_ReadAck();
	MPU6050_I2C_SendByte(reg);
	MPU6050_I2C_ReadAck();
	
	while(len--)
	{
		MPU6050_I2C_SendByte(*(buf++));
		MPU6050_I2C_ReadAck();
	}
	MPU6050_I2C_P();
	return 0;
}
	
/**
* 函    数：MPU6050连续读寄存器
* 参    数：Addr:0X68
			reg:当前寄存器地址
			len:要读取的寄存器长度
			buf:储存数据的数组
* 返 回 值：0
* 说    明：适配mpu6050 dmp库的连续读寄存器代码
*/

uint8_t MPU6050_Burst_Read(uint8_t Addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	MPU6050_I2C_S();
	MPU6050_I2C_SendByte((Addr<<1)|0);
	MPU6050_I2C_ReadAck();
	MPU6050_I2C_SendByte(reg);
	MPU6050_I2C_ReadAck();
	MPU6050_I2C_S();
	MPU6050_I2C_SendByte((Addr<<1)|1);
	MPU6050_I2C_ReadAck();
	while(len)
	{
		*(buf++) = MPU6050_I2C_ReadByte();

		switch(len){
			case 1:MPU6050_I2C_SendAck(1);break;
			default:MPU6050_I2C_SendAck(0);break;
		}
		len--;
	}
	MPU6050_I2C_P();
	return 0;
}

void MPU_6050_Init(void)
{
	MPU6050_I2C_Init();
	MPU6050_WriteReg(0x6B,0x80);//复位
	delay_ms(500);              //等待复位完成
	MPU6050_WriteReg(0x6B,0x00);//取消睡眠
	MPU6050_WriteReg(0x1B,0x08);//设置陀螺仪量程为500°/s
	MPU6050_WriteReg(0x1C,0x00);//设置加速度计量程2G
	
	MPU6050_WriteReg(0x1A,0x04);//配置滤波器参数
	MPU6050_WriteReg(0x19,0x00);//配置分频频率不分频，1000khz输出
	
	MPU6050_WriteReg(0x6B,0x02);//使用陀螺仪的晶振作为时钟
	MPU6050_WriteReg(0x6C,0x00);
}


void MPU_6050_GetData(int16_t *buf) 
{
	uint8_t i;
	uint8_t temp[14]={0};
	MPU6050_Burst_Read(0x68, 0x3B, 14, temp);
	for(i=0; i<14; i+=2)
	{
		buf[i/2] = (temp[i]<<8)|temp[i+1];
	}
}
