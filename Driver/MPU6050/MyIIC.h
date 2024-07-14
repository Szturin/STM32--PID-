#ifndef __MyIIC_H__
#define __MyIIC_H__
void MPU6050_IIC_Init(void);
void MPU6050_IIC_Start(void);
void MPU6050_IIC_Stop(void);//仅有停止单元SDA以高电平结束
void MPU6050_IIC_SendByte(uint8_t Byte);
uint8_t MPU6050_IIC_ReceiveByte(void);
void MPU6050_IIC_SendAck(uint8_t AckBit);
uint8_t MPU6050_IIC_ReceiveAck(void);
#endif