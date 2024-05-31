#ifndef __MPU6050_H__
#define __MPU6050_H__
#define q30  1073741824.0f
void MPU6050_WriteReg_Byte(uint8_t RegAddress,uint8_t Data);
uint8_t MPU6050_ReadReg_Byte(uint8_t RegAddress);
void MPU6050_Init(void);
void MPU6050_GetData(int16_t *AccX,int16_t *AccY,int16_t *AccZ,
						int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ);
uint8_t MPU6050_GetID(void);
uint8_t MPU6050_WriteReg_Str(uint8_t DevAddress, uint8_t RegAddress,uint8_t len,uint8_t *Data);
uint8_t MPU6050_ReadReg_Str(uint8_t DevAddress, uint8_t RegAddress,uint8_t len,uint8_t *Data);
void MPU6050_GetData_T(int16_t *AccX,int16_t *AccY,int16_t *AccZ,
						int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ);
void MPU6050_Pose(void);
void MPU6050_DMP_Init(void);
void LED_GPIO_Config(void);
#endif