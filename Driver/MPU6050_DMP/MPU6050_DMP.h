#ifndef __MPU6050_DMP_H__
#define __MPU6050_DMP_H__
int MPU6050_DMP_Init(void);
uint8_t MPU_DMP_Get_Data(float *pitch,float *roll,float *yaw);
#endif