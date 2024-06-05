#ifndef __PID_H__
#define __PID_H__

float Vertical_PID(float Pitch);
float Velocity_PID(float velocity,float velocity_calcu);
float Turn_PID(float yaw,float yaw_calcu);
void Turn_PID_Init();
void Velociy_PID_Init();
void Vertical_PID_Init();
void I_amplitude_limiting(float number,float *Error_sum);
#endif