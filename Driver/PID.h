#ifndef __PID_H__
#define __PID_H__

float Vertical_PID(float Pitch);
float Velocity_PID(float velocity);
void Velociy_PID_Init();
void Vertical_PID_Init();
void I_amplitude_limiting(float number,float *Error_sum);
#endif