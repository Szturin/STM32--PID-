#ifndef __ENCODER_H__
#define __ENCODER_H__

void Encoder1_Init(void);
void Encoder2_Init(void);
	
int16_t Encoder1_GetCounter(void);
int16_t Encoder2_GetCounter(void);

#endif