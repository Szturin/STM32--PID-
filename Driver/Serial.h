#ifndef __SERIAL_H
#define __SERIAL_H
#include <stdio.h>

extern char Serial_RxPacket[];
extern uint8_t Serial_RxFlag;
extern uint8_t RxState;
extern uint8_t pRxState;//表示当前接收的是第几个变量
extern uint8_t RxData_type;
extern uint8_t Sign_Flag;

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char * String);
void Serial_SendNumber(uint32_t Number,uint8_t Length);
void Serial_Printf(char * format,...);
void USART3_Init(void);

#endif