#ifndef __MOTOR_H__
#define __MOTOR_H__
void Motor_PWM_Init(void);
void Motor_SpeedSet_Vertical(int16_t Speed);
void Motor_GPIO_Init();
void motor_gpio_init();
void Motor_PWMSet(int16_t PWM1,int16_t PWM2);
#endif