#ifndef __PID_H
#define __PID_H
#include <stdint.h>
#include "stm32f4xx.h"
typedef struct{
	float Kp;
	float Ki;
	float Kd;
	
	float error[3];
	float u[2];
	float filter_encoder ; // qua bo loc thong thap 1/(0.1s +1) tin hieu encoder.
	uint8_t En;
	float duty ; 
	float speed_set;
	float speed[2] ;
}  pidUint;
extern TIM_HandleTypeDef htim4;
void Pid_Init_Left (pidUint *p);
void Pid_Init_Right (pidUint *p);
void reset_out_pwm (pidUint *p);
void he_so_Pid_right(void);
void he_so_Pid_left(void);
void Pid_Ctrl(pidUint *pleft, pidUint *pright);
//void Pid_Ctrl_left(pidUint *pleft);
//void Pid_Ctrl_right(pidUint *pright);
void Set_Pwm(float motorLeft,float motorRight);
float Pid_Common(pidUint *p,float speed_current,float set_point);
//float Pid_Common_left(pidUint *pleft,float speed_current_left,float set_point_left);
//float Pid_Common_right(pidUint *pright,float speed_current_right,float set_point_right);
#endif
