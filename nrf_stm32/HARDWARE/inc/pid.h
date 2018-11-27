#ifndef __PID_H__
#define __PID_H__
#include "stm32f10x.h"

void PID_Motor_Init(void);						//设置电机PID参数的初始值
int32_t IncPIDCalc_Motor(s32 NextPoint);		//电机PID运算得出增量值

void PID_Steer_Parameter_1(void);

#endif
