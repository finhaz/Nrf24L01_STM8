#include "PID.h"

#define Kp   1.4        //PID调节的比例常数
#define Ti   0.04     //PID调节的积分常数
#define Td   0     //PID调节的微分时间常数
#define T    0.02     //采样周期

#define Kpp   Kp * ( 1 + (T / Ti) + (Td / T) )
#define Ki   	Kp * ( 1 + (2 * Td / T ) )
#define Kd    Kp * Td / T




//-----------------------电机PID参数-----------------------------
volatile double p_motor = Kpp;
volatile double i_motor = Ki;
volatile double d_motor = Kd;
volatile int SETPOINT_MOTOR = 270;		 //  线/100ms




extern volatile uint32_t Car_Speed;

//定义PID结构体（增量式）
typedef struct PID
{
    int SetPoint;      			//设定目标Desired Value
    double Proportion; 			//比例常数Proportional Const
    double Integral; 			//积分常数Integral Const
    double Derivative; 			//微分常数Derivative Const
    int LastError;    			//Error[-1]
    int PrevError;    			//Error[-2]
} PID;



volatile static PID sPID_motor;
volatile static PID *sptr_motor = &sPID_motor;



//--------------------------电机PID------------------------------
//函数名:PID_Motor_Init
//参  数:电机的PID初始化函数，用于设置电机PID的初始值
//返回值:无
//功  能:设置电机PID参数的初始值
void PID_Motor_Init(void)
{
	sptr_motor->LastError = 0;       		//Error[-1]
	sptr_motor->PrevError = 0;       		//Error[-2]
	sptr_motor->Proportion = p_motor;  		//比例常数Proportional Const
	sptr_motor->Integral = i_motor;    		//积分常数Integral Const
	sptr_motor->Derivative = d_motor;  		//微分常数Derivative Const
	sptr_motor->SetPoint = SETPOINT_MOTOR; 	//
}


//函数名:IncPIDCalc_Motor
//参  数:NextPoint：设定的实测值
//返回值:返回增量值
//功  能:电机PID运算得出增量值
int32_t IncPIDCalc_Motor(int NextPoint)
{
	int32_t iError, iIncpid;                       				//当前误差
	iError = sptr_motor->SetPoint - NextPoint;            		//增量计算
   
	iIncpid = sptr_motor->Proportion * iError             		//E[k]项
             - sptr_motor->Integral * sptr_motor->LastError    	//E[k-1]项
             + sptr_motor->Derivative * sptr_motor->PrevError; 	//E[k-2]项
    sptr_motor->PrevError = sptr_motor->LastError;  			//存储误差，用于下次计算
    sptr_motor->LastError = iError;           					//存储误差，用于下次计算

    return(iIncpid);                    						//返回增量值
}










