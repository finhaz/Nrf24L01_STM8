#include "PID.h"

#define Kp   1.4        //PID���ڵı�������
#define Ti   0.04     //PID���ڵĻ��ֳ���
#define Td   0     //PID���ڵ�΢��ʱ�䳣��
#define T    0.02     //��������

#define Kpp   Kp * ( 1 + (T / Ti) + (Td / T) )
#define Ki   	Kp * ( 1 + (2 * Td / T ) )
#define Kd    Kp * Td / T




//-----------------------���PID����-----------------------------
volatile double p_motor = Kpp;
volatile double i_motor = Ki;
volatile double d_motor = Kd;
volatile int SETPOINT_MOTOR = 270;		 //  ��/100ms




extern volatile uint32_t Car_Speed;

//����PID�ṹ�壨����ʽ��
typedef struct PID
{
    int SetPoint;      			//�趨Ŀ��Desired Value
    double Proportion; 			//��������Proportional Const
    double Integral; 			//���ֳ���Integral Const
    double Derivative; 			//΢�ֳ���Derivative Const
    int LastError;    			//Error[-1]
    int PrevError;    			//Error[-2]
} PID;



volatile static PID sPID_motor;
volatile static PID *sptr_motor = &sPID_motor;



//--------------------------���PID------------------------------
//������:PID_Motor_Init
//��  ��:�����PID��ʼ���������������õ��PID�ĳ�ʼֵ
//����ֵ:��
//��  ��:���õ��PID�����ĳ�ʼֵ
void PID_Motor_Init(void)
{
	sptr_motor->LastError = 0;       		//Error[-1]
	sptr_motor->PrevError = 0;       		//Error[-2]
	sptr_motor->Proportion = p_motor;  		//��������Proportional Const
	sptr_motor->Integral = i_motor;    		//���ֳ���Integral Const
	sptr_motor->Derivative = d_motor;  		//΢�ֳ���Derivative Const
	sptr_motor->SetPoint = SETPOINT_MOTOR; 	//
}


//������:IncPIDCalc_Motor
//��  ��:NextPoint���趨��ʵ��ֵ
//����ֵ:��������ֵ
//��  ��:���PID����ó�����ֵ
int32_t IncPIDCalc_Motor(int NextPoint)
{
	int32_t iError, iIncpid;                       				//��ǰ���
	iError = sptr_motor->SetPoint - NextPoint;            		//��������
   
	iIncpid = sptr_motor->Proportion * iError             		//E[k]��
             - sptr_motor->Integral * sptr_motor->LastError    	//E[k-1]��
             + sptr_motor->Derivative * sptr_motor->PrevError; 	//E[k-2]��
    sptr_motor->PrevError = sptr_motor->LastError;  			//�洢�������´μ���
    sptr_motor->LastError = iError;           					//�洢�������´μ���

    return(iIncpid);                    						//��������ֵ
}










