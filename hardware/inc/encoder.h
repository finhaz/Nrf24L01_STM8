#ifndef __ENCODER_H
#define __ENCODER_H

#define ECDPeriod      20000
#define prd     ECDPeriod
#define Vbreak  ECDPeriod/2



/* ----------------------- Data Struct ------------------------------------- */ 


typedef struct
{
	s16 V2;
	s16 V3;
	s16 V4;
	s16 V5;
	s16 cnt2;
	s16 cnt3;
	s16 cnt4;
	s16 cnt5;
	s16 rcnt2;
	s16 rcnt3;
	s16 rcnt4;
	s16 rcnt5;
	s32 CNT2;
	s32 CNT3;
	s32 CNT4;
	s32 CNT5;
}EncoderType;



void Init_Timer1(void);
void Get_Encoder_ONE(void);



#endif