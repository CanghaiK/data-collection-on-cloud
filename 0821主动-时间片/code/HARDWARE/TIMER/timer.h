#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 

//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   
#define  MinInterval  1


extern u8 cnt_1s;
extern u16 Interval_NUM;
void TIM3_Int_Init(u16 arr,u16 psc);
void TIM4_Int_Init(u16 arr,u16 psc);
void TIM5_Int_Init(u16 arr,u16 psc);
u8 Time_Init(void);
void Time_App(void);

 
#endif
