#include "delay.h"
#include "sys.h"
//延时函数满足基本需要非精准


// us延时函数
void delay_us(u32 nus)
{
	u32 tmp;
	u32 count=nus*6 ;
	for(tmp=0;tmp<count;tmp++);
}
//ms延时函数
void delay_ms(u16 nms)
{	 		  	  
	u16	tmp;
	for(tmp=0;tmp<nms;tmp++)
	{
		delay_us(1000);
	}
} 
//s延时函数
void delay_s(u16 ns)
{
	u16 i;
	for(i=0;i<ns;i++)
	{
		delay_ms (1000);
	}
}



