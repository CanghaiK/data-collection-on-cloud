#include "delay.h"
#include "sys.h"
//��ʱ�������������Ҫ�Ǿ�׼


// us��ʱ����
void delay_us(u32 nus)
{
	u32 tmp;
	u32 count=nus*6 ;
	for(tmp=0;tmp<count;tmp++);
}
//ms��ʱ����
void delay_ms(u16 nms)
{	 		  	  
	u16	tmp;
	for(tmp=0;tmp<nms;tmp++)
	{
		delay_us(1000);
	}
} 
//s��ʱ����
void delay_s(u16 ns)
{
	u16 i;
	for(i=0;i<ns;i++)
	{
		delay_ms (1000);
	}
}



