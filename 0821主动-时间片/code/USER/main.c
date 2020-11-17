#if  1
#include "stm32f10x.h"
#include "sys.h"
#include "usart.h"
#include "GPRS.h"
#include "rs485.h"
#include "timer.h"
#include "w25qxx.h"

/************************************************
//DTU  V1.0  20190624
//��Ʒ��������Ŀdemo
//���ߣ�hs
//�������ڣ�20190624
************************************************/

/////////////////////////////////////////////////
/*
�޸ı�ע��
*/
/////////////////////////////////////////////////
//u8 get_cmd[8]={0x02,0x03,0x00,0x00,0x00,0x0A,0xC5,0xFE};
//u8 get_state[8]={0x02,0x03,0x00,0x0B,0x00,0x01,0xF5,0xFB};





 int main(void)
 {	
//	 SCB->VTOR = FLASH_BASE | 0x20000;
//	 u8 Send_flag=0;
	 	sys = &systm;			//ϵͳ
	  cfgsys = &cfg_sys_t;	//ϵͳ������Ϣ	4K
    uart_init(115200);
		M_printf("*******************\r\n");
		M_printf("        CJLU       \r\n");
		M_printf("     NYL1-DTU-4G   \r\n");
		M_printf("      2019-08-08   \r\n");
		M_printf("*******************\r\n");
		M_printf("USART1  ok.....\r\n");
		SysTick_Init();                                            //�δ��ʱ������
		RS485_Init(9600);
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				    //�����ж����ȼ���Ϊ2����ռ���ȼ��ʹ����ȼ�����λ(����0��3) 
		M_printf("NVIC    ok.....\r\n");
		//	 W25QXX_Init(); 
		Cf_init();
		M_printf("TIMER   ok.....\r\n");
	 
		 while(1)
		{
				processRunningMode();
		}
 }
 #endif

