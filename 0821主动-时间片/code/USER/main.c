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
//产品：温岭项目demo
//作者：hs
//创建日期：20190624
************************************************/

/////////////////////////////////////////////////
/*
修改备注：
*/
/////////////////////////////////////////////////
//u8 get_cmd[8]={0x02,0x03,0x00,0x00,0x00,0x0A,0xC5,0xFE};
//u8 get_state[8]={0x02,0x03,0x00,0x0B,0x00,0x01,0xF5,0xFB};





 int main(void)
 {	
//	 SCB->VTOR = FLASH_BASE | 0x20000;
//	 u8 Send_flag=0;
	 	sys = &systm;			//系统
	  cfgsys = &cfg_sys_t;	//系统配置信息	4K
    uart_init(115200);
		M_printf("*******************\r\n");
		M_printf("        CJLU       \r\n");
		M_printf("     NYL1-DTU-4G   \r\n");
		M_printf("      2019-08-08   \r\n");
		M_printf("*******************\r\n");
		M_printf("USART1  ok.....\r\n");
		SysTick_Init();                                            //滴答计时器开启
		RS485_Init(9600);
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				    //设置中断优先级组为2，先占优先级和从优先级各两位(可设0～3) 
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

