#ifndef __RS485_H
#define __RS485_H			 
#include "sys.h"	 								  
//////////////////////////////////////////////////////////////////////////////////	 
							  
//////////////////////////////////////////////////////////////////////////////////
	
extern u8 RS485_RX_BUF[160]; 		//接收缓冲,最大64个字节
extern u8 RS485_RX_CNT;   			//接收到的数据长度
extern u8 RS485Flag;            //485端口数据接收完成

#if V3
//模式控制
#define RS485_TX_EN		PDout(7)	//485模式控制.0,接收;1,发送.
#endif 
 
 #if Mini
//模式控制
#define RS485_TX_EN		PAout(4)	//485模式控制.0,接收;1,发送.
#endif 
//如果想串口中断接收，请不要注释以下宏定义
#define EN_USART2_RX 	1			//0,不接收;1,接收.

void RS485_Init(u32 bound);
void RS485_Send_Data(u8 *buf,u8 len);
void RS485_Receive_Data(u8 *buf,u8 *len);
void Clear_rs485_buf(void);
void Clear_databuff(void);

#endif	   
















