#ifndef __RS485_H
#define __RS485_H			 
#include "sys.h"	 								  
//////////////////////////////////////////////////////////////////////////////////	 
							  
//////////////////////////////////////////////////////////////////////////////////
	
extern u8 RS485_RX_BUF[160]; 		//���ջ���,���64���ֽ�
extern u8 RS485_RX_CNT;   			//���յ������ݳ���
extern u8 RS485Flag;            //485�˿����ݽ������

#if V3
//ģʽ����
#define RS485_TX_EN		PDout(7)	//485ģʽ����.0,����;1,����.
#endif 
 
 #if Mini
//ģʽ����
#define RS485_TX_EN		PAout(4)	//485ģʽ����.0,����;1,����.
#endif 
//����봮���жϽ��գ��벻Ҫע�����º궨��
#define EN_USART2_RX 	1			//0,������;1,����.

void RS485_Init(u32 bound);
void RS485_Send_Data(u8 *buf,u8 len);
void RS485_Receive_Data(u8 *buf,u8 *len);
void Clear_rs485_buf(void);
void Clear_databuff(void);

#endif	   
















