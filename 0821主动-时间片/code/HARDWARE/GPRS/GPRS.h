#ifndef __GPRS_H
#define __GPRS_H			 
#include "sys.h"	 								  
//////////////////////////////////////////////////////////////////////////////////	 
								  
//////////////////////////////////////////////////////////////////////////////////
#include "stm32f10x.h"
//#include "sys.h"
//#include <stm32f10x.h>
//#include "func_com.h"
//#include <stdio.h>
//#include <string.h>
#include <stm32f10x.h>
#include "sys.h"
//#include "cmd_process.h"
#define DATA_BUF_LEN
#define CMD_HEAD "HEAD"
#define CMD_INFO_LEN sizeof(SDK_CMD_INFO)
	#if V3
#define Power_4G PBout(6)	// PB6
#endif 
	#if Mini
#define Power_4G PBout(6)	// PB6
#endif 
#define UART3_SendLR() UART3_SendString("\r\n")
#define GPRS_REV_BUF_LEN 30*1024 					  //串口2缓存长度

typedef struct
{
	u8 header[4];///数据头 48 45 41 44    "HEAD"assic码
	u16 len;///<数据长度 包括头
	u16 cmd_no;///<命令号
	u8 sn[4];  ///设备编号
	u32 second;
	u32 ms;
	u32 sys_num1;
	u32 sys_num2;
	u8 data[158];///<命令附加的数据
	u8 crc[2];///<crc16校验位  crc[0]低字节 crc[1] 高字节
}SDK_CMD_INFO;
extern u8 GPRSstaFlag;
extern u8 beatflag;
extern char Uart3_GPRS_Buf[GPRS_REV_BUF_LEN]; //串口2接收缓存
extern SDK_CMD_INFO sdk_cmd_info;
void USART3_Init_Config(u32 bound);
void UART3_SendString(char* s);
void USART3_IRQHandler(void);
void init_net_module(void);
static void Wait_CREG(void);
void connect_tcp_server(void);
static void init_net_config(void);
static u8 Find(const char *a);
static int8_t send_at_cmd_ack(char *s_at_cmd, char *s_cmd_ret,u8 wait_time,u16 cmd_len);
static void send_at_cmd_noack(char *s_at_cmd,u16 cmd_len);
static void connect_4G_tcp_server(void);
void GPRS_APP(void);
void Send_DATA(u8 *buf,u32 len);
u8 GPRS_send_DATA(u8 *p, u32 len);
u8 send_beat_to_sever(void);
//u8 send_data_to_sever(u8 num);
//u8 send_data_to_sever(u8 *buffer, u8 info);
void send_data_to_sever(u8 *buffer, u8 info ,u8 len);
void Start_4G(u8 num);
void GPRS_init(void);
void heartbeat(u8 num);
void gprs_rest(void);
u32 revl(u32 number);
#endif	   
















