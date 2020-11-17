#include "sys.h"		    
#include "GPRS.h"	 
#include "delay.h"
#include "rs485.h"
#include "timer.h"
////////////////////////////////////////////////////////////////////////////////////	 
////ͨ��ģ�����ģʽ/Air720/EC20/SIM7000a								  
////////////////////////////////////////////////////////////////////////////////////



//#define GPRS_REV_BUF_LEN 30*1024 					  //����2���泤��
#define TCP_SERVER_INFO "AT+CIPSTART=\"TCP\",\"180.76.153.66\",8181\r\n"

u8 GPRSstaFlag=0;//1��TCP��������  0��TCP�����쳣������
u8 beatflag=0;
char Uart3_GPRS_Buf[GPRS_REV_BUF_LEN]; //����2���ջ���
char *p_gprs_buf; 
char *p_gprs_rd = NULL;
char RX_buff[50];
SDK_CMD_INFO Uatdata1,Iatdata2;


/*******************************************************************************
* ������  : USART3_Init_Config
* ����    : USART3��ʼ������
* ����    : bound��������(���ã�2400��4800��9600��19200��38400��115200��)
* ���    : ��
* ����    : �� 
* ˵��    : ��
*******************************************************************************/
void USART3_Init_Config(u32 bound)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		
		/*ʹ��USART2����ʱ��*/  
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA, ENABLE);	
		/*��λ����2*/
		USART_DeInit(USART3);  
#if V3		
	 /*4gģ�鿪���������ų�ʼ��*/
	 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;				 //PB6�˿�����
 	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	  GPIO_Init(GPIOB, &GPIO_InitStructure);				 //��ʼ��IO��
   	GPIO_SetBits(GPIOB,GPIO_Pin_6);						 //PA5 ����� �˴������޸�io��
	
	
	/*USART3_GPIO��ʼ������*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			//USART2_TXD(PB.10)     
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		//�����������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//������������������Ϊ50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);				//���ÿ⺯���е�GPIO��ʼ����������ʼ��USART1_TXD(PB.10)  
     PBout(6)=0;//
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				//USART2_RXD(PB.11)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//��������
    GPIO_Init(GPIOB, &GPIO_InitStructure);					//���ÿ⺯���е�GPIO��ʼ����������ʼ��USART1_RXD(PB.11)
	#endif
	
	#if Mini
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;				 //PB6�˿�����
 	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	  GPIO_Init(GPIOB, &GPIO_InitStructure);				 //��ʼ��IO��
   	GPIO_SetBits(GPIOB,GPIO_Pin_6);						 //PA5 ����� �˴������޸�io��
	/*USART3_GPIO��ʼ������*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			//USART2_TXD(PB.10)     
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		//�����������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//������������������Ϊ50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);				//���ÿ⺯���е�GPIO��ʼ����������ʼ��USART1_TXD(PB.10)  
     PBout(6)=0;//
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				//USART2_RXD(PB.11)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//��������
    GPIO_Init(GPIOB, &GPIO_InitStructure);					//���ÿ⺯���е�GPIO��ʼ����������ʼ��USART1_RXD(PB.11)
	#endif

		 /*USART2 ��ʼ������*/
		USART_InitStructure.USART_BaudRate = bound;										//���ò�����
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//8λ���ݸ�ʽ
		USART_InitStructure.USART_StopBits = USART_StopBits_1;							//1��ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;								//����żУ��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//��Ӳ������������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//����ģʽ����Ϊ�շ�ģʽ
		USART_Init(USART3, &USART_InitStructure);		//��ʼ������3
		
		//�жϳ�ʼ��
		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; //ʹ�ܴ���2�ж�
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //��ռ���ȼ�2��
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; //�����ȼ�2��
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ʹ���ⲿ�ж�ͨ��
		NVIC_Init(&NVIC_InitStructure); //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
		
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);			//ʹ�ܴ���3�����ж�

		USART_Cmd(USART3, ENABLE);                    			//ʹ�ܴ��� 
		USART_ClearFlag(USART3, USART_FLAG_TC);					//���������ɱ�־
		M_printf("USART3 Init ok !\r\n");
}
/*******************************************************************************
* ������  : UART3_SendString
* ����    : USART3�����ַ���
* ����    : *s�ַ���ָ��
* ���    : ��
* ����    : �� 
* ˵��    : ��
*******************************************************************************/
void UART3_SendString(char* s)
{
	if(cfgsys->LogLev>=SIMPLELOG)
	{
		M_printf (s);
		M_printf("\r\n");
	}
	
	while(*s != '\0')//����ַ���������
	{
		while(USART_GetFlagStatus(USART3, USART_FLAG_TC)==RESET); 
		USART_SendData(USART3 ,*s++);//���͵�ǰ�ַ�
	}
}
///*******************************************************************************
//* ������  : USART3_IRQHandler
//* ����    : ����3�жϷ������
//* ����    : ��
//* ����    : �� 
//* ˵��    : 
//*******************************************************************************/
void USART3_IRQHandler(void)               	
{
		u8 Res=0;
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
		{
			Res=USART_ReceiveData(USART3);
			*p_gprs_buf = Res;  	  //�����յ����ַ����浽������
			p_gprs_buf++;                			//����ָ������ƶ�
			if(p_gprs_buf>=&Uart3_GPRS_Buf[GPRS_REV_BUF_LEN])       		//���������,������ָ��ָ�򻺴���׵�ַ
			{
				p_gprs_buf=Uart3_GPRS_Buf;
			} 
		}			
} 
/*******************************************************************************
* ������ : crc16_check
* ����   : crc16У��
* ����   : 
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/
uint16_t crc16_check(uint8_t *Pushdata,uint8_t length)
{
    uint16_t Reg_CRC=0xffff;
    uint8_t i,j;
    for( i = 0; i<length; i ++)
    {
           Reg_CRC^= *Pushdata++;
           for (j = 0; j<8; j++)
           {
            if (Reg_CRC & 0x0001)
                Reg_CRC=Reg_CRC>>1^0xA001;
            else
                Reg_CRC >>=1;
           }
    }
    return Reg_CRC;
}

///*******************************************************************************
//* ������ : revl
//* ����   : u32�ֽ���ת��
//* ����   : 
//* ���   : 
//* ����   : 
//* ע��   : 
//*******************************************************************************/
u32 revl(u32 number)
{
	u32 temp = 0;
	temp = ((number&0x000000ff) << 24) + ((number&0x0000ff00) << 8) + 
	((number&0x00ff0000) >> 8) + ((number&0xff000000) >> 24);
	return temp;
}

/*******************************************************************************
* ������ : revs
* ����   : u16�ֽ���ת��
* ����   : 
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/
u16 revs(u16 number)
{
	u32 temp = 0;
	temp = ((number&0xff00) >> 8) + ((number&0x00ff) << 8);
	return temp;
}
/*******************************************************************************
* ������ : CLR_GPRS_Buf
* ����   : �������2��������
* ����   : 
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/
static void CLR_GPRS_Buf(void)
{
	u16 k;
	for(k=0;k<GPRS_REV_BUF_LEN;k++)      //��������������
	{
		Uart3_GPRS_Buf[k] = 0x00;
	}
  p_gprs_buf=Uart3_GPRS_Buf;               //�����ַ�������ʼ�洢λ��
	p_gprs_rd = Uart3_GPRS_Buf;    //��ʼ����ָ��
}


///*******************************************************************************
//* ������ : Start_4G
//* ����   : 4Gģ������
//* ����   : u8
//* ���   : 
//* ����   : 
//* ע��   : 
//*******************************************************************************/
void Start_4G(u8 num)
{
	if(0==num)
		M_printf("4G  Starting********\r\n");
	else 
		M_printf("4G  ReStarting********\r\n");
	Power_4G=0;
	delay_s(1);
	Power_4G=1;
	delay_s (30);
}

/*******************************************************************************
* ������ : Find
* ����   : �жϻ������Ƿ���ָ�����ַ���
* ����   : 
* ���   : 
* ����   : unsigned char:1 �ҵ�ָ���ַ���0 δ�ҵ�ָ���ַ� 
* ע��   : 
*******************************************************************************/
static u8 Find(const char *a)
{ 
  if(strstr(Uart3_GPRS_Buf,a)!=NULL)
	{
//			M_printf("Find %s successed.....\r\n",a);
	    return 1;
	}

	else
	{
//		M_printf("Find %s failed.....\r\n",a);
			return 0;
	}
}

#if 0
//����ֵ��1�����ͳɹ�
//   			0������ʧ��
static int8_t send_at_cmd_ack(char *s_at_cmd, char *s_cmd_ret,u8 wait_time,u16 cmd_len)         
{
	u8 i,j=0;
	int8_t ret = ERROR;
//	M_printf(s_at_cmd);
	CLR_GPRS_Buf(); 
  i = 0;
	while(i <= wait_time)                    
	{
		if(!Find(s_cmd_ret)) 
		{
				delay_ms(2000); 
				for (j=0; *s_at_cmd!='\0' && j < cmd_len;s_at_cmd++,j++)
				{
					while(USART_GetFlagStatus(USART3, USART_FLAG_TC)==RESET);
					USART_SendData(USART3,*s_at_cmd);//UART3_SendData(*b);
				}
//				UART3_SendLR();
				delay_ms(1000); 
//				UART1_SendString(Uart3_GPRS_Buf);///<���Դ�����ʾ4Gģ�鷵��ֵ
				M_printf("GPRS[rec]:%s\r\n",Uart3_GPRS_Buf);
				i++;
    }
 	  else
		{
			M_printf("cmd  send  successed.....\r\n");
			ret = 1;//���ͳɹ�
			break;
		}
		M_printf("cmd  send  failed.....\r\n");
		ret = 0;//����ʧ��
	}
	M_printf("\r\n");
	CLR_GPRS_Buf(); 
	return ret;
}
#endif 
/*******************************************************************************
* ������ : send_at_cmd_noack
* ����   : ����ATָ���
* ����   : �������ݵ�ָ�롢���͵ȴ�ʱ��(��λ��S)
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/

static void send_at_cmd_noack(char *s_at_cmd,u16 cmd_len)         
{
	#if 1
	u16 i;
	CLR_GPRS_Buf(); 
//	M_printf(s_at_cmd);
//	M_printf("\r\n");
	for (i=0; *s_at_cmd!='\0' && i < cmd_len;s_at_cmd++,i++)
	{
		while(USART_GetFlagStatus(USART3, USART_FLAG_TC)==RESET);
		USART_SendData(USART3,*s_at_cmd);//UART2_SendData(*b);
	}
//	UART3_SendLR();
	delay_ms(1000); 
	M_printf("GPRS[rec]:%s\r\n",Uart3_GPRS_Buf);
	CLR_GPRS_Buf(); 
	return;
	#endif 
	#if 0

	uint8_t count = 0;
	for(; count < cmd_len; count++)
	{
		USART_SendData(USART3, *s_at_cmd++);								                      	// ��������
		while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);		        // �ȴ��������
	}

	#endif 
	
}

#if 0
/*******************************************************************************
* ������ : connect_4G_tcp_server
* ����   : GPRS���ӷ���������,͸��ģʽ
* ����   : 
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/
static void connect_4G_tcp_server(void)
{
	  u8 cnt=0;
	u8 num=0;
	Restart:
	  Start_4G(0);
		
		delay_s(30);
//	  UART3_SendLR();
	send_at_cmd_noack("AT\r\n", strlen("AT\r\n"));
//		while(1)			
//			M_printf(Uart3_GPRS_Buf);
		while(!Find("READY"))
		{
			M_printf("waitting READY.........\r\n");
			delay_s(6);
			cnt++;
			if(cnt==5)
			{
				cnt=0;
				CLR_GPRS_Buf();
				goto Restart;
			}
		}
		
	TCP_CONNECT:
		CLR_GPRS_Buf();
	  M_printf("CLR_GPRS_Buf ok!\r\n");
		#if 1
//	quit_data_mode("+++", "OK", 5,sizeof("+++"));
		send_at_cmd_noack("AT\r\n", strlen("AT\r\n"));
		send_at_cmd_noack("ATE0\r\n", strlen("ATE0\r\n"));
	  send_at_cmd_noack("AT+CPIN?\r\n", strlen("AT+CPIN?\r\n"));
	  send_at_cmd_noack("AT+CSQ\r\n", strlen("AT+CSQ\r\n"));
		send_at_cmd_noack("AT+CEREG?\r\n", strlen("AT+CEREG?\r\n"));
		send_at_cmd_noack("AT+CGATT?\r\n", strlen("AT+CGATT?\r\n"));
		if(1!=send_at_cmd_ack(TCP_SERVER_INFO,"CONNECT OK" ,8,strlen(TCP_SERVER_INFO)))
		{
			M_printf("connect 4G failed\r\n");
			GPRSstaFlag=0;
//			Start_4G(1);
			delay_s(2);//��ʱ2s
			num++;
			if(3==num)
			{
				Start_4G(1);
				goto Restart;
			}
			goto TCP_CONNECT;
			
		}
		else {
			M_printf("connect 4G Successed\r\n");
			GPRSstaFlag=1;
		}
	//send_at_cmd_noack("AT+QISTATE=1,0", sizeof("AT+QISTATE=1,0"));
	#endif
	CLR_GPRS_Buf();
	return;
}
#endif
void GPRS_init(void)
{
	p_gprs_buf = Uart3_GPRS_Buf;
	p_gprs_rd = Uart3_GPRS_Buf; 
	USART3_Init_Config(115200);
	CLR_GPRS_Buf();
//	sys->gprsstate =1;	//����gprs
//	gprs_rest();
	
}
//����:sys->gprsstate =1 ����,0,������
//���:sys->gprscheck = 1;	check
void gprs_rest(void)
{
	M_printf("sys->gprstimecount_s=%d\r\n",	sys->gprstimecount_s);
//	M_printf("sys->gprsstate=%d\r\n",sys->gprsstate);
	switch(sys->gprsstate)
	{
			M_printf("sys->gprstimecount_s=%d\r\n",	sys->gprstimecount_s);
		case 1:								//����
			Power_4G=0;
			sys->gprsstate = 2;				//������
			sys->gprstimecount_10msEn =1;
			sys->gprstimecount_10ms = 0;
			CLR_GPRS_Buf ();
		break;
		case 2:
			if(sys->gprstimecount_10ms > 5)		////50ms
			{
//				M_printf ("50ms\r\n");
				//sys->gprstimecount_10msEn = 0;
				sys->gprstimecount_10msEn =0;
				sys->gprstimecount_10ms = 0;
				sys->gprstimecount_sEn =1;
				sys->gprstimecount_s =0;
                Power_4G=1;
				if(cfgsys->LogLev>=SIMPLELOG)
				{				
					M_printf("[GPRS-R:2]RESET OK\r\n");		//GPRSSTATE 2
				}
					sys->gprsstate = 3;			//����OK
					sys->gprscheckcount = 0;
				
//				if(sys->gprssyncbaud == 0)		//������Ӧ������
//				{
//					sys->gprsstate = 3;			//����OK
//					sys->gprscheckcount = 0;
//				}
//				else//����Ӧ�����ʣ�ֱ�ӷ�AT
//				{
//					//�״�ģ���޷�Ӧ����
//					sys->gprscheckcount = 0;
//					sys->gprsstate = 0;				//ready ok 
//					sys->gprsack = 0;
//					sys->gprscheck = 1;				//check

//				}
				
			}				
		break;
		case 3://������ʱ
			if(sys->gprstimecount_s > 50)		///�ȴ�10s 
			{
				M_printf("delay30S    ok .......\r\n");
				sys->gprstimecount_sEn =0;
				sys->gprstimecount_s =0;					
//				sys->gprsstate =4;
				sys->gprstimecount_10msEn =1;
				sys->gprstimecount_10ms =0;
				UART3_SendString("AT\r\n");
				# if 1
				  sys->gprscheckcount = 0;
					sys->gprsstate = 0;				//ready ok 
					sys->gprsack = 0;
					sys->gprscheck = 1;				//check
					CLR_GPRS_Buf ();
					sys->gprstimecount_10msEn =0;
					sys->gprstimecount_10ms = 0;
			  	sys->gprsrestEN=1;
					M_printf("GPRS  start ok.......\r\n");
				#endif
			}
		break;
			case 4://���ready
			if(sys->gprstimecount_10ms > 100)		///1S���һ��
			{
				if(cfgsys->LogLev>=SIMPLELOG)
				{	
					M_printf("GPRS-R:4[rec]:%s\r\n",Uart3_GPRS_Buf);	
				}	
				sys->gprstimecount_10ms =0;			
				sys->gprscheckcount++;
				if(!Find("READY"))
				{
					if(sys->gprscheckcount>20)
					{
						if(cfgsys->LogLev>=SIMPLELOG)
						{
							M_printf("[GPRS-R:4]GET NO 'READY', TIME OUT..\r\n");
						}	
						CLR_GPRS_Buf ();
						sys->gprscheckcount = 0;
						sys->gprsstate = 1;			//����	
					}	
				}	
				else
				{
					if(cfgsys->LogLev>=SIMPLELOG)
					{
						M_printf("[GPRS-R:4]GET 'READY' OK\r\n");
					}						
					sys->gprscheckcount = 0;
					sys->gprsstate = 0;				//ready ok 
					sys->gprsack = 0;
					sys->gprscheck = 1;				//check
					CLR_GPRS_Buf ();
					sys->gprstimecount_10msEn =0;
					sys->gprstimecount_10ms = 0;
				}
			}
		break;
			case 5:
				
				break;
		default:
		break;			
	}	
}	
u8  CYCLE=15;
#if 1
//����:	1	gprsack		
//		2	gprscheck
//���:	1	sys->netok == 1 �����ɹ�
void gprs_check(void)
{
	u8 temp=0;
	char str[20];
	char R_buff[30];
	switch(sys->gprsack)		//����ָ���־
	{
		case 0:		//������� ����gprscheck �жϷ��;���ָ��
			switch (sys->gprscheck)
			{
				case 1:
					UART3_SendString("AT+CIPCLOSE\r\n"); //�ر�TCP����
				  CLR_GPRS_Buf();//������ջ���������
				  sys->gprsack = 1;
					sys->gprscheckcount = 0;				
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				case 2:		
					CLR_GPRS_Buf();//������ջ���������
					UART3_SendString("AT\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;				
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				break;

				case 3:
					UART3_SendString("ATE0\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				break;
				
				case 4:
					UART3_SendString("AT+CPIN?\r\n");
					sys->gprsack = 2;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				break;
				
//				
//				case 3:
//					UART3_SendString("AT+CREG=1\r\n");
//					sys->gprsack = 1;
//					sys->gprscheckcount = 0;
//					sys->gprstimecount_10msEn =1;
//					sys->gprstimecount_10ms = 0;
//				break;
				
				case 5:
					UART3_SendString("AT+CGREG?\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				break;
				
				case 6:		
					UART3_SendString("AT+CSQ\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				break;	
				
//				case 6:
//					UART3_SendString("AT+CGREG=1\r\n");
//					sys->gprsack = 1;
//					sys->gprscheckcount = 0;
//					sys->gprstimecount_10msEn =1;
//					sys->gprstimecount_10ms = 0;
//				break;
				
				case 7:
					UART3_SendString("AT+CGATT?\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				break;
//				
				case 8:
					UART3_SendString("AT+CIPSHUT\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				break;
				case 9:
					UART3_SendString("AT+CIPMUX=0\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				break;
				case 10:
					UART3_SendString("AT+CIPQSEND=0\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				break;
				case 11:
					UART3_SendString("AT+CSTT=\"CMNET\"\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				break;
				case 12:
					UART3_SendString("AT+CIICR\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				break;
				case 13:
					UART3_SendString("AT+CIFSR\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				break;
				case 14:
					UART3_SendString("AT+CIPSTATUS\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				break;
				case 15:
//					if(sys->modetype==MODE_ENG)		//����ע��
//					{
//						sprintf(str,"AT+QIOPEN=\"TCP\",\"%s\",\"%d\"\r",cfgsys->ip,cfgsys->port);
						//UART3_SendString("AT+QIOPEN=\"TCP\",\"42.121.97.22\",\"31000\"\r");
						UART3_SendString(TCP_SERVER_INFO);
						sys->gprsack = 3;
						sys->gprscheckcount = 0;
						sys->gprstimecount_10msEn =1;
						sys->gprstimecount_10ms = 0;
//					}	
				break;
				case 16:	//case16~case20ͬ������ʱ��
					UART3_SendString("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
					break;
				case 17:
					UART3_SendString("AT+SAPBR=3,1,\"APN\",\"CMNET\"\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
					break;
				case 18:
					UART3_SendString("AT+SAPBR=1,1\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
					break;
				case 19:
					UART3_SendString("AT+CNTPCID=1\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
					break;
				case 20:
					UART3_SendString("AT+CNTP\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
					break;
				case 21:
					CLR_GPRS_Buf();
					UART3_SendString("AT+CCLK?\r\n");
					sys->gprsack = 7;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
					break;
		
				//��������   ��ʱ����
				case 30:	//AT
						sprintf(str,"AT+CIPSEND=%d\r\n",sys->gprssendlen);
						//GPRS_send_Cmd("AT+QISEND=",sys->gprssendlen);
						UART3_SendString(str);
						sys->gprsack = 4;			//���<
						sys->gprscheckcount = 0;
				break;
				
//				case 13:								//��������		Э��ͷ				
//						UART3_SendString((u8*)0X1A);		//���ͽ�����
//						sys->gprsack = 5;	
//						sys->gprscheckcount = 0;
//				break;

//				case 14:      //  ��������   ��ִ��12��ִ��14
//						Send_DATA ((u8 *)beat,sizeof (beat));
//						sys->gprsack = 5;	
//						sys->gprscheckcount = 0;
//				default:
//				break;
			}	
		break;

		case 1:			//�ظ�ok
			if(sys->gprstimecount_10ms > 5)		///50msһ��
			{
				if(sys->gprscheck ==1||sys->gprscheck ==13)
				{
						sys->gprstimecount_10msEn =0;
						sys->gprstimecount_10ms = 0;
						sys->gprsack = 0;										
						sys->gprscheck++;
					  CLR_GPRS_Buf();
				}
				else{	
								if(cfgsys->LogLev>=SIMPLELOG)
								{
									M_printf("GPRS-C:OK[rce]:%s \r\n",Uart3_GPRS_Buf);
								}	
								sys->gprstimecount_10ms =0;			
								sys->gprscheckcount++;
								if(!Find("OK"))
								{
									if(Find("ERROR"))
									{
										sys->gprsack = 0;
										sys->gprsrestEN =1;
										sys->gprsstate=0;
										sys->gprscheck =1;
									}
									else{
											if(sys->gprscheckcount>10)		//����1.5s
												{
													if(sys->gprsresendcheck++>2)
													{
														sys->gprscheckcount =0;
														sys->gprsresendcheck=0;
														if(sys->gprscheck ==2)
														{
															if(cfgsys->LogLev>=SIMPLELOG)
																{
																	M_printf("[GPRS]restart GPRS...\r\n");
																}	
															sys->gprsack = 0;
															sys->gprsrestEN =0;
															sys->gprsstate=1;
															sys->gprscheck =0;
														}
														else sys->gprscheck --;
														
													}
													else	
													{
														sys->gprsack =0;
														if(cfgsys->LogLev>=SIMPLELOG)
																{
																	M_printf("[GPRS]check cmd num :%d\r\n",sys->gprsresendcheck);
																}	
													}											
												}
												else
												{
													if(cfgsys->LogLev>=SIMPLELOG)
													{					
														M_printf("[GPRS]WAIT 'OK', COUNT:%d\r\n",sys->gprscheckcount);
													}	
												}	
										
									}	
													
								}
								else		//��ȡ��ok
								{
									if(cfgsys->LogLev>=SIMPLELOG)
										{
											M_printf("GET 'OK' OK\r\n");
										}
									sys->gprstimecount_10msEn =0;
									sys->gprstimecount_10ms = 0;
									sys->gprsack = 0;										
									sys->gprscheck++;
									CLR_GPRS_Buf();
									sys->gprscheckcount =0;
									sys->gprsresendcheck=0;
								}	
							}		
					}				
		break;

		case 2:
			if(sys->gprstimecount_10ms > 10)		///1sһ��
			{
				sys->gprstimecount_10ms =0;			
				sys->gprscheckcount++;		
				if(!Find("READY"))
				{
					if(sys->gprscheckcount>10)		//����40s
					{
						if(sys->gprsresendcheck++>2)
						{
							sys->gprscheckcount=0;
							sys->gprsresendcheck=0;
							sys->gprsack=4;
//							sys->gprstimecount_10msEn=0;
							if(cfgsys->LogLev>=SIMPLELOG)
								{
									M_printf("[GPRS]  check sim  fail	..................\r\n");
								}	
						}else{
							sys->gprsack=0;
						}
//						sys->gprscheckcount =0;
//						sys->gprsack = 0;
//						sys->gprscheck = 1;			//����check	
//						sys->gprsstate = 0;	//����
//						if(cfgsys->LogLev>=SIMPLELOG)
//						{
//							M_printf("[GPRS]GET NO 'READY', TIMEOUT...\r\n");
//						}	
					}
					else
					{
						if(cfgsys->LogLev>=DETAILLOG)
						{
							M_printf("[GPRS]WAIT 'READY', COUNT:%d\r\n",sys->gprscheckcount);
						}	
					}	
				}
				else		//��ȡ��READY
				{
					if(cfgsys->LogLev>=SIMPLELOG)
					{
						M_printf("[GPRS]GET 'READY' OK\r\n");
						M_printf ("[GPRS]------------------------------\r\n");
					}
					sys->gprsack = 0;
					sys->gprscheck++;
					sys->gprscheckcount=0;
					sys->gprsresendcheck=0;
				}	
			}	
		break;
			
		case 3:
			if(sys->gprstimecount_10ms > 10)		///100msһ��
			{
				sys->gprstimecount_10ms =0;			
				sys->gprscheckcount++;	
				if(!Find("CONNECT OK"))
				{
					if(sys->gprscheckcount>10)		//����5s
					{
						if(sys->gprsTCPchecknum++>10)
						{
							sys->gprsTCPchecknum=0;
							sys->gprscheckcount =0;
							sys->gprsack = 0;
							sys->gprscheck = 0;			//
							sys->gprsstate = 1;			//����
							sys->gprsrestEN =0;
							if(cfgsys->LogLev>=SIMPLELOG)
							{
								M_printf("[GPRS]restart GPRS.....................\r\n");
								CLR_GPRS_Buf();
							}	
						}
						else{
							sys->gprscheckcount =0;
							sys->gprsack = 0;
							sys->gprscheck = 1;			//
							sys->gprsstate = 1;			//����
							if(cfgsys->LogLev>=SIMPLELOG)
							{
								M_printf("[GPRS]GET NO 'CONNECT OK', TIMEOUT...\r\n");
								M_printf("[GPRS]recheck GPRS.....................\r\n");	
							}	
							CLR_GPRS_Buf();
						}
						
					}
					else
					{
						if(cfgsys->LogLev>=SIMPLELOG)
						{
							M_printf("[GPRS]WAIT 'CONNECT OK', COUNT:%d\r\n",sys->gprscheckcount);
							
						}
					}					
				}
				else		//��ȡ��ok
				{
					if(cfgsys->LogLev>=SIMPLELOG)
					{
						M_printf("[GPRS]GET 'CONNECT OK' OK\r\n");
						M_printf ("-------TCP  connect  OK-------\r\n");
					}
					CLR_GPRS_Buf();
					sys->gprscheckcount =0;
					sys->gprsresendcheck=0;					
					sys->gprsstate =1;
					sys->gprsack = 0;
					sys->gprscheck=1;
					sys->gprstimecount_10msEn =0;
					sys->gprstimecount_10ms = 0;
					sys->netok = 1;
					CLR_GPRS_Buf();
					TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx
					M_printf("TIME3  ENABLE........\r\n");
				}	
			}
		break;
		case 4:
			if(cfgsys->LogLev>=SIMPLELOG)
						{
							M_printf("[GPRS]ENTER FLY MODE\r\n");		
						}
				UART3_SendString("AT+CFUN=0\r\n");           //ѡ��ģ�鹦��
				sys->gprsack=6;
						
			break;
		case 5:
			if(cfgsys->LogLev>=SIMPLELOG)
						{
							M_printf("[GPRS]QUIT FLY MODE\r\n");		
						}
				UART3_SendString("AT+CFUN=1\r\n");
				sys->gprsack=0;
				sys->gprscheck--;
			break;
		case 6:
			if(sys->gprstimecount_10ms >500)//��ʱ5S
			{			
				sys->gprsack=5;
				sys->gprstimecount_10msEn=0;
				sys->gprstimecount_10ms=0;
			}
			break;
		case 7:               //ʵʱʱ��
			if(sys->gprstimecount_10ms > 5)		///50msһ��
			{
				sys->gprstimecount_10ms=0;
				sys->gprscheckcount++;
				if(Find("+CCLK")&&Find("OK"))
				{
					memcpy(R_buff,Uart3_GPRS_Buf+10,17);
					if(cfgsys->LogLev>=SIMPLELOG)
						{
							M_printf("[gprs]%s\r\n",R_buff);
						}
						sscanf(R_buff,"%2d/%d/%d,%d:%d:%2d",&time_t .year ,&time_t.month,&time_t.day ,&time_t.hour ,&time_t.minute,&time_t.second );	
					if(cfgsys->LogLev>=SIMPLELOG)
						{
							M_printf("[GPRS-7:time_t]%2d/%2d/%2d,%2d:%2d:%2d    \r\n" , time_t .year ,time_t.month,time_t.day ,time_t.hour ,time_t.minute,time_t.second );
						}
					sys_second=time_to_second();
						if(cfgsys->LogLev>=SIMPLELOG)
						{
							M_printf("[GPRS-7:]sys_second:%d    \r\n" , sys_second );
						}
					if(sys_second!=0)
					{
						if(cfgsys->LogLev>=SIMPLELOG)
						{
							M_printf("[gprs]net time update ok...\r\n");
							sys->sysgprstimegetEN=1;
							sys->gprstimecount_10msEn=0;
							sys->gprstimecount_10ms=0;
							sys->gprscheckcount=0;
							sys->systimerCountEn=1;
							sys->systimerCount_ms =0;
							sys->gprsack=0;
							sys->gprscheck =1;
//							sys->systimerCount_s =0;
						}
					}
					else
					{
						if(cfgsys->LogLev>=SIMPLELOG)
						{
							M_printf("[gprs]time  change error  ...net time update faile...\r\n");
						}
					}
				}
				else
				{
						if(sys->gprscheckcount>30)
						{
							sys->gprscheckcount=0;
							sys->gprstimecount_10msEn=0;
							sys->gprsack=0;
							sys->gprscheck=16;
							if(cfgsys->LogLev>=SIMPLELOG)
							{
								M_printf("[gprs]  get no time .....net time update faile...\r\n");
							}
						}
				}
			}
			break;
#if 0
		case 4:
			if(sys->gprstimecount_10ms > 2)		///20msһ��
			{
				sys->gprstimecount_10ms =0;			
				sys->gprscheckcount++;
				if(!Find("<"))			//��Ϊ��·���ӣ�û��<
				{
					if(sys->gprscheckcount>CYCLE)		//����2s
					{
						sys->gprscheckcount =0;
//						sys->gprsack = 0;
//						sys->gprscheck = 1;			//����check			
						sys->gprsack=0;			//
						sys->gprscheck++;		//��������		/��Ϊ��·���ӣ�û��<
						if(cfgsys->LogLev>=SIMPLELOG)
						{
							M_printf("GET NO '<', TIMEOUT...\r\n");
							M_printf("IF  AT+QIMUX=1  NO '<', SEND DATA GO ON...\r\n");
						}
					}
					else
					{
						if(cfgsys->LogLev>=DETAILLOG)
							M_printf("WAIT '<', TIMES...%d\r\n",sys->gprscheckcount);
						}	
						{
					}					
				}
				else		//��Ϊ��·���ӣ�û��<
				{
					if(cfgsys->LogLev>=SIMPLELOG)
					{
						M_printf("GET '<' OK\r\n");
					}
					if(1== beatflag )
					{
						sys->gprsack = 0;						
						sys->gprscheck=14;
					}
					sys->gprsack = 0;						
					sys->gprscheck++;	
						
				}	
			}
		break;
		case 5:
			if(sys->gprstimecount_10ms > 10)		///100msһ��
			{
				sys->gprstimecount_10ms =0;			
				sys->gprscheckcount++;
				if(!Find("SEND OK"))	//�ж������Ƿ������
				{
					if(sys->gprscheckcount>50)		//����5s
					{
						sys->gprscheckcount =0;
//						sys->gprsack = 0;
//						sys->gprscheck = 1;			//����check			
						sys->gprsack=0;				//
						sys->gprscheck++;			//��������		/��Ϊ��·���ӣ�û��<
						if(cfgsys->LogLev>=SIMPLELOG)
						{						
							M_printf("GET NO '<', TIMEOUT...\r\n");
							M_printf("IF  AT+QIMUX=1  NO '<', SEND DATA GO ON...\r\n");
						}	
					}
					else
					{
						if(cfgsys->LogLev>=DETAILLOG)
						{
							M_printf("WAIT '<', TIMES...%d\r\n",sys->gprscheckcount);
						}	
					}					
				}
				else		//
				{
					if(cfgsys->LogLev>=SIMPLELOG)
					{
						M_printf("GET 'SEND OK' OK\r\n");
					}	
					sys->gprsack = 0;						
					
					sys->gprscheck=0;	
						
				}	
			}
		break;
#endif
		default:
		break;
	}	
}	
#endif 
#if 0
//����:	1	gprsack		
//		2	gprscheck
//���:	1	sys->netok == 1 �����ɹ�
void gprs_check(void)
{
	char str[20];

	switch(sys->gprsack)		//����ָ���־
	{
		case 0:		//������� ����gprscheck �жϷ��;���ָ��
			switch (sys->gprscheck)
			{
				case 1:
					UART3_SendString("AT+CIPCLOSE\r\n"); //�ر�TCP����
				  CLR_GPRS_Buf();//������ջ���������
				  sys->gprsack = 1;
					sys->gprscheckcount = 0;				
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				case 2:		
					CLR_GPRS_Buf();//������ջ���������
					UART3_SendString("AT\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;				
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				break;

				case 3:
					UART3_SendString("ATE0\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				break;
				
				case 4:
					UART3_SendString("AT+CPIN?\r\n");
					sys->gprsack = 2;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				break;
				
//				
//				case 3:
//					UART3_SendString("AT+CREG=1\r\n");
//					sys->gprsack = 1;
//					sys->gprscheckcount = 0;
//					sys->gprstimecount_10msEn =1;
//					sys->gprstimecount_10ms = 0;
//				break;
				
				case 5:
					UART3_SendString("AT+CEREG?\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				break;
				
				case 6:		
					UART3_SendString("AT+CSQ\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				break;	
				
//				case 6:
//					UART3_SendString("AT+CGREG=1\r\n");
//					sys->gprsack = 1;
//					sys->gprscheckcount = 0;
//					sys->gprstimecount_10msEn =1;
//					sys->gprstimecount_10ms = 0;
//				break;
				
				case 7:
					UART3_SendString("AT+CGREG?\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				break;
//				
				case 8:
					UART3_SendString("AT+CGATT=?\r\n");
					sys->gprsack = 1;
					sys->gprscheckcount = 0;
					sys->gprstimecount_10msEn =1;
					sys->gprstimecount_10ms = 0;
				break;

				case 9:
//					if(sys->modetype==MODE_ENG)		//����ע��
//					{
//						sprintf(str,"AT+QIOPEN=\"TCP\",\"%s\",\"%d\"\r",cfgsys->ip,cfgsys->port);
						//UART3_SendString("AT+QIOPEN=\"TCP\",\"42.121.97.22\",\"31000\"\r");
						UART3_SendString(TCP_SERVER_INFO);
						sys->gprsack = 3;
						sys->gprscheckcount = 0;
						sys->gprstimecount_10msEn =1;
						sys->gprstimecount_10ms = 0;
//					}	
				break;
		
				//��������   ��ʱ����
				case 12:	//AT
						sprintf(str,"AT+CIPSEND=%d\r\n",sys->gprssendlen);
						//GPRS_send_Cmd("AT+QISEND=",sys->gprssendlen);
						UART3_SendString(str);
						sys->gprsack = 4;			//���<
						sys->gprscheckcount = 0;
				break;
				
//				case 13:								//��������		Э��ͷ				
//						UART3_SendString((u8*)0X1A);		//���ͽ�����
//						sys->gprsack = 5;	
//						sys->gprscheckcount = 0;
//				break;

//				case 14:      //  ��������   ��ִ��12��ִ��14
//						Send_DATA ((u8 *)beat,sizeof (beat));
//						sys->gprsack = 5;	
//						sys->gprscheckcount = 0;
//				default:
//				break;
			}	
		break;

		case 1:			//�ظ�ok
			if(sys->gprstimecount_10ms > 5)		///100msһ��
			{
				if(sys->gprscheck ==1)
				{
						sys->gprstimecount_10msEn =0;
						sys->gprstimecount_10ms = 0;
						sys->gprsack = 0;										
						sys->gprscheck++;
					  CLR_GPRS_Buf();
				}
				else{	
								M_printf("GPRS-C:OK[rce]:%s \r\n",Uart3_GPRS_Buf);
								sys->gprstimecount_10ms =0;			
								sys->gprscheckcount++;
								if(!Find("OK"))
								{
									if(sys->gprscheckcount>10)		//����1.5s
									{
										if(sys->gprsresendcheck++>2)
										{
											sys->gprscheckcount =0;
											sys->gprsresendcheck=0;
											if(sys->gprscheck ==2)
											{
												if(cfgsys->LogLev>=SIMPLELOG)
													{
														M_printf("[GPRS]restart GPRS...\r\n");
													}	
												sys->gprsack = 0;
												sys->gprsrestEN =0;
												sys->gprsstate=1;
												sys->gprscheck =0;
											}
											else sys->gprscheck --;
											
										}
										else	
										{
											sys->gprsack =0;
											if(cfgsys->LogLev>=SIMPLELOG)
													{
														M_printf("[GPRS]check cmd num :%d\r\n",sys->gprsresendcheck);
													}	
										}											
									}
									else
									{
										if(cfgsys->LogLev>=SIMPLELOG)
										{					
											M_printf("[GPRS]WAIT 'OK', COUNT:%d\r\n",sys->gprscheckcount);
										}	
									}					
								}
								else		//��ȡ��ok
								{
									if(cfgsys->LogLev>=SIMPLELOG)
										M_printf("GET 'OK' OK\r\n");
										sys->gprstimecount_10msEn =0;
										sys->gprstimecount_10ms = 0;
										sys->gprsack = 0;										
										sys->gprscheck++;
										CLR_GPRS_Buf();
										sys->gprscheckcount =0;
										sys->gprsresendcheck=0;
								}	
							}		
					}				
		break;

		case 2:
			if(sys->gprstimecount_10ms > 10)		///1sһ��
			{
				sys->gprstimecount_10ms =0;			
				sys->gprscheckcount++;		
				if(!Find("READY"))
				{
					if(sys->gprscheckcount>10)		//����40s
					{
						if(sys->gprsresendcheck++>2)
						{
							sys->gprscheckcount=0;
							sys->gprsresendcheck=0;
							sys->gprsack=4;
//							sys->gprstimecount_10msEn=0;
							if(cfgsys->LogLev>=SIMPLELOG)
								{
									M_printf("[GPRS]  check sim  fail	..................\r\n");
								}	
						}else{
							sys->gprsack=0;
						}
//						sys->gprscheckcount =0;
//						sys->gprsack = 0;
//						sys->gprscheck = 1;			//����check	
//						sys->gprsstate = 0;	//����
//						if(cfgsys->LogLev>=SIMPLELOG)
//						{
//							M_printf("[GPRS]GET NO 'READY', TIMEOUT...\r\n");
//						}	
					}
					else
					{
						if(cfgsys->LogLev>=DETAILLOG)
						{
							M_printf("[GPRS]WAIT 'READY', COUNT:%d\r\n",sys->gprscheckcount);
						}	
					}	
				}
				else		//��ȡ��READY
				{
					if(cfgsys->LogLev>=SIMPLELOG)
					{
						M_printf("[GPRS]GET 'READY' OK\r\n");
						M_printf ("[GPRS]------------------------------\r\n");
					}
					sys->gprsack = 0;
					sys->gprscheck++;
					sys->gprscheckcount=0;
					sys->gprsresendcheck=0;
				}	
			}	
		break;
			
		case 3:
			if(sys->gprstimecount_10ms > 10)		///100msһ��
			{
				sys->gprstimecount_10ms =0;			
				sys->gprscheckcount++;	
				if(!Find("CONNECT OK"))
				{
					if(sys->gprscheckcount>10)		//����5s
					{
						if(sys->gprsTCPchecknum++>10)
						{
							sys->gprsTCPchecknum=0;
							sys->gprscheckcount =0;
							sys->gprsack = 0;
							sys->gprscheck = 0;			//
							sys->gprsstate = 1;			//����
							sys->gprsrestEN =0;
							if(cfgsys->LogLev>=SIMPLELOG)
							{
								M_printf("[GPRS]restart GPRS.....................\r\n");
								CLR_GPRS_Buf();
							}	
						}
						else{
							sys->gprscheckcount =0;
							sys->gprsack = 0;
							sys->gprscheck = 1;			//
							sys->gprsstate = 1;			//����
							if(cfgsys->LogLev>=SIMPLELOG)
							{
								M_printf("[GPRS]GET NO 'CONNECT OK', TIMEOUT...\r\n");
								M_printf("[GPRS]recheck GPRS.....................\r\n");
								CLR_GPRS_Buf();
							}	
							
						}
						
					}
					else
					{
						if(cfgsys->LogLev>=SIMPLELOG)
						{
							M_printf("[GPRS]WAIT 'CONNECT OK', COUNT:%d\r\n",sys->gprscheckcount);
							
						}
					}					
				}
				else		//��ȡ��ok
				{
					if(cfgsys->LogLev>=SIMPLELOG)
					{
						M_printf("[GPRS]GET 'CONNECT OK' OK\r\n");
						M_printf ("-------TCP  connect  OK-------\r\n");
					}
					sys->gprsstate =1;
					sys->gprsack = 0;
					sys->gprscheck=1;
					sys->gprstimecount_10msEn =0;
					sys->gprstimecount_10ms = 0;
					sys->netok = 1;
					CLR_GPRS_Buf();
					TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx
//					M_printf("TIME3  ENABLE........\r\n");
				}	
			}
		break;
		case 4:
			if(cfgsys->LogLev>=SIMPLELOG)
						{
							M_printf("[GPRS]ENTER FLY MODE\r\n");		
						}
				UART3_SendString("AT+CFUN=0\r\n");
				sys->gprsack=6;
						
			break;
		case 5:
			if(cfgsys->LogLev>=SIMPLELOG)
						{
							M_printf("[GPRS]QUIT FLY MODE\r\n");		
						}
				UART3_SendString("AT+CFUN=1\r\n");
				sys->gprsack=0;
				sys->gprscheck--;
			break;
		case 6:
			if(sys->gprstimecount_10ms >500)//��ʱ5S
			{			
				sys->gprsack=5;
				sys->gprstimecount_10msEn=0;
				sys->gprstimecount_10ms=0;
			}
			break;
#if 0
		case 4:
			if(sys->gprstimecount_10ms > 2)		///20msһ��
			{
				sys->gprstimecount_10ms =0;			
				sys->gprscheckcount++;
				if(!Find("<"))			//��Ϊ��·���ӣ�û��<
				{
					if(sys->gprscheckcount>CYCLE)		//����2s
					{
						sys->gprscheckcount =0;
//						sys->gprsack = 0;
//						sys->gprscheck = 1;			//����check			
						sys->gprsack=0;			//
						sys->gprscheck++;		//��������		/��Ϊ��·���ӣ�û��<
						if(cfgsys->LogLev>=SIMPLELOG)
						{
							M_printf("GET NO '<', TIMEOUT...\r\n");
							M_printf("IF  AT+QIMUX=1  NO '<', SEND DATA GO ON...\r\n");
						}
					}
					else
					{
						if(cfgsys->LogLev>=DETAILLOG)
						{
							M_printf("WAIT '<', TIMES...%d\r\n",sys->gprscheckcount);
						}	
					}					
				}
				else		//��Ϊ��·���ӣ�û��<
				{
					if(cfgsys->LogLev>=SIMPLELOG)
					{
						M_printf("GET '<' OK\r\n");
					}
					if(1== beatflag )
					{
						sys->gprsack = 0;						
						sys->gprscheck=14;
					}
					sys->gprsack = 0;						
					sys->gprscheck++;	
						
				}	
			}
		break;
		case 5:
			if(sys->gprstimecount_10ms > 10)		///100msһ��
			{
				sys->gprstimecount_10ms =0;			
				sys->gprscheckcount++;
				if(!Find("SEND OK"))	//�ж������Ƿ������
				{
					if(sys->gprscheckcount>50)		//����5s
					{
						sys->gprscheckcount =0;
//						sys->gprsack = 0;
//						sys->gprscheck = 1;			//����check			
						sys->gprsack=0;				//
						sys->gprscheck++;			//��������		/��Ϊ��·���ӣ�û��<
						if(cfgsys->LogLev>=SIMPLELOG)
						{						
							M_printf("GET NO '<', TIMEOUT...\r\n");
							M_printf("IF  AT+QIMUX=1  NO '<', SEND DATA GO ON...\r\n");
						}	
					}
					else
					{
						if(cfgsys->LogLev>=DETAILLOG)
						{
							M_printf("WAIT '<', TIMES...%d\r\n",sys->gprscheckcount);
						}	
					}					
				}
				else		//
				{
					if(cfgsys->LogLev>=SIMPLELOG)
					{
						M_printf("GET 'SEND OK' OK\r\n");
					}	
					sys->gprsack = 0;						
					
					sys->gprscheck=0;	
						
				}	
			}
		break;
#endif
		default:
		break;
	}	
}	
#endif 
void GPRS_APP(void)
{
	static u8 num=16;
//	M_printf("GPRS_App.........\r\n");
	if(sys->netok == 0)
	{	
		if(sys->gprsrestEN ==0)
		    gprs_rest();
		gprs_check();	
	}
	else{
		if(sys->sysgprstimegetEN==0)
		{
			if (sys->gprscheck==1)
			{
				sys->gprsack =0;
				sys->gprscheck =16;
			}
			gprs_check();
		}
	}	
}


//��������
void Send_DATA(u8 *buf,u32 len)
{
	u16 t;
	for(t=0;t<len ;t++)		//ѭ����������
	{
	  	while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET); //�ȴ����ͽ���		
    	USART_SendData(USART3,buf[t]); //��������	
	}	 

	while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET); //�ȴ����ͽ���	
}


#if 1
u8 cle=50;
uint16_t crc = 0;
SDK_CMD_INFO sdk_cmd_info;
//  �ɹ�����1  ʧ�ܷ��� 2  ����0
void send_data_to_sever(u8 *buffer, u8 info ,u8 len)  //�����0Ϊ����
{ 
	u8 j=0;
	char str[20];
	if(cfgsys->LogLev>=SIMPLELOG)
	{
		M_printf("[GPRS:]sys->datasendcheck:%d.....\r\n",sys->datasendcheck);
	}		
	switch(sys->datasendcheck )
	{
		case 0:  // ָ�����͵��ֽڳ���
						if (0!=info)
				    {
							sdk_cmd_info.cmd_no = revs(info);///<���͵�����תΪ���ģʽ
							sdk_cmd_info.header[0] = 'H';
							sdk_cmd_info.header[1] = 'E';
							sdk_cmd_info.header[2] = 'A';
							sdk_cmd_info.header[3] = 'D';
							for(j=0;j<4;j++)
							sdk_cmd_info.sn[j]=SN[j];
//							if(sys->htydatasendEN==0)
//							{
//							sdk_cmd_info.second =revl(sdk_cmd_info.second);
//							sdk_cmd_info.ms=revl(sdk_cmd_info.ms);
//							sdk_cmd_info.sys_num1=revl(Random_num1);	
//							sdk_cmd_info.sys_num2=revl(Random_num2);
//							}
							sdk_cmd_info.len = revs(sizeof(sdk_cmd_info));
							memcpy(sdk_cmd_info.data,buffer,158);
							crc = crc16_check((uint8_t*)&sdk_cmd_info,sizeof(sdk_cmd_info) -2);
							sdk_cmd_info.crc[0] = crc & 0x00ff;
							sdk_cmd_info.crc[1] = (crc & 0xff00) >> 8;
//							if(cfgsys->LogLev>=SIMPLELOG)
//								{
//									M_printf("[GPRS:sys_second]%d    \r\n" , sys_second );
//								}
							if(cfgsys->LogLev>=SIMPLELOG)
								{
									M_printf("[GPRS:sdk_cmd_info]%d      %d      %d       %d  \r\n" , sdk_cmd_info.second,sdk_cmd_info.ms,sdk_cmd_info.sys_num1,sdk_cmd_info.sys_num2 );
								}
								
							sprintf(str,"AT+CIPSEND=%d\r\n",sizeof(sdk_cmd_info));
				   }else
							sprintf(str,"AT+CIPSEND=%d\r\n",2);		     
		        CLR_GPRS_Buf();
		      	UART3_SendString(str);
			      sys->datasendcheck=1;
						sys->reccheckcount=0; 
						sys->gprstimecount_5msEn=1;
						sys->gprstimecount_5ms=0;
//						sys->datdsendcletimes=0;
					break;
		case 1://����<
						if (sys->gprstimecount_5ms>0)//   10ms����һ��
						{
							sys->gprstimecount_5ms =0;
							if(Find (">"))
							{
								if(cfgsys->LogLev>=SIMPLELOG)
									{
										M_printf("[GPRS:1]find >  ok.....\r\n");
									}	
								
								sys->datasendcheck=2;
								sys->reccheckcount=0; 
								sys->gprstimecount_5msEn=0;
								sys->gprstimecount_5ms=0;
								if(Get_data_EN ==1)
								{		
									sys->datacheck =3;									
								}
							}
							else
							{
								if(sys->reccheckcount ++>2)  //��ⲻ��  ���粻ͨ
								{
									if(cfgsys->LogLev>=SIMPLELOG)
									{
											M_printf("[GPRS:1]find    >    fail........\r\n");
									}
								
									sys->datasendcheck =0;
									sys->reccheckcount=0; 
									sys->gprstimecount_5msEn=0;
									sys->gprstimecount_5ms=0;
									sys->netok =0;
									if(1==Get_data_EN )
									{
										sys->datacheck=5;
										sys->datasaveEN =1;
										RS485Flag =0;
										sys->gprsdatabusy =0;
										sys->gprsbusy =0;
									}
									if(sys->gprsbeatbusy ==1)
									{
										sys->gprsbeatbusy =0;
										sys->gprsbusy =0;
										beatflag=0;
									}
									if(sys->htydatasendEN ==1)
									{
									sys->htysendcheck =0;
									sys->htydatasendEN =0;
									}
								}
								else{
									if(cfgsys->LogLev>=SIMPLELOG)
									{
											M_printf("[GPRS:1]reccheckcount  >  :%d\r\n",sys->reccheckcount);
									}
									
									sys->datasendcheck =1;
									sys->gprstimecount_5msEn=1;
									sys->gprstimecount_5ms=0;
								}
							}
						}
						break;
			case 2://������Ҫ�ϴ�������
				if(cfgsys->LogLev>=SIMPLELOG)
					{
							M_printf("[GPRS:2]start send data.....\r\n");
					}
				
			  CLR_GPRS_Buf ();
				if (0!=info)
				{
					Send_DATA ((u8 *)&sdk_cmd_info ,sizeof(sdk_cmd_info));
				}else{
					Send_DATA (buffer ,2);
				}
				sys->datasendcheck=3;
				sys->reccheckcount=0; 
				sys->gprstimecount_5msEn=1;
				sys->gprstimecount_5ms=0;
				if(Get_data_EN ==1)
				{
//					 sys->datacheck =3;
					 sys->datasaveEN =0;
				}
				sys->htysendcheck =0;
				if(cfgsys->LogLev>=SIMPLELOG)
					{
							M_printf("[GPRS:2] data send  OK.....\r\n");
					}
				
				break;
			case 3://�ȴ�send  ok
				if (sys->gprstimecount_5ms>0)//   20ms����һ��
						{
							sys->gprstimecount_5ms =0;
							if(Find ("SEND OK"))
							{
								if(cfgsys->LogLev>=SIMPLELOG)
								{
										M_printf("[GPRS:3]GET send ok ...............\r\n");
								}
							
								sys->datasendcheck=0;
								sys->reccheckcount=0; 
								sys->gprstimecount_5msEn=0;
								sys->gprstimecount_5ms=0;
								sys->gprssendcletimes=0;
								sys->datasendflag=1;  ///���ͳɹ�				
								if(sys->gprsbeatbusy==1)
								{
									sys->gprsbeatbusy =0;
									sys->gprsbusy =0;
									beatflag=0;
								}
								if(1==Get_data_EN &&sys->htydatasendEN ==0)
								{
									sys->gprsdatabusy =0;
									if(sys->datacheck ==3)
								  sys->datacheck=4;
									RS485Flag =0;
									sys->gprsbusy =0;
									sdk_cmd_info.second=0;
									sdk_cmd_info.ms=0;
									sdk_cmd_info.sys_num1=0;
									sdk_cmd_info.sys_num2=0;
//									sys->rs485busy =0;
								}
								if(sys->htydatasendEN ==1)
								{
								sys->htysendcheck =1;
									sdk_cmd_info.second=0;
									sdk_cmd_info.ms=0;
									sdk_cmd_info.sys_num1=0;
									sdk_cmd_info.sys_num2=0;
								}
							}
							else
							{
								if(Find("SEND FAIL"))
								{
														if(cfgsys->LogLev>=SIMPLELOG)
													{
																M_printf("[GPRS:3]GET  SEND OK  fail.....\r\n");
													}
								
													sys->gprssendcletimes=0;
													sys->datasendcheck =0;
													sys->reccheckcount=0; 
													sys->gprstimecount_5msEn=0;
													sys->gprstimecount_5ms=0;	
													sys->gprssendcletimes=0;
													sys->netok =0;
													sys->datasendflag =2;//����ʧ��
													sys->datasaveEN =1;
													if(sys->gprsbeatbusy ==1)
													{
															sys->gprsbeatbusy =0;
															sys->gprsbusy =0;
															beatflag=0;
													}
													if(Get_data_EN ==1)
													{
													sys->gprsdatabusy =0;
													sys->datacheck=5;
													RS485Flag =0;	
													sys->gprsbusy =0;
			//										sys->rs485busy=0;
													}
													if(sys->htydatasendEN ==1)
													sys->htysendcheck =0;
													sys->htydatasendEN =0;
								}									
								else{
											if(cfgsys->LogLev>=SIMPLELOG)
												{
													M_printf("[gprs:3]sys->reccheckcount=%d\r\n    ",sys->reccheckcount);
												}
											
											if(sys->reccheckcount++ >cle)  //��ⲻ��  ���粻ͨ
											{
												if(cfgsys->LogLev>=SIMPLELOG)
												{
													M_printf("[GPRS:3]find    SEND OK     fail........\r\n");
												}
												
												sys->datasendcheck =0;
												sys->reccheckcount=0; 
												sys->gprstimecount_5msEn=0;
												sys->gprstimecount_5ms=0;	
												sys->gprssendcletimes++;
												if(cfgsys->LogLev>=SIMPLELOG)
												{
													M_printf("[GPRS:3]GPRS_resend:%d\r\n",sys->gprssendcletimes );
												}
												
												sys->datasendflag =0;  ///����������
												if(sys->gprssendcletimes>2)
												{
													if(cfgsys->LogLev>=SIMPLELOG)
														{
															M_printf("[GPRS:3]GET  SEND OK  fail.....\r\n");
														}
													
													sys->gprssendcletimes=0;
													sys->datasendcheck =0;
													sys->reccheckcount=0; 
													sys->gprstimecount_5msEn=0;
													sys->gprstimecount_5ms=0;	
													sys->gprssendcletimes=0;
													sys->netok =0;
													sys->datasendflag =2;//����ʧ��
													sys->datasaveEN =1;
													if(sys->gprsbeatbusy ==1)
													{
															sys->gprsbeatbusy =0;
															sys->gprsbusy =0;
															beatflag=0;
													}
													if(Get_data_EN ==1)
													{
													sys->gprsdatabusy =0;
													sys->datacheck=5;
													RS485Flag =0;	
													sys->gprsbusy =0;
			//										sys->rs485busy=0;
													}
													if(sys->htydatasendEN ==1)
													sys->htysendcheck =0;
													sys->htydatasendEN =0;
												}
											}
											else{
											sys->datasendcheck=3;
											sys->gprstimecount_5msEn=1;
											sys->gprstimecount_5ms=0;
			//								sys->datasendflag=0;  ///��������
											}
											
										
									}
							}
						}
				break;	
				default:break;
	}
}
#endif


#if 0
u8 cle=100;
uint16_t crc = 0;
SDK_CMD_INFO sdk_cmd_info;
//  �ɹ�����1  ʧ�ܷ��� 2  ����0
void send_data_to_sever(u8 *buffer, u8 info ,u8 len)  //�����0Ϊ����
{ 
	u8 j=0;
	char str[20];
	if(cfgsys->LogLev>=SIMPLELOG)
	{
		M_printf("[GPRS:]sys->datasendcheck:%d.....\r\n",sys->datasendcheck);
	}		
	switch(sys->datasendcheck )
	{
		case 0:  // ָ�����͵��ֽڳ���
						if (0!=info)
				    {
							sdk_cmd_info.cmd_no = revs(info);///<���͵�����תΪ���ģʽ
							sdk_cmd_info.header[0] = 'H';
							sdk_cmd_info.header[1] = 'E';
							sdk_cmd_info.header[2] = 'A';
							sdk_cmd_info.header[3] = 'D';
							for(j=0;j<4;j++)
							sdk_cmd_info.sn[j]=SN[j];
							sdk_cmd_info.len = revs(sizeof(sdk_cmd_info));
							memcpy(sdk_cmd_info.data,buffer,158);
							crc = crc16_check((uint8_t*)&sdk_cmd_info,sizeof(sdk_cmd_info) -2);
							sdk_cmd_info.crc[0] = crc & 0x00ff;
							sdk_cmd_info.crc[1] = (crc & 0xff00) >> 8;
							sprintf(str,"AT+CIPSEND=%d\r\n",sizeof(sdk_cmd_info));
				   }else
							sprintf(str,"AT+CIPSEND=%d\r\n",2);		     
		        CLR_GPRS_Buf();
		      	UART3_SendString(str);
			      sys->datasendcheck=1;
						sys->reccheckcount=0; 
						sys->gprstimecount_5msEn=1;
						sys->gprstimecount_5ms=0;
//						sys->datdsendcletimes=0;
					break;
		case 1://����<
						if (sys->gprstimecount_5ms>0)//   10ms����һ��
						{
							sys->gprstimecount_5ms =0;
							if(Find (">"))
							{
								if(cfgsys->LogLev>=SIMPLELOG)
									{
										M_printf("[GPRS:1]find >  ok.....\r\n");
									}	
								
								sys->datasendcheck=2;
								sys->reccheckcount=0; 
								sys->gprstimecount_5msEn=0;
								sys->gprstimecount_5ms=0;
								if(Get_data_EN ==1)
								{		
									sys->datacheck =3;									
								}
							}
							else
							{
								if(sys->reccheckcount ++>3)  //��ⲻ��  ���粻ͨ
								{
									if(cfgsys->LogLev>=SIMPLELOG)
									{
											M_printf("[GPRS:1]find    >    fail........\r\n");
									}
								
									sys->datasendcheck =0;
									sys->reccheckcount=0; 
									sys->gprstimecount_5msEn=0;
									sys->gprstimecount_5ms=0;
									sys->netok =0;
									if(1==Get_data_EN )
									{
										sys->datacheck=5;
										sys->datasaveEN =1;
										RS485Flag =0;
										sys->gprsdatabusy =0;
										sys->gprsbusy =0;
									}
									if(sys->gprsbeatbusy ==1)
									{
										sys->gprsbeatbusy =0;
										sys->gprsbusy =0;
										beatflag=0;
									}
									if(sys->htydatasendEN ==1)
									{
									sys->htysendcheck =0;
									sys->htydatasendEN =0;
									}
								}
								else{
									if(cfgsys->LogLev>=SIMPLELOG)
									{
											M_printf("[GPRS:1]reccheckcount  >  :%d\r\n",sys->reccheckcount);
									}
									
									sys->datasendcheck =1;
									sys->gprstimecount_5msEn=1;
									sys->gprstimecount_5ms=0;
								}
							}
						}
						break;
			case 2://������Ҫ�ϴ�������
				if(cfgsys->LogLev>=SIMPLELOG)
					{
							M_printf("[GPRS:2]start send data.....\r\n");
					}
				
			  CLR_GPRS_Buf ();
				if (0!=info)
				{
					Send_DATA ((u8 *)&sdk_cmd_info ,sizeof(sdk_cmd_info));
				}else{
					Send_DATA (buffer ,2);
				}
				sys->datasendcheck=3;
				sys->reccheckcount=0; 
				sys->gprstimecount_5msEn=1;
				sys->gprstimecount_5ms=0;
				if(Get_data_EN ==1)
				{
//					 sys->datacheck =3;
					 sys->datasaveEN =0;
				}
				sys->htysendcheck =0;
				if(cfgsys->LogLev>=SIMPLELOG)
					{
							M_printf("[GPRS:2] data send  OK.....\r\n");
					}
				
				break;
			case 3://�ȴ�send  ok
				if (sys->gprstimecount_5ms>2)//   5ms����һ��
						{
							sys->gprstimecount_5ms =0;
							if(Find ("SEND OK"))
							{
								if(cfgsys->LogLev>=SIMPLELOG)
								{
										M_printf("[GPRS:2]GET send ok ...............\r\n");
								}
							
								sys->datasendcheck=0;
								sys->reccheckcount=0; 
								sys->gprstimecount_5msEn=0;
								sys->gprstimecount_5ms=0;
								sys->gprssendcletimes=0;
								sys->datasendflag=1;  ///���ͳɹ�				
								if(sys->gprsbeatbusy==1)
								{
									sys->gprsbeatbusy =0;
									sys->gprsbusy =0;
									beatflag=0;
								}
								if(1==Get_data_EN &&sys->htydatasendEN ==0)
								{
									sys->gprsdatabusy =0;
									if(sys->datacheck ==3)
								  sys->datacheck=4;
									RS485Flag =0;
									sys->gprsbusy =0;
//									sys->rs485busy =0;
								}
								if(sys->htydatasendEN ==1)
								{
								sys->htysendcheck =1;
								}
							}
							else
							{
								if(Find("SEND FAIL"))
								{
									if(cfgsys->LogLev>=SIMPLELOG)
								{
											M_printf("[GPRS:]GET  SEND OK  fail.....\r\n");
								}
								
													sys->gprssendcletimes=0;
													sys->datasendcheck =0;
													sys->reccheckcount=0; 
													sys->gprstimecount_5msEn=0;
													sys->gprstimecount_5ms=0;	
													sys->gprssendcletimes=0;
													sys->netok =0;
													sys->datasendflag =2;//����ʧ��
													sys->datasaveEN =1;
													if(sys->gprsbeatbusy ==1)
													{
															sys->gprsbeatbusy =0;
															sys->gprsbusy =0;
															beatflag=0;
													}
													if(Get_data_EN ==1)
													{
													sys->gprsdatabusy =0;
													sys->datacheck=5;
													RS485Flag =0;	
													sys->gprsbusy =0;
			//										sys->rs485busy=0;
													}
													if(sys->htydatasendEN ==1)
													sys->htysendcheck =0;
													sys->htydatasendEN =0;
								}									
								else{
											if(cfgsys->LogLev>=SIMPLELOG)
												{
													M_printf("sys->reccheckcount=%d\r\n    ",sys->reccheckcount);
												}
											
											if(sys->reccheckcount++ >cle)  //��ⲻ��  ���粻ͨ
											{
												if(cfgsys->LogLev>=SIMPLELOG)
												{
													M_printf("[GPRS:2]find    SEND OK     fail........\r\n");
												}
												
												sys->datasendcheck =0;
												sys->reccheckcount=0; 
												sys->gprstimecount_5msEn=0;
												sys->gprstimecount_5ms=0;	
												sys->gprssendcletimes++;
												if(cfgsys->LogLev>=SIMPLELOG)
												{
													M_printf("[GPRS:]GPRS_resend:%d\r\n",sys->gprssendcletimes );
												}
												
												sys->datasendflag =0;  ///����������
												if(sys->gprssendcletimes>2)
												{
													if(cfgsys->LogLev>=SIMPLELOG)
														{
															M_printf("[GPRS:]GET  SEND OK  fail.....\r\n");
														}
													
													sys->gprssendcletimes=0;
													sys->datasendcheck =0;
													sys->reccheckcount=0; 
													sys->gprstimecount_5msEn=0;
													sys->gprstimecount_5ms=0;	
													sys->gprssendcletimes=0;
													sys->netok =0;
													sys->datasendflag =2;//����ʧ��
													sys->datasaveEN =1;
													if(sys->gprsbeatbusy ==1)
													{
															sys->gprsbeatbusy =0;
															sys->gprsbusy =0;
															beatflag=0;
													}
													if(Get_data_EN ==1)
													{
													sys->gprsdatabusy =0;
													sys->datacheck=5;
													RS485Flag =0;	
													sys->gprsbusy =0;
			//										sys->rs485busy=0;
													}
													if(sys->htydatasendEN ==1)
													sys->htysendcheck =0;
													sys->htydatasendEN =0;
												}
											}
											else{
											sys->datasendcheck=3;
											sys->gprstimecount_5msEn=1;
											sys->gprstimecount_5ms=0;
			//								sys->datasendflag=0;  ///��������
											}
											
										
									}
							}
						}
				break;	
				default:break;
	}
}
#endif

#if 0
u8 cle=100;
uint16_t crc = 0;
SDK_CMD_INFO sdk_cmd_info;
//  �ɹ�����1  ʧ�ܷ��� 2  ����0
void send_data_to_sever(u8 *buffer, u8 info ,u8 len)  //�����0Ϊ����
{ 
	u8 j=0;
	char str[20];
	M_printf("[GPRS:]sys->datasendcheck:%d.....\r\n",sys->datasendcheck);
	switch(sys->datasendcheck )
	{
		case 0:  // ָ�����͵��ֽڳ���
						if (0!=info)
				    {
							sdk_cmd_info.cmd_no = revs(info);///<���͵�����תΪ���ģʽ
							sdk_cmd_info.header[0] = 'H';
							sdk_cmd_info.header[1] = 'E';
							sdk_cmd_info.header[2] = 'A';
							sdk_cmd_info.header[3] = 'D';
							for(j=0;j<4;j++)
							sdk_cmd_info.sn[j]=SN[j];
							sdk_cmd_info.len = revs(sizeof(sdk_cmd_info));
							memcpy(sdk_cmd_info.data,buffer,158);
							crc = crc16_check((uint8_t*)&sdk_cmd_info,sizeof(sdk_cmd_info) -2);
							sdk_cmd_info.crc[0] = crc & 0x00ff;
							sdk_cmd_info.crc[1] = (crc & 0xff00) >> 8;
							sprintf(str,"AT+CIPSEND=%d\r\n",sizeof(sdk_cmd_info));
				   }else
							sprintf(str,"AT+CIPSEND=%d\r\n",2);		     
		        CLR_GPRS_Buf();
		      	UART3_SendString(str);
			      sys->datasendcheck=1;
						sys->reccheckcount=0; 
						sys->gprstimecount_5msEn=1;
						sys->gprstimecount_5ms=0;
//						sys->datdsendcletimes=0;
					break;
		case 1://����<
						if (sys->gprstimecount_5ms>0)//   10ms����һ��
						{
							sys->gprstimecount_5ms =0;
							if(Find (">"))
							{
								M_printf("[GPRS:1]find >  ok.....\r\n");
								sys->datasendcheck=2;
								sys->reccheckcount=0; 
								sys->gprstimecount_5msEn=0;
								sys->gprstimecount_5ms=0;
								if(Get_data_EN ==1)
								{		
									sys->datacheck =3;									
								}
							}
							else
							{
								if(sys->reccheckcount ++>3)  //��ⲻ��  ���粻ͨ
								{
									M_printf("[GPRS:1]find    >    fail........\r\n");
									sys->datasendcheck =0;
									sys->reccheckcount=0; 
									sys->gprstimecount_5msEn=0;
									sys->gprstimecount_5ms=0;
									sys->netok =0;
									if(1==Get_data_EN )
									{
										sys->datacheck=5;
										sys->datasaveEN =1;
										RS485Flag =0;
										sys->gprsdatabusy =0;
										sys->gprsbusy =0;
									}
									if(sys->gprsbeatbusy ==1)
									{
										sys->gprsbeatbusy =0;
										sys->gprsbusy =0;
										beatflag=0;
									}
									if(sys->htydatasendEN ==1)
									{
									sys->htysendcheck =0;
									sys->htydatasendEN =0;
									}
								}
								else{
									M_printf("[GPRS:1]reccheckcount  >  :%d\r\n",sys->reccheckcount);
									sys->datasendcheck =1;
									sys->gprstimecount_5msEn=1;
									sys->gprstimecount_5ms=0;
								}
							}
						}
						break;
			case 2://������Ҫ�ϴ�������
				M_printf("[GPRS:2]start send data.....\r\n");
			  CLR_GPRS_Buf ();
				if (0!=info)
				{
					Send_DATA ((u8 *)&sdk_cmd_info ,sizeof(sdk_cmd_info));
				}else{
					Send_DATA (buffer ,2);
				}
				sys->datasendcheck=3;
				sys->reccheckcount=0; 
				sys->gprstimecount_5msEn=1;
				sys->gprstimecount_5ms=0;
				if(Get_data_EN ==1)
				{
//					 sys->datacheck =3;
					 sys->datasaveEN =0;
				}
				sys->htysendcheck =0;
				M_printf("[GPRS:2] data send  OK.....\r\n");
				break;
			case 3://�ȴ�send  ok
				if (sys->gprstimecount_5ms>2)//   5ms����һ��
						{
							sys->gprstimecount_5ms =0;
							if(Find ("SEND OK"))
							{
								M_printf("[GPRS:2]GET send ok ...............\r\n");
								sys->datasendcheck=0;
								sys->reccheckcount=0; 
								sys->gprstimecount_5msEn=0;
								sys->gprstimecount_5ms=0;
								sys->gprssendcletimes=0;
								sys->datasendflag=1;  ///���ͳɹ�				
								if(sys->gprsbeatbusy==1)
								{
									sys->gprsbeatbusy =0;
									sys->gprsbusy =0;
									beatflag=0;
								}
								if(1==Get_data_EN &&sys->htydatasendEN ==0)
								{
									sys->gprsdatabusy =0;
									if(sys->datacheck ==3)
								  sys->datacheck=4;
									RS485Flag =0;
									sys->gprsbusy =0;
//									sys->rs485busy =0;
								}
								if(sys->htydatasendEN ==1)
								{
								sys->htysendcheck =1;
								}
							}
							else
							{
								M_printf("sys->reccheckcount=%d\r\n    ",sys->reccheckcount);
								if(sys->reccheckcount++ >cle)  //��ⲻ��  ���粻ͨ
								{
									M_printf("[GPRS:2]find    SEND OK     fail........\r\n");
									sys->datasendcheck =0;
									sys->reccheckcount=0; 
									sys->gprstimecount_5msEn=0;
									sys->gprstimecount_5ms=0;	
									sys->gprssendcletimes++;
									M_printf("[GPRS:]GPRS_resend:%d\r\n",sys->gprssendcletimes );
									sys->datasendflag =0;  ///����������
									if(sys->gprssendcletimes>2)
									{
										M_printf("[GPRS:]GET  SEND OK  fail.....\r\n");
										sys->gprssendcletimes=0;
										sys->datasendcheck =0;
										sys->reccheckcount=0; 
										sys->gprstimecount_5msEn=0;
										sys->gprstimecount_5ms=0;	
										sys->gprssendcletimes=0;
										sys->netok =0;
										sys->datasendflag =2;//����ʧ��
										sys->datasaveEN =1;
										if(sys->gprsbeatbusy ==1)
										{
												sys->gprsbeatbusy =0;
												sys->gprsbusy =0;
												beatflag=0;
										}
										if(Get_data_EN ==1)
										{
										sys->gprsdatabusy =0;
										sys->datacheck=5;
										RS485Flag =0;	
										sys->gprsbusy =0;
//										sys->rs485busy=0;
										}
										if(sys->htydatasendEN ==1)
										sys->htysendcheck =0;
										sys->htydatasendEN =0;
									}
								}
								else{
								sys->datasendcheck=3;
								sys->gprstimecount_5msEn=1;
								sys->gprstimecount_5ms=0;
//								sys->datasendflag=0;  ///��������
								}
								
							
							}
						}
				break;	
				default:break;
	}
}
#endif


#if 0
u8 send_data_to_sever(u8 *buffer, u8 info)
{
	u8 j=0,flag=0,cnt=0;
	uint16_t crc = 0;
	SDK_CMD_INFO sdk_cmd_info;
	sdk_cmd_info.cmd_no = revs(info);///<���͵�����תΪ���ģʽ
	sdk_cmd_info.header[0] = 'H';
	sdk_cmd_info.header[1] = 'E';
	sdk_cmd_info.header[2] = 'A';
	sdk_cmd_info.header[3] = 'D';
	for(j=0;j<4;j++)
	sdk_cmd_info.sn[j]=SN[j];
	sdk_cmd_info.len = revs(sizeof(sdk_cmd_info));
	memcpy(sdk_cmd_info.data,buffer,158);
	crc = crc16_check((uint8_t*)&sdk_cmd_info,sizeof(sdk_cmd_info) -2);
	sdk_cmd_info.crc[0] = crc & 0x00ff;
	sdk_cmd_info.crc[1] = (crc & 0xff00) >> 8;
	flag=GPRS_send_DATA((u8 *)&sdk_cmd_info,sizeof(sdk_cmd_info));
	return flag;
}
u8 send_beat_to_sever(void)
{
	u8 flag=0,cnt=0;
	u8 beat[2]={0x00,0x00};
	Rebeat:
	flag=GPRS_send_DATA((u8 *)&beat,sizeof(beat));
//	M_printf("flag[beat��]:%d",flag);
	while(!flag)
		{
			delay_s(2);
			M_printf("beat resend......\r\n");
			flag=GPRS_send_DATA((u8 *)&beat,sizeof(beat));
			cnt++;
			if(3==cnt)
				{
					cnt=0;
					GPRSstaFlag=0;
					M_printf("beat resend  faile..........\r\n");
//					connect_4G_tcp_server();
					sys->netok=0;
					break;
				}
		}
	
}
#endif 

//����������0 ������� 1 ��������
void heartbeat(u8 num)
{
	if(num==0)
		{
			if(cfgsys->LogLev>=SIMPLELOG)
				{
							M_printf("clear beat.....\r\n");
				}
			
			beatflag=0;
			cnt_1s=0;
			TIM_Cmd(TIM3,DISABLE);//ʹ�ܶ�ʱ��4
			TIM_SetCounter(TIM3,0);//��������� 
		}
		else if(num==1)
		{
			if(cfgsys->LogLev>=SIMPLELOG)
				{
							M_printf("set beat.....\r\n");
				}
			
			cnt_1s=0;
			TIM_Cmd(TIM3,ENABLE);//ʹ�ܶ�ʱ��4
			TIM_SetCounter(TIM3,0);//��������� 	
		}
}


