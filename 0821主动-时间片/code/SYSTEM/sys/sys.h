#ifndef __SYS_H
#define __SYS_H	
#include "stm32f10x.h"
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "delay.h"
#include "usart.h"	
//////////////////////////////////////////////////////////////////////////////////	 

																	    
	 
//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).
//IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO�ڵ�ַӳ��
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����
//������ƥ��
#define V3   0
#define Mini 1
#define TotalStorage 	     1000
#define CFGSYSADDATART	   0
#define HISData1ADDSTART 	 1024
#define HISData2ADDSTART 	 1024*1024
#define DataBLOCK          256
#define Passive             1
#define Initiative          2
#define first               1
//ģʽ����
#define MODE_PRO 1		//����ģʽ
#define MODE_ENG 2		//����ģʽ
#define MODE_RUN 3		//����ģʽ
#define MODE_SLE 4		//����ģʽ
//��־��ӡ����
#define NOLOG	  	0
#define SIMPLELOG	1
#define DETAILLOG	2
#define ALLLOG		3


extern struct flash_t uatflash,iatfalsh;
extern u8 Get_data_EN;
extern u8 Get_State_EN;
extern struct sys_t systm,*sys;
extern struct h_cfg_sys_t *cfgsys,cfg_sys_t;
extern struct TIME_t time_t;
extern u32 sys_second;
extern u8 u8_4_buffer[4];
extern u32 u32_temp;
extern u32 Random_num1;
extern u32 Random_num2;
struct flash_t
{
	u16 flash_RW_1num;
  u16 flash_RD_1num;
	u8 flash_send_1En;
	u16 flash_RW_2num;
  u16 flash_RD_2num;
	u8 flash_send_2En;
};
struct TIME_t
{
	int year;
	int month;
	int day;
	int hour;
	int minute;
	int second;
};

struct h_cfg_sys_t
{
	u32 devid;				//�豸ID
	char ip[20];			//������IP
	u16 port;				//�������˿�
	char updataIP[20];		//����
	u16 updataPort;
	char apnName[32];		//apn
	char apnUser[32];
	char apnPass[32];	
	u8 TransferCycle;		//���մ����ϴ����
	u8 SupplyPowerType;		//�������
	u16 ProModeExitTimer;	//����ģʽ�����˳�ʱ��	
	u16 EngModeMaxTimer;	//����ģʽ�������ʱ��
	u16 RunModeMaxTimer;	//����ģʽ�������ʱ��	
	u8 LogLev;				//��־����ȼ�
	u8 EngTestImgTmr;		//���̲���ͼ����ʱ��
	u8 EngTestImgNum;		//���̲���ͼ������
	u8 EngModeMaxTimerUnSle;//����ģʽ����ʹ��   0����  1������
};

//ϵͳ����
struct sys_t
{
	u8 modetype;		       	//����ģʽ
	u32 systimerCountEn;		//ʹ��count�ۼ�
	u16 systimerCount_s;	  //��ʱs����
	u16 systimerCount_ms;	  //��ʱms����
	u8 sysgprstimegetEN;    //�Ƿ��ȡϵͳʱ��

	u32 systmr;       //ϵͳ���ļ�����
//�������ر�ʶ
//	u8 ledDriverEn;			//ָʾ����������״̬
//	u8 debugDriverEn;
//	u8 key1DriverEn;
//	u8 rtcDriverEn;
//	u8 flashDriverEn;
//	u8 cameraDriverEn;
//	u8 powermonitorDriverEn;
	u8 gprsDriverEn;
//	u8 jlinkDriverEn;
	u8 timerDriverEn;	
	u8 datasaveEN;
////led	
//	u16 ledtimerCount_ms;			//��ʱ���ۼ� 1000
//	u8 led1state;			//ָʾ��״̬	
////key1	
//	u8 key1scan;			//����1ɨ��ʱ�䵽
//	u16 key1count_20ms;		//					20
//	u8 key1statetemp;		//������ʱ״̬
//	u8 key1state;			//����1״̬	
////debug	
//	u8 debugcount_10ms;		//					10
//	u8 debugscan;	
//	

////flash
//	u8 saveFlashimageEn;		//���д洢
//	u8 flashstate;			
//	
//	u8 readflashtitleok;		//��ȡͳ������ok
//	u8 readflashimgeok;			//��ȡFLASHͼ������OK

//gprs
	u8 netstate;
	u8 gprsstate;		///	1����
	u8 gprssyncbaud;	//����Ӱ������
	u8 gprscheck;
	u8 gprsack;
	u8 gprsresendcheck;
	u8 gprstimecount_10msEn;
	u16 gprstimecount_10ms;
	u16 gprstimecount_ms;
	u8 gprstimecount_5msEn;
	u8 gprstimecount_5ms;
	u8 gprstimecount_sEn;
	u8 gprstimecount_s;
	u16 gprscheckcount;
	u16 gprssendlen;		//���ͳ���
	u8 gprssendpacket;		//���Ͱ�
	u8 gprssendEn;
	u8 gprssendstate;
	u8 gprsdatabusy;
	u8 gprsbeatbusy;
	u8 gprshtybusy;
	u8 gprsbusy;
	u8 netok;
	u8 gprsrestEN;
	u8 gprssendcletimes;
	u8 Callreadyflag;
	u8 gprsTCPchecknum;
	
	
	u8 datatimecount_5msEn;
	u16 datatimecount_ms;
	u16 datatimecount_5ms;  
	u8 Rs485cletimes;
	u8 rs485gettimes;
	u8 rs485busy;
	u8 datacheckcount;  //485���ռ������
	u8 datacheck;       //���ݼ���
	
	u8 datasendEN;   //ʵʱ���ݷ���
	u8 datasendcheck; //���ͼ���
	u8 reccheckcount;//GPRS�������
	u8 datdsendcletimes;    //�ط�����
	u8 datasendflag;//���ͳɹ�ʧ�ܱ�־λ
	 
	
	u8 htytimecount_5msEn;
	u16 htytimecount_ms;
	u16 htytimecount_5ms;
	u8 htydatasendEN;
	u8 htysendcheck;
	u8 htyreccount;
	u8 htycletimes;
	u8 htysendstate;
};
extern u8 SN[4];
extern struct sys_t *sys;
//����Ϊ��ຯ��
void WFI_SET(void);		//ִ��WFIָ��
void INTX_DISABLE(void);//�ر������ж�
void INTX_ENABLE(void);	//���������ж�
void MSR_MSP(u32 addr);	//���ö�ջ��ַ
void timerTick_app(void);//ϵͳ���ĺ���
u8 SysTick_Init(void); //�δ��ʱ����ʼ��
void data_send_check(void);
void Datasend(void);
void Data_save(u8 num);
void Cf_init(void);
void processRunningMode(void);
void processRunningApp(void);
void    RunningApp(void);
void beat_APP(void );
void get_APP(void);
u32 time_to_second(void );
u32 Get_random_num(void);
void u32_to_u8(u32 *num ,u8 *buffer);
void u8_to_u32(u32 *num ,u8 *buffer);
void Adc_Init(void);
#endif
