#ifndef __SYS_H
#define __SYS_H	
#include "stm32f10x.h"
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "delay.h"
#include "usart.h"	
//////////////////////////////////////////////////////////////////////////////////	 

																	    
	 
//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
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
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入
//开发板匹配
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
//模式定义
#define MODE_PRO 1		//生产模式
#define MODE_ENG 2		//工程模式
#define MODE_RUN 3		//运行模式
#define MODE_SLE 4		//休眠模式
//日志打印定义
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
	u32 devid;				//设备ID
	char ip[20];			//服务器IP
	u16 port;				//服务器端口
	char updataIP[20];		//升级
	u16 updataPort;
	char apnName[32];		//apn
	char apnUser[32];
	char apnPass[32];	
	u8 TransferCycle;		//拍照次数上传间隔
	u8 SupplyPowerType;		//电池类型
	u16 ProModeExitTimer;	//生产模式空闲退出时间	
	u16 EngModeMaxTimer;	//工程模式运行最大时长
	u16 RunModeMaxTimer;	//运行模式运行最大时长	
	u8 LogLev;				//日志输出等级
	u8 EngTestImgTmr;		//工程测试图像间隔时间
	u8 EngTestImgNum;		//工程测试图像张数
	u8 EngModeMaxTimerUnSle;//工程模式休眠使能   0休眠  1不休眠
};

//系统定义
struct sys_t
{
	u8 modetype;		       	//运行模式
	u32 systimerCountEn;		//使能count累加
	u16 systimerCount_s;	  //延时s计数
	u16 systimerCount_ms;	  //延时ms计数
	u8 sysgprstimegetEN;    //是否获取系统时钟

	u32 systmr;       //系统节拍计数器
//驱动加载标识
//	u8 ledDriverEn;			//指示灯驱动加载状态
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
//	u16 ledtimerCount_ms;			//定时器累加 1000
//	u8 led1state;			//指示灯状态	
////key1	
//	u8 key1scan;			//按键1扫描时间到
//	u16 key1count_20ms;		//					20
//	u8 key1statetemp;		//按键临时状态
//	u8 key1state;			//按键1状态	
////debug	
//	u8 debugcount_10ms;		//					10
//	u8 debugscan;	
//	

////flash
//	u8 saveFlashimageEn;		//进行存储
//	u8 flashstate;			
//	
//	u8 readflashtitleok;		//获取统计数据ok
//	u8 readflashimgeok;			//获取FLASH图像数据OK

//gprs
	u8 netstate;
	u8 gprsstate;		///	1重启
	u8 gprssyncbaud;	//自摄影波特率
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
	u16 gprssendlen;		//发送长度
	u8 gprssendpacket;		//发送包
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
	u8 datacheckcount;  //485接收检验次数
	u8 datacheck;       //数据检验
	
	u8 datasendEN;   //实时数据发送
	u8 datasendcheck; //发送检验
	u8 reccheckcount;//GPRS检验次数
	u8 datdsendcletimes;    //重发次数
	u8 datasendflag;//发送成功失败标志位
	 
	
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
//以下为汇编函数
void WFI_SET(void);		//执行WFI指令
void INTX_DISABLE(void);//关闭所有中断
void INTX_ENABLE(void);	//开启所有中断
void MSR_MSP(u32 addr);	//设置堆栈地址
void timerTick_app(void);//系统节拍函数
u8 SysTick_Init(void); //滴答计时器初始化
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
