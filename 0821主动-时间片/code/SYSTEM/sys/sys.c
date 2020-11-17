#include "sys.h"
#include "w25qxx.h"
#include "GPRS.h"
#include "RS485.h"
#include "timer.h"
#include "GPRS.h"
//////////////////////////////////////////////////////////////////////////////////	 
//static u32 SysTickDelayTime;
//volatile u32 sys_time_s = 0;
#define  State 0
#define Data_MAX_num   720
#define  Data  1
#define data_block 41
#define is_leap_year(y) (((y) % 4  == 0 && (y) % 100 != 0) || (y) % 400 == 0)
u8 SN[4]={0x19,0x07,0x14,0x01};
u8 get_data[8]={0x02,0x03,0x00,0x00,0x00,0x0A,0xC5,0xFE};
u8 get_state[8]={0x02,0x03,0x00,0x0B,0x00,0x01,0xF5,0xFB};
u8 beat[2]={0x00,0x00};
struct flash_t flash;
struct sys_t systm,*sys;
struct h_cfg_sys_t *cfgsys,cfg_sys_t;
struct TIME_t time_t;
u8 Data_buff[158];
u8 Get_data_EN=0;  //获取数据使能  0：不获取  1：获取
u8 Get_State_EN=0; //获取状态使能  0：不获取  1：获取
u8 last_num=0;
u8 this_num=0;
u8 data_save[29520];
u8 send_buff[158];
u8 u8_4_buffer[4];
u32 u32_temp=0;
u32 Random_num1=0;
u32 Random_num2=0;
u32 Second_temp=0;
u32 Ms_temp=0;
u32 Random1_temp1=0;
u32 Random2_temp2=0;
u32 RD_num=0;
u32 RW_num=0;
u8 sendhistorydataEN=0;
u32 sys_second=0;

void WFI_SET(void)
{
	__ASM volatile("wfi");		  
}
//关闭所有中断
void INTX_DISABLE(void)
{		  
	__ASM volatile("cpsid i");
}
//开启所有中断
void INTX_ENABLE(void)
{
	__ASM volatile("cpsie i");		  
}
//设置栈顶地址
//addr:栈顶地址
__asm void MSR_MSP(u32 addr) 
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
}

void u32_to_u8(u32 *num ,u8 *buffer)
{
	buffer[0]=buffer[1]=buffer[2]=buffer[3]=0xFF;
	buffer[3]&=*num;
	buffer[2]&=*num>>8;
	buffer[1]&=*num>>16;
	buffer[0]&=*num>>24;
}
void u8_to_u32(u32 *num ,u8 *buffer)
{
//  buffer[0]=buffer[1]=buffer[2]=buffer[3]=0xFF;
//	buffer[3]&=num;
//	buffer[2]&=num>>8;
//	buffer[1]&=num>>16;
//	buffer[0]&=num>>24;
	*num = (buffer[0]<<24)|(buffer[1]<< 16)|(buffer [2]<<8)|buffer[3];
}
u32 time_to_second(void )
{
	int i = 0;
	u32 sec = 0;
	int days[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

	if(time_t.year!=0)
	{
		for(i = 19; i < time_t.year; i++)
    {
        if(is_leap_year(i))
            sec += 366 * 24 * 60 * 60;
        else
            sec += 365 * 24 * 60 * 60;
    }
		
    for(i = 1; i < time_t.month ; i++)
    {
        sec += days[i] * 24 * 60 * 60;
        if(i == 2 && is_leap_year(time_t.year))
        {
            sec += 24 * 60 * 60;
        }
    }

    sec += (time_t.day - 1) * 24 * 60 * 60;

    sec += time_t.hour  * 60 * 60 + time_t.minute  * 60 + time_t.second ;

    return sec;
	}
	else return 0;
}

void Adc_Init(void)
{
ADC_InitTypeDef ADC_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_ADC1, ENABLE );

RCC_ADCCLKConfig(RCC_PCLK2_Div6);//设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M


GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
GPIO_Init(GPIOC, &GPIO_InitStructure);
ADC_DeInit(ADC1);

ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
ADC_InitStructure.ADC_ScanConvMode = DISABLE;
ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
ADC_InitStructure.ADC_NbrOfChannel = 1;
ADC_Init(ADC1, &ADC_InitStructure);
ADC_Cmd(ADC1, ENABLE);
ADC_ResetCalibration(ADC1);
while(ADC_GetResetCalibrationStatus(ADC1));
ADC_StartCalibration(ADC1);
while(ADC_GetCalibrationStatus(ADC1));

} 
u16 Get_Adc(u8 ch)   
{
  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通道,采样时间为239.5周期	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}




u16 iSeed=0,iRand=0;//
u32 Get_random_num(void)
{
Adc_Init();//ADC???

iSeed=Get_Adc(ADC_Channel_14);
srand(iSeed);
return iRand=1+(u32)(rand()%2147483647);
}

//系统滴答计时器初始化
//输出:1正常	0配置失败
u8 SysTick_Init(void)
{   
	if (SysTick_Config(SystemCoreClock / 1000) == 0)  	//V1.30 stm32l15x
	{   
//	  M_printf("SysTick_Init ok\r\n");
		return 1;	/////////////正常
	}
	else
	{
//		M_printf("SysTick_Init err\r\n");
		return 0;	//
	}
}

//滴答计时器获取系统节拍
void timerTick_app(void)
{
//	sys->systmr++;
//	if(sys->systimerCountEn)
//	{
//	sys->systimerCount_ms++;
//	}
	if(sys->timerDriverEn == 1)
	{	
		
		if(sys->systimerCountEn==1)   //时间戳获取到网络时间进行维护
		{
			if(sys->systimerCount_ms++>1000)	
			{
				sys->systimerCount_ms = 0;
				sys_second++;
//				sys->systimerCount_s++;		//秒累加
			}		
		}
//		if(sys->ledDriverEn == 1)		//led时钟
//		{
//			sys->ledtimerCount_ms++;	
//		}

//		if(sys->key1DriverEn == 1 )			//key时钟
//		{
//			sys->key1count_20ms++;
//			if(sys->key1count_20ms>=20) 
//			{  
//				sys->key1count_20ms=0;
//				sys-> key1scan=1;           //20ms检测一次                       
//			}
//		}
//		if(sys->debugDriverEn == 1)
//		{
//			sys->debugcount_10ms++;
//			if(sys->debugcount_10ms>=10) 
//			{  
//				sys->debugcount_10ms=0;
//				sys-> debugscan=1;           //10ms检测一次                       
//			}		
//		}
//		if(sys->cameraDriverEn == 1)
//		{	
//			if(sys->sendcmd == 1)	//有发送命令
//			{
//				sys->getimgetmrcount_s++;
//				if(sys->getimgetmrcount_s>=4000) 
//				{
//					sys->getimgetmrcount_s=0;				
//					sys->sendcmd = 0;	
//					sys->getimgewaitok = 1;
//				}	
//			}	
//		}
		if(sys->gprstimecount_sEn == 1)
		{	
			if(sys->gprstimecount_ms++>1000)
			{
//				M_printf("delay 1S ...ok  \r\n");
//	     M_printf("GPRS-SYS-[rec]:%s\r\n",Uart3_GPRS_Buf);
				sys->gprstimecount_ms = 0;
				sys->gprstimecount_s++;
			}	
		}
		if(sys->gprstimecount_10msEn == 1)
		{	
			if(sys->gprstimecount_ms++>10)
			{
				sys->gprstimecount_ms = 0;
				sys->gprstimecount_10ms++;
			}	
		}
		if(sys->gprstimecount_5msEn == 1)
		{	
			if(sys->gprstimecount_ms++>5)
			{
				sys->gprstimecount_ms = 0;
				sys->gprstimecount_5ms++;
			}	
		}
		if(sys->datatimecount_5msEn ==1)
		{
			if(sys->datatimecount_ms++>5)
			{
				sys->datatimecount_ms = 0;
				sys->datatimecount_5ms++;
			}	
		}
		if(sys->htytimecount_5msEn ==1)
		{
			if(sys->htytimecount_ms++>5)
			{
				sys->htytimecount_ms = 0;
				sys->htytimecount_5ms++;
			}	
		}

//		if(sys->dtutimecount_3minEn == 1)
//		{
//			//if(sys->dtutimecount_m++>60000)
//			if(sys->dtutimecount_m++ > ((cfgsys->EngTestImgTmr) *1000))
//			{
//				sys->dtutimecount_m = 0;
//				sys->dtutimecount_3minF=1;
//			//	sys->dtuwaitcount++;
//			}
//		}	
	}
}
//参数初始化：历史数据存储初始化
void Cf_init(void )
{
// u8 bufff[300]={0};
//	#if first
//	flash.flash_RD_1num =0;
//  flash.flash_RD_2num =0;
//	flash.flash_RW_1num =0;
//	flash.flash_RW_2num =0;
//	flash.flash_send_1En =0;
//	flash.flash_send_2En =0;	
//	W25QXX_Write((u8 *)&flash,0,6);
//	#endif
//	W25QXX_Read((u8 *)&flash,0,6);
//  data_send_check();
//	memset(Data_buff ,0,158);
	cfgsys->LogLev=SIMPLELOG;
	sys->sysgprstimegetEN=0;
	sys->datacheck=0;
	sys->datacheckcount =0;
	sys->datasaveEN=0;
	sys->datatimecount_5ms =0;
	sys->datatimecount_5msEn=0;
	sys->datatimecount_ms=0;
	sys->netok = 0;	
	sys->netstate = 0;
	sys->gprsstate = 0;
	sys->gprscheck = 0;
	sys->gprsack = 0;
	sys->gprstimecount_10msEn = 0;
	sys->gprstimecount_10ms = 0;
	sys->gprstimecount_5msEn =0;
	sys->datatimecount_5ms =0;
	sys->gprstimecount_ms = 0;
	sys->gprscheckcount = 0;
	sys->gprssendlen = 0;
	sys->gprssendpacket = 0;
	sys->gprssendEn = 0;
	sys->gprssendstate = 0;
	sys->netok = 0;	
	sys->systimerCountEn=0;
	sys->timerDriverEn =0;
	sys->systimerCount_ms=0;
	sys->systimerCount_s =0;
	sys->gprsdatabusy=0;
	sys->gprsbeatbusy=0; 
	sys->gprshtybusy =0;
	sys->gprsbusy=0;
	sys->gprsresendcheck=0;
	sys->gprsTCPchecknum=0;
	sys->gprssendcletimes=0;
	sys->htycletimes=0;
	sys->htydatasendEN=0;
	sys->gprsrestEN=0;


	 sys->gprstimecount_sEn=0;
	 sys->gprstimecount_s=0;
	 sys->Rs485cletimes=0;
	 sys->rs485gettimes=0;
	 sys->rs485busy=0;
	
	 sys->datasendEN=0;   //实时数据发送
	 sys->datasendcheck=0; //发送检验
	 sys->reccheckcount=0;//GPRS检验次数
	 sys->datdsendcletimes=0;    //重发次数
	 sys->datasendflag=0;//发送成功失败标志位
	 
	
	 sys->htytimecount_5msEn=0;
	 sys->htytimecount_ms=0;
	 sys->htytimecount_5ms=0;
	 sys->htydatasendEN=0;
	 sys->htysendcheck=0;
	 sys-> htyreccount=0;
	 sys->htycletimes=0;
	 sys->htysendstate=0;
	 
	 memset(data_save ,0,2500);
	 M_printf("sys_cf    ok ......\r\n");
}
void  clear_sendbuf(void)
{
	u8 i=0;
	for(i=0;i<25;i++)
	send_buff[i]=0;
}

//数据缓存写
//输入：buff要缓存的数据，addr：要存的起始位置，Dnum缓存的数据长度
void data_write(u8* buff,u32 addr,u16 Dnum )
{
	u16 i=0;
	
	u32 startaddr=addr;
	for (i=0;i<Dnum;i++)
	{
		data_save[startaddr]=buff[i];
		startaddr ++;
	}
}
//数据缓存写
//输入：buff要缓存读取的数据，addr：要读取的起始位置，Dnum读取的数据长度
void  data_read(u8 *buff,u32 addr,u16 Dnum) 
{
	u16 i=0;
	u32 startaddr=addr;
	for (i=0;i<Dnum;i++)
	{
		buff[i]=data_save[startaddr];
		startaddr ++;
	}
}
//检验是否需要发送历史数据
void data_send_check(void)
{
	#if 0
//	M_printf("send check...\r\n");
//	M_printf ("RD_num1:%d    RW_num1:%d",flash.flash_RD_1num,flash.flash_RW_1num);
	if(flash.flash_RD_1num!=flash.flash_RW_1num)
		flash.flash_send_1En=1;
	else flash.flash_send_1En=0;
//	M_printf ("RD_num2:%d    RW_num2:%d",flash.flash_RD_1num,flash.flash_RW_1num);
	if(flash.flash_RD_2num!=flash.flash_RW_2num)
		flash.flash_send_2En=1;
	else flash.flash_send_2En=0;
	#endif
	if (RD_num!=RW_num)
	{
		if(cfgsys->LogLev>=SIMPLELOG)
			{
				M_printf("[check:]RD_num:%d      RW_num:%d\r\n",RD_num,RW_num);
			}	
		
	  sys->htydatasendEN=1;
	}else sys->htydatasendEN=0;
}

//发送历史数据
void HtyDatasend(void)
{
	u8 i=0;
	data_send_check();
//	M_printf("htydatasendAPP.....\r\n");
	if(1==sys->htydatasendEN&&sys->sysgprstimegetEN==1)
	{
			if(sys->gprsbusy==0||sys->gprshtybusy ==1)
			{
				if(sys->gprshtybusy ==0)
				{
					sys->gprshtybusy=1;
				}
				  sys->gprsbusy=1;
				if(cfgsys->LogLev>=DETAILLOG)
					{
						M_printf("[hty]History Data need to send......\r\n");
					}
					
				if(cfgsys->LogLev>=DETAILLOG)
					{
						M_printf("[hty]:sys->gprsdatabusy:%d  sys->gprsbeatbusy:%d \r\n",sys->gprsdatabusy,sys->gprsbeatbusy);
					}
				
				if((sys->gprsdatabusy==0)&&(sys->gprsbeatbusy ==0))
				{
					
					if(cfgsys->LogLev>=DETAILLOG)
					{
						M_printf("[hty]sys->htysendcheck:%d     sys->datasendcheck:%d\r\n",sys->htysendcheck ,sys->datasendcheck );
					}
					
					switch(sys->htysendcheck)
					{
						case 0:
								if(cfgsys->LogLev>=DETAILLOG)
									{
										M_printf("[hty-0]start to send htydata\r\n");
									}
								
								heartbeat(0);
//						    M_printf("RD_num:%d\r\n",RD_num);
								clear_sendbuf ();
								data_read (u8_4_buffer,RD_num*data_block+25,4);
								
								if(cfgsys->LogLev>=DETAILLOG)
								{
									for(i=0;i<4;i++)
									{
									M_printf("[hty-0]data_save[%d]：%d\r\n",i,data_save[RD_num*data_block+25+i]);
									}
									M_printf("\r\n");
									for(i=0;i<4;i++)
									{
									M_printf("[hty-0]u8_4_buffer[%d]：%d\r\n",i,u8_4_buffer[i]);
									}
									M_printf("\r\n");
								}									
								u8_to_u32(&u32_temp,u8_4_buffer);
									
								if(cfgsys->LogLev>=SIMPLELOG)
								{
									M_printf("[hty-0]u32_temp：%d\r\n",u32_temp);
								}
								if(u32_temp!=0)
								{
									if(cfgsys->LogLev>=SIMPLELOG)
									{
										M_printf("[hty-0]start to send htydata(include time)\r\n");
									}
									data_read (u8_4_buffer,RD_num*data_block+25,4);
									u8_to_u32(&sdk_cmd_info.second,u8_4_buffer);
									M_printf("[hty-0]%d\r\n",sdk_cmd_info.second);
									sdk_cmd_info.second=revl(sdk_cmd_info.second);
									
									data_read (u8_4_buffer,RD_num*data_block+29,4);
									u8_to_u32(&sdk_cmd_info.ms,u8_4_buffer);
									sdk_cmd_info.ms=revl(sdk_cmd_info.ms);
									
									data_read (u8_4_buffer,RD_num*data_block+33,4);
									u8_to_u32(&sdk_cmd_info.sys_num1,u8_4_buffer);
									sdk_cmd_info.sys_num1=revl(sdk_cmd_info.sys_num1);
									
									data_read (u8_4_buffer,RD_num*data_block+37,4);
									u8_to_u32(&sdk_cmd_info.sys_num2,u8_4_buffer);
									sdk_cmd_info.sys_num2=revl(sdk_cmd_info.sys_num2);
								}
								else
								{
									if(cfgsys->LogLev>=SIMPLELOG)
									{
										M_printf("[hty-0]start to send htydata(no  time)\r\n");
									}
									sdk_cmd_info.second=revl(sys_second);
									sdk_cmd_info.ms=revl(sys->systimerCount_ms);
									
									sdk_cmd_info.sys_num1=Get_random_num();
									sdk_cmd_info.sys_num1=revl(sdk_cmd_info.sys_num1);
									
									sdk_cmd_info.sys_num2=Get_random_num();	
									sdk_cmd_info.sys_num2=revl(sdk_cmd_info.sys_num2);
									
									sdk_cmd_info.second=revl(sdk_cmd_info.second);
									u32_to_u8(&sdk_cmd_info.second,u8_4_buffer);				
									data_write (u8_4_buffer,RD_num*data_block+25,4);
									sdk_cmd_info.ms=revl(sdk_cmd_info.ms);
									u32_to_u8(&sdk_cmd_info.ms,u8_4_buffer);	
									data_write (u8_4_buffer,RD_num*data_block+29,4);
									sdk_cmd_info.sys_num1=revl(sdk_cmd_info.sys_num1);
									u32_to_u8(&sdk_cmd_info.sys_num1,u8_4_buffer);	
									data_write (u8_4_buffer,RD_num*data_block+33,4);
									sdk_cmd_info.sys_num2=revl(sdk_cmd_info.sys_num2);
									u32_to_u8(&sdk_cmd_info.sys_num2,u8_4_buffer);	
									data_write (u8_4_buffer,RD_num*data_block+37,4);
									
								}
								data_read(send_buff,RD_num*data_block,25);	
									if(cfgsys->LogLev>=DETAILLOG)
									{
										for(i=0;i<25;i++)
										M_printf("% 02X ",send_buff [i]);
										M_printf("\r\n");
									}
								send_data_to_sever (send_buff ,5,25);
//								sys->htysendcheck =1;
						break;
						case 1:
							if(cfgsys->LogLev>=SIMPLELOG)
									{
										M_printf("[hty-1]sys->datasendflag :%d......\r\n",sys->datasendflag );
									}
							
							if(0==sys->datasendflag )//重试中继续等待
							{
								sys->htysendcheck =1;
							}
							else if(2==sys->datasendflag)//
							{
								if(cfgsys->LogLev>=SIMPLELOG)
									{
											M_printf("[hty-1]data send to server fail .......\r\n");
									}
								if(u32_temp==0)
								{
									revl(sdk_cmd_info.second);
									u32_to_u8(&sdk_cmd_info.second,u8_4_buffer);				
									data_write (u8_4_buffer,RW_num*data_block+25,4);
									revl(sdk_cmd_info.ms);
									u32_to_u8(&sdk_cmd_info.ms,u8_4_buffer);	
									data_write (u8_4_buffer,RW_num*data_block+29,4);
									revl(sdk_cmd_info.sys_num1);
									u32_to_u8(&sdk_cmd_info.sys_num1,u8_4_buffer);	
									data_write (u8_4_buffer,RW_num*data_block+33,4);
									revl(sdk_cmd_info.sys_num2);
									u32_to_u8(&sdk_cmd_info.sys_num2,u8_4_buffer);	
									data_write (u8_4_buffer,RW_num*data_block+37,4);
								}									
								sys->datasendflag=0;
								sys->htydatasendEN=0;
								sys->htysendcheck=0;
								sys->gprshtybusy=0;
								sys->gprsbusy=0;
								sys->netok=0;
								clear_sendbuf(); 
							}
							else if(1==sys->datasendflag )
							{
								sys->datasendflag=0;			
								sys->htydatasendEN=0;
								if(cfgsys->LogLev>=SIMPLELOG)
									{
											M_printf("[hty-1]data send to server ok .......\r\n");
									}
								
								heartbeat(1);
								if(RD_num==Data_MAX_num)
								{
									RD_num=0;
								}
								else RD_num++;
								sys->htysendcheck=0;
								sys->gprshtybusy=0;
								sys->gprsbusy=0;
								clear_sendbuf();
								if(sys->htysendcheck==1)
									sys->htysendcheck =0;
                heartbeat(1);								//清除发送缓存
							}
							break;
				   default:break;
					}
			
				}
		  }
			else sys->htydatasendEN=0;
	}
//	while(flash.flash_send_1En)
//	{
//		heartbeat(0);
//		Clear_databuff();
//		if(flash.flash_RD_1num==1000)
//			W25QXX_Read(Data_buff ,HISData1ADDSTART,158);
//		else 
//		   W25QXX_Read(Data_buff ,HISData1ADDSTART+flash.flash_RD_1num*DataBLOCK ,158);
//		for(i=0;i<160;i++)
//		M_printf(" %02x ",Data_buff[i]);
//		M_printf("\r\n");
//		flag=send_data_to_sever(Data_buff ,4);
//		if(flag==1)
//			{
//				if(flash.flash_RD_1num==1000)
//					flash.flash_RD_1num=0;
//				else flash.flash_RD_1num++;
//			}
//		W25QXX_Write((u8 *)&flash,0,sizeof(uatflash));
//		data_send_check();
//		heartbeat(1);
//	}
//	while(flash.flash_send_2En)
//	{
//		heartbeat(0);
//		Clear_databuff();
//		if(flash .flash_RD_2num ==1000)
//			W25QXX_Read(Data_buff ,HISData2ADDSTART,25);
//		else W25QXX_Read(Data_buff ,HISData2ADDSTART+flash.flash_RD_2num*DataBLOCK ,25);
//		for(i=0;i<158;i++)
//		M_printf(" %02x ",Data_buff[i]);
//		M_printf("\r\n");
//		flag =send_data_to_sever(Data_buff ,5);
//		if(flag ==1)
//		{
//			if(flash.flash_RD_2num==1000)
//				flash.flash_RD_2num=0;
//			else flash.flash_RD_2num++;
//		}
//		W25QXX_Write((u8 *)&flash,0,sizeof(uatflash));
//		data_send_check();
//		heartbeat(1);
//	}
}
//数据存储
void Data_save(u8 num)
{
//	u8 i=0;
	W25QXX_Read((u8 *)&flash,0,sizeof(uatflash));
//	for(i=0;i<158;i++)
//		M_printf(" %02x ",RS485_RX_BUF[i]);
//		M_printf("\r\n");
	if(Passive==num)
	{
		if(flash.flash_RW_1num==1000)
			W25QXX_Write(RS485_RX_BUF ,HISData1ADDSTART,158);
		else W25QXX_Write(RS485_RX_BUF ,HISData1ADDSTART+flash.flash_RW_1num*DataBLOCK ,158);
		if(flash.flash_RW_1num==1000)
			flash.flash_RW_1num=0;
		else flash.flash_RW_1num++;
	}
	if(Initiative==num)
	{
		if(flash.flash_RW_2num==1000)
		{
			W25QXX_Write(RS485_RX_BUF ,HISData2ADDSTART,158);
//			Clear_rs485_buf ();
//			W25QXX_Read(RS485_RX_BUF ,HISData2ADDSTART,158);	
//			for(i=0;i<158;i++)
//			M_printf(" %02x ",RS485_RX_BUF[i]);
//			M_printf("\r\n");
		}
		else 
			{
			W25QXX_Write(RS485_RX_BUF ,HISData2ADDSTART+flash.flash_RD_2num*DataBLOCK ,158);
//			Clear_rs485_buf ();
//			W25QXX_Read(RS485_RX_BUF ,HISData2ADDSTART+flash.flash_RD_2num*DataBLOCK,158);	
//			for(i=0;i<158;i++)
//			M_printf(" %02x ",RS485_RX_BUF[i]);
//			M_printf("\r\n");
		   }
		if(flash.flash_RW_2num==1000)
			flash.flash_RW_2num =0;
		else flash.flash_RW_2num++;
	}
	data_send_check();
	W25QXX_Write((u8 *)&flash,0,sizeof(uatflash));
}
void Clear_databuff(void)
{
	memset(Data_buff,0,160);
}

void Sys_clk_driver(u8 state)
{
	if(cfgsys->LogLev>=DETAILLOG)
	{
		M_printf("[Sys_clk]Sys_clk_DIRVER:");
	}	
	sys->systimerCount_s= 0;  //
	if(state)
	{
		if(sys->timerDriverEn== 0)
		{
			if(cfgsys->LogLev>=DETAILLOG)
			{		
				M_printf("ENABLE NOW!!!\r\n");	
			}	
			// user code
//			if(SysTick_Init() == 1)
//			{
				//M_printf ("SysTick  OK\r\n");
			SysTick_Init();
			Time_Init();
				sys->timerDriverEn = 1;
//		}
//			else
//			{
//				if(cfgsys->LogLev>=DETAILLOG)
//				{			
//					M_printf ("SysTick  ERR\r\n");
//				}
//			}
		}
		else
		{
			if(cfgsys->LogLev>=DETAILLOG)
			{		
				M_printf("ENABLE already\r\n");
			}	
		}
	}		
}
//系统时钟使能
void timer_driver(u8 state)
{
	if(cfgsys->LogLev>=DETAILLOG)
	{
		M_printf("[TIMER]TIMER_DIRVER:");
	}	
	sys->systimerCount_s= 0;  //
	if(state)
	{
		if(sys->timerDriverEn== 0)
		{
			if(cfgsys->LogLev>=DETAILLOG)
			{		
				M_printf("ENABLE NOW!!!\r\n");	
			}	
			// user code
			if(Time_Init()==1 )
			{
				//M_printf ("SysTick  OK\r\n");
				sys->timerDriverEn = 1;
			}
			else
			{
				if(cfgsys->LogLev>=DETAILLOG)
				{			
					M_printf ("SysTick  ERR\r\n");
				}
			}
		}
		else
		{
			if(cfgsys->LogLev>=DETAILLOG)
			{		
				M_printf("ENABLE already\r\n");
			}	
		}
	}
}
void gprs_driver(u8 state)
{
	if(cfgsys->LogLev>=DETAILLOG)
	{
		M_printf("[GPRS]GPRS_DIRVER:");	
	}	
	if(state)
	{
		if(sys->gprsDriverEn== 0)
		{
			sys->gprsDriverEn = 1;
			if(cfgsys->LogLev>=DETAILLOG)
			{
				M_printf("ENABLE NOW!!!\r\n");	
			}	
			GPRS_init();
		}	
		else
		{	
			if(cfgsys->LogLev>=DETAILLOG)
			{		
				M_printf("ENABLE already\r\n");
			}	
		}	
	}
	
}


void init_driver_mode_run(void)
{
	if(cfgsys->LogLev>=SIMPLELOG)
	{
		M_printf("[MODE_RUN]INIT DRIVER START:\r\n");
	}	
	Sys_clk_driver(ENABLE);	//系统时钟使能	
//	timer_driver(ENABLE);
	gprs_driver(ENABLE);
	if(cfgsys->LogLev>=SIMPLELOG)
	{
		M_printf("[MODE_RUN]mode_run init driver end\r\n");
	}	
}
void		DataSave(void)
{
	
}
void processRunningApp(void)
{		
	   GPRS_APP();	
	  if(sys->netok == 1)   //网络没问题进行数据、心跳的发送
		{		
			HtyDatasend();
			beat_APP();			
		}
	   get_APP();	
}

void beat_APP(void )
{
//	M_printf("beat_APP.....\r\n");
	if(beatflag==1)  //心跳APP	
	{
		data_send_check();
		if(cfgsys->LogLev>=SIMPLELOG)
			{
					M_printf("[beat]sys->htydatasendEN：%d\r\n",sys->htydatasendEN);
			}
		
		if(0==sys->htydatasendEN )
		{
			if(cfgsys->LogLev>=SIMPLELOG)
				{
						M_printf("[beat]sys->gprsbusy ：%d    sys->gprsbeatbusy:%d  \r\n",sys->gprsbusy,sys->gprsbeatbusy);
				}
			
				if(sys->gprsbusy ==0||sys->gprsbeatbusy ==1)
				{
					if(sys->gprsbeatbusy==0)
							sys->datasendcheck=0; 
					sys->gprsbeatbusy=1; 
					sys->gprsbusy =1;
		//			M_printf("sys->datasendcheck:%d",sys->datasendcheck );
					send_data_to_sever ((u8 *)&beat,0,2);
				}
				else
				{					
					beatflag=0;
				sys->gprsbeatbusy =0;
				}
	  }	
		else
		{
			beatflag=0;	
			sys->gprsbeatbusy =0;
		}
	}
			
		
}

//校验状态位
//返回值 0:无有效数据产生，1：又有效数据产生，2：没有接收的数据
u8 State_check()
{
		
		if(0x00==RS485_RX_BUF [4])
			return 0;
		else
			return 1;
}

void get_APP(void)
{
	u8 i=0;

		if (1==Get_State_EN)
		{
			if(cfgsys->LogLev>=SIMPLELOG)
				{
						M_printf("[rs485:1]sys->rs485busy:%d   sys->datacheck:%d.....\r\n",sys->rs485busy,sys->datacheck);
				}
			
			if(sys->rs485busy==0)
			{
			switch (sys->datacheck )
			{
				case 0:		
						if(cfgsys->LogLev>=SIMPLELOG)
							{
									M_printf("[rs485:1-0]Start get  machine  state ......... \r\n");
							}
					
						Clear_rs485_buf();    //清除接收缓存区
						RS485_Send_Data(get_state,sizeof (get_state));
						sys->datatimecount_5msEn=1;
						sys->datatimecount_5ms=0;
						sys->datatimecount_ms =0;
						sys->datacheck =1;
						sys->datacheckcount=0;
					break;
				
				case 1:
					if(sys->datatimecount_5ms>1)//15ms检测一次 
					{
						sys->datatimecount_5ms=0;                  
						sys->datatimecount_ms =0;
						sys->datacheckcount++;
						if(RS485Flag ==1)
						{
							if(cfgsys->LogLev>=DETAILLOG)
							{
									M_printf("[rs485:1-0]Get  machine  state for check......... \r\n");
							}
							RS485Flag=0;
							this_num=State_check();
							Get_State_EN=0;
							Clear_rs485_buf ();
							sys->datacheckcount=0;
							switch(this_num )
							{
								case 0: 
									last_num=this_num;
									sys->datacheck =0;
								break;
								case 1: 						
											if(0==last_num )
											{
												Get_data_EN=1;
												last_num=this_num;
												sys->datacheck =1;
												sys->rs485busy=1;
											}
											else
											{
												last_num=this_num;
												sys->datacheck =0;												
											}
								break;
								default:   break;
							}
							if(cfgsys->LogLev>=SIMPLELOG)
							{
									M_printf("[rs485:1-0]  state  check  over......... \r\n");
							}
						}
						else 
								{
									  if(sys->datacheckcount>6)
											{
												sys->datacheck =0;
												if(cfgsys->LogLev>=SIMPLELOG)
													{
														M_printf("[rs485:1-1]get state fail.......\r\n");
														M_printf("sys->datacheck :%d\r\n",sys->datacheck);
													}
												
												sys->datacheckcount=0;
												sys->datatimecount_5msEn=0;
												sys->datatimecount_5ms=0;
												sys->datatimecount_ms =0;
											
												Get_State_EN=0;
												Clear_rs485_buf ();
													if(sys->datacheck !=0)
													{
														sys->datacheck=0;
													}
										  }								
											else{
												sys->datacheck=1;
												sys->datatimecount_5msEn=1;
												sys->datatimecount_5ms=0;
												sys->datatimecount_ms =0;			
											}												
								}
					}
					break;
					default:   break;
				}
			}
			else Get_State_EN=0;
		}
	
		if(1==Get_data_EN)
		{
			if(cfgsys->LogLev>=SIMPLELOG)
				{
					M_printf("[rs485:2-1]sys->datacheck:%d.....\r\n",sys->datacheck);
				}
			
			switch(sys->datacheck)
			{
				case 1:
						if(cfgsys->LogLev>=SIMPLELOG)
							{
									M_printf("[rs485:2-1]Get  machine  data ......... \r\n");
							}
					
						sys->rs485busy =1;
						Clear_rs485_buf();    //清除接收缓存区
						RS485_Send_Data(get_data,sizeof (get_data));
						sys->datatimecount_5msEn=1;
						sys->datatimecount_5ms=0;
						sys->datatimecount_ms =0;
						sys->datacheck =2;
//						sys->datdsendcletimes=0;
				break;
				case 2:
						if(sys->datatimecount_5ms>0)//5ms检测一次
						{
							sys->datatimecount_5ms=0;
							sys->datatimecount_ms =0;	
							if(1==RS485Flag)
							{
								sys->datdsendcletimes =0;
								if(cfgsys->LogLev>=SIMPLELOG)
									{
											M_printf("[rs485:2-2]\r\n");
										for(i=0;i<25;i++)
										M_printf("% 02X ",RS485_RX_BUF [i]);
										M_printf("\r\n");
									}
								
													
								heartbeat(0);
									if(cfgsys->LogLev>=SIMPLELOG)
									{
											M_printf("[rs485:2-2]sys->netok:%d.....\r\n",sys->netok);
									}
								
									if(sys->sysgprstimegetEN==1)                      //************************************************
									{
										if(sys->gprsbusy==0)
										{
										sdk_cmd_info.second=revl(sys_second);
										
										sdk_cmd_info.ms=revl(sys->gprstimecount_ms);
										
										Random_num1 =Get_random_num ();
										
										sdk_cmd_info.sys_num1=revl(Random_num1);
										
										Random_num2 =Get_random_num ();
										
										sdk_cmd_info.sys_num2=revl(Random_num2);
										}else{
											Second_temp=revl(sys_second);
											
											Ms_temp=revl(sys->gprstimecount_ms);
											
											Random1_temp1=Get_random_num ();										
											Random1_temp1=revl(Random1_temp1);	
											
											Random2_temp2 =Get_random_num ();
											Random2_temp2=revl(Random2_temp2);		//***********************************************************
										}
										
									}
								sys->datacheck =3;
							}
							else{
								if(sys->datdsendcletimes++>30)
									{
										if(cfgsys->LogLev>=SIMPLELOG)
											{
												M_printf("[rs485:2-2]sys->datdsendcletimes=%d ..............\r\n",sys->datdsendcletimes);
												M_printf("[rs485:2-2]RS485 get data fail..............\r\n");
											}
										
										Get_data_EN=0;
										sys->datdsendcletimes=0;
										sys->datatimecount_5msEn=0;
										sys->datatimecount_5ms=0;
										sys->datatimecount_ms =0;
										sys->datacheck =0;	
										sys->rs485busy=0;
									}	
                 else	{
											sys->datacheck=2;
											sys->datatimecount_5msEn=1;
											sys->datatimecount_5ms=0;
											sys->datatimecount_ms =0;
											}								
							}
						}
				break;
						
					case 3:
								data_send_check();
								if(cfgsys->LogLev>=SIMPLELOG)
									{
											M_printf("[rs485:2-3]sys->netok:%d.....\r\n",sys->netok);
									}
								
								if(0==sys->htydatasendEN &&1==sys->netok&&sys->sysgprstimegetEN==1 )
								{
									if(sys->gprsbusy ==0||sys->gprsdatabusy==1)
									{
//												M_printf("Free to send data.....\r\n");
												if(sys->gprsdatabusy ==0)
												{
													sys->datasendcheck =0; 
												}
												sys->gprsbusy =1;
										    sys->gprsdatabusy=1;
												if(cfgsys->LogLev>=SIMPLELOG)
													{
															M_printf("[rs485:2-3]sys->datasendcheck:%d.....\r\n",sys->datasendcheck);
													}
												
												send_data_to_sever(RS485_RX_BUF ,5,25);											
									}
									else{
										if(cfgsys->LogLev>=SIMPLELOG)
											{
													M_printf("[rs485:2-3]busy to send data.....\r\n");
											}
										
//										     sys->datasaveEN=1;
										 sys->datacheck =5;
									}
								}
								else {
//								sys->datasaveEN=1;
								sys->datacheck =5;
								}							
						
				break;
						
				case 4://等待发送结果
					if(cfgsys->LogLev>=SIMPLELOG)
						{
								M_printf("[485:2-4]sys->datasendflag :%d......\r\n",sys->datasendflag );
						}
					
					if(1==sys->datasaveEN)
					{ 
						sys->datasaveEN=0;
						sys->datacheck =5;
					}else if(0==sys->datasendflag )//重试中继续等待
					{
						sys->datacheck =4;
					}
					else if(2==sys->datasendflag)//   发送失败
					{
						sys->datacheck=5; 
						 sys->gprsdatabusy=0;
							sys->gprsbusy =0;
							sys->rs485busy=0;
						sys->datasendflag=0;
					}
					else if(1==sys->datasendflag )
					{
						sys->datasaveEN=0;
						RS485Flag=0;
						if(cfgsys->LogLev>=SIMPLELOG)
							{
									M_printf("[rs485]data send to server ok .......\r\n");
							}
						
						heartbeat(1);
						sys->datacheck =0;
						Get_data_EN=0;
						Clear_rs485_buf();
						sys->gprsdatabusy=0;
						sys->gprsbusy =0;
						sys->rs485busy=0;
						sys->datasendflag=0;
					}
				break;	
				case 5://发送失败进行存储
					if(cfgsys->LogLev>=SIMPLELOG)
						{
								M_printf("[rs485:2-5]data save....\r\n");
						}
					
				data_write(RS485_RX_BUF,RW_num*data_block,25);
				M_printf("[rs485:2-5]Second_temp:%d\r\n",Second_temp);	
				M_printf("[rs485:2-5]sdk_cmd_info.second:%d\r\n",sdk_cmd_info.second);
				M_printf("[rs485:2-5]sys->gprsbusy:%d\r\n",sys->gprsbusy);			
				if(sys->sysgprstimegetEN==1)
				{
					if(Second_temp!=0)
					{
						
						if(cfgsys->LogLev>=SIMPLELOG)
							{
									M_printf("[rs485:2-5]data save(include time2)....\r\n");
							}
							M_printf("[rs485:2-5]Second_temp:%d\r\n",Second_temp);
						Second_temp=revl(Second_temp);
							M_printf("[rs485:2-5]Second_temp:%d\r\n",Second_temp);
						u32_to_u8(&Second_temp,u8_4_buffer);	
							for(i=0;i<4;i++)
							M_printf("% 02X ",u8_4_buffer[i]);
							M_printf("\r\n");
						data_write (u8_4_buffer,RW_num*data_block+25,4);
							
						Ms_temp=revl(Ms_temp);
						u32_to_u8(&Ms_temp,u8_4_buffer);	
						data_write (u8_4_buffer,RW_num*data_block+29,4);
							
						Random1_temp1=revl(Random1_temp1);
						u32_to_u8(&Random1_temp1,u8_4_buffer);	
						data_write (u8_4_buffer,RW_num*data_block+33,4);
							
						Random2_temp2=revl(Random2_temp2);
						u32_to_u8(&Random2_temp2,u8_4_buffer);	
						data_write (u8_4_buffer,RW_num*data_block+37,4);
						
						Second_temp=0;
						Ms_temp=0;
						Random1_temp1=0;
						Random2_temp2=0;
												
					}
					else if(sys->gprsbusy ==0&&sdk_cmd_info.second!=0)
					{
						if(cfgsys->LogLev>=SIMPLELOG)
							{
									M_printf("[rs485:2-5]data save(include time1)....\r\n");
							}
							M_printf("[rs485:2-5]sdk_cmd_info.second:%d\r\n",sdk_cmd_info.second);
						sdk_cmd_info.second=revl(sdk_cmd_info.second);
							M_printf("[rs485:2-5]sdk_cmd_info.second:%d\r\n",sdk_cmd_info.second);
						u32_to_u8(&sdk_cmd_info.second,u8_4_buffer);	
							for(i=0;i<4;i++)
							M_printf("% 02X ",u8_4_buffer[i]);
							M_printf("\r\n");
						data_write (u8_4_buffer,RW_num*data_block+25,4);
						sdk_cmd_info.ms=revl(sdk_cmd_info.ms);
						u32_to_u8(&sdk_cmd_info.ms,u8_4_buffer);	
						data_write (u8_4_buffer,RW_num*data_block+29,4);
						sdk_cmd_info.sys_num1=revl(sdk_cmd_info.sys_num1);
						u32_to_u8(&sdk_cmd_info.sys_num1,u8_4_buffer);	
						data_write (u8_4_buffer,RW_num*data_block+33,4);
						sdk_cmd_info.sys_num2=revl(sdk_cmd_info.sys_num2);
						u32_to_u8(&sdk_cmd_info.sys_num2,u8_4_buffer);	
						data_write (u8_4_buffer,RW_num*data_block+37,4);
						sdk_cmd_info.second=0;
						sdk_cmd_info.ms=0;
						sdk_cmd_info.sys_num1=0;
						sdk_cmd_info.sys_num2=0;
					}
				}else
				{
					if(cfgsys->LogLev>=SIMPLELOG)
						{
								M_printf("[rs485:2-5]data save(no time)....\r\n");
						}
					memset(u8_4_buffer,0,4);
					data_write (u8_4_buffer,RW_num*data_block+25,4);					
					data_write (u8_4_buffer,RW_num*data_block+29,4);	
					data_write (u8_4_buffer,RW_num*data_block+33,4);
					data_write (u8_4_buffer,RW_num*data_block+37,4);
				}					
//				data_read(send_buff ,RD_num*data_block,25);
				if(cfgsys->LogLev>=SIMPLELOG)
				{
					for(i=0;i<25;i++)
					M_printf("% 02X ",data_save[RW_num*data_block+i]);
					M_printf("\r\n");
				}
				
				if(RW_num==Data_MAX_num)
					RW_num=0;
				else RW_num++;
				sys->datacheck =0;
				Get_data_EN=0;	
				RS485Flag=0;
				Clear_rs485_buf ();
				sys->rs485busy=0;
				sys->gprsbusy =0;
				sys->gprsdatabusy=0;
				heartbeat(1);
				if(cfgsys->LogLev>=SIMPLELOG)
					{
							M_printf("[rs485:2-5]data save finished\r\n");
					}
				
				break;
				default:break;
	     }
 
	}
}



void processRunningMode(void)
{	
	if(cfgsys->LogLev>=SIMPLELOG)
	{	
		M_printf("[MODE_RUN]============================START\r\n");	
	}	
	init_driver_mode_run();
	Adc_Init();
	sys->gprsstate =1;	//启动gprs

		while(1)
	{
		processRunningApp();
	}
}
