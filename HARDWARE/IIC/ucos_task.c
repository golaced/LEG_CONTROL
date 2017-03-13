#include "include.h" 
#include "iic_soft.h"
#include "hml_sample.h"
#include "ms5611.h"
#include "ms5611_2.h"
#include "hml5833l.h"
#include "alt_kf.h"
#include "flash.h"
#include "led_fc.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "ucos_task.h"
#include "pwm_in.h"
#include "flow.h"
#include "circle.h"
#include "eso.h"
#include "gps.h"
#include "m100.h"
//==============================传感器 任务函数==========================
OS_STK MEMS_TASK_STK[MEMS_STK_SIZE];
void mems_task(void *pdata)
{		static u8 cnt,cnt1;						 
 	while(1)
	{
  get_force();
		
		
		
	delay_ms(10);
	}
}		
//--------------------

OS_STK INNER_TASK_STK[INNER_STK_SIZE];
float inner_loop_time;
int Rc_Pwm_off[8];
void inner_task(void *pdata)
{NVIC_InitTypeDef NVIC_InitStructure;
 u8 i;
 static u8 dj_fly_line=0;
 static u8 init;	
 static int flag_scan=1;
 	while(1)
	{
	inner_loop_time = Get_Cycle_T(GET_T_INNER); 						//获取内环准确的执行周期
  LEG_POWER(leg.leg_power);
		
		
		
		
		
		
	if(leg.control_mode)
	cal_sita_from_pos(&leg,leg.pos_tar[2].x,leg.pos_tar[2].y,leg.pos_tar[2].z);	
	else
	cal_sita_from_pos(&leg,leg.init_end_pos.x,leg.init_end_pos.y,leg.init_end_pos.z);
	cal_pwm_from_sita(&leg);	
	SetPwm(Rc_Pwm_Out_mine,Rc_Pwm_off,pwmin.min,pwmin.max);
	delay_ms(5);
	}
}		



//========================外环  任务函数============================
OS_STK OUTER_TASK_STK[OUTER_STK_SIZE];
float outer_loop_time;
float Pitch_R,Roll_R,Yaw_R;
void outer_task(void *pdata)
{	static u8 cnt,cnt1,cnt2;						  
 	while(1)
	{	
	outer_loop_time = Get_Cycle_T(GET_T_OUTTER);								//获取外环准确的执行周期
	
	delay_ms(5);
	}
}		
//=========================射频 任务函数======================
OS_STK NRF_TASK_STK[NRF_STK_SIZE];
u8 en_shoot=0;
void nrf_task(void *pdata)
{							 
	static u8 cnt,cnt2;
 	while(1)
	{
  	
	}
}		

//气压计 任务函数
OS_STK BARO_TASK_STK[BARO_STK_SIZE];
void baro_task(void *pdata)
{							  
 	while(1)
	{ static u8 cnt;
	 
	}
}	

//=======================超声波 任务函数==================
OS_STK SONAR_TASK_STK[SONAR_STK_SIZE];
void sonar_task(void *pdata)
{							  
 	while(1)
	{
		
		delay_ms(100);
	}
	  
	
}	

#include "AttitudeEKF.h"
//=======================光流 任务函数==================
OS_STK FLOW_TASK_STK[FLOW_STK_SIZE];

void flow_task(void *pdata)
{	
 static float hc_speed_i[2],h_speed[2],wz_speed_0[2],tempacc_lpf[2];				
 float c_nb_dtb[3][3],a_br[3],tmp[3],acc[3];	
 	while(1)
	{
	
	delay_ms(5);
	}
}	
	
//=======================M100 任务函数==================
OS_STK M100_TASK_STK[M100_STK_SIZE];
void m100_task(void *pdata)
{		
	static u8 cnt_m100;
		static u8 en_vrcr,flag1;
	static int m100_Rc_gr;
	
		while(1)
	{

	
	delay_ms(5);
	}
}

//=======================串口 任务函数===========================
OS_STK  UART_TASK_STK[UART_STK_SIZE];
u8 UART_UP_LOAD_SEL=4;//<------------------------------UART UPLOAD DATA SEL
u8 state_v_test=0;
u8 num_need_to_check;
void uart_task(void *pdata)
{	static u8 cnt[4];	
  static u8 sd_sel;	
 	while(1)
	{
				//GPS		
				if(cnt[0]++>1){cnt[0]=0;
						#if EN_DMA_UART3 
					if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET)//等待DMA2_Steam7传输完成
								{ 
							DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//清除DMA2_Steam7传输完成标志
							//data_per_uart3();
						  USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
							MYDMA_Enable(DMA1_Stream3,SEND_BUF_SIZE3);     //开始一次DMA传输！	
								}	
						#else
						#define NUM_OFF_STATE 50	
						#define UART_ODROID_SEL 0
						#if		UART_ODROID_SEL
						static u8 odroid_up_sel=0;
							if(odroid_up_sel){odroid_up_sel=0;
								if(state_v_test!=0)
								UsartSend_GPS(state_v_test+NUM_OFF_STATE);
								else
								UsartSend_GPS(state_v+NUM_OFF_STATE);
							}
							else
								{odroid_up_sel=1;
								UsartSend_GPS(num_need_to_check);}
						#else		
								CPU_LINK_TASK();//to lds.,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,xsodroid
						#endif		
						#endif
							}		
				
				
				//UPLOAD			
				
					if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//等待DMA2_Steam7传输完成
							{ 	
							DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//清除DMA2_Steam7传输完成标志
							clear_leg_uart();		
							GOL_LINK_TASK_DMA();	
							USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
							MYDMA_Enable(DMA2_Stream7,leg_uart_cnt+2);     //开始一次DMA传输！	  
							}	
						
	
											
		delay_ms(5);  
	}
}	


//------------------------------软件定时器--------------------------------//
OS_TMR   * tmr1;			//软件定时器1
OS_TMR   * tmr2;			//软件定时器2
OS_TMR   * tmr3;			//软件定时器3

//软件定时器1的回调函数	
//每100ms执行一次,用于显示CPU使用率和内存使用率		
 u16 cpuusage=0;
void tmr1_callback(OS_TMR *ptmr,void *p_arg) 
{
	static u8 tcnt=0;	    

	if(tcnt==5)
	{
 		
		cpuusage=0;
		tcnt=0; 
	}
	cpuusage+=OSCPUUsage;
	tcnt++;				    
}
 #include "circle.h"
//软件定时器2的回调函数				  50ms	 
u8 dji_rst_protect;
u8 dji_rst;
u8 DJI_CONNECT;
u16 dji_miss_cnt;
u8 dji_rc_miss;
u16 dji_rc_miss_cnt;
void tmr2_callback(OS_TMR *ptmr,void *p_arg) 
{	
static u16 cnt_1,cnt_2;	
static u8 cnt;
//	LEDRGB_STATE();
	
}
//软件定时器3的回调函数				  	   
void tmr3_callback(OS_TMR *ptmr,void *p_arg) 
{	
 
} 


//