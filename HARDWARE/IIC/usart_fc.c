
#include "include.h"
#include "usart_fc.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"






void Usart1_Init(u32 br_num)//-------UPload_board1
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART1, ENABLE); 


}
float uart_time;
 void Data_IMU(u8 *data_buf,u8 num)
{ static u8 cnt;
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
  if(*(data_buf+2)==leg.id)//FLOW_MINE_frame
  {
	 uart_time = Get_Cycle_T(GET_T_OUTTER);	
	 if(cnt++>100){cnt=0;
	 LEDRGB_STATE();}
	
	  leg.pos_tar[0].x=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/10;
		leg.pos_tar[0].y=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/10;
	  leg.pos_tar[0].z=(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/10;
		leg.pos_tar[1].x=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/10;
		leg.pos_tar[1].y=(float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/10;
		leg.pos_tar[1].z=(float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/10;
		leg.pos_tar[2].x=(float)((int16_t)(*(data_buf+16)<<8)|*(data_buf+17))/10;
		leg.pos_tar[2].y=(float)((int16_t)(*(data_buf+18)<<8)|*(data_buf+19)/10);
		leg.pos_tar[2].z=(float)((int16_t)(*(data_buf+20)<<8)|*(data_buf+21))/10;
		leg.sita_tar[0]		 =(float)((int16_t)(*(data_buf+22)<<8)|*(data_buf+23))/10;
		leg.sita_tar[1]		 =(float)((int16_t)(*(data_buf+24)<<8)|*(data_buf+25))/10;
		leg.sita_tar[2]		 =(float)((int16_t)(*(data_buf+26)<<8)|*(data_buf+27))/10;
	  leg.control_mode		 							 =*(data_buf+28);
	 
	 
	 
	}		
}

u8 TxBuffer[256];
u8 TxCounter=0;
u8 count=0; 

u8 Rx_Buf[256];	//串口接收缓存
u8 RxBuffer[50];
u8 RxState = 0;
u8 RxBufferNum = 0;
u8 RxBufferCnt = 0;
u8 RxLen = 0;
u8 com_data ;
static u8 _data_len = 0,_data_cnt = 0;
void USART1_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	
	if(USART1->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART1->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//清除中断标志
   
		com_data = USART1->DR;
		if(RxState==0&&com_data==0xAA)
		{
			RxState=1;
			RxBuffer[0]=com_data;
		}
		else if(RxState==1&&com_data==0xAF)
		{
			RxState=2;
			RxBuffer[1]=com_data;
		}
		else if(RxState==2&&com_data>0&&com_data<0XF1)
		{
			RxState=3;
			RxBuffer[2]=com_data;
		}
		else if(RxState==3&&com_data<50)
		{
			RxState = 4;
			RxBuffer[3]=com_data;
			_data_len = com_data;
			_data_cnt = 0;
		}
		else if(RxState==4&&_data_len>0)
		{
			_data_len--;
			RxBuffer[4+_data_cnt++]=com_data;
			if(_data_len==0)
				RxState = 5;
		}
		else if(RxState==5)
		{
			RxState = 0;
			RxBuffer[4+_data_cnt]=com_data;
			Data_IMU(RxBuffer,_data_cnt+5);
		}
		else
			RxState = 0;
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(USART1,USART_IT_TXE ) )
	{
				
		USART1->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}

   OSIntExit(); 

}

void Usart3_Init(u32 br_num)//-------GPS_board
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 

	
   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART3, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART3, ENABLE); 
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}


}


u8 Rx_Buf3[256];	//串口接收缓存
u8 RxBuffer3[50];
u8 RxState3 = 0;
u8 RxBufferNum3 = 0;
u8 RxBufferCnt3 = 0;
u8 RxLen3 = 0;
static u8 _data_len3 = 0,_data_cnt3 = 0;
void USART3_IRQHandler(void)
{  OSIntEnter();  
	u8 com_data;
	
	if(USART3->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART3->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART3,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);//清除中断标志

		com_data = USART3->DR;
		
				if(RxState3==0&&com_data==0xAA)
		{
			RxState3=1;
			RxBuffer3[0]=com_data;
		}
		else if(RxState3==1&&com_data==0xAF)
		{
			RxState3=2;
			RxBuffer3[1]=com_data;
		}
		else if(RxState3==2&&com_data>0&&com_data<0XF1)
		{
			RxState3=3;
			RxBuffer3[2]=com_data;
		}
		else if(RxState3==3&&com_data<50)
		{
			RxState3 = 4;
			RxBuffer3[3]=com_data;
			_data_len3 = com_data;
			_data_cnt3 = 0;
		}
		else if(RxState3==4&&_data_len3>0)
		{
			_data_len3--;
			RxBuffer3[4+_data_cnt3++]=com_data;
			if(_data_len3==0)
				RxState3 = 5;
		}
		else if(RxState3==5)
		{
			RxState3 = 0;
			RxBuffer3[4+_data_cnt3]=com_data;
			
		}
		else
			RxState3 = 0;
	
	}
	
 OSIntExit();        
}

u8 SendBuff1[SEND_BUF_SIZE1];
u8 SendBuff3[SEND_BUF_SIZE3];

u16 leg_uart_cnt;
void data_per_uart1(u8 sel)
{
	u8 i;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;
	vs16 _temp;
  vs32 _temp32;

switch(sel){
	case SEND_LEG:
	cnt_reg=leg_uart_cnt;
  SendBuff1[leg_uart_cnt++]=0xAA;
	SendBuff1[leg_uart_cnt++]=0xAF;
	SendBuff1[leg_uart_cnt++]=leg.id;//功能字
	SendBuff1[leg_uart_cnt++]=0;//数据量
  
	_temp = leg.pos_now[0].x*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	_temp = leg.pos_now[0].y*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	_temp = leg.pos_now[0].z*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	
	_temp = leg.pos_now[1].x*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	_temp = leg.pos_now[1].y*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	_temp = leg.pos_now[1].z*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	
	_temp = leg.pos_now[2].x*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	_temp = leg.pos_now[2].y*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	_temp = leg.pos_now[2].z*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);

  
	_temp = leg.sita[0]*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	_temp = leg.sita[1]*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	_temp = leg.sita[2]*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
  
	_temp = leg.leg_ground;
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
  _temp = leg.err;
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	
  _temp = leg.leg_end_force[0]*1000;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	



	SendBuff1[cnt_reg+3] = leg_uart_cnt-cnt_reg-4;
	for( i=cnt_reg;i< leg_uart_cnt;i++)
		sum += SendBuff1[i];
	SendBuff1[leg_uart_cnt++] = sum;
	break;//------------

	default:break;
}
}

void GOL_LINK_TASK_DMA(void)//5ms
{
static u8 cnt[10];
static u8 flag[10];
	
//传感器值
data_per_uart1(SEND_LEG);

end_gol_link1:;
}

void clear_leg_uart(void)
{u16 i;
leg_uart_cnt=0;
for(i=0;i<SEND_BUF_SIZE1;i++)
SendBuff1[i]=0;

}

