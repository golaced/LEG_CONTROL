

#include "led_fc.h"
#include "include.h"

void LEDRGB_COLOR(u8 color);
void LED_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(ANO_RCC_LED,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = ANO_Pin_LED1| ANO_Pin_LED2| ANO_Pin_LED3;
	GPIO_Init(ANO_GPIO_LED, &GPIO_InitStructure);


  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOA,GPIOE时钟
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9; //KEY0 KEY1 KEY2对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOE2,3,4
}



/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/


void LEDRGB(u8 sel,u8 on)
{
switch(sel)
{
case RED:
if(!on)
GPIO_ResetBits(GPIOA,GPIO_Pin_4);
else
GPIO_SetBits(GPIOA,GPIO_Pin_4);
break;
case GREEN:
if(!on)
GPIO_ResetBits(GPIOA,GPIO_Pin_4);
else
GPIO_SetBits(GPIOA,GPIO_Pin_4);
break;
case BLUE:
if(!on)
GPIO_ResetBits(GPIOA,GPIO_Pin_4);
else
GPIO_SetBits(GPIOA,GPIO_Pin_4);
break;
case 12:
if(!on)
GPIO_ResetBits(GPIOA,GPIO_Pin_4);
else
GPIO_SetBits(GPIOA,GPIO_Pin_4);
break;
}
}

void LEDRGB_COLOR(u8 color)
{
switch (color)
{
case RED:
LEDRGB(RED,1);
LEDRGB(BLUE,0);
LEDRGB(GREEN,0);
break;
case BLUE:
LEDRGB(RED,0);
LEDRGB(BLUE,1);
LEDRGB(GREEN,0);
break;
case GREEN:
LEDRGB(RED,0);
LEDRGB(BLUE,0);
LEDRGB(GREEN,1);
break;
case WHITE:
LEDRGB(RED,1);
LEDRGB(BLUE,1);
LEDRGB(GREEN,1);
break;
case BLACK:
LEDRGB(RED,0);
LEDRGB(BLUE,0);
LEDRGB(GREEN,0);
break;
case YELLOW:
LEDRGB(RED,1);
LEDRGB(BLUE,0);
LEDRGB(GREEN,1);
break;
}
}
#define IDLE 0
#define CAL_MPU 1
#define CAL_M  2
#include "circle.h"
void LEDRGB_STATE(void)
{
static u8 main_state;
static u8 mpu_state,m_state,idle_state;
static u16 cnt,cnt_idle;
u8 mode_control;
	
if(main_state){main_state=0;
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);

}else{main_state=1;

GPIO_SetBits(GPIOA,GPIO_Pin_4);
	
}

}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

