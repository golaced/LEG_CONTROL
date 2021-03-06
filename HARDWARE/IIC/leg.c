#include "include.h" 
LEG_STRUCT leg;
//% leg ending positon calculate'
//%         _                side view            ----  ------>x  back view
//%         |  l1                               3  ||   |
//%      __ O                o---> y               OO   |
//%     1  //                |                     ||   \/ Z 
//%       //     l2          |                     ||
//%      O                 Z \/                    OO
//%    /  \\                                       ||
//%   / 2  \\    l3                                ||
//%          O                                     OO
void READ_LEG_ID(LEG_STRUCT *in)
{
u8 temp=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8)<<1|GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9);
in->id=temp+1;
}

void leg_init( LEG_STRUCT *in)
{
READ_LEG_ID(in);	
in->init_mode=0;	
in->l1=7.5;
in->l2=8;
in->l3=11;	

in->init_end_pos.x=0;	
in->init_end_pos.y=0;	
in->init_end_pos.z=15;	
	
in->limit.x=(in->l1+in->l2+in->l3)*0.98;	
in->limit.y=(in->l1+in->l2+in->l3)*0.98;	
in->limit.z=(in->l1+in->l2+in->l3)*0.98;	
	
in->PWM_OFF[0]=1500;	
in->PWM_OFF[1]=1500;	
in->PWM_OFF[2]=1500;		

in->PWM_MIN[0]=1000;	
in->PWM_MIN[1]=1000;	
in->PWM_MIN[2]=1000;	

in->PWM_MAX[0]=2000;	
in->PWM_MAX[1]=2000;	
in->PWM_MAX[2]=2000;	

in->PWM_PER_DEGREE=180/1000.;		
in->sita_flag[0]=1;
in->sita_flag[1]=1;	
in->sita_flag[2]=1;
}	

u8 pos_range_check(LEG_STRUCT * in,float x,float y,float z)
{
 float r=(in->l1+in->l2+in->l3)*0.95;
 float r_pos=sqrt(x*x+y*y+z*z);
 if(r_pos>r||r_pos<(in->l3-(in->l2-in->l1))*1.05)
	 return 0;
 else
	 return 1;
}


void cal_sita_from_pos(LEG_STRUCT * in,float x_i,float y_i,float z_i)
{           
float x=LIMIT(x_i,-in->limit.x,in->limit.x);	
float y=LIMIT(y_i,-in->limit.y,in->limit.y);
float z=LIMIT(z_i,-in->limit.z,in->limit.z);	
	if(pos_range_check(in,x,y,z))	
	{
	in->err=0;	
	in->sita[2]=atan(x/z)*RtA;
	float l1=cos(in->sita[2]*AtR)*in->l1;
	float l2=cos(in->sita[2]*AtR)*in->l2;
	float l3=cos(in->sita[2]*AtR)*in->l3;
	float temp=z-l1;
	float l4=sqrt(y*y+temp*temp);
	temp=(l2*l2+l3*l3-l4*l4)/(2*l2*l3);
	float sita5=acos(temp)*RtA;
	in->sita[1]=(180-sita5);             
	temp=(l2*l2+l4*l4-l3*l3)/(2*l2*l4);
	in->sita[0]=180-acos(y/l4)*RtA-acos(temp)*RtA*cos(in->sita[2]*AtR);
	//recal_pos 
	in->pos_now[0].x=l1*sin(in->sita[2]*AtR);in->pos_now[0].y=0;in->pos_now[0].z=cos(in->sita[2]*AtR)*l1;
	float h1=sin(in->sita[0]*AtR)*l2;
	float h2=sin((180-in->sita[0]-in->sita[1])*AtR)*l3;
	float d1=cos(in->sita[2]*AtR)*l2;
	float d2=cos(in->sita[2]*AtR)*l3;
	in->pos_now[1].x=(l1+h1)*sin(in->sita[2]*AtR);in->pos_now[1].y=-cos(in->sita[0]*AtR)*d1;in->pos_now[1].z=cos(in->sita[2]*AtR)*(l1+h1);
	in->pos_now[2].x=(l1+h1+h2)*sin(in->sita[2]*AtR);in->pos_now[2].y=-cos(in->sita[0]*AtR)*d1+cos((180-in->sita[0]-in->sita[1])*AtR)*d2;in->pos_now[2].z=cos(in->sita[2]*AtR)*(l1+h1+h2);	
	}
	else
	in->err=1;
}	


void cal_pos_from_sita(LEG_STRUCT * in,float sita1,float sita2,float sita3)
{                                                                                                                                                                                                            

float l1=cos(sita3*AtR)*in->l1;
float l2=cos(sita3*AtR)*in->l2;
float l3=cos(sita3*AtR)*in->l3;
//recal_pos 
in->pos_now[0].x=l1*sin(sita3*AtR);in->pos_now[0].y=0;in->pos_now[0].z=cos(sita3*AtR)*l1;
float h1=sin(sita1*AtR)*l2;
float h2=sin((180-sita1-sita2)*AtR)*l3;
float d1=cos(sita3*AtR)*l2;
float d2=cos(sita3*AtR)*l3;
in->pos_now[1].x=(l1+h1)*sin(sita3*AtR);in->pos_now[1].y=-cos(sita1*AtR)*d1;in->pos_now[1].z=cos(sita3*AtR)*(l1+h1);
in->pos_now[2].x=(l1+h1+h2)*sin(sita3*AtR);in->pos_now[2].y=-cos(sita1*AtR)*d1+cos((180-sita1-sita2)*AtR)*d2;in->pos_now[2].z=cos(sita3*AtR)*(l1+h1+h2);	

}	

void cal_pwm_from_sita(LEG_STRUCT * in)
{ u8 i;
	for(i=0;i<3;i++)
	in->PWM_OUT[i]=LIMIT(in->PWM_OFF[i]+in->sita_flag[i]*in->sita[i]*in->PWM_PER_DEGREE,in->PWM_MIN[i],in->PWM_MAX[i]);
	
}	