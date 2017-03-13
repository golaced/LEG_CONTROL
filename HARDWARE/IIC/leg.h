
#include "include.h"

typedef struct 
{
	float x;
	float y;
	float z;
	
}POS;

typedef struct 
{ u8 id;
	u8 control_mode;
	u8 leg_switch_flag;
	POS pos_now[3];
	POS pos_tar[3];
	POS init_end_pos;
	POS limit;
	float sita[3];
	float sita_tar[3];
	float dsita[3];
	float dsita_tar[3];
	float init_sita[3];
	int sita_flag[3];
	u8 init_mode; //0->openloop 1->closeloop
	float l1;
	float l2;
	float l3;
	float leg_end_force[4];
	float leg_meme_angle[4];
	u8 leg_ground;
	u8 leg_power;
	u8 err;
	
	u16 PWM_OFF[3];
	u16 PWM_INIT[3];
	u16 PWM_MIN[3],PWM_MAX[3];
	float PWM_OUT[3];
	float PWM_PER_DEGREE;
	
	
}LEG_STRUCT;

extern LEG_STRUCT leg;

void cal_sita_from_pos( LEG_STRUCT *in,float x_i,float y_i,float z_i);
void cal_pos_from_sita(LEG_STRUCT * in,float sita1,float sita2,float sita3);
void leg_init( LEG_STRUCT *in);
void cal_pwm_from_sita(LEG_STRUCT * in);
u8 pos_range_check(LEG_STRUCT * in,float x,float y,float z);
