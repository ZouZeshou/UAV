#ifndef __KEYBOARD_H
#define __KEYBOARD_H
#include "stdint.h"

#define KEY_V		0x4000
#define KEY_C		0x2000
#define KEY_X		0x1000
#define KEY_Z		0x0800
#define KEY_G		0x0400
#define KEY_F		0x0200
#define KEY_R		0x0100
#define KEY_E		0x0080
#define KEY_Q		0x0040
#define KEY_CTRL	0x0020
#define KEY_SHIFT	0x0010
#define KEY_D		0x0008
#define KEY_A		0x0004
#define KEY_S		0x0002
#define KEY_W		0x0001

#define KEYUP		0X01
#define KEYDOWN  	0X02
#define KEYMID 		0X03

#define Q_X 0
#define Q_Y 5000
#define E_X 0
#define E_Y 2000
#define SHIFT_X 5000
#define SHIFT_Y 3000
#define MOVE_X 6000
#define MOVE_Y 8000

#define MOUSE_YAW_CONST -0.01f
#define MOUSE_PITCH_CONST -0.01f

typedef struct
{
	int fric_start;
	int stir_start;
	int stir_start_onebyone;
	int use_vision;
	int laser_on;
	int BGR;
	
}KeyMouse;

extern KeyMouse KeyMousedata;

void DealKeyMousedata(void);

#endif
