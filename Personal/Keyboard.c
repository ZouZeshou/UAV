#include "Keyboard.h"
#include "DBUS.h"
#include "ramp.h"
#include "camera.h"
KeyMouse KeyMousedata = {0};

/**
 * @brief deal data from key and mouse
 * @param None
 * @return None
 * @attention None
 */
void DealKeyMousedata(void)
{
	static int key_Shift_up,key_Ctrl_up;
	
	if(RC_Ctl.key.v & KEY_Z)
	{
		KeyMousedata.fric_start = 1;
	}
	else if(RC_Ctl.key.v & KEY_X)
	{
		KeyMousedata.fric_start = 0;
		KeyMousedata.stir_start = 0;
	}
	
	if((RC_Ctl.key.v & KEY_C) && KeyMousedata.fric_start)
	{
		KeyMousedata.stir_start = 1;
	}
	else if(RC_Ctl.key.v & KEY_V)
	{
		KeyMousedata.stir_start = 0;
	}
	
	
	if(RC_Ctl.key.v & KEY_R)
	{
		KeyMousedata.sentrymode = 0;
	}
	else if(RC_Ctl.key.v & KEY_Q)
	{
		KeyMousedata.sentrymode = 1;
	}
	else if(RC_Ctl.key.v & KEY_W)
	{
		KeyMousedata.sentrymode = 2;
	}
	else if(RC_Ctl.key.v & KEY_E)
	{
		KeyMousedata.sentrymode = 3;
	}


	if(RC_Ctl.key.v & KEY_A)
	{
		KeyMousedata.Base_or_robot = 0;
	}
	else if(RC_Ctl.key.v & KEY_S)
	{
		KeyMousedata.Base_or_robot = 1;
	}
	else if(RC_Ctl.key.v & KEY_D)
	{
		KeyMousedata.Base_or_robot = 2;
	}

	if((RC_Ctl.key.v & KEY_SHIFT) && key_Shift_up)
	{
		center_y += 20;
		key_Shift_up = 0;
		
	}
	else if((RC_Ctl.key.v & KEY_SHIFT)==0)
	{
		key_Shift_up = 1;
	}
	
	if((RC_Ctl.key.v & KEY_CTRL) && key_Ctrl_up)
	{
		center_y -= 20;
		key_Ctrl_up = 0;
	}
	else if((RC_Ctl.key.v & KEY_CTRL)==0)
	{
		key_Ctrl_up = 1;
	}

	
}