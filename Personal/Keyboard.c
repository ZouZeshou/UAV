#include "Keyboard.h"
#include "DBUS.h"
#include "ramp.h"

KeyMouse KeyMousedata = {0};

/**
 * @brief deal data from key and mouse
 * @param None
 * @return None
 * @attention None
 */
void DealKeyMousedata(void)
{
	static int key_E_up,key_D_up;
	
	if(RC_Ctl.key.v & KEY_A)
	{
		KeyMousedata.fric_start = 1;
	}
	else if(RC_Ctl.key.v & KEY_Z)
	{
		KeyMousedata.fric_start = 0;
		KeyMousedata.stir_start = 0;
	}
	
	if((RC_Ctl.key.v & KEY_F) && KeyMousedata.fric_start)
	{
		KeyMousedata.stir_start = 1;
	}
	else if(RC_Ctl.key.v & KEY_V)
	{
		KeyMousedata.stir_start = 0;
	}
	
	if((RC_Ctl.key.v & KEY_E)&&key_E_up)
	{
		KeyMousedata.pitchup = 1;
		key_E_up = 0;
	}
	else if((RC_Ctl.key.v & KEY_E)==0)
	{
		key_E_up = 1;
		KeyMousedata.pitchup = 0;
	}
	if((RC_Ctl.key.v & KEY_D)&&key_D_up)
	{
		KeyMousedata.pitchdown = -1;
		key_D_up = 0;
	}
	else if((RC_Ctl.key.v & KEY_D)==0)
	{
		key_D_up = 1;
		KeyMousedata.pitchdown = 0;
	}
	
	if(RC_Ctl.key.v & KEY_Q)
	{
		KeyMousedata.sentrymode = 0;
	}
	else if(RC_Ctl.key.v & KEY_W)
	{
		KeyMousedata.sentrymode = 1;
	}
	else if(RC_Ctl.key.v & KEY_S)
	{
		KeyMousedata.sentrymode = 2;
	}
	else if(RC_Ctl.key.v & KEY_X)
	{
		KeyMousedata.sentrymode = 3;
	}


	
}