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
	
	if(RC_Ctl.key.v & KEY_CTRL)
	{
		KeyMousedata.motor_start = 0;
	}
	else if(RC_Ctl.key.v & KEY_SHIFT)
	{
		KeyMousedata.motor_start = 1;
	}
}