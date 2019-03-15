#include "Keyboard.h"
#include "DBUS.h"
#include "ramp.h"

KeyMouse KeyMousedata;

/**
 * @brief deal data from key and mouse
 * @param None
 * @return None
 * @attention None
 */
void DealKeyMousedata(void)
{
	if(RC_Ctl.key.v & KEY_Q)
	{
		KeyMousedata.Ax = Q_X;
		KeyMousedata.Ay = Q_Y;
	}
	else if(RC_Ctl.key.v & KEY_E)
	{
		KeyMousedata.Ax = E_X;
		KeyMousedata.Ay = E_Y;
	}
	else if(RC_Ctl.key.v & KEY_W)
	{
		KeyMousedata.Ay = MOVE_Y;
	}
	if(RC_Ctl.key.v & KEY_S)
	{
		KeyMousedata.Ay = -MOVE_Y;
	}
	
	
	if(RC_Ctl.key.v & KEY_A)
	{
		KeyMousedata.Ax = -MOVE_X;
	}
	else if(RC_Ctl.key.v & KEY_D)
	{
		KeyMousedata.Ax = MOVE_X;
	}
	
	
	if((RC_Ctl.key.v & KEY_Q)||(RC_Ctl.key.v & KEY_W)||(RC_Ctl.key.v & KEY_E)||(RC_Ctl.key.v & KEY_S))
	{
		if(RC_Ctl.key.v & KEY_SHIFT)
		{
			KeyMousedata.Vy = KeyMousedata.Ay ;
			ramp_cal(&ramp_y);
		}
		else
		{
			KeyMousedata.Vy = KeyMousedata.Ay * ramp_cal(&ramp_y);
		}	
	}
	else
	{
		ramp_init(&ramp_y);
	}

	if((RC_Ctl.key.v & KEY_A)||(RC_Ctl.key.v & KEY_D))
	{
		if(RC_Ctl.key.v & KEY_SHIFT)
		{
			KeyMousedata.Vx = KeyMousedata.Ax ;
			ramp_cal(&ramp_x);
		}
		else
		{
			KeyMousedata.Vx = KeyMousedata.Ax * ramp_cal(&ramp_x);
		}
	}
	else
	{
		ramp_init(&ramp_x);
	}
}