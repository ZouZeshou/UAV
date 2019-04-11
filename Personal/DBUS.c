#include "DBUS.h"
#include "BSP_usart.h"
#include "usart.h"
#include "BSP_can.h"
RC_Ctl_t RC_Ctl = {0};
uint8_t Remotebuffer[25];

/**
 * @brief Receive data from remote device
 * @param None
 * @return None
 * @attention  None
 */
void GetDBUS (void)
{
	if(Remotebuffer[12] < 0x02 && Remotebuffer[13] < 0x02)
	{
		fps.DBUS++;
		RC_Ctl.rc.ch0 = (Remotebuffer[0]| (Remotebuffer[1] << 8)) & 0x07ff; //!< Channel 0
		RC_Ctl.rc.ch1 = ((Remotebuffer[1] >> 3) | (Remotebuffer[2] << 5)) & 0x07ff; //!< Channel 1
		RC_Ctl.rc.ch2 = ((Remotebuffer[2] >> 6) | (Remotebuffer[3] << 2) | (Remotebuffer[4] << 10)) & 0x07ff;//!< Channel 2
		RC_Ctl.rc.ch3 = ((Remotebuffer[4] >> 1) | (Remotebuffer[5] << 7)) & 0x07ff; //!< Channel 3
		RC_Ctl.rc.s1 = ((Remotebuffer[5] >> 4)& 0x000C) >> 2; //!< Switch left
		RC_Ctl.rc.s2 = ((Remotebuffer[5] >> 4)& 0x0003); //!< Switch right9 / 9
		RC_Ctl.mouse.x = Remotebuffer[6] | (Remotebuffer[7] << 8); //!< Mouse X axis
		RC_Ctl.mouse.y = Remotebuffer[8] | (Remotebuffer[9] << 8); //!< Mouse Y axis
		RC_Ctl.mouse.z = Remotebuffer[10] | (Remotebuffer[11] << 8); //!< Mouse Z axis
		RC_Ctl.mouse.press_l = Remotebuffer[12]; //!< Mouse Left Is Press ?
		RC_Ctl.mouse.press_r = Remotebuffer[13]; //!< Mouse Right Is Press ?
		RC_Ctl.key.v = Remotebuffer[14] | (Remotebuffer[15] << 8); //!< KeyBoard value
		RC_Ctl.rc.ch0 = (abs(RC_Ctl.rc.ch0 - 1024) > 10 ? RC_Ctl.rc.ch0 : 1024);
		RC_Ctl.rc.ch1 = (abs(RC_Ctl.rc.ch1 - 1024) > 10 ? RC_Ctl.rc.ch1 : 1024);
		RC_Ctl.rc.ch2 = (abs(RC_Ctl.rc.ch2 - 1024) > 10 ? RC_Ctl.rc.ch2 : 1024);
		RC_Ctl.rc.ch3 = (abs(RC_Ctl.rc.ch3 - 1024) > 10 ? RC_Ctl.rc.ch3 : 1024);
	}		
	__HAL_UART_CLEAR_PEFLAG(&huart1);
}

