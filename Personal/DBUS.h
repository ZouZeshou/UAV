#ifndef __DBUS_H
#define __DBUS_H
#include "stdint.h"


typedef struct
{
struct
{
uint16_t ch0;
uint16_t ch1;
uint16_t ch2;
uint16_t ch3;
uint8_t s1;
uint8_t s2;
}rc;
struct
{
int16_t x;
int16_t y;
int16_t z;
uint8_t press_l;
uint8_t press_r;
}mouse;
struct
{
uint16_t v;
}key;
}RC_Ctl_t;


extern RC_Ctl_t RC_Ctl;
extern uint8_t Remotebuffer[25];

void GetDBUS (void);

#endif
