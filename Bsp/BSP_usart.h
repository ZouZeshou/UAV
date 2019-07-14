#ifndef __BSPUSART__H
#define __BSPUSART__H
#include "stdint.h"

void USART7_Enable(void);
void USART3_IDLE_IRQ(void);
void USART3_Enable(void);
void USART2_Enable(void);
void USART1_IDLE_IRQ(void);
void USART1_Enable(void);
void USART6_Enable(void);
void Plot_in_UpperMonitor (void);
int send_by_register(uint8_t* data);
#endif
