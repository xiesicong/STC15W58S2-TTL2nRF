#ifndef __UART_H
#define __UART_H

#include	"STC15.H"

void UartInit();
void SendData(unsigned char ch);
void SendString(char *s);



#endif