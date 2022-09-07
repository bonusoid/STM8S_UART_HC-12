//writer : bonus adityas (bonusonic@gmail.com)
//29 january 2022


#ifndef __UART_FUNC_H
#define __UART_FUNC_H

void UART_sendchar(unsigned char usend);  //send 1 character via UART
void UART_sendtext(unsigned char *usend); //send text via UART
void UART_sendnum(unsigned int unum);	  //send integer via UART
unsigned char UART_recvchar();		  //receive 1 character via UART (for Polling mode)

#endif
