//writer : bonus adityas (bonusonic@gmail.com)
//29 january 2022


#ifndef __UART_FUNC_H
#define __UART_FUNC_H

void UART_sendchar(unsigned char usend);
void UART_sendtext(unsigned char *usend);
void UART_sendnum(unsigned int unum);
unsigned char UART_recvchar();

#endif
