//writer : bonus adityas (bonusonic@gmail.com)
//29 january 2022

#include"uart_func.h"
#include"periph_stm8s.h"

//^^^^^^^^^^ UART FUNCTION ^^^^^^^^^^//
void UART_sendchar(unsigned char usend) //send 1 character via UART
{
	uart1_send(usend);
}

void UART_sendtext(unsigned char *usend) //send text via UART
{
	unsigned int stridx = 0;

	while(usend[stridx] != 0) //scan characters in string
	{
		uart1_send(usend[stridx]); //print each character
		stridx++;
	}
}

void UART_sendnum(unsigned int unum) //send integer via UART
{
	unsigned char ibuff[6]; //MAX : 5 DIGIT : 65535

	unsigned char ndigit=0,nd;
	unsigned int numb; //must unsigned, so max. number can be 65535
			   //if set to signed, max. number only 55536

	numb = unum;
	while(numb!=0)
	  {
	  	ndigit++;
		numb /= 10; //count decimal digit	
	  }
	for(nd=0;nd<ndigit;nd++)
	  {
		numb = unum%10;
		unum = unum/10;
		ibuff[ndigit-(nd+1)] = numb + '0'; //start from last_index-1
	  }
	ibuff[ndigit] = '\0'; //last character is null

	UART_sendtext(ibuff);
}

unsigned char UART_recvchar() //receive 1 character via UART (for Polling mode)
{
	unsigned char urecv;

	urecv = uart1_recv();
	
	return urecv;
}
//__________ UART FUNCTION __________//
