//2022-09-06
//Created by : bonusoid
//HC-12 Demo

#include"delay.h"
#include"delay.c"
#include"periph_stm8s.h"
#include"periph_stm8s.c"
#include"REG/stm8s_gpio.h"
#include"uart_func.h"
#include"uart_func.c"
//#include"REG/stm8s_itc.h"

#define ACCXP	P2	//Pin for Accelerometer X
#define ACCX	3	//ADC channel for Accelerometer X
#define ACCYP	P3	//Pin for Accelerometer Y
#define ACCY	4	//ADC channel for Accelerometer Y
#define ACCXDDR	PD_DDR  
#define ACCXCR1	PD_CR1
#define ACCXCR2 PD_CR2

#define ACCZP	P4	//Pin for Accelerometer Z
#define ACCZ	2	//ADC channel for Accelerometer Z
#define ACCZDDR	PC_DDR
#define ACCZCR1	PC_CR1
#define ACCZCR2	PC_CR2

#define LED1	P5	//LED in B5, active low
#define LEDDDR	PB_DDR
#define LEDODR	PB_ODR
#define LEDCR1	PB_CR1
#define LEDCR2	PB_CR2

unsigned char cri; //data received
unsigned int acc; //ADC value

void gpio_init();
void loop();

//^^^^^^^^^^ INTERRUPTS ^^^^^^^^^^//
void isr_UART1_RX() __interrupt UART_RX_INTERRUPT_VECTOR //ISR for UART Receiver Mode
{
	cri = uart1_recv_i();	//receive 1 byte data
	if((cri=='o')||(cri=='O')) LEDODR &= ~(1<<LED1); //LED ON (active low)
	else if((cri=='x')||(cri=='X')) LEDODR |= 1<<LED1; //LED OFF (active low)
	else;
}
//__________ INTERRUPTS __________//



//^^^^^^^^^^ INIT ^^^^^^^^^^//
int main()
{
  clock_init();
  delay_init();
  gpio_init();
  adc_init();
  uart1_init(UART_RX_INTERRUPT_ENABLED); //UART RX using Interrupt

  cri = 0; //init value of UART RX buffer

  enable_interrupts();

  loop();
  return 0;
}
//__________ INIT __________//

//^^^^^^^^^^ LOOP ^^^^^^^^^^//
void loop()
{
	while(1)
	{
		acc = read_adc(ACCX); //read Accelerometer X value
		UART_sendtext("| X = ");
		UART_sendnum(acc); //send Accelerometer X value

		acc = read_adc(ACCY); //read Accelerometer Y value
		UART_sendtext(" | Y = ");
		UART_sendnum(acc); //send Accelerometer Y value

		acc = read_adc(ACCZ); //read Accelerometer Z value
		UART_sendtext(" | Z = ");
		UART_sendnum(acc); //send Accelerometer Z value
		UART_sendtext(" |");

		//send 'ENTER' for new line
		UART_sendchar('\r'); //return
		UART_sendchar('\n'); //newline
		
		delay_ms(500);
	} 	
}
//__________ LOOP __________//


//^^^^^^^^^^ GPIO INIT ^^^^^^^^^^//
void gpio_init()
{
	//GPIO setting for Accelerometer
  	ACCXDDR |= (INPUT<<ACCX) | (INPUT<<ACCY);
	ACCXCR1 |= (floating<<ACCX) | (floating<<ACCY);
	ACCXCR2 |= (exti_disabled<<ACCX) | (exti_disabled<<ACCY);

	ACCZDDR |= (INPUT<<ACCZ);
	ACCZCR1 |= (floating<<ACCZ);
	ACCZCR2 |= (exti_disabled<<ACCZ);

	//GPIO setting for LED
	LEDDDR |= (OUTPUT<<LED1) | (OUTPUT<<LED1);
	LEDCR1 |= (pushpull<<LED1) | (pushpull<<LED1);
	LEDCR2 |= (speed_2MHz<<LED1) | (speed_2MHz<<LED1);
}
//__________ GPIO INIT __________//
