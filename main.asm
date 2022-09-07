;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.5.0 #9253 (Apr  3 2018) (Linux)
; This file was generated Wed Sep  7 19:46:05 2022
;--------------------------------------------------------
	.module main
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _main
	.globl _isr_UART1_RX
	.globl _acc
	.globl _cri
	.globl _readreg
	.globl _delay_init
	.globl _delay_us
	.globl _delay_ms
	.globl _delay_timer
	.globl _clock_init
	.globl _i2c_init
	.globl _i2c_set_start
	.globl _i2c_set_address
	.globl _i2c_set_stop
	.globl _i2c_clear_ack
	.globl _i2c_set_ack
	.globl _i2c_ack_pos_current
	.globl _i2c_ack_pos_next
	.globl _i2c_poll_SB
	.globl _i2c_poll_ADDR
	.globl _i2c_poll_BTF
	.globl _i2c_poll_TXE
	.globl _i2c_poll_RXNE
	.globl _i2c_clear_bits
	.globl _i2c_clear_ADDR
	.globl _i2c_enable_interrupts
	.globl _i2c_disable_interrupts
	.globl _adc_init
	.globl _read_adc
	.globl _uart1_init
	.globl _uart1_send
	.globl _uart1_recv
	.globl _uart1_recv_i
	.globl _pwm1_init
	.globl _pwm2_init
	.globl _pwm1ch1_enable
	.globl _pwm1ch1_disable
	.globl _pwm2ch1_enable
	.globl _pwm2ch1_disable
	.globl _pwm1_update
	.globl _pwm2_update
	.globl _UART_sendchar
	.globl _UART_sendtext
	.globl _UART_sendnum
	.globl _UART_recvchar
	.globl _loop
	.globl _gpio_init
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area DATA
_readreg::
	.ds 1
_cri::
	.ds 1
_acc::
	.ds 2
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area INITIALIZED
;--------------------------------------------------------
; Stack segment in internal ram 
;--------------------------------------------------------
	.area	SSEG
__start__stack:
	.ds	1

;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area DABS (ABS)
;--------------------------------------------------------
; interrupt vector 
;--------------------------------------------------------
	.area HOME
__interrupt_vect:
	int s_GSINIT ;reset
	int 0x0000 ;trap
	int 0x0000 ;int0
	int 0x0000 ;int1
	int 0x0000 ;int2
	int 0x0000 ;int3
	int 0x0000 ;int4
	int 0x0000 ;int5
	int 0x0000 ;int6
	int 0x0000 ;int7
	int 0x0000 ;int8
	int 0x0000 ;int9
	int 0x0000 ;int10
	int 0x0000 ;int11
	int 0x0000 ;int12
	int 0x0000 ;int13
	int 0x0000 ;int14
	int 0x0000 ;int15
	int 0x0000 ;int16
	int 0x0000 ;int17
	int _isr_UART1_RX ;int18
	int 0x0000 ;int19
	int 0x0000 ;int20
	int 0x0000 ;int21
	int 0x0000 ;int22
	int 0x0000 ;int23
	int 0x0000 ;int24
	int 0x0000 ;int25
	int 0x0000 ;int26
	int 0x0000 ;int27
	int 0x0000 ;int28
	int 0x0000 ;int29
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME
	.area GSINIT
	.area GSFINAL
	.area GSINIT
__sdcc_gs_init_startup:
__sdcc_init_data:
; stm8_genXINIT() start
	ldw x, #l_DATA
	jreq	00002$
00001$:
	clr (s_DATA - 1, x)
	decw x
	jrne	00001$
00002$:
	ldw	x, #l_INITIALIZER
	jreq	00004$
00003$:
	ld	a, (s_INITIALIZER - 1, x)
	ld	(s_INITIALIZED - 1, x), a
	decw	x
	jrne	00003$
00004$:
; stm8_genXINIT() end
	.area GSFINAL
	jp	__sdcc_program_startup
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME
	.area HOME
__sdcc_program_startup:
	jp	_main
;	return from main will return to caller
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CODE
;	delay.c: 7: void delay_init()
;	-----------------------------------------
;	 function delay_init
;	-----------------------------------------
_delay_init:
;	delay.c: 9: TIM4_PSCR = 4; // CLK/16
	mov	0x5347+0, #0x04
	ret
;	delay.c: 12: void delay_us(unsigned long delus)
;	-----------------------------------------
;	 function delay_us
;	-----------------------------------------
_delay_us:
	sub	sp, #6
;	delay.c: 16: for(du=0;du<(delus/10);du++)
	push	#0x0a
	clrw	x
	pushw	x
	push	#0x00
	ldw	x, (0x0f, sp)
	pushw	x
	ldw	x, (0x0f, sp)
	pushw	x
	call	__divulong
	addw	sp, #8
	ldw	(0x05, sp), x
	ldw	(0x03, sp), y
	clrw	x
	ldw	(0x01, sp), x
00103$:
	ldw	x, (0x01, sp)
	clrw	y
	cpw	x, (0x05, sp)
	ld	a, yl
	sbc	a, (0x04, sp)
	ld	a, yh
	sbc	a, (0x03, sp)
	jrnc	00101$
;	delay.c: 18: delay_timer(100);
	push	#0x64
	call	_delay_timer
	pop	a
;	delay.c: 16: for(du=0;du<(delus/10);du++)
	ldw	x, (0x01, sp)
	incw	x
	ldw	(0x01, sp), x
	jra	00103$
00101$:
;	delay.c: 20: delay_timer(delus%10);
	push	#0x0a
	clrw	x
	pushw	x
	push	#0x00
	ldw	x, (0x0f, sp)
	pushw	x
	ldw	x, (0x0f, sp)
	pushw	x
	call	__modulong
	addw	sp, #8
	ld	a, xl
	push	a
	call	_delay_timer
	addw	sp, #7
	ret
;	delay.c: 23: void delay_ms(unsigned long delms)
;	-----------------------------------------
;	 function delay_ms
;	-----------------------------------------
_delay_ms:
	sub	sp, #8
;	delay.c: 27: for(dm=0;dm<(delms*100);dm++)
	ldw	x, (0x0d, sp)
	pushw	x
	ldw	x, (0x0d, sp)
	pushw	x
	push	#0x64
	clrw	x
	pushw	x
	push	#0x00
	call	__mullong
	addw	sp, #8
	ldw	(0x07, sp), x
	ldw	(0x05, sp), y
	clrw	x
	clr	a
	clr	(0x01, sp)
00103$:
	push	a
	cpw	x, (0x08, sp)
	ld	a, (1, sp)
	sbc	a, (0x07, sp)
	ld	a, (0x02, sp)
	sbc	a, (0x06, sp)
	pop	a
	jrnc	00105$
;	delay.c: 29: delay_timer(100);
	push	a
	pushw	x
	push	#0x64
	call	_delay_timer
	pop	a
	popw	x
	pop	a
;	delay.c: 27: for(dm=0;dm<(delms*100);dm++)
	addw	x, #0x0001
	adc	a, #0x00
	push	a
	ld	a, (0x02, sp)
	adc	a, #0x00
	ld	(0x02, sp), a
	pop	a
	jra	00103$
00105$:
	addw	sp, #8
	ret
;	delay.c: 33: void delay_timer(unsigned char deltim)
;	-----------------------------------------
;	 function delay_timer
;	-----------------------------------------
_delay_timer:
;	delay.c: 35: TIM4_CR1 = (1<<TIM4_CR1_CEN);
	mov	0x5340+0, #0x01
;	delay.c: 36: while(TIM4_CNTR<deltim);
00101$:
	ldw	x, #0x5346
	ld	a, (x)
	cp	a, (0x03, sp)
	jrc	00101$
;	delay.c: 37: TIM4_CR1 = (0<<TIM4_CR1_CEN);
	mov	0x5340+0, #0x00
;	delay.c: 38: TIM4_CNTR = 0; //reset timer	
	mov	0x5346+0, #0x00
	ret
;	periph_stm8s.c: 16: void clock_init()
;	-----------------------------------------
;	 function clock_init
;	-----------------------------------------
_clock_init:
;	periph_stm8s.c: 18: CLK_CKDIVR = 0x00; //fMASTER = fCPU = fHSI = 16MHz
	mov	0x50c6+0, #0x00
;	periph_stm8s.c: 19: CLK_ICKR = (1<<CLK_ICKR_HSIEN);	//High Speed Internal RC Oscillator enabled
	mov	0x50c0+0, #0x01
	ret
;	periph_stm8s.c: 24: void i2c_init()
;	-----------------------------------------
;	 function i2c_init
;	-----------------------------------------
_i2c_init:
;	periph_stm8s.c: 26: I2C_CR1 = (0<<I2C_CR1_PE); //Disable I2C before configuration starts
	mov	0x5210+0, #0x00
;	periph_stm8s.c: 27: I2C_FREQR = 16;	//fCLK = 16 MHz
	mov	0x5212+0, #0x10
;	periph_stm8s.c: 28: I2C_CCRH = (0<<I2C_CCRH_FS)|(0<<I2C_CCRH_CCR11)|(0<<I2C_CCRH_CCR10)|(0<<I2C_CCRH_CCR9)|(0<<I2C_CCRH_CCR8); //Standard Mode
	mov	0x521c+0, #0x00
;	periph_stm8s.c: 29: I2C_CCRL = 0x80;  //Clock Speed = 100 kHz
	mov	0x521b+0, #0x80
;	periph_stm8s.c: 31: I2C_OARH = (0<<I2C_OARH_ADDMODE)|(1<<I2C_OARH_ADDCONF); //7-bit address mode, ADDCONF always must be 1
	mov	0x5214+0, #0x40
;	periph_stm8s.c: 32: I2C_TRISER = 17;  //Setup Bus Characteristic
	mov	0x521d+0, #0x11
;	periph_stm8s.c: 37: I2C_CR1 = (1<<I2C_CR1_PE);  //Enable I2C after configuration complete
	mov	0x5210+0, #0x01
	ret
;	periph_stm8s.c: 40: void i2c_set_start()
;	-----------------------------------------
;	 function i2c_set_start
;	-----------------------------------------
_i2c_set_start:
;	periph_stm8s.c: 42: I2C_CR2 |= (1<<I2C_CR2_START);
	bset	0x5211, #0
	ret
;	periph_stm8s.c: 45: void i2c_set_address(unsigned char addr, unsigned char dir)
;	-----------------------------------------
;	 function i2c_set_address
;	-----------------------------------------
_i2c_set_address:
;	periph_stm8s.c: 47: if(dir==I2C_READ) I2C_DR = (addr<<1)|dir;
	ld	a, (0x03, sp)
	ld	xl, a
	sllw	x
	ld	a, (0x04, sp)
	cp	a, #0x01
	jrne	00104$
	ld	a, xl
	or	a, (0x04, sp)
	ldw	x, #0x5216
	ld	(x), a
	jra	00106$
00104$:
;	periph_stm8s.c: 48: else if(dir==I2C_WRITE) I2C_DR = (addr<<1)&dir;
	ld	a, (0x04, sp)
	cp	a, #0xfe
	jrne	00106$
	ld	a, xl
	and	a, (0x04, sp)
	ldw	x, #0x5216
	ld	(x), a
00106$:
	ret
;	periph_stm8s.c: 52: void i2c_set_stop()
;	-----------------------------------------
;	 function i2c_set_stop
;	-----------------------------------------
_i2c_set_stop:
;	periph_stm8s.c: 54: I2C_CR2 |= (1<<I2C_CR2_STOP);
	ldw	x, #0x5211
	ld	a, (x)
	or	a, #0x02
	ld	(x), a
	ret
;	periph_stm8s.c: 57: void i2c_clear_ack()
;	-----------------------------------------
;	 function i2c_clear_ack
;	-----------------------------------------
_i2c_clear_ack:
;	periph_stm8s.c: 59: I2C_CR2 &= ~(1<<I2C_CR2_ACK); //Disable Acknowledge
	ldw	x, #0x5211
	ld	a, (x)
	and	a, #0xfb
	ld	(x), a
	ret
;	periph_stm8s.c: 62: void i2c_set_ack()
;	-----------------------------------------
;	 function i2c_set_ack
;	-----------------------------------------
_i2c_set_ack:
;	periph_stm8s.c: 64: I2C_CR2 |= (1<<I2C_CR2_ACK); //Enable Acknowledge
	ldw	x, #0x5211
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
	ret
;	periph_stm8s.c: 67: void i2c_ack_pos_current()
;	-----------------------------------------
;	 function i2c_ack_pos_current
;	-----------------------------------------
_i2c_ack_pos_current:
;	periph_stm8s.c: 69: I2C_CR2 &= ~(1<<I2C_CR2_POS); //ACK bit controls the (N)ACK of the current byte being received in the shift register
	ldw	x, #0x5211
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
	ret
;	periph_stm8s.c: 72: void i2c_ack_pos_next()
;	-----------------------------------------
;	 function i2c_ack_pos_next
;	-----------------------------------------
_i2c_ack_pos_next:
;	periph_stm8s.c: 74: I2C_CR2 |= (1<<I2C_CR2_POS); //ACK bit controls the (N)ACK of the next byte which will be received in the shift register
	ldw	x, #0x5211
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
	ret
;	periph_stm8s.c: 77: void i2c_poll_SB()
;	-----------------------------------------
;	 function i2c_poll_SB
;	-----------------------------------------
_i2c_poll_SB:
;	periph_stm8s.c: 79: while((I2C_SR1&(1<<I2C_SR1_SB))!=(1<<I2C_SR1_SB)){}
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	and	a, #0x01
	cp	a, #0x01
	jrne	00101$
	ret
;	periph_stm8s.c: 82: void i2c_poll_ADDR()
;	-----------------------------------------
;	 function i2c_poll_ADDR
;	-----------------------------------------
_i2c_poll_ADDR:
;	periph_stm8s.c: 84: while((I2C_SR1&(1<<I2C_SR1_ADDR))!=(1<<I2C_SR1_ADDR)){}
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	and	a, #0x02
	cp	a, #0x02
	jrne	00101$
	ret
;	periph_stm8s.c: 87: void i2c_poll_BTF()
;	-----------------------------------------
;	 function i2c_poll_BTF
;	-----------------------------------------
_i2c_poll_BTF:
;	periph_stm8s.c: 89: while((I2C_SR1&(1<<I2C_SR1_BTF))!=(1<<I2C_SR1_BTF)){}
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	and	a, #0x04
	cp	a, #0x04
	jrne	00101$
	ret
;	periph_stm8s.c: 92: void i2c_poll_TXE()
;	-----------------------------------------
;	 function i2c_poll_TXE
;	-----------------------------------------
_i2c_poll_TXE:
;	periph_stm8s.c: 94: while((I2C_SR1&(1<<I2C_SR1_TXE))!=(1<<I2C_SR1_TXE)){}
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	and	a, #0x80
	cp	a, #0x80
	jrne	00101$
	ret
;	periph_stm8s.c: 97: void i2c_poll_RXNE()
;	-----------------------------------------
;	 function i2c_poll_RXNE
;	-----------------------------------------
_i2c_poll_RXNE:
;	periph_stm8s.c: 99: while((I2C_SR1&(1<<I2C_SR1_RXNE))!=(1<<I2C_SR1_RXNE)){}
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	and	a, #0x40
	cp	a, #0x40
	jrne	00101$
	ret
;	periph_stm8s.c: 102: void i2c_clear_bits()
;	-----------------------------------------
;	 function i2c_clear_bits
;	-----------------------------------------
_i2c_clear_bits:
;	periph_stm8s.c: 104: readreg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
	ld	_readreg+0, a
	ret
;	periph_stm8s.c: 107: void i2c_clear_ADDR()
;	-----------------------------------------
;	 function i2c_clear_ADDR
;	-----------------------------------------
_i2c_clear_ADDR:
;	periph_stm8s.c: 109: readreg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
;	periph_stm8s.c: 110: readreg = I2C_SR3;
	ldw	x, #0x5219
	ld	a, (x)
	ld	_readreg+0, a
	ret
;	periph_stm8s.c: 113: void i2c_enable_interrupts()
;	-----------------------------------------
;	 function i2c_enable_interrupts
;	-----------------------------------------
_i2c_enable_interrupts:
;	periph_stm8s.c: 115: I2C_ITR = 0x07;
	mov	0x521a+0, #0x07
	ret
;	periph_stm8s.c: 117: void i2c_disable_interrupts()
;	-----------------------------------------
;	 function i2c_disable_interrupts
;	-----------------------------------------
_i2c_disable_interrupts:
;	periph_stm8s.c: 119: I2C_ITR = 0x00;
	mov	0x521a+0, #0x00
	ret
;	periph_stm8s.c: 124: void adc_init()
;	-----------------------------------------
;	 function adc_init
;	-----------------------------------------
_adc_init:
;	periph_stm8s.c: 126: ADC1_CR1 = fADC_fMASTER_8<<ADC1_CR1_SPSEL; // ADCCLK = MCLK/8
	mov	0x5401+0, #0x40
;	periph_stm8s.c: 127: ADC1_CR2 = (1<<ADC1_CR2_ALIGN);  // right alignment adc data
	mov	0x5402+0, #0x08
;	periph_stm8s.c: 129: ADC1_CR1 |= (1<<ADC1_CR1_ADON);  // turn on ADC
	bset	0x5401, #0
	ret
;	periph_stm8s.c: 133: unsigned int read_adc(unsigned char adcch)
;	-----------------------------------------
;	 function read_adc
;	-----------------------------------------
_read_adc:
	sub	sp, #4
;	periph_stm8s.c: 137: ADC1_CSR &= 0xF0;  // select
	ldw	x, #0x5400
	ld	a, (x)
	and	a, #0xf0
	ld	(x), a
;	periph_stm8s.c: 138: ADC1_CSR |= adcch; // channel
	ldw	x, #0x5400
	ld	a, (x)
	or	a, (0x07, sp)
	ldw	x, #0x5400
	ld	(x), a
;	periph_stm8s.c: 141: ADC1_CR1 |= (1<<ADC1_CR1_ADON); // start conversion
	bset	0x5401, #0
;	periph_stm8s.c: 142: while(!((ADC1_CSR)&(1<<ADC1_CSR_EOC)));; // conversion is in progress
00101$:
	ldw	x, #0x5400
	ld	a, (x)
	tnz	a
	jrpl	00101$
;	periph_stm8s.c: 143: adcval = (ADC1_DRH<<8) + ADC1_DRL;
	ldw	x, #0x5404
	ld	a, (x)
	clr	(0x03, sp)
	ld	(0x01, sp), a
	clr	(0x02, sp)
	ldw	x, #0x5405
	ld	a, (x)
	clrw	x
	ld	xl, a
	addw	x, (0x01, sp)
;	periph_stm8s.c: 144: ADC1_CSR |= (0<<ADC1_CSR_EOC); // reset EOC
	ldw	y, #0x5400
	ld	a, (y)
	ldw	y, #0x5400
	ld	(y), a
;	periph_stm8s.c: 146: return adcval;
	addw	sp, #4
	ret
;	periph_stm8s.c: 151: void uart1_init(unsigned char rxien) //UART Initialization
;	-----------------------------------------
;	 function uart1_init
;	-----------------------------------------
_uart1_init:
;	periph_stm8s.c: 155: UART1_BRR1 = 0x68;
	mov	0x5232+0, #0x68
;	periph_stm8s.c: 156: UART1_BRR2 = 0x03;
	mov	0x5233+0, #0x03
;	periph_stm8s.c: 158: UART1_CR1 |= (0<<UART1_CR1_M)|(0<<UART1_CR1_PCEN); //8 bit Data; No Parity
	ldw	x, #0x5234
	ld	a, (x)
	ldw	x, #0x5234
	ld	(x), a
;	periph_stm8s.c: 159: UART1_CR3 |= (0<<UART1_CR3_STOP); //Stop Bit = 1
	ldw	x, #0x5236
	ld	a, (x)
	ldw	x, #0x5236
	ld	(x), a
;	periph_stm8s.c: 161: if(rxien==1) 
	ld	a, (0x03, sp)
	cp	a, #0x01
	jrne	00102$
;	periph_stm8s.c: 163: UART1_CR2 |= (1<<UART1_CR2_RIEN); //Enable Interrupt on Receiver Mode
	ldw	x, #0x5235
	ld	a, (x)
	or	a, #0x20
	ld	(x), a
;	periph_stm8s.c: 164: ITC_SPR5 = (level_2<<ITC_SPR5_VECT18); //UART Interrupt Setting
	mov	0x7f74+0, #0x00
00102$:
;	periph_stm8s.c: 167: UART1_CR2 |= (1<<UART1_CR2_TEN); //Enable Transmitter Mode
	ldw	x, #0x5235
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	periph_stm8s.c: 168: UART1_CR2 |= (1<<UART1_CR2_REN); //Enable Receiver Mode
	ldw	x, #0x5235
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
	ret
;	periph_stm8s.c: 171: void uart1_send(unsigned char usend) //UART Transmit a Byte
;	-----------------------------------------
;	 function uart1_send
;	-----------------------------------------
_uart1_send:
;	periph_stm8s.c: 173: UART1_DR = usend; //Write to UART Data Register
	ldw	x, #0x5231
	ld	a, (0x03, sp)
	ld	(x), a
;	periph_stm8s.c: 174: while((UART1_SR&(1<<UART1_SR_TXE))!=(1<<UART1_SR_TXE)); //Wait until Transmission complete
00101$:
	ldw	x, #0x5230
	ld	a, (x)
	and	a, #0x80
	cp	a, #0x80
	jrne	00101$
	ret
;	periph_stm8s.c: 177: unsigned char uart1_recv() //UART Receive a Byte (using Polling)
;	-----------------------------------------
;	 function uart1_recv
;	-----------------------------------------
_uart1_recv:
;	periph_stm8s.c: 180: if((UART1_SR&(1<<UART1_SR_RXNE))==(1<<UART1_SR_RXNE)) //Check if any data in Data Register
	ldw	x, #0x5230
	ld	a, (x)
	and	a, #0x20
	cp	a, #0x20
	jrne	00102$
;	periph_stm8s.c: 182: urecv = UART1_DR; //Read Data Register (RXNE cleared automatically)
	ldw	x, #0x5231
	ld	a, (x)
;	periph_stm8s.c: 184: else urecv=0;
	.byte 0x21
00102$:
	clr	a
00103$:
;	periph_stm8s.c: 185: return urecv;
	ret
;	periph_stm8s.c: 188: unsigned char uart1_recv_i() //UART Receive a Byte (using Interrupt)
;	-----------------------------------------
;	 function uart1_recv_i
;	-----------------------------------------
_uart1_recv_i:
;	periph_stm8s.c: 191: urecv = UART1_DR; //Read Data Register (RXNE cleared automatically)
	ldw	x, #0x5231
	ld	a, (x)
;	periph_stm8s.c: 192: return urecv;
	ret
;	periph_stm8s.c: 198: void pwm1_init(unsigned int timval)
;	-----------------------------------------
;	 function pwm1_init
;	-----------------------------------------
_pwm1_init:
	sub	sp, #2
;	periph_stm8s.c: 200: TIM1_PSCRH = 0x00; //TIM_CLK = CLK
	mov	0x5260+0, #0x00
;	periph_stm8s.c: 201: TIM1_PSCRL = 0x00; //TIM_CLK = CLK
	mov	0x5261+0, #0x00
;	periph_stm8s.c: 202: TIM1_ARRH = (timval >> 8); //TIM RELOAD
	ld	a, (0x05, sp)
	clr	(0x01, sp)
	ldw	x, #0x5262
	ld	(x), a
;	periph_stm8s.c: 203: TIM1_ARRL = (timval & 0x00FF); //TIM RELOAD
	ld	a, (0x06, sp)
	ld	xh, a
	clr	a
	ld	a, xh
	ldw	x, #0x5263
	ld	(x), a
;	periph_stm8s.c: 204: pwm1ch1_enable();
	call	_pwm1ch1_enable
;	periph_stm8s.c: 205: TIM1_CCER1 |= (0<<TIM1_CCER1_CC1P); //Output active high
	ldw	x, #0x525c
	ld	a, (x)
	ldw	x, #0x525c
	ld	(x), a
;	periph_stm8s.c: 206: TIM1_CCMR1 = (TIM1_OCxREF_PWM_mode1<<TIM1_CCMR1_OC1M); //PWM MODE 1 for Channel 1
	mov	0x5258+0, #0x60
;	periph_stm8s.c: 207: pwm1_update(0x0000); //Start Value
	clrw	x
	pushw	x
	call	_pwm1_update
	addw	sp, #2
;	periph_stm8s.c: 208: TIM1_BKR = (1<<TIM1_BKR_MOE); //ENABLE MAIN OUTPUT 
	mov	0x526d+0, #0x80
;	periph_stm8s.c: 209: TIM1_CR1 |= (1<<TIM1_CR1_CEN); //ENABLE TIM
	ldw	x, #0x5250
	ld	a, (x)
	or	a, #0x01
	ld	(x), a
	addw	sp, #2
	ret
;	periph_stm8s.c: 212: void pwm2_init(unsigned int timval)
;	-----------------------------------------
;	 function pwm2_init
;	-----------------------------------------
_pwm2_init:
	sub	sp, #2
;	periph_stm8s.c: 214: TIM2_PSCR = 0x00; //TIM_CLK = CLK
	mov	0x530e+0, #0x00
;	periph_stm8s.c: 215: TIM2_ARRH = (timval >> 8); //TIM RELOAD
	ld	a, (0x05, sp)
	clr	(0x01, sp)
	ldw	x, #0x530f
	ld	(x), a
;	periph_stm8s.c: 216: TIM2_ARRL = (timval & 0x00FF); //TIM RELOAD
	ld	a, (0x06, sp)
	ld	xh, a
	clr	a
	ld	a, xh
	ldw	x, #0x5310
	ld	(x), a
;	periph_stm8s.c: 217: pwm2ch1_enable();
	call	_pwm2ch1_enable
;	periph_stm8s.c: 218: TIM2_CCER1 |= (0<<TIM2_CCER1_CC1P); //Output active high
	ldw	x, #0x530a
	ld	a, (x)
	ldw	x, #0x530a
	ld	(x), a
;	periph_stm8s.c: 219: TIM2_CCMR1 = (TIM2_OCxREF_PWM_mode1<<TIM2_CCMR1_OC1M); //PWM MODE 1 for Channel 1 
	mov	0x5307+0, #0x60
;	periph_stm8s.c: 220: pwm2_update(0x0000); //Start Value
	clrw	x
	pushw	x
	call	_pwm2_update
	addw	sp, #2
;	periph_stm8s.c: 221: TIM2_CR1 |= (1<<TIM2_CR1_CEN); //ENABLE TIM
	ldw	x, #0x5300
	ld	a, (x)
	or	a, #0x01
	ld	(x), a
	addw	sp, #2
	ret
;	periph_stm8s.c: 224: void pwm1ch1_enable()
;	-----------------------------------------
;	 function pwm1ch1_enable
;	-----------------------------------------
_pwm1ch1_enable:
;	periph_stm8s.c: 226: TIM1_CCER1 |= (1<<TIM1_CCER1_CC1E);
	bset	0x525c, #0
	ret
;	periph_stm8s.c: 229: void pwm1ch1_disable()
;	-----------------------------------------
;	 function pwm1ch1_disable
;	-----------------------------------------
_pwm1ch1_disable:
;	periph_stm8s.c: 231: TIM1_CCER1 &= ~(1<<TIM1_CCER1_CC1E);
	bres	0x525c, #0
	ret
;	periph_stm8s.c: 234: void pwm2ch1_enable()
;	-----------------------------------------
;	 function pwm2ch1_enable
;	-----------------------------------------
_pwm2ch1_enable:
;	periph_stm8s.c: 236: TIM2_CCER1 |= (1<<TIM2_CCER1_CC1E);
	bset	0x530a, #0
	ret
;	periph_stm8s.c: 239: void pwm2ch1_disable()
;	-----------------------------------------
;	 function pwm2ch1_disable
;	-----------------------------------------
_pwm2ch1_disable:
;	periph_stm8s.c: 241: TIM2_CCER1 &= ~(1<<TIM2_CCER1_CC1E);
	bres	0x530a, #0
	ret
;	periph_stm8s.c: 244: void pwm1_update(unsigned int pwmval)
;	-----------------------------------------
;	 function pwm1_update
;	-----------------------------------------
_pwm1_update:
	sub	sp, #2
;	periph_stm8s.c: 246: TIM1_CCR1L = (pwmval & 0x00FF);
	ld	a, (0x06, sp)
	ld	xh, a
	clr	a
	ld	a, xh
	ldw	x, #0x5266
	ld	(x), a
;	periph_stm8s.c: 247: TIM1_CCR1H = (pwmval >> 8);
	ld	a, (0x05, sp)
	clr	(0x01, sp)
	ldw	x, #0x5265
	ld	(x), a
	addw	sp, #2
	ret
;	periph_stm8s.c: 250: void pwm2_update(unsigned int pwmval)
;	-----------------------------------------
;	 function pwm2_update
;	-----------------------------------------
_pwm2_update:
	sub	sp, #2
;	periph_stm8s.c: 252: TIM2_CCR1L = (pwmval & 0x00FF);
	ld	a, (0x06, sp)
	ld	xh, a
	clr	a
	ld	a, xh
	ldw	x, #0x5312
	ld	(x), a
;	periph_stm8s.c: 253: TIM2_CCR1H = (pwmval >> 8);
	ld	a, (0x05, sp)
	clr	(0x01, sp)
	ldw	x, #0x5311
	ld	(x), a
	addw	sp, #2
	ret
;	uart_func.c: 8: void UART_sendchar(unsigned char usend) //send 1 character via UART
;	-----------------------------------------
;	 function UART_sendchar
;	-----------------------------------------
_UART_sendchar:
;	uart_func.c: 10: uart1_send(usend);
	ld	a, (0x03, sp)
	push	a
	call	_uart1_send
	pop	a
	ret
;	uart_func.c: 13: void UART_sendtext(unsigned char *usend) //send text via UART
;	-----------------------------------------
;	 function UART_sendtext
;	-----------------------------------------
_UART_sendtext:
	sub	sp, #2
;	uart_func.c: 17: while(usend[stridx] != 0) //scan characters in string
	clrw	x
	ldw	(0x01, sp), x
00101$:
	ldw	x, (0x05, sp)
	addw	x, (0x01, sp)
	ld	a, (x)
	tnz	a
	jreq	00104$
;	uart_func.c: 19: uart1_send(usend[stridx]); //print each character
	push	a
	call	_uart1_send
	pop	a
;	uart_func.c: 20: stridx++;
	ldw	x, (0x01, sp)
	incw	x
	ldw	(0x01, sp), x
	jra	00101$
00104$:
	addw	sp, #2
	ret
;	uart_func.c: 24: void UART_sendnum(unsigned int unum) //send integer via UART
;	-----------------------------------------
;	 function UART_sendnum
;	-----------------------------------------
_UART_sendnum:
	sub	sp, #12
;	uart_func.c: 32: numb = unum;
	ldw	x, (0x0f, sp)
;	uart_func.c: 33: while(numb!=0)
	clr	a
00101$:
	tnzw	x
	jreq	00114$
;	uart_func.c: 35: ndigit++;
	inc	a
;	uart_func.c: 36: numb /= 10; //count decimal digit	
	ldw	y, #0x000a
	divw	x, y
	jra	00101$
00114$:
	ld	(0x09, sp), a
;	uart_func.c: 38: for(nd=0;nd<ndigit;nd++)
	clr	a
	ldw	x, sp
	incw	x
	ldw	(0x0a, sp), x
00106$:
	cp	a, (0x09, sp)
	jrnc	00104$
;	uart_func.c: 40: numb = unum%10;
	ldw	x, (0x0f, sp)
	ldw	y, #0x000a
	divw	x, y
	ldw	(0x07, sp), y
;	uart_func.c: 41: unum = unum/10;
	ldw	x, (0x0f, sp)
	ldw	y, #0x000a
	divw	x, y
	ldw	(0x0f, sp), x
;	uart_func.c: 42: ibuff[ndigit-(nd+1)] = numb + '0'; //start from last_index-1
	inc	a
	ld	(0x0c, sp), a
	ld	a, (0x09, sp)
	sub	a, (0x0c, sp)
	clrw	x
	ld	xl, a
	addw	x, (0x0a, sp)
	ld	a, (0x08, sp)
	add	a, #0x30
	ld	(x), a
;	uart_func.c: 38: for(nd=0;nd<ndigit;nd++)
	ld	a, (0x0c, sp)
	jra	00106$
00104$:
;	uart_func.c: 44: ibuff[ndigit] = '\0'; //last character is null
	clrw	x
	ld	a, (0x09, sp)
	ld	xl, a
	addw	x, (0x0a, sp)
	clr	(x)
;	uart_func.c: 46: UART_sendtext(ibuff);
	ldw	x, (0x0a, sp)
	pushw	x
	call	_UART_sendtext
	addw	sp, #14
	ret
;	uart_func.c: 49: unsigned char UART_recvchar() //receive 1 character via UART (for Polling mode)
;	-----------------------------------------
;	 function UART_recvchar
;	-----------------------------------------
_UART_recvchar:
;	uart_func.c: 53: urecv = uart1_recv();
;	uart_func.c: 55: return urecv;
	jp	_uart1_recv
;	main.c: 41: void isr_UART1_RX() __interrupt UART_RX_INTERRUPT_VECTOR //ISR for UART Receiver Mode
;	-----------------------------------------
;	 function isr_UART1_RX
;	-----------------------------------------
_isr_UART1_RX:
;	main.c: 43: cri = uart1_recv_i();	//receive 1 byte data
	call	_uart1_recv_i
;	main.c: 44: if((cri=='o')||(cri=='O')) LEDODR &= ~(1<<LED1); //LED ON (active low)
	ld	_cri+0, a
	cp	a, #0x6f
	jreq	00104$
	ld	a, _cri+0
	cp	a, #0x4f
	jrne	00105$
00104$:
	ldw	x, #0x5005
	ld	a, (x)
	and	a, #0xdf
	ld	(x), a
	jra	00108$
00105$:
;	main.c: 45: else if((cri=='x')||(cri=='X')) LEDODR |= 1<<LED1; //LED OFF (active low)
	ld	a, _cri+0
	cp	a, #0x78
	jreq	00101$
	ld	a, _cri+0
	cp	a, #0x58
	jrne	00108$
00101$:
	ldw	x, #0x5005
	ld	a, (x)
	or	a, #0x20
	ld	(x), a
00108$:
	iret
;	main.c: 53: int main()
;	-----------------------------------------
;	 function main
;	-----------------------------------------
_main:
;	main.c: 55: clock_init();
	call	_clock_init
;	main.c: 56: delay_init();
	call	_delay_init
;	main.c: 57: gpio_init();
	call	_gpio_init
;	main.c: 58: adc_init();
	call	_adc_init
;	main.c: 59: uart1_init(UART_RX_INTERRUPT_ENABLED); //UART RX using Interrupt
	push	#0x01
	call	_uart1_init
	pop	a
;	main.c: 61: cri = 0; //init value of UART RX buffer
	clr	_cri+0
;	main.c: 63: enable_interrupts();
	rim 
;	main.c: 65: loop();
	call	_loop
;	main.c: 66: return 0;
	clrw	x
	ret
;	main.c: 71: void loop()
;	-----------------------------------------
;	 function loop
;	-----------------------------------------
_loop:
;	main.c: 73: while(1)
00102$:
;	main.c: 75: acc = read_adc(ACCX); //read Accelerometer X value
	push	#0x03
	call	_read_adc
	pop	a
	ldw	_acc+0, x
;	main.c: 76: UART_sendtext("| X = ");
	ldw	x, #___str_0+0
	pushw	x
	call	_UART_sendtext
	addw	sp, #2
;	main.c: 77: UART_sendnum(acc); //send Accelerometer X value
	push	_acc+1
	push	_acc+0
	call	_UART_sendnum
	addw	sp, #2
;	main.c: 79: acc = read_adc(ACCY); //read Accelerometer Y value
	push	#0x04
	call	_read_adc
	pop	a
	ldw	_acc+0, x
;	main.c: 80: UART_sendtext(" | Y = ");
	ldw	x, #___str_1+0
	pushw	x
	call	_UART_sendtext
	addw	sp, #2
;	main.c: 81: UART_sendnum(acc); //send Accelerometer Y value
	push	_acc+1
	push	_acc+0
	call	_UART_sendnum
	addw	sp, #2
;	main.c: 83: acc = read_adc(ACCZ); //read Accelerometer Z value
	push	#0x02
	call	_read_adc
	pop	a
	ldw	_acc+0, x
;	main.c: 84: UART_sendtext(" | Z = ");
	ldw	x, #___str_2+0
	pushw	x
	call	_UART_sendtext
	addw	sp, #2
;	main.c: 85: UART_sendnum(acc); //send Accelerometer Z value
	push	_acc+1
	push	_acc+0
	call	_UART_sendnum
	addw	sp, #2
;	main.c: 86: UART_sendtext(" |");
	ldw	x, #___str_3+0
	pushw	x
	call	_UART_sendtext
	addw	sp, #2
;	main.c: 89: UART_sendchar('\r'); //return
	push	#0x0d
	call	_UART_sendchar
	pop	a
;	main.c: 90: UART_sendchar('\n'); //newline
	push	#0x0a
	call	_UART_sendchar
	pop	a
;	main.c: 92: delay_ms(500);
	push	#0xf4
	push	#0x01
	clrw	x
	pushw	x
	call	_delay_ms
	addw	sp, #4
	jra	00102$
	ret
;	main.c: 99: void gpio_init()
;	-----------------------------------------
;	 function gpio_init
;	-----------------------------------------
_gpio_init:
;	main.c: 102: ACCXDDR |= (INPUT<<ACCX) | (INPUT<<ACCY);
	ldw	x, #0x5011
	ld	a, (x)
	ldw	x, #0x5011
	ld	(x), a
;	main.c: 103: ACCXCR1 |= (floating<<ACCX) | (floating<<ACCY);
	ldw	x, #0x5012
	ld	a, (x)
	ldw	x, #0x5012
	ld	(x), a
;	main.c: 104: ACCXCR2 |= (exti_disabled<<ACCX) | (exti_disabled<<ACCY);
	ldw	x, #0x5013
	ld	a, (x)
	ldw	x, #0x5013
	ld	(x), a
;	main.c: 106: ACCZDDR |= (INPUT<<ACCZ);
	ldw	x, #0x500c
	ld	a, (x)
	ldw	x, #0x500c
	ld	(x), a
;	main.c: 107: ACCZCR1 |= (floating<<ACCZ);
	ldw	x, #0x500d
	ld	a, (x)
	ldw	x, #0x500d
	ld	(x), a
;	main.c: 108: ACCZCR2 |= (exti_disabled<<ACCZ);
	ldw	x, #0x500e
	ld	a, (x)
	ldw	x, #0x500e
	ld	(x), a
;	main.c: 111: LEDDDR |= (OUTPUT<<LED1) | (OUTPUT<<LED1);
	ldw	x, #0x5007
	ld	a, (x)
	or	a, #0x20
	ld	(x), a
;	main.c: 112: LEDCR1 |= (pushpull<<LED1) | (pushpull<<LED1);
	ldw	x, #0x5008
	ld	a, (x)
	or	a, #0x20
	ld	(x), a
;	main.c: 113: LEDCR2 |= (speed_2MHz<<LED1) | (speed_2MHz<<LED1);
	ldw	x, #0x5009
	ld	a, (x)
	ldw	x, #0x5009
	ld	(x), a
	ret
	.area CODE
___str_0:
	.ascii "| X = "
	.db 0x00
___str_1:
	.ascii " | Y = "
	.db 0x00
___str_2:
	.ascii " | Z = "
	.db 0x00
___str_3:
	.ascii " |"
	.db 0x00
	.area INITIALIZER
	.area CABS (ABS)
