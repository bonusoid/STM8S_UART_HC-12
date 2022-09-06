                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ANSI-C Compiler
                                      3 ; Version 3.5.0 #9253 (Apr  3 2018) (Linux)
                                      4 ; This file was generated Tue Sep  6 22:50:25 2022
                                      5 ;--------------------------------------------------------
                                      6 	.module main
                                      7 	.optsdcc -mstm8
                                      8 	
                                      9 ;--------------------------------------------------------
                                     10 ; Public variables in this module
                                     11 ;--------------------------------------------------------
                                     12 	.globl _main
                                     13 	.globl _isr_UART1_RX
                                     14 	.globl _acc
                                     15 	.globl _cri
                                     16 	.globl _readreg
                                     17 	.globl _delay_init
                                     18 	.globl _delay_us
                                     19 	.globl _delay_ms
                                     20 	.globl _delay_timer
                                     21 	.globl _clock_init
                                     22 	.globl _i2c_init
                                     23 	.globl _i2c_set_start
                                     24 	.globl _i2c_set_address
                                     25 	.globl _i2c_set_stop
                                     26 	.globl _i2c_clear_ack
                                     27 	.globl _i2c_set_ack
                                     28 	.globl _i2c_ack_pos_current
                                     29 	.globl _i2c_ack_pos_next
                                     30 	.globl _i2c_poll_SB
                                     31 	.globl _i2c_poll_ADDR
                                     32 	.globl _i2c_poll_BTF
                                     33 	.globl _i2c_poll_TXE
                                     34 	.globl _i2c_poll_RXNE
                                     35 	.globl _i2c_clear_bits
                                     36 	.globl _i2c_clear_ADDR
                                     37 	.globl _i2c_enable_interrupts
                                     38 	.globl _i2c_disable_interrupts
                                     39 	.globl _adc_init
                                     40 	.globl _read_adc
                                     41 	.globl _uart1_init
                                     42 	.globl _uart1_send
                                     43 	.globl _uart1_recv
                                     44 	.globl _uart1_recv_i
                                     45 	.globl _pwm1_init
                                     46 	.globl _pwm2_init
                                     47 	.globl _pwm1ch1_enable
                                     48 	.globl _pwm1ch1_disable
                                     49 	.globl _pwm2ch1_enable
                                     50 	.globl _pwm2ch1_disable
                                     51 	.globl _pwm1_update
                                     52 	.globl _pwm2_update
                                     53 	.globl _UART_sendchar
                                     54 	.globl _UART_sendtext
                                     55 	.globl _UART_recvchar
                                     56 	.globl _UART_sendnum
                                     57 	.globl _loop
                                     58 	.globl _gpio_init
                                     59 ;--------------------------------------------------------
                                     60 ; ram data
                                     61 ;--------------------------------------------------------
                                     62 	.area DATA
      000001                         63 _readreg::
      000001                         64 	.ds 1
      000002                         65 _cri::
      000002                         66 	.ds 1
      000003                         67 _acc::
      000003                         68 	.ds 2
                                     69 ;--------------------------------------------------------
                                     70 ; ram data
                                     71 ;--------------------------------------------------------
                                     72 	.area INITIALIZED
                                     73 ;--------------------------------------------------------
                                     74 ; Stack segment in internal ram 
                                     75 ;--------------------------------------------------------
                                     76 	.area	SSEG
      000005                         77 __start__stack:
      000005                         78 	.ds	1
                                     79 
                                     80 ;--------------------------------------------------------
                                     81 ; absolute external ram data
                                     82 ;--------------------------------------------------------
                                     83 	.area DABS (ABS)
                                     84 ;--------------------------------------------------------
                                     85 ; interrupt vector 
                                     86 ;--------------------------------------------------------
                                     87 	.area HOME
      008000                         88 __interrupt_vect:
      008000 82 00 80 83             89 	int s_GSINIT ;reset
      008004 82 00 00 00             90 	int 0x0000 ;trap
      008008 82 00 00 00             91 	int 0x0000 ;int0
      00800C 82 00 00 00             92 	int 0x0000 ;int1
      008010 82 00 00 00             93 	int 0x0000 ;int2
      008014 82 00 00 00             94 	int 0x0000 ;int3
      008018 82 00 00 00             95 	int 0x0000 ;int4
      00801C 82 00 00 00             96 	int 0x0000 ;int5
      008020 82 00 00 00             97 	int 0x0000 ;int6
      008024 82 00 00 00             98 	int 0x0000 ;int7
      008028 82 00 00 00             99 	int 0x0000 ;int8
      00802C 82 00 00 00            100 	int 0x0000 ;int9
      008030 82 00 00 00            101 	int 0x0000 ;int10
      008034 82 00 00 00            102 	int 0x0000 ;int11
      008038 82 00 00 00            103 	int 0x0000 ;int12
      00803C 82 00 00 00            104 	int 0x0000 ;int13
      008040 82 00 00 00            105 	int 0x0000 ;int14
      008044 82 00 00 00            106 	int 0x0000 ;int15
      008048 82 00 00 00            107 	int 0x0000 ;int16
      00804C 82 00 00 00            108 	int 0x0000 ;int17
      008050 82 00 83 F7            109 	int _isr_UART1_RX ;int18
      008054 82 00 00 00            110 	int 0x0000 ;int19
      008058 82 00 00 00            111 	int 0x0000 ;int20
      00805C 82 00 00 00            112 	int 0x0000 ;int21
      008060 82 00 00 00            113 	int 0x0000 ;int22
      008064 82 00 00 00            114 	int 0x0000 ;int23
      008068 82 00 00 00            115 	int 0x0000 ;int24
      00806C 82 00 00 00            116 	int 0x0000 ;int25
      008070 82 00 00 00            117 	int 0x0000 ;int26
      008074 82 00 00 00            118 	int 0x0000 ;int27
      008078 82 00 00 00            119 	int 0x0000 ;int28
      00807C 82 00 00 00            120 	int 0x0000 ;int29
                                    121 ;--------------------------------------------------------
                                    122 ; global & static initialisations
                                    123 ;--------------------------------------------------------
                                    124 	.area HOME
                                    125 	.area GSINIT
                                    126 	.area GSFINAL
                                    127 	.area GSINIT
      008083                        128 __sdcc_gs_init_startup:
      008083                        129 __sdcc_init_data:
                                    130 ; stm8_genXINIT() start
      008083 AE 00 04         [ 2]  131 	ldw x, #l_DATA
      008086 27 07            [ 1]  132 	jreq	00002$
      008088                        133 00001$:
      008088 72 4F 00 00      [ 1]  134 	clr (s_DATA - 1, x)
      00808C 5A               [ 2]  135 	decw x
      00808D 26 F9            [ 1]  136 	jrne	00001$
      00808F                        137 00002$:
      00808F AE 00 00         [ 2]  138 	ldw	x, #l_INITIALIZER
      008092 27 09            [ 1]  139 	jreq	00004$
      008094                        140 00003$:
      008094 D6 86 63         [ 1]  141 	ld	a, (s_INITIALIZER - 1, x)
      008097 D7 00 04         [ 1]  142 	ld	(s_INITIALIZED - 1, x), a
      00809A 5A               [ 2]  143 	decw	x
      00809B 26 F7            [ 1]  144 	jrne	00003$
      00809D                        145 00004$:
                                    146 ; stm8_genXINIT() end
                                    147 	.area GSFINAL
      00809D CC 80 80         [ 2]  148 	jp	__sdcc_program_startup
                                    149 ;--------------------------------------------------------
                                    150 ; Home
                                    151 ;--------------------------------------------------------
                                    152 	.area HOME
                                    153 	.area HOME
      008080                        154 __sdcc_program_startup:
      008080 CC 84 27         [ 2]  155 	jp	_main
                                    156 ;	return from main will return to caller
                                    157 ;--------------------------------------------------------
                                    158 ; code
                                    159 ;--------------------------------------------------------
                                    160 	.area CODE
                                    161 ;	delay.c: 7: void delay_init()
                                    162 ;	-----------------------------------------
                                    163 ;	 function delay_init
                                    164 ;	-----------------------------------------
      0080A0                        165 _delay_init:
                                    166 ;	delay.c: 9: TIM4_PSCR = 4; // CLK/16
      0080A0 35 04 53 47      [ 1]  167 	mov	0x5347+0, #0x04
      0080A4 81               [ 4]  168 	ret
                                    169 ;	delay.c: 12: void delay_us(unsigned long delus)
                                    170 ;	-----------------------------------------
                                    171 ;	 function delay_us
                                    172 ;	-----------------------------------------
      0080A5                        173 _delay_us:
      0080A5 52 06            [ 2]  174 	sub	sp, #6
                                    175 ;	delay.c: 16: for(du=0;du<(delus/10);du++)
      0080A7 4B 0A            [ 1]  176 	push	#0x0a
      0080A9 5F               [ 1]  177 	clrw	x
      0080AA 89               [ 2]  178 	pushw	x
      0080AB 4B 00            [ 1]  179 	push	#0x00
      0080AD 1E 0F            [ 2]  180 	ldw	x, (0x0f, sp)
      0080AF 89               [ 2]  181 	pushw	x
      0080B0 1E 0F            [ 2]  182 	ldw	x, (0x0f, sp)
      0080B2 89               [ 2]  183 	pushw	x
      0080B3 CD 85 8E         [ 4]  184 	call	__divulong
      0080B6 5B 08            [ 2]  185 	addw	sp, #8
      0080B8 1F 05            [ 2]  186 	ldw	(0x05, sp), x
      0080BA 17 03            [ 2]  187 	ldw	(0x03, sp), y
      0080BC 5F               [ 1]  188 	clrw	x
      0080BD 1F 01            [ 2]  189 	ldw	(0x01, sp), x
      0080BF                        190 00103$:
      0080BF 1E 01            [ 2]  191 	ldw	x, (0x01, sp)
      0080C1 90 5F            [ 1]  192 	clrw	y
      0080C3 13 05            [ 2]  193 	cpw	x, (0x05, sp)
      0080C5 90 9F            [ 1]  194 	ld	a, yl
      0080C7 12 04            [ 1]  195 	sbc	a, (0x04, sp)
      0080C9 90 9E            [ 1]  196 	ld	a, yh
      0080CB 12 03            [ 1]  197 	sbc	a, (0x03, sp)
      0080CD 24 0D            [ 1]  198 	jrnc	00101$
                                    199 ;	delay.c: 18: delay_timer(100);
      0080CF 4B 64            [ 1]  200 	push	#0x64
      0080D1 CD 81 3A         [ 4]  201 	call	_delay_timer
      0080D4 84               [ 1]  202 	pop	a
                                    203 ;	delay.c: 16: for(du=0;du<(delus/10);du++)
      0080D5 1E 01            [ 2]  204 	ldw	x, (0x01, sp)
      0080D7 5C               [ 2]  205 	incw	x
      0080D8 1F 01            [ 2]  206 	ldw	(0x01, sp), x
      0080DA 20 E3            [ 2]  207 	jra	00103$
      0080DC                        208 00101$:
                                    209 ;	delay.c: 20: delay_timer(delus%10);
      0080DC 4B 0A            [ 1]  210 	push	#0x0a
      0080DE 5F               [ 1]  211 	clrw	x
      0080DF 89               [ 2]  212 	pushw	x
      0080E0 4B 00            [ 1]  213 	push	#0x00
      0080E2 1E 0F            [ 2]  214 	ldw	x, (0x0f, sp)
      0080E4 89               [ 2]  215 	pushw	x
      0080E5 1E 0F            [ 2]  216 	ldw	x, (0x0f, sp)
      0080E7 89               [ 2]  217 	pushw	x
      0080E8 CD 85 1E         [ 4]  218 	call	__modulong
      0080EB 5B 08            [ 2]  219 	addw	sp, #8
      0080ED 9F               [ 1]  220 	ld	a, xl
      0080EE 88               [ 1]  221 	push	a
      0080EF CD 81 3A         [ 4]  222 	call	_delay_timer
      0080F2 5B 07            [ 2]  223 	addw	sp, #7
      0080F4 81               [ 4]  224 	ret
                                    225 ;	delay.c: 23: void delay_ms(unsigned long delms)
                                    226 ;	-----------------------------------------
                                    227 ;	 function delay_ms
                                    228 ;	-----------------------------------------
      0080F5                        229 _delay_ms:
      0080F5 52 08            [ 2]  230 	sub	sp, #8
                                    231 ;	delay.c: 27: for(dm=0;dm<(delms*100);dm++)
      0080F7 1E 0D            [ 2]  232 	ldw	x, (0x0d, sp)
      0080F9 89               [ 2]  233 	pushw	x
      0080FA 1E 0D            [ 2]  234 	ldw	x, (0x0d, sp)
      0080FC 89               [ 2]  235 	pushw	x
      0080FD 4B 64            [ 1]  236 	push	#0x64
      0080FF 5F               [ 1]  237 	clrw	x
      008100 89               [ 2]  238 	pushw	x
      008101 4B 00            [ 1]  239 	push	#0x00
      008103 CD 85 E8         [ 4]  240 	call	__mullong
      008106 5B 08            [ 2]  241 	addw	sp, #8
      008108 1F 07            [ 2]  242 	ldw	(0x07, sp), x
      00810A 17 05            [ 2]  243 	ldw	(0x05, sp), y
      00810C 5F               [ 1]  244 	clrw	x
      00810D 4F               [ 1]  245 	clr	a
      00810E 0F 01            [ 1]  246 	clr	(0x01, sp)
      008110                        247 00103$:
      008110 88               [ 1]  248 	push	a
      008111 13 08            [ 2]  249 	cpw	x, (0x08, sp)
      008113 7B 01            [ 1]  250 	ld	a, (1, sp)
      008115 12 07            [ 1]  251 	sbc	a, (0x07, sp)
      008117 7B 02            [ 1]  252 	ld	a, (0x02, sp)
      008119 12 06            [ 1]  253 	sbc	a, (0x06, sp)
      00811B 84               [ 1]  254 	pop	a
      00811C 24 19            [ 1]  255 	jrnc	00105$
                                    256 ;	delay.c: 29: delay_timer(100);
      00811E 88               [ 1]  257 	push	a
      00811F 89               [ 2]  258 	pushw	x
      008120 4B 64            [ 1]  259 	push	#0x64
      008122 CD 81 3A         [ 4]  260 	call	_delay_timer
      008125 84               [ 1]  261 	pop	a
      008126 85               [ 2]  262 	popw	x
      008127 84               [ 1]  263 	pop	a
                                    264 ;	delay.c: 27: for(dm=0;dm<(delms*100);dm++)
      008128 1C 00 01         [ 2]  265 	addw	x, #0x0001
      00812B A9 00            [ 1]  266 	adc	a, #0x00
      00812D 88               [ 1]  267 	push	a
      00812E 7B 02            [ 1]  268 	ld	a, (0x02, sp)
      008130 A9 00            [ 1]  269 	adc	a, #0x00
      008132 6B 02            [ 1]  270 	ld	(0x02, sp), a
      008134 84               [ 1]  271 	pop	a
      008135 20 D9            [ 2]  272 	jra	00103$
      008137                        273 00105$:
      008137 5B 08            [ 2]  274 	addw	sp, #8
      008139 81               [ 4]  275 	ret
                                    276 ;	delay.c: 33: void delay_timer(unsigned char deltim)
                                    277 ;	-----------------------------------------
                                    278 ;	 function delay_timer
                                    279 ;	-----------------------------------------
      00813A                        280 _delay_timer:
                                    281 ;	delay.c: 35: TIM4_CR1 = (1<<TIM4_CR1_CEN);
      00813A 35 01 53 40      [ 1]  282 	mov	0x5340+0, #0x01
                                    283 ;	delay.c: 36: while(TIM4_CNTR<deltim);
      00813E                        284 00101$:
      00813E AE 53 46         [ 2]  285 	ldw	x, #0x5346
      008141 F6               [ 1]  286 	ld	a, (x)
      008142 11 03            [ 1]  287 	cp	a, (0x03, sp)
      008144 25 F8            [ 1]  288 	jrc	00101$
                                    289 ;	delay.c: 37: TIM4_CR1 = (0<<TIM4_CR1_CEN);
      008146 35 00 53 40      [ 1]  290 	mov	0x5340+0, #0x00
                                    291 ;	delay.c: 38: TIM4_CNTR = 0; //reset timer	
      00814A 35 00 53 46      [ 1]  292 	mov	0x5346+0, #0x00
      00814E 81               [ 4]  293 	ret
                                    294 ;	periph_stm8s.c: 16: void clock_init()
                                    295 ;	-----------------------------------------
                                    296 ;	 function clock_init
                                    297 ;	-----------------------------------------
      00814F                        298 _clock_init:
                                    299 ;	periph_stm8s.c: 18: CLK_CKDIVR = 0x00; //fMASTER = fCPU = fHSI = 16MHz
      00814F 35 00 50 C6      [ 1]  300 	mov	0x50c6+0, #0x00
                                    301 ;	periph_stm8s.c: 19: CLK_ICKR = (1<<CLK_ICKR_HSIEN);	//High Speed Internal RC Oscillator enabled
      008153 35 01 50 C0      [ 1]  302 	mov	0x50c0+0, #0x01
      008157 81               [ 4]  303 	ret
                                    304 ;	periph_stm8s.c: 24: void i2c_init()
                                    305 ;	-----------------------------------------
                                    306 ;	 function i2c_init
                                    307 ;	-----------------------------------------
      008158                        308 _i2c_init:
                                    309 ;	periph_stm8s.c: 26: I2C_CR1 = (0<<I2C_CR1_PE); //Disable I2C before configuration starts
      008158 35 00 52 10      [ 1]  310 	mov	0x5210+0, #0x00
                                    311 ;	periph_stm8s.c: 27: I2C_FREQR = 16;	//fCLK = 16 MHz
      00815C 35 10 52 12      [ 1]  312 	mov	0x5212+0, #0x10
                                    313 ;	periph_stm8s.c: 28: I2C_CCRH = (0<<I2C_CCRH_FS)|(0<<I2C_CCRH_CCR11)|(0<<I2C_CCRH_CCR10)|(0<<I2C_CCRH_CCR9)|(0<<I2C_CCRH_CCR8); //Standard Mode
      008160 35 00 52 1C      [ 1]  314 	mov	0x521c+0, #0x00
                                    315 ;	periph_stm8s.c: 29: I2C_CCRL = 0x80;  //Clock Speed = 100 kHz
      008164 35 80 52 1B      [ 1]  316 	mov	0x521b+0, #0x80
                                    317 ;	periph_stm8s.c: 31: I2C_OARH = (0<<I2C_OARH_ADDMODE)|(1<<I2C_OARH_ADDCONF); //7-bit address mode, ADDCONF always must be 1
      008168 35 40 52 14      [ 1]  318 	mov	0x5214+0, #0x40
                                    319 ;	periph_stm8s.c: 32: I2C_TRISER = 17;  //Setup Bus Characteristic
      00816C 35 11 52 1D      [ 1]  320 	mov	0x521d+0, #0x11
                                    321 ;	periph_stm8s.c: 37: I2C_CR1 = (1<<I2C_CR1_PE);  //Enable I2C after configuration complete
      008170 35 01 52 10      [ 1]  322 	mov	0x5210+0, #0x01
      008174 81               [ 4]  323 	ret
                                    324 ;	periph_stm8s.c: 40: void i2c_set_start()
                                    325 ;	-----------------------------------------
                                    326 ;	 function i2c_set_start
                                    327 ;	-----------------------------------------
      008175                        328 _i2c_set_start:
                                    329 ;	periph_stm8s.c: 42: I2C_CR2 |= (1<<I2C_CR2_START);
      008175 72 10 52 11      [ 1]  330 	bset	0x5211, #0
      008179 81               [ 4]  331 	ret
                                    332 ;	periph_stm8s.c: 45: void i2c_set_address(unsigned char addr, unsigned char dir)
                                    333 ;	-----------------------------------------
                                    334 ;	 function i2c_set_address
                                    335 ;	-----------------------------------------
      00817A                        336 _i2c_set_address:
                                    337 ;	periph_stm8s.c: 47: if(dir==I2C_READ) I2C_DR = (addr<<1)|dir;
      00817A 7B 03            [ 1]  338 	ld	a, (0x03, sp)
      00817C 97               [ 1]  339 	ld	xl, a
      00817D 58               [ 2]  340 	sllw	x
      00817E 7B 04            [ 1]  341 	ld	a, (0x04, sp)
      008180 A1 01            [ 1]  342 	cp	a, #0x01
      008182 26 09            [ 1]  343 	jrne	00104$
      008184 9F               [ 1]  344 	ld	a, xl
      008185 1A 04            [ 1]  345 	or	a, (0x04, sp)
      008187 AE 52 16         [ 2]  346 	ldw	x, #0x5216
      00818A F7               [ 1]  347 	ld	(x), a
      00818B 20 0D            [ 2]  348 	jra	00106$
      00818D                        349 00104$:
                                    350 ;	periph_stm8s.c: 48: else if(dir==I2C_WRITE) I2C_DR = (addr<<1)&dir;
      00818D 7B 04            [ 1]  351 	ld	a, (0x04, sp)
      00818F A1 FE            [ 1]  352 	cp	a, #0xfe
      008191 26 07            [ 1]  353 	jrne	00106$
      008193 9F               [ 1]  354 	ld	a, xl
      008194 14 04            [ 1]  355 	and	a, (0x04, sp)
      008196 AE 52 16         [ 2]  356 	ldw	x, #0x5216
      008199 F7               [ 1]  357 	ld	(x), a
      00819A                        358 00106$:
      00819A 81               [ 4]  359 	ret
                                    360 ;	periph_stm8s.c: 52: void i2c_set_stop()
                                    361 ;	-----------------------------------------
                                    362 ;	 function i2c_set_stop
                                    363 ;	-----------------------------------------
      00819B                        364 _i2c_set_stop:
                                    365 ;	periph_stm8s.c: 54: I2C_CR2 |= (1<<I2C_CR2_STOP);
      00819B AE 52 11         [ 2]  366 	ldw	x, #0x5211
      00819E F6               [ 1]  367 	ld	a, (x)
      00819F AA 02            [ 1]  368 	or	a, #0x02
      0081A1 F7               [ 1]  369 	ld	(x), a
      0081A2 81               [ 4]  370 	ret
                                    371 ;	periph_stm8s.c: 57: void i2c_clear_ack()
                                    372 ;	-----------------------------------------
                                    373 ;	 function i2c_clear_ack
                                    374 ;	-----------------------------------------
      0081A3                        375 _i2c_clear_ack:
                                    376 ;	periph_stm8s.c: 59: I2C_CR2 &= ~(1<<I2C_CR2_ACK); //Disable Acknowledge
      0081A3 AE 52 11         [ 2]  377 	ldw	x, #0x5211
      0081A6 F6               [ 1]  378 	ld	a, (x)
      0081A7 A4 FB            [ 1]  379 	and	a, #0xfb
      0081A9 F7               [ 1]  380 	ld	(x), a
      0081AA 81               [ 4]  381 	ret
                                    382 ;	periph_stm8s.c: 62: void i2c_set_ack()
                                    383 ;	-----------------------------------------
                                    384 ;	 function i2c_set_ack
                                    385 ;	-----------------------------------------
      0081AB                        386 _i2c_set_ack:
                                    387 ;	periph_stm8s.c: 64: I2C_CR2 |= (1<<I2C_CR2_ACK); //Enable Acknowledge
      0081AB AE 52 11         [ 2]  388 	ldw	x, #0x5211
      0081AE F6               [ 1]  389 	ld	a, (x)
      0081AF AA 04            [ 1]  390 	or	a, #0x04
      0081B1 F7               [ 1]  391 	ld	(x), a
      0081B2 81               [ 4]  392 	ret
                                    393 ;	periph_stm8s.c: 67: void i2c_ack_pos_current()
                                    394 ;	-----------------------------------------
                                    395 ;	 function i2c_ack_pos_current
                                    396 ;	-----------------------------------------
      0081B3                        397 _i2c_ack_pos_current:
                                    398 ;	periph_stm8s.c: 69: I2C_CR2 &= ~(1<<I2C_CR2_POS); //ACK bit controls the (N)ACK of the current byte being received in the shift register
      0081B3 AE 52 11         [ 2]  399 	ldw	x, #0x5211
      0081B6 F6               [ 1]  400 	ld	a, (x)
      0081B7 A4 F7            [ 1]  401 	and	a, #0xf7
      0081B9 F7               [ 1]  402 	ld	(x), a
      0081BA 81               [ 4]  403 	ret
                                    404 ;	periph_stm8s.c: 72: void i2c_ack_pos_next()
                                    405 ;	-----------------------------------------
                                    406 ;	 function i2c_ack_pos_next
                                    407 ;	-----------------------------------------
      0081BB                        408 _i2c_ack_pos_next:
                                    409 ;	periph_stm8s.c: 74: I2C_CR2 |= (1<<I2C_CR2_POS); //ACK bit controls the (N)ACK of the next byte which will be received in the shift register
      0081BB AE 52 11         [ 2]  410 	ldw	x, #0x5211
      0081BE F6               [ 1]  411 	ld	a, (x)
      0081BF AA 08            [ 1]  412 	or	a, #0x08
      0081C1 F7               [ 1]  413 	ld	(x), a
      0081C2 81               [ 4]  414 	ret
                                    415 ;	periph_stm8s.c: 77: void i2c_poll_SB()
                                    416 ;	-----------------------------------------
                                    417 ;	 function i2c_poll_SB
                                    418 ;	-----------------------------------------
      0081C3                        419 _i2c_poll_SB:
                                    420 ;	periph_stm8s.c: 79: while((I2C_SR1&(1<<I2C_SR1_SB))!=(1<<I2C_SR1_SB)){}
      0081C3                        421 00101$:
      0081C3 AE 52 17         [ 2]  422 	ldw	x, #0x5217
      0081C6 F6               [ 1]  423 	ld	a, (x)
      0081C7 A4 01            [ 1]  424 	and	a, #0x01
      0081C9 A1 01            [ 1]  425 	cp	a, #0x01
      0081CB 26 F6            [ 1]  426 	jrne	00101$
      0081CD 81               [ 4]  427 	ret
                                    428 ;	periph_stm8s.c: 82: void i2c_poll_ADDR()
                                    429 ;	-----------------------------------------
                                    430 ;	 function i2c_poll_ADDR
                                    431 ;	-----------------------------------------
      0081CE                        432 _i2c_poll_ADDR:
                                    433 ;	periph_stm8s.c: 84: while((I2C_SR1&(1<<I2C_SR1_ADDR))!=(1<<I2C_SR1_ADDR)){}
      0081CE                        434 00101$:
      0081CE AE 52 17         [ 2]  435 	ldw	x, #0x5217
      0081D1 F6               [ 1]  436 	ld	a, (x)
      0081D2 A4 02            [ 1]  437 	and	a, #0x02
      0081D4 A1 02            [ 1]  438 	cp	a, #0x02
      0081D6 26 F6            [ 1]  439 	jrne	00101$
      0081D8 81               [ 4]  440 	ret
                                    441 ;	periph_stm8s.c: 87: void i2c_poll_BTF()
                                    442 ;	-----------------------------------------
                                    443 ;	 function i2c_poll_BTF
                                    444 ;	-----------------------------------------
      0081D9                        445 _i2c_poll_BTF:
                                    446 ;	periph_stm8s.c: 89: while((I2C_SR1&(1<<I2C_SR1_BTF))!=(1<<I2C_SR1_BTF)){}
      0081D9                        447 00101$:
      0081D9 AE 52 17         [ 2]  448 	ldw	x, #0x5217
      0081DC F6               [ 1]  449 	ld	a, (x)
      0081DD A4 04            [ 1]  450 	and	a, #0x04
      0081DF A1 04            [ 1]  451 	cp	a, #0x04
      0081E1 26 F6            [ 1]  452 	jrne	00101$
      0081E3 81               [ 4]  453 	ret
                                    454 ;	periph_stm8s.c: 92: void i2c_poll_TXE()
                                    455 ;	-----------------------------------------
                                    456 ;	 function i2c_poll_TXE
                                    457 ;	-----------------------------------------
      0081E4                        458 _i2c_poll_TXE:
                                    459 ;	periph_stm8s.c: 94: while((I2C_SR1&(1<<I2C_SR1_TXE))!=(1<<I2C_SR1_TXE)){}
      0081E4                        460 00101$:
      0081E4 AE 52 17         [ 2]  461 	ldw	x, #0x5217
      0081E7 F6               [ 1]  462 	ld	a, (x)
      0081E8 A4 80            [ 1]  463 	and	a, #0x80
      0081EA A1 80            [ 1]  464 	cp	a, #0x80
      0081EC 26 F6            [ 1]  465 	jrne	00101$
      0081EE 81               [ 4]  466 	ret
                                    467 ;	periph_stm8s.c: 97: void i2c_poll_RXNE()
                                    468 ;	-----------------------------------------
                                    469 ;	 function i2c_poll_RXNE
                                    470 ;	-----------------------------------------
      0081EF                        471 _i2c_poll_RXNE:
                                    472 ;	periph_stm8s.c: 99: while((I2C_SR1&(1<<I2C_SR1_RXNE))!=(1<<I2C_SR1_RXNE)){}
      0081EF                        473 00101$:
      0081EF AE 52 17         [ 2]  474 	ldw	x, #0x5217
      0081F2 F6               [ 1]  475 	ld	a, (x)
      0081F3 A4 40            [ 1]  476 	and	a, #0x40
      0081F5 A1 40            [ 1]  477 	cp	a, #0x40
      0081F7 26 F6            [ 1]  478 	jrne	00101$
      0081F9 81               [ 4]  479 	ret
                                    480 ;	periph_stm8s.c: 102: void i2c_clear_bits()
                                    481 ;	-----------------------------------------
                                    482 ;	 function i2c_clear_bits
                                    483 ;	-----------------------------------------
      0081FA                        484 _i2c_clear_bits:
                                    485 ;	periph_stm8s.c: 104: readreg = I2C_SR1;
      0081FA AE 52 17         [ 2]  486 	ldw	x, #0x5217
      0081FD F6               [ 1]  487 	ld	a, (x)
      0081FE C7 00 01         [ 1]  488 	ld	_readreg+0, a
      008201 81               [ 4]  489 	ret
                                    490 ;	periph_stm8s.c: 107: void i2c_clear_ADDR()
                                    491 ;	-----------------------------------------
                                    492 ;	 function i2c_clear_ADDR
                                    493 ;	-----------------------------------------
      008202                        494 _i2c_clear_ADDR:
                                    495 ;	periph_stm8s.c: 109: readreg = I2C_SR1;
      008202 AE 52 17         [ 2]  496 	ldw	x, #0x5217
      008205 F6               [ 1]  497 	ld	a, (x)
                                    498 ;	periph_stm8s.c: 110: readreg = I2C_SR3;
      008206 AE 52 19         [ 2]  499 	ldw	x, #0x5219
      008209 F6               [ 1]  500 	ld	a, (x)
      00820A C7 00 01         [ 1]  501 	ld	_readreg+0, a
      00820D 81               [ 4]  502 	ret
                                    503 ;	periph_stm8s.c: 113: void i2c_enable_interrupts()
                                    504 ;	-----------------------------------------
                                    505 ;	 function i2c_enable_interrupts
                                    506 ;	-----------------------------------------
      00820E                        507 _i2c_enable_interrupts:
                                    508 ;	periph_stm8s.c: 115: I2C_ITR = 0x07;
      00820E 35 07 52 1A      [ 1]  509 	mov	0x521a+0, #0x07
      008212 81               [ 4]  510 	ret
                                    511 ;	periph_stm8s.c: 117: void i2c_disable_interrupts()
                                    512 ;	-----------------------------------------
                                    513 ;	 function i2c_disable_interrupts
                                    514 ;	-----------------------------------------
      008213                        515 _i2c_disable_interrupts:
                                    516 ;	periph_stm8s.c: 119: I2C_ITR = 0x00;
      008213 35 00 52 1A      [ 1]  517 	mov	0x521a+0, #0x00
      008217 81               [ 4]  518 	ret
                                    519 ;	periph_stm8s.c: 124: void adc_init()
                                    520 ;	-----------------------------------------
                                    521 ;	 function adc_init
                                    522 ;	-----------------------------------------
      008218                        523 _adc_init:
                                    524 ;	periph_stm8s.c: 126: ADC1_CR1 = fADC_fMASTER_8<<ADC1_CR1_SPSEL; // ADCCLK = MCLK/8
      008218 35 40 54 01      [ 1]  525 	mov	0x5401+0, #0x40
                                    526 ;	periph_stm8s.c: 127: ADC1_CR2 = (1<<ADC1_CR2_ALIGN);  // right alignment adc data
      00821C 35 08 54 02      [ 1]  527 	mov	0x5402+0, #0x08
                                    528 ;	periph_stm8s.c: 129: ADC1_CR1 |= (1<<ADC1_CR1_ADON);  // turn on ADC
      008220 72 10 54 01      [ 1]  529 	bset	0x5401, #0
      008224 81               [ 4]  530 	ret
                                    531 ;	periph_stm8s.c: 133: unsigned int read_adc(unsigned char adcch)
                                    532 ;	-----------------------------------------
                                    533 ;	 function read_adc
                                    534 ;	-----------------------------------------
      008225                        535 _read_adc:
      008225 52 04            [ 2]  536 	sub	sp, #4
                                    537 ;	periph_stm8s.c: 137: ADC1_CSR &= 0xF0;  // select
      008227 AE 54 00         [ 2]  538 	ldw	x, #0x5400
      00822A F6               [ 1]  539 	ld	a, (x)
      00822B A4 F0            [ 1]  540 	and	a, #0xf0
      00822D F7               [ 1]  541 	ld	(x), a
                                    542 ;	periph_stm8s.c: 138: ADC1_CSR |= adcch; // channel
      00822E AE 54 00         [ 2]  543 	ldw	x, #0x5400
      008231 F6               [ 1]  544 	ld	a, (x)
      008232 1A 07            [ 1]  545 	or	a, (0x07, sp)
      008234 AE 54 00         [ 2]  546 	ldw	x, #0x5400
      008237 F7               [ 1]  547 	ld	(x), a
                                    548 ;	periph_stm8s.c: 141: ADC1_CR1 |= (1<<ADC1_CR1_ADON); // start conversion
      008238 72 10 54 01      [ 1]  549 	bset	0x5401, #0
                                    550 ;	periph_stm8s.c: 142: while(!((ADC1_CSR)&(1<<ADC1_CSR_EOC)));; // conversion is in progress
      00823C                        551 00101$:
      00823C AE 54 00         [ 2]  552 	ldw	x, #0x5400
      00823F F6               [ 1]  553 	ld	a, (x)
      008240 4D               [ 1]  554 	tnz	a
      008241 2A F9            [ 1]  555 	jrpl	00101$
                                    556 ;	periph_stm8s.c: 143: adcval = (ADC1_DRH<<8) + ADC1_DRL;
      008243 AE 54 04         [ 2]  557 	ldw	x, #0x5404
      008246 F6               [ 1]  558 	ld	a, (x)
      008247 0F 03            [ 1]  559 	clr	(0x03, sp)
      008249 6B 01            [ 1]  560 	ld	(0x01, sp), a
      00824B 0F 02            [ 1]  561 	clr	(0x02, sp)
      00824D AE 54 05         [ 2]  562 	ldw	x, #0x5405
      008250 F6               [ 1]  563 	ld	a, (x)
      008251 5F               [ 1]  564 	clrw	x
      008252 97               [ 1]  565 	ld	xl, a
      008253 72 FB 01         [ 2]  566 	addw	x, (0x01, sp)
                                    567 ;	periph_stm8s.c: 144: ADC1_CSR |= (0<<ADC1_CSR_EOC); // reset EOC
      008256 90 AE 54 00      [ 2]  568 	ldw	y, #0x5400
      00825A 90 F6            [ 1]  569 	ld	a, (y)
      00825C 90 AE 54 00      [ 2]  570 	ldw	y, #0x5400
      008260 90 F7            [ 1]  571 	ld	(y), a
                                    572 ;	periph_stm8s.c: 146: return adcval;
      008262 5B 04            [ 2]  573 	addw	sp, #4
      008264 81               [ 4]  574 	ret
                                    575 ;	periph_stm8s.c: 151: void uart1_init(unsigned char rxien) //UART Initialization
                                    576 ;	-----------------------------------------
                                    577 ;	 function uart1_init
                                    578 ;	-----------------------------------------
      008265                        579 _uart1_init:
                                    580 ;	periph_stm8s.c: 155: UART1_BRR1 = 0x68;
      008265 35 68 52 32      [ 1]  581 	mov	0x5232+0, #0x68
                                    582 ;	periph_stm8s.c: 156: UART1_BRR2 = 0x03;
      008269 35 03 52 33      [ 1]  583 	mov	0x5233+0, #0x03
                                    584 ;	periph_stm8s.c: 158: UART1_CR1 |= (0<<UART1_CR1_M)|(0<<UART1_CR1_PCEN); //8 bit Data; No Parity
      00826D AE 52 34         [ 2]  585 	ldw	x, #0x5234
      008270 F6               [ 1]  586 	ld	a, (x)
      008271 AE 52 34         [ 2]  587 	ldw	x, #0x5234
      008274 F7               [ 1]  588 	ld	(x), a
                                    589 ;	periph_stm8s.c: 159: UART1_CR3 |= (0<<UART1_CR3_STOP); //Stop Bit = 1
      008275 AE 52 36         [ 2]  590 	ldw	x, #0x5236
      008278 F6               [ 1]  591 	ld	a, (x)
      008279 AE 52 36         [ 2]  592 	ldw	x, #0x5236
      00827C F7               [ 1]  593 	ld	(x), a
                                    594 ;	periph_stm8s.c: 161: if(rxien==1) 
      00827D 7B 03            [ 1]  595 	ld	a, (0x03, sp)
      00827F A1 01            [ 1]  596 	cp	a, #0x01
      008281 26 0B            [ 1]  597 	jrne	00102$
                                    598 ;	periph_stm8s.c: 163: UART1_CR2 |= (1<<UART1_CR2_RIEN); //Enable Interrupt on Receiver Mode
      008283 AE 52 35         [ 2]  599 	ldw	x, #0x5235
      008286 F6               [ 1]  600 	ld	a, (x)
      008287 AA 20            [ 1]  601 	or	a, #0x20
      008289 F7               [ 1]  602 	ld	(x), a
                                    603 ;	periph_stm8s.c: 164: ITC_SPR5 = (level_2<<ITC_SPR5_VECT18); //UART Interrupt Setting
      00828A 35 00 7F 74      [ 1]  604 	mov	0x7f74+0, #0x00
      00828E                        605 00102$:
                                    606 ;	periph_stm8s.c: 167: UART1_CR2 |= (1<<UART1_CR2_TEN); //Enable Transmitter Mode
      00828E AE 52 35         [ 2]  607 	ldw	x, #0x5235
      008291 F6               [ 1]  608 	ld	a, (x)
      008292 AA 08            [ 1]  609 	or	a, #0x08
      008294 F7               [ 1]  610 	ld	(x), a
                                    611 ;	periph_stm8s.c: 168: UART1_CR2 |= (1<<UART1_CR2_REN); //Enable Receiver Mode
      008295 AE 52 35         [ 2]  612 	ldw	x, #0x5235
      008298 F6               [ 1]  613 	ld	a, (x)
      008299 AA 04            [ 1]  614 	or	a, #0x04
      00829B F7               [ 1]  615 	ld	(x), a
      00829C 81               [ 4]  616 	ret
                                    617 ;	periph_stm8s.c: 171: void uart1_send(unsigned char usend) //UART Transmit a Byte
                                    618 ;	-----------------------------------------
                                    619 ;	 function uart1_send
                                    620 ;	-----------------------------------------
      00829D                        621 _uart1_send:
                                    622 ;	periph_stm8s.c: 173: UART1_DR = usend; //Write to UART Data Register
      00829D AE 52 31         [ 2]  623 	ldw	x, #0x5231
      0082A0 7B 03            [ 1]  624 	ld	a, (0x03, sp)
      0082A2 F7               [ 1]  625 	ld	(x), a
                                    626 ;	periph_stm8s.c: 174: while((UART1_SR&(1<<UART1_SR_TXE))!=(1<<UART1_SR_TXE)); //Wait until Transmission complete
      0082A3                        627 00101$:
      0082A3 AE 52 30         [ 2]  628 	ldw	x, #0x5230
      0082A6 F6               [ 1]  629 	ld	a, (x)
      0082A7 A4 80            [ 1]  630 	and	a, #0x80
      0082A9 A1 80            [ 1]  631 	cp	a, #0x80
      0082AB 26 F6            [ 1]  632 	jrne	00101$
      0082AD 81               [ 4]  633 	ret
                                    634 ;	periph_stm8s.c: 177: unsigned char uart1_recv() //UART Receive a Byte (using Polling)
                                    635 ;	-----------------------------------------
                                    636 ;	 function uart1_recv
                                    637 ;	-----------------------------------------
      0082AE                        638 _uart1_recv:
                                    639 ;	periph_stm8s.c: 180: if((UART1_SR&(1<<UART1_SR_RXNE))==(1<<UART1_SR_RXNE)) //Check if any data in Data Register
      0082AE AE 52 30         [ 2]  640 	ldw	x, #0x5230
      0082B1 F6               [ 1]  641 	ld	a, (x)
      0082B2 A4 20            [ 1]  642 	and	a, #0x20
      0082B4 A1 20            [ 1]  643 	cp	a, #0x20
      0082B6 26 05            [ 1]  644 	jrne	00102$
                                    645 ;	periph_stm8s.c: 182: urecv = UART1_DR; //Read Data Register (RXNE cleared automatically)
      0082B8 AE 52 31         [ 2]  646 	ldw	x, #0x5231
      0082BB F6               [ 1]  647 	ld	a, (x)
                                    648 ;	periph_stm8s.c: 184: else urecv=0;
      0082BC 21                     649 	.byte 0x21
      0082BD                        650 00102$:
      0082BD 4F               [ 1]  651 	clr	a
      0082BE                        652 00103$:
                                    653 ;	periph_stm8s.c: 185: return urecv;
      0082BE 81               [ 4]  654 	ret
                                    655 ;	periph_stm8s.c: 188: unsigned char uart1_recv_i() //UART Receive a Byte (using Interrupt)
                                    656 ;	-----------------------------------------
                                    657 ;	 function uart1_recv_i
                                    658 ;	-----------------------------------------
      0082BF                        659 _uart1_recv_i:
                                    660 ;	periph_stm8s.c: 191: urecv = UART1_DR; //Read Data Register (RXNE cleared automatically)
      0082BF AE 52 31         [ 2]  661 	ldw	x, #0x5231
      0082C2 F6               [ 1]  662 	ld	a, (x)
                                    663 ;	periph_stm8s.c: 192: return urecv;
      0082C3 81               [ 4]  664 	ret
                                    665 ;	periph_stm8s.c: 198: void pwm1_init(unsigned int timval)
                                    666 ;	-----------------------------------------
                                    667 ;	 function pwm1_init
                                    668 ;	-----------------------------------------
      0082C4                        669 _pwm1_init:
      0082C4 52 02            [ 2]  670 	sub	sp, #2
                                    671 ;	periph_stm8s.c: 200: TIM1_PSCRH = 0x00; //TIM_CLK = CLK
      0082C6 35 00 52 60      [ 1]  672 	mov	0x5260+0, #0x00
                                    673 ;	periph_stm8s.c: 201: TIM1_PSCRL = 0x00; //TIM_CLK = CLK
      0082CA 35 00 52 61      [ 1]  674 	mov	0x5261+0, #0x00
                                    675 ;	periph_stm8s.c: 202: TIM1_ARRH = (timval >> 8); //TIM RELOAD
      0082CE 7B 05            [ 1]  676 	ld	a, (0x05, sp)
      0082D0 0F 01            [ 1]  677 	clr	(0x01, sp)
      0082D2 AE 52 62         [ 2]  678 	ldw	x, #0x5262
      0082D5 F7               [ 1]  679 	ld	(x), a
                                    680 ;	periph_stm8s.c: 203: TIM1_ARRL = (timval & 0x00FF); //TIM RELOAD
      0082D6 7B 06            [ 1]  681 	ld	a, (0x06, sp)
      0082D8 95               [ 1]  682 	ld	xh, a
      0082D9 4F               [ 1]  683 	clr	a
      0082DA 9E               [ 1]  684 	ld	a, xh
      0082DB AE 52 63         [ 2]  685 	ldw	x, #0x5263
      0082DE F7               [ 1]  686 	ld	(x), a
                                    687 ;	periph_stm8s.c: 204: pwm1ch1_enable();
      0082DF CD 83 3A         [ 4]  688 	call	_pwm1ch1_enable
                                    689 ;	periph_stm8s.c: 205: TIM1_CCER1 |= (0<<TIM1_CCER1_CC1P); //Output active high
      0082E2 AE 52 5C         [ 2]  690 	ldw	x, #0x525c
      0082E5 F6               [ 1]  691 	ld	a, (x)
      0082E6 AE 52 5C         [ 2]  692 	ldw	x, #0x525c
      0082E9 F7               [ 1]  693 	ld	(x), a
                                    694 ;	periph_stm8s.c: 206: TIM1_CCMR1 = (TIM1_OCxREF_PWM_mode1<<TIM1_CCMR1_OC1M); //PWM MODE 1 for Channel 1
      0082EA 35 60 52 58      [ 1]  695 	mov	0x5258+0, #0x60
                                    696 ;	periph_stm8s.c: 207: pwm1_update(0x0000); //Start Value
      0082EE 5F               [ 1]  697 	clrw	x
      0082EF 89               [ 2]  698 	pushw	x
      0082F0 CD 83 4E         [ 4]  699 	call	_pwm1_update
      0082F3 5B 02            [ 2]  700 	addw	sp, #2
                                    701 ;	periph_stm8s.c: 208: TIM1_BKR = (1<<TIM1_BKR_MOE); //ENABLE MAIN OUTPUT 
      0082F5 35 80 52 6D      [ 1]  702 	mov	0x526d+0, #0x80
                                    703 ;	periph_stm8s.c: 209: TIM1_CR1 |= (1<<TIM1_CR1_CEN); //ENABLE TIM
      0082F9 AE 52 50         [ 2]  704 	ldw	x, #0x5250
      0082FC F6               [ 1]  705 	ld	a, (x)
      0082FD AA 01            [ 1]  706 	or	a, #0x01
      0082FF F7               [ 1]  707 	ld	(x), a
      008300 5B 02            [ 2]  708 	addw	sp, #2
      008302 81               [ 4]  709 	ret
                                    710 ;	periph_stm8s.c: 212: void pwm2_init(unsigned int timval)
                                    711 ;	-----------------------------------------
                                    712 ;	 function pwm2_init
                                    713 ;	-----------------------------------------
      008303                        714 _pwm2_init:
      008303 52 02            [ 2]  715 	sub	sp, #2
                                    716 ;	periph_stm8s.c: 214: TIM2_PSCR = 0x00; //TIM_CLK = CLK
      008305 35 00 53 0E      [ 1]  717 	mov	0x530e+0, #0x00
                                    718 ;	periph_stm8s.c: 215: TIM2_ARRH = (timval >> 8); //TIM RELOAD
      008309 7B 05            [ 1]  719 	ld	a, (0x05, sp)
      00830B 0F 01            [ 1]  720 	clr	(0x01, sp)
      00830D AE 53 0F         [ 2]  721 	ldw	x, #0x530f
      008310 F7               [ 1]  722 	ld	(x), a
                                    723 ;	periph_stm8s.c: 216: TIM2_ARRL = (timval & 0x00FF); //TIM RELOAD
      008311 7B 06            [ 1]  724 	ld	a, (0x06, sp)
      008313 95               [ 1]  725 	ld	xh, a
      008314 4F               [ 1]  726 	clr	a
      008315 9E               [ 1]  727 	ld	a, xh
      008316 AE 53 10         [ 2]  728 	ldw	x, #0x5310
      008319 F7               [ 1]  729 	ld	(x), a
                                    730 ;	periph_stm8s.c: 217: pwm2ch1_enable();
      00831A CD 83 44         [ 4]  731 	call	_pwm2ch1_enable
                                    732 ;	periph_stm8s.c: 218: TIM2_CCER1 |= (0<<TIM2_CCER1_CC1P); //Output active high
      00831D AE 53 0A         [ 2]  733 	ldw	x, #0x530a
      008320 F6               [ 1]  734 	ld	a, (x)
      008321 AE 53 0A         [ 2]  735 	ldw	x, #0x530a
      008324 F7               [ 1]  736 	ld	(x), a
                                    737 ;	periph_stm8s.c: 219: TIM2_CCMR1 = (TIM2_OCxREF_PWM_mode1<<TIM2_CCMR1_OC1M); //PWM MODE 1 for Channel 1 
      008325 35 60 53 07      [ 1]  738 	mov	0x5307+0, #0x60
                                    739 ;	periph_stm8s.c: 220: pwm2_update(0x0000); //Start Value
      008329 5F               [ 1]  740 	clrw	x
      00832A 89               [ 2]  741 	pushw	x
      00832B CD 83 64         [ 4]  742 	call	_pwm2_update
      00832E 5B 02            [ 2]  743 	addw	sp, #2
                                    744 ;	periph_stm8s.c: 221: TIM2_CR1 |= (1<<TIM2_CR1_CEN); //ENABLE TIM
      008330 AE 53 00         [ 2]  745 	ldw	x, #0x5300
      008333 F6               [ 1]  746 	ld	a, (x)
      008334 AA 01            [ 1]  747 	or	a, #0x01
      008336 F7               [ 1]  748 	ld	(x), a
      008337 5B 02            [ 2]  749 	addw	sp, #2
      008339 81               [ 4]  750 	ret
                                    751 ;	periph_stm8s.c: 224: void pwm1ch1_enable()
                                    752 ;	-----------------------------------------
                                    753 ;	 function pwm1ch1_enable
                                    754 ;	-----------------------------------------
      00833A                        755 _pwm1ch1_enable:
                                    756 ;	periph_stm8s.c: 226: TIM1_CCER1 |= (1<<TIM1_CCER1_CC1E);
      00833A 72 10 52 5C      [ 1]  757 	bset	0x525c, #0
      00833E 81               [ 4]  758 	ret
                                    759 ;	periph_stm8s.c: 229: void pwm1ch1_disable()
                                    760 ;	-----------------------------------------
                                    761 ;	 function pwm1ch1_disable
                                    762 ;	-----------------------------------------
      00833F                        763 _pwm1ch1_disable:
                                    764 ;	periph_stm8s.c: 231: TIM1_CCER1 &= ~(1<<TIM1_CCER1_CC1E);
      00833F 72 11 52 5C      [ 1]  765 	bres	0x525c, #0
      008343 81               [ 4]  766 	ret
                                    767 ;	periph_stm8s.c: 234: void pwm2ch1_enable()
                                    768 ;	-----------------------------------------
                                    769 ;	 function pwm2ch1_enable
                                    770 ;	-----------------------------------------
      008344                        771 _pwm2ch1_enable:
                                    772 ;	periph_stm8s.c: 236: TIM2_CCER1 |= (1<<TIM2_CCER1_CC1E);
      008344 72 10 53 0A      [ 1]  773 	bset	0x530a, #0
      008348 81               [ 4]  774 	ret
                                    775 ;	periph_stm8s.c: 239: void pwm2ch1_disable()
                                    776 ;	-----------------------------------------
                                    777 ;	 function pwm2ch1_disable
                                    778 ;	-----------------------------------------
      008349                        779 _pwm2ch1_disable:
                                    780 ;	periph_stm8s.c: 241: TIM2_CCER1 &= ~(1<<TIM2_CCER1_CC1E);
      008349 72 11 53 0A      [ 1]  781 	bres	0x530a, #0
      00834D 81               [ 4]  782 	ret
                                    783 ;	periph_stm8s.c: 244: void pwm1_update(unsigned int pwmval)
                                    784 ;	-----------------------------------------
                                    785 ;	 function pwm1_update
                                    786 ;	-----------------------------------------
      00834E                        787 _pwm1_update:
      00834E 52 02            [ 2]  788 	sub	sp, #2
                                    789 ;	periph_stm8s.c: 246: TIM1_CCR1L = (pwmval & 0x00FF);
      008350 7B 06            [ 1]  790 	ld	a, (0x06, sp)
      008352 95               [ 1]  791 	ld	xh, a
      008353 4F               [ 1]  792 	clr	a
      008354 9E               [ 1]  793 	ld	a, xh
      008355 AE 52 66         [ 2]  794 	ldw	x, #0x5266
      008358 F7               [ 1]  795 	ld	(x), a
                                    796 ;	periph_stm8s.c: 247: TIM1_CCR1H = (pwmval >> 8);
      008359 7B 05            [ 1]  797 	ld	a, (0x05, sp)
      00835B 0F 01            [ 1]  798 	clr	(0x01, sp)
      00835D AE 52 65         [ 2]  799 	ldw	x, #0x5265
      008360 F7               [ 1]  800 	ld	(x), a
      008361 5B 02            [ 2]  801 	addw	sp, #2
      008363 81               [ 4]  802 	ret
                                    803 ;	periph_stm8s.c: 250: void pwm2_update(unsigned int pwmval)
                                    804 ;	-----------------------------------------
                                    805 ;	 function pwm2_update
                                    806 ;	-----------------------------------------
      008364                        807 _pwm2_update:
      008364 52 02            [ 2]  808 	sub	sp, #2
                                    809 ;	periph_stm8s.c: 252: TIM2_CCR1L = (pwmval & 0x00FF);
      008366 7B 06            [ 1]  810 	ld	a, (0x06, sp)
      008368 95               [ 1]  811 	ld	xh, a
      008369 4F               [ 1]  812 	clr	a
      00836A 9E               [ 1]  813 	ld	a, xh
      00836B AE 53 12         [ 2]  814 	ldw	x, #0x5312
      00836E F7               [ 1]  815 	ld	(x), a
                                    816 ;	periph_stm8s.c: 253: TIM2_CCR1H = (pwmval >> 8);
      00836F 7B 05            [ 1]  817 	ld	a, (0x05, sp)
      008371 0F 01            [ 1]  818 	clr	(0x01, sp)
      008373 AE 53 11         [ 2]  819 	ldw	x, #0x5311
      008376 F7               [ 1]  820 	ld	(x), a
      008377 5B 02            [ 2]  821 	addw	sp, #2
      008379 81               [ 4]  822 	ret
                                    823 ;	uart_func.c: 8: void UART_sendchar(unsigned char usend)
                                    824 ;	-----------------------------------------
                                    825 ;	 function UART_sendchar
                                    826 ;	-----------------------------------------
      00837A                        827 _UART_sendchar:
                                    828 ;	uart_func.c: 10: uart1_send(usend);
      00837A 7B 03            [ 1]  829 	ld	a, (0x03, sp)
      00837C 88               [ 1]  830 	push	a
      00837D CD 82 9D         [ 4]  831 	call	_uart1_send
      008380 84               [ 1]  832 	pop	a
      008381 81               [ 4]  833 	ret
                                    834 ;	uart_func.c: 13: void UART_sendtext(unsigned char *usend)
                                    835 ;	-----------------------------------------
                                    836 ;	 function UART_sendtext
                                    837 ;	-----------------------------------------
      008382                        838 _UART_sendtext:
      008382 52 02            [ 2]  839 	sub	sp, #2
                                    840 ;	uart_func.c: 17: while(usend[stridx] != 0) //scan characters in string
      008384 5F               [ 1]  841 	clrw	x
      008385 1F 01            [ 2]  842 	ldw	(0x01, sp), x
      008387                        843 00101$:
      008387 1E 05            [ 2]  844 	ldw	x, (0x05, sp)
      008389 72 FB 01         [ 2]  845 	addw	x, (0x01, sp)
      00838C F6               [ 1]  846 	ld	a, (x)
      00838D 4D               [ 1]  847 	tnz	a
      00838E 27 0C            [ 1]  848 	jreq	00104$
                                    849 ;	uart_func.c: 19: uart1_send(usend[stridx]); //print each character
      008390 88               [ 1]  850 	push	a
      008391 CD 82 9D         [ 4]  851 	call	_uart1_send
      008394 84               [ 1]  852 	pop	a
                                    853 ;	uart_func.c: 20: stridx++;
      008395 1E 01            [ 2]  854 	ldw	x, (0x01, sp)
      008397 5C               [ 2]  855 	incw	x
      008398 1F 01            [ 2]  856 	ldw	(0x01, sp), x
      00839A 20 EB            [ 2]  857 	jra	00101$
      00839C                        858 00104$:
      00839C 5B 02            [ 2]  859 	addw	sp, #2
      00839E 81               [ 4]  860 	ret
                                    861 ;	uart_func.c: 24: unsigned char UART_recvchar()
                                    862 ;	-----------------------------------------
                                    863 ;	 function UART_recvchar
                                    864 ;	-----------------------------------------
      00839F                        865 _UART_recvchar:
                                    866 ;	uart_func.c: 28: urecv = uart1_recv();
                                    867 ;	uart_func.c: 30: return urecv;
      00839F CC 82 AE         [ 2]  868 	jp	_uart1_recv
                                    869 ;	uart_func.c: 33: void UART_sendnum(unsigned int unum)
                                    870 ;	-----------------------------------------
                                    871 ;	 function UART_sendnum
                                    872 ;	-----------------------------------------
      0083A2                        873 _UART_sendnum:
      0083A2 52 0C            [ 2]  874 	sub	sp, #12
                                    875 ;	uart_func.c: 41: numb = unum;
      0083A4 1E 0F            [ 2]  876 	ldw	x, (0x0f, sp)
                                    877 ;	uart_func.c: 42: while(numb!=0)
      0083A6 4F               [ 1]  878 	clr	a
      0083A7                        879 00101$:
      0083A7 5D               [ 2]  880 	tnzw	x
      0083A8 27 08            [ 1]  881 	jreq	00114$
                                    882 ;	uart_func.c: 44: ndigit++;
      0083AA 4C               [ 1]  883 	inc	a
                                    884 ;	uart_func.c: 45: numb /= 10; //count decimal digit	
      0083AB 90 AE 00 0A      [ 2]  885 	ldw	y, #0x000a
      0083AF 65               [ 2]  886 	divw	x, y
      0083B0 20 F5            [ 2]  887 	jra	00101$
      0083B2                        888 00114$:
      0083B2 6B 09            [ 1]  889 	ld	(0x09, sp), a
                                    890 ;	uart_func.c: 47: for(nd=0;nd<ndigit;nd++)
      0083B4 4F               [ 1]  891 	clr	a
      0083B5 96               [ 1]  892 	ldw	x, sp
      0083B6 1C 00 03         [ 2]  893 	addw	x, #3
      0083B9 1F 0B            [ 2]  894 	ldw	(0x0b, sp), x
      0083BB                        895 00106$:
      0083BB 11 09            [ 1]  896 	cp	a, (0x09, sp)
      0083BD 24 27            [ 1]  897 	jrnc	00104$
                                    898 ;	uart_func.c: 49: numb = unum%10;
      0083BF 1E 0F            [ 2]  899 	ldw	x, (0x0f, sp)
      0083C1 90 AE 00 0A      [ 2]  900 	ldw	y, #0x000a
      0083C5 65               [ 2]  901 	divw	x, y
      0083C6 17 01            [ 2]  902 	ldw	(0x01, sp), y
                                    903 ;	uart_func.c: 50: unum = unum/10;
      0083C8 1E 0F            [ 2]  904 	ldw	x, (0x0f, sp)
      0083CA 90 AE 00 0A      [ 2]  905 	ldw	y, #0x000a
      0083CE 65               [ 2]  906 	divw	x, y
      0083CF 1F 0F            [ 2]  907 	ldw	(0x0f, sp), x
                                    908 ;	uart_func.c: 51: ibuff[ndigit-(nd+1)] = numb + '0'; //start from last_index-1
      0083D1 4C               [ 1]  909 	inc	a
      0083D2 6B 0A            [ 1]  910 	ld	(0x0a, sp), a
      0083D4 7B 09            [ 1]  911 	ld	a, (0x09, sp)
      0083D6 10 0A            [ 1]  912 	sub	a, (0x0a, sp)
      0083D8 5F               [ 1]  913 	clrw	x
      0083D9 97               [ 1]  914 	ld	xl, a
      0083DA 72 FB 0B         [ 2]  915 	addw	x, (0x0b, sp)
      0083DD 7B 02            [ 1]  916 	ld	a, (0x02, sp)
      0083DF AB 30            [ 1]  917 	add	a, #0x30
      0083E1 F7               [ 1]  918 	ld	(x), a
                                    919 ;	uart_func.c: 47: for(nd=0;nd<ndigit;nd++)
      0083E2 7B 0A            [ 1]  920 	ld	a, (0x0a, sp)
      0083E4 20 D5            [ 2]  921 	jra	00106$
      0083E6                        922 00104$:
                                    923 ;	uart_func.c: 53: ibuff[ndigit] = '\0'; //last character is null
      0083E6 5F               [ 1]  924 	clrw	x
      0083E7 7B 09            [ 1]  925 	ld	a, (0x09, sp)
      0083E9 97               [ 1]  926 	ld	xl, a
      0083EA 72 FB 0B         [ 2]  927 	addw	x, (0x0b, sp)
      0083ED 7F               [ 1]  928 	clr	(x)
                                    929 ;	uart_func.c: 55: UART_sendtext(ibuff);
      0083EE 1E 0B            [ 2]  930 	ldw	x, (0x0b, sp)
      0083F0 89               [ 2]  931 	pushw	x
      0083F1 CD 83 82         [ 4]  932 	call	_UART_sendtext
      0083F4 5B 0E            [ 2]  933 	addw	sp, #14
      0083F6 81               [ 4]  934 	ret
                                    935 ;	main.c: 41: void isr_UART1_RX() __interrupt UART_RX_INTERRUPT_VECTOR //ISR for UART Receiver Mode
                                    936 ;	-----------------------------------------
                                    937 ;	 function isr_UART1_RX
                                    938 ;	-----------------------------------------
      0083F7                        939 _isr_UART1_RX:
                                    940 ;	main.c: 43: cri = uart1_recv_i();	//receive 1 byte data
      0083F7 CD 82 BF         [ 4]  941 	call	_uart1_recv_i
                                    942 ;	main.c: 44: if((cri=='o')||(cri=='O')) LEDODR &= ~(1<<LED1); //LED ON (active low)
      0083FA C7 00 02         [ 1]  943 	ld	_cri+0, a
      0083FD A1 6F            [ 1]  944 	cp	a, #0x6f
      0083FF 27 07            [ 1]  945 	jreq	00104$
      008401 C6 00 02         [ 1]  946 	ld	a, _cri+0
      008404 A1 4F            [ 1]  947 	cp	a, #0x4f
      008406 26 09            [ 1]  948 	jrne	00105$
      008408                        949 00104$:
      008408 AE 50 05         [ 2]  950 	ldw	x, #0x5005
      00840B F6               [ 1]  951 	ld	a, (x)
      00840C A4 DF            [ 1]  952 	and	a, #0xdf
      00840E F7               [ 1]  953 	ld	(x), a
      00840F 20 15            [ 2]  954 	jra	00108$
      008411                        955 00105$:
                                    956 ;	main.c: 45: else if((cri=='x')||(cri=='X')) LEDODR |= 1<<LED1; //LED OFF (active low)
      008411 C6 00 02         [ 1]  957 	ld	a, _cri+0
      008414 A1 78            [ 1]  958 	cp	a, #0x78
      008416 27 07            [ 1]  959 	jreq	00101$
      008418 C6 00 02         [ 1]  960 	ld	a, _cri+0
      00841B A1 58            [ 1]  961 	cp	a, #0x58
      00841D 26 07            [ 1]  962 	jrne	00108$
      00841F                        963 00101$:
      00841F AE 50 05         [ 2]  964 	ldw	x, #0x5005
      008422 F6               [ 1]  965 	ld	a, (x)
      008423 AA 20            [ 1]  966 	or	a, #0x20
      008425 F7               [ 1]  967 	ld	(x), a
      008426                        968 00108$:
      008426 80               [11]  969 	iret
                                    970 ;	main.c: 53: int main()
                                    971 ;	-----------------------------------------
                                    972 ;	 function main
                                    973 ;	-----------------------------------------
      008427                        974 _main:
                                    975 ;	main.c: 55: clock_init();
      008427 CD 81 4F         [ 4]  976 	call	_clock_init
                                    977 ;	main.c: 56: delay_init();
      00842A CD 80 A0         [ 4]  978 	call	_delay_init
                                    979 ;	main.c: 57: gpio_init();
      00842D CD 84 BD         [ 4]  980 	call	_gpio_init
                                    981 ;	main.c: 58: adc_init();
      008430 CD 82 18         [ 4]  982 	call	_adc_init
                                    983 ;	main.c: 59: uart1_init(UART_RX_INTERRUPT_ENABLED); //UART RX using Interrupt
      008433 4B 01            [ 1]  984 	push	#0x01
      008435 CD 82 65         [ 4]  985 	call	_uart1_init
      008438 84               [ 1]  986 	pop	a
                                    987 ;	main.c: 61: cri = 0; //init value of UART RX buffer
      008439 72 5F 00 02      [ 1]  988 	clr	_cri+0
                                    989 ;	main.c: 63: enable_interrupts();
      00843D 9A               [ 1]  990 	rim 
                                    991 ;	main.c: 65: loop();
      00843E CD 84 43         [ 4]  992 	call	_loop
                                    993 ;	main.c: 66: return 0;
      008441 5F               [ 1]  994 	clrw	x
      008442 81               [ 4]  995 	ret
                                    996 ;	main.c: 71: void loop()
                                    997 ;	-----------------------------------------
                                    998 ;	 function loop
                                    999 ;	-----------------------------------------
      008443                       1000 _loop:
                                   1001 ;	main.c: 73: while(1)
      008443                       1002 00102$:
                                   1003 ;	main.c: 75: acc = read_adc(ACCX); //read Accelerometer X value
      008443 4B 03            [ 1] 1004 	push	#0x03
      008445 CD 82 25         [ 4] 1005 	call	_read_adc
      008448 84               [ 1] 1006 	pop	a
      008449 CF 00 03         [ 2] 1007 	ldw	_acc+0, x
                                   1008 ;	main.c: 76: UART_sendtext("| X = ");
      00844C AE 85 04         [ 2] 1009 	ldw	x, #___str_0+0
      00844F 89               [ 2] 1010 	pushw	x
      008450 CD 83 82         [ 4] 1011 	call	_UART_sendtext
      008453 5B 02            [ 2] 1012 	addw	sp, #2
                                   1013 ;	main.c: 77: UART_sendnum(acc); //send Accelerometer X value
      008455 3B 00 04         [ 1] 1014 	push	_acc+1
      008458 3B 00 03         [ 1] 1015 	push	_acc+0
      00845B CD 83 A2         [ 4] 1016 	call	_UART_sendnum
      00845E 5B 02            [ 2] 1017 	addw	sp, #2
                                   1018 ;	main.c: 79: acc = read_adc(ACCY); //read Accelerometer Y value
      008460 4B 04            [ 1] 1019 	push	#0x04
      008462 CD 82 25         [ 4] 1020 	call	_read_adc
      008465 84               [ 1] 1021 	pop	a
      008466 CF 00 03         [ 2] 1022 	ldw	_acc+0, x
                                   1023 ;	main.c: 80: UART_sendtext(" | Y = ");
      008469 AE 85 0B         [ 2] 1024 	ldw	x, #___str_1+0
      00846C 89               [ 2] 1025 	pushw	x
      00846D CD 83 82         [ 4] 1026 	call	_UART_sendtext
      008470 5B 02            [ 2] 1027 	addw	sp, #2
                                   1028 ;	main.c: 81: UART_sendnum(acc); //send Accelerometer Y value
      008472 3B 00 04         [ 1] 1029 	push	_acc+1
      008475 3B 00 03         [ 1] 1030 	push	_acc+0
      008478 CD 83 A2         [ 4] 1031 	call	_UART_sendnum
      00847B 5B 02            [ 2] 1032 	addw	sp, #2
                                   1033 ;	main.c: 83: acc = read_adc(ACCZ); //read Accelerometer Z value
      00847D 4B 02            [ 1] 1034 	push	#0x02
      00847F CD 82 25         [ 4] 1035 	call	_read_adc
      008482 84               [ 1] 1036 	pop	a
      008483 CF 00 03         [ 2] 1037 	ldw	_acc+0, x
                                   1038 ;	main.c: 84: UART_sendtext(" | Z = ");
      008486 AE 85 13         [ 2] 1039 	ldw	x, #___str_2+0
      008489 89               [ 2] 1040 	pushw	x
      00848A CD 83 82         [ 4] 1041 	call	_UART_sendtext
      00848D 5B 02            [ 2] 1042 	addw	sp, #2
                                   1043 ;	main.c: 85: UART_sendnum(acc); //send Accelerometer Z value
      00848F 3B 00 04         [ 1] 1044 	push	_acc+1
      008492 3B 00 03         [ 1] 1045 	push	_acc+0
      008495 CD 83 A2         [ 4] 1046 	call	_UART_sendnum
      008498 5B 02            [ 2] 1047 	addw	sp, #2
                                   1048 ;	main.c: 86: UART_sendtext(" |");
      00849A AE 85 1B         [ 2] 1049 	ldw	x, #___str_3+0
      00849D 89               [ 2] 1050 	pushw	x
      00849E CD 83 82         [ 4] 1051 	call	_UART_sendtext
      0084A1 5B 02            [ 2] 1052 	addw	sp, #2
                                   1053 ;	main.c: 89: UART_sendchar('\r'); //return
      0084A3 4B 0D            [ 1] 1054 	push	#0x0d
      0084A5 CD 83 7A         [ 4] 1055 	call	_UART_sendchar
      0084A8 84               [ 1] 1056 	pop	a
                                   1057 ;	main.c: 90: UART_sendchar('\n'); //newline
      0084A9 4B 0A            [ 1] 1058 	push	#0x0a
      0084AB CD 83 7A         [ 4] 1059 	call	_UART_sendchar
      0084AE 84               [ 1] 1060 	pop	a
                                   1061 ;	main.c: 92: delay_ms(500);
      0084AF 4B F4            [ 1] 1062 	push	#0xf4
      0084B1 4B 01            [ 1] 1063 	push	#0x01
      0084B3 5F               [ 1] 1064 	clrw	x
      0084B4 89               [ 2] 1065 	pushw	x
      0084B5 CD 80 F5         [ 4] 1066 	call	_delay_ms
      0084B8 5B 04            [ 2] 1067 	addw	sp, #4
      0084BA 20 87            [ 2] 1068 	jra	00102$
      0084BC 81               [ 4] 1069 	ret
                                   1070 ;	main.c: 99: void gpio_init()
                                   1071 ;	-----------------------------------------
                                   1072 ;	 function gpio_init
                                   1073 ;	-----------------------------------------
      0084BD                       1074 _gpio_init:
                                   1075 ;	main.c: 102: ACCXDDR |= (INPUT<<ACCX) | (INPUT<<ACCY);
      0084BD AE 50 11         [ 2] 1076 	ldw	x, #0x5011
      0084C0 F6               [ 1] 1077 	ld	a, (x)
      0084C1 AE 50 11         [ 2] 1078 	ldw	x, #0x5011
      0084C4 F7               [ 1] 1079 	ld	(x), a
                                   1080 ;	main.c: 103: ACCXCR1 |= (floating<<ACCX) | (floating<<ACCY);
      0084C5 AE 50 12         [ 2] 1081 	ldw	x, #0x5012
      0084C8 F6               [ 1] 1082 	ld	a, (x)
      0084C9 AE 50 12         [ 2] 1083 	ldw	x, #0x5012
      0084CC F7               [ 1] 1084 	ld	(x), a
                                   1085 ;	main.c: 104: ACCXCR2 |= (exti_disabled<<ACCX) | (exti_disabled<<ACCY);
      0084CD AE 50 13         [ 2] 1086 	ldw	x, #0x5013
      0084D0 F6               [ 1] 1087 	ld	a, (x)
      0084D1 AE 50 13         [ 2] 1088 	ldw	x, #0x5013
      0084D4 F7               [ 1] 1089 	ld	(x), a
                                   1090 ;	main.c: 106: ACCZDDR |= (INPUT<<ACCZ);
      0084D5 AE 50 0C         [ 2] 1091 	ldw	x, #0x500c
      0084D8 F6               [ 1] 1092 	ld	a, (x)
      0084D9 AE 50 0C         [ 2] 1093 	ldw	x, #0x500c
      0084DC F7               [ 1] 1094 	ld	(x), a
                                   1095 ;	main.c: 107: ACCZCR1 |= (floating<<ACCZ);
      0084DD AE 50 0D         [ 2] 1096 	ldw	x, #0x500d
      0084E0 F6               [ 1] 1097 	ld	a, (x)
      0084E1 AE 50 0D         [ 2] 1098 	ldw	x, #0x500d
      0084E4 F7               [ 1] 1099 	ld	(x), a
                                   1100 ;	main.c: 108: ACCZCR2 |= (exti_disabled<<ACCZ);
      0084E5 AE 50 0E         [ 2] 1101 	ldw	x, #0x500e
      0084E8 F6               [ 1] 1102 	ld	a, (x)
      0084E9 AE 50 0E         [ 2] 1103 	ldw	x, #0x500e
      0084EC F7               [ 1] 1104 	ld	(x), a
                                   1105 ;	main.c: 111: LEDDDR |= (OUTPUT<<LED1) | (OUTPUT<<LED1);
      0084ED AE 50 07         [ 2] 1106 	ldw	x, #0x5007
      0084F0 F6               [ 1] 1107 	ld	a, (x)
      0084F1 AA 20            [ 1] 1108 	or	a, #0x20
      0084F3 F7               [ 1] 1109 	ld	(x), a
                                   1110 ;	main.c: 112: LEDCR1 |= (pushpull<<LED1) | (pushpull<<LED1);
      0084F4 AE 50 08         [ 2] 1111 	ldw	x, #0x5008
      0084F7 F6               [ 1] 1112 	ld	a, (x)
      0084F8 AA 20            [ 1] 1113 	or	a, #0x20
      0084FA F7               [ 1] 1114 	ld	(x), a
                                   1115 ;	main.c: 113: LEDCR2 |= (speed_2MHz<<LED1) | (speed_2MHz<<LED1);
      0084FB AE 50 09         [ 2] 1116 	ldw	x, #0x5009
      0084FE F6               [ 1] 1117 	ld	a, (x)
      0084FF AE 50 09         [ 2] 1118 	ldw	x, #0x5009
      008502 F7               [ 1] 1119 	ld	(x), a
      008503 81               [ 4] 1120 	ret
                                   1121 	.area CODE
      008504                       1122 ___str_0:
      008504 7C 20 58 20 3D 20     1123 	.ascii "| X = "
      00850A 00                    1124 	.db 0x00
      00850B                       1125 ___str_1:
      00850B 20 7C 20 59 20 3D 20  1126 	.ascii " | Y = "
      008512 00                    1127 	.db 0x00
      008513                       1128 ___str_2:
      008513 20 7C 20 5A 20 3D 20  1129 	.ascii " | Z = "
      00851A 00                    1130 	.db 0x00
      00851B                       1131 ___str_3:
      00851B 20 7C                 1132 	.ascii " |"
      00851D 00                    1133 	.db 0x00
                                   1134 	.area INITIALIZER
                                   1135 	.area CABS (ABS)
