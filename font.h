//2020-11-27
//bonusoid

#ifndef __FONT_H
#define __FONT_H

const unsigned char font_arr[640] = {

	//OFFSET=0, Offset from Ascii = 32
	0x00,0x00,0x00,0x00,0x00, //SPACE	//0 (32)
	0x00,0x00,0x5F,0x00,0x00, //!
	0x05,0x03,0x00,0x05,0x03, //"
	0x14,0x7F,0x14,0x7F,0x14, //#
	0x24,0x2A,0x7F,0x2A,0x12, //$
	0x23,0x13,0x08,0x64,0x62, //%
	0x36,0x49,0x55,0x22,0x50, //&
	0x00,0x05,0x03,0x00,0x00, //'
	0x00,0x1C,0x22,0x41,0x00, //(
	0x00,0x41,0x22,0x1C,0x00, //)
	0x0A,0x04,0x1F,0x04,0x0A, //*
	0x08,0x08,0x3E,0x08,0x08, //+
	0x00,0x50,0x30,0x00,0x00, //,
	0x08,0x08,0x08,0x08,0x08, //-
	0x00,0x60,0x60,0x00,0x00, //.
	0x20,0x10,0x08,0x04,0x02, ///
	0x3E,0x51,0x49,0x45,0x3E, //0
	0x00,0x42,0x7F,0x40,0x00, //1
	0x42,0x61,0x51,0x49,0x46, //2
	0x22,0x41,0x49,0x49,0x36, //3
	0x18,0x14,0x12,0x7F,0x10, //4
	0x27,0x45,0x45,0x45,0x39, //5
	0x3E,0x49,0x49,0x49,0x32, //6
	0x61,0x11,0x09,0x05,0x03, //7
	0x36,0x49,0x49,0x49,0x36, //8
	0x26,0x49,0x49,0x49,0x3E, //9
	0x00,0x36,0x36,0x00,0x00, //:
	0x00,0x56,0x36,0x00,0x00, //;
	0x00,0x08,0x14,0x22,0x00, //<
	0x14,0x14,0x14,0x14,0x14, //=
	0x00,0x22,0x14,0x08,0x00, //>
	0x02,0x01,0x51,0x09,0x06, //?		//31 (63)

	0x32,0x49,0x79,0x41,0x3E, //@		//32 (64)
	0x7C,0x12,0x11,0x12,0x7C, //A
	0x7F,0x49,0x49,0x49,0x36, //B
	0x3E,0x41,0x41,0x41,0x22, //C
	0x7F,0x41,0x41,0x22,0x1C, //D
	0x7F,0x49,0x49,0x49,0x49, //E
	0x7F,0x09,0x09,0x09,0x09, //F
	0x3E,0x41,0x49,0x49,0x3A, //G
	0x7F,0x08,0x08,0x08,0x7F, //H
	0x00,0x41,0x7F,0x41,0x00, //I
	0x20,0x40,0x41,0x3F,0x01, //J
	0x7F,0x08,0x14,0x22,0x41, //K
	0x7F,0x40,0x40,0x40,0x40, //L
	0x7F,0x02,0x0C,0x02,0x7F, //M
	0x7F,0x04,0x08,0x10,0x7F, //N
	0x3E,0x41,0x41,0x41,0x3E, //O
	0x7F,0x09,0x09,0x09,0x06, //P
	0x3E,0x41,0x51,0x21,0x5E, //Q
	0x7F,0x09,0x19,0x29,0x46, //R
	0x26,0x49,0x49,0x49,0x32, //S
	0x01,0x01,0x7F,0x01,0x01, //T
	0x3F,0x40,0x40,0x40,0x3F, //U
	0x1F,0x20,0x40,0x20,0x1F, //V
	0x3F,0x40,0x38,0x40,0x3F, //W
	0x63,0x14,0x08,0x14,0x63, //X
	0x07,0x08,0x70,0x08,0x07, //Y
	0x61,0x51,0x49,0x45,0x43, //Z
	0x00,0x7F,0x41,0x41,0x00, //[
	0x02,0x04,0x08,0x10,0x20, //(\)
	0x00,0x41,0x41,0x7F,0x00, //]
	0x04,0x02,0x01,0x02,0x04, //^
	0x40,0x40,0x40,0x40,0x40, //_		//63 (95)	

	0x00,0x01,0x02,0x04,0x00, //`		//64 (96)
	0x20,0x54,0x54,0x54,0x78, //a
	0x7F,0x50,0x48,0x48,0x30, //b
	0x38,0x44,0x44,0x44,0x28, //c
	0x30,0x48,0x48,0x50,0x7F, //d
	0x38,0x54,0x54,0x54,0x18, //e
	0x08,0x7E,0x09,0x09,0x02, //f
	0x08,0x54,0x54,0x54,0x3C, //g
	0x7F,0x10,0x08,0x08,0x70, //h
	0x00,0x48,0x7A,0x40,0x00, //i
	0x20,0x40,0x48,0x3A,0x00, //j
	0x7F,0x10,0x28,0x44,0x00, //k
	0x00,0x41,0x7F,0x40,0x00, //l
	0x7C,0x04,0x7C,0x04,0x78, //m
	0x7C,0x08,0x04,0x04,0x78, //n
	0x38,0x44,0x44,0x44,0x38, //o
	0x7C,0x14,0x14,0x14,0x08, //p
	0x08,0x14,0x14,0x18,0x7C, //q
	0x7C,0x08,0x04,0x04,0x08, //r
	0x48,0x54,0x54,0x54,0x20, //s
	0x04,0x3F,0x44,0x44,0x20, //t
	0x3C,0x40,0x40,0x20,0x7C, //u
	0x1C,0x20,0x40,0x20,0x1C, //v
	0x3C,0x40,0x38,0x40,0x3C, //w
	0x44,0x28,0x10,0x28,0x44, //x
	0x0C,0x50,0x50,0x50,0x3C, //y
	0x44,0x64,0x54,0x4C,0x44, //z
	0x00,0x08,0x36,0x41,0x00, //{
	0x00,0x00,0x7F,0x00,0x00, //|
	0x00,0x41,0x36,0x08,0x00, //}
	0x10,0x08,0x08,0x10,0x08, //~
	0x06,0x09,0x09,0x06,0x00, //degree	//95 (127)

	//OFFSET=480, Offset from Ascii = 128
	0x00,0x00,0x00,0xF8,0xF8,0x18,0x18,0x18, //frame : top-left	//0 (96) (128)
	0x18,0x18,0x18,0xF8,0xF8,0x18,0x18,0x18, //frame : top
	0x18,0x18,0x18,0xF8,0xF8,0x00,0x00,0x00, //frame : top-right
	0x00,0x00,0x00,0xFF,0xFF,0x18,0x18,0x18, //frame : mid-left
	0x18,0x18,0x18,0xFF,0xFF,0x18,0x18,0x18, //frame : center
 	0x18,0x18,0x18,0xFF,0xFF,0x00,0x00,0x00, //frame : mid-right
	0x00,0x00,0x00,0x1F,0x1F,0x18,0x18,0x18, //frame : bot-left
	0x18,0x18,0x18,0x1F,0x1F,0x18,0x18,0x18, //frame : bot
	0x18,0x18,0x18,0x1F,0x1F,0x00,0x00,0x00, //frame : bot-right
	0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18, //frame : horizontal
	0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00, //frame : vertical	//10 (106) (138)

	0x18,0x0C,0x06,0xFF,0xFF,0x06,0x0C,0x18, //arrow : up		//11 (107) (139)
	0x18,0x30,0x60,0xFF,0xFF,0x60,0x30,0x18, //arrow : down		
	0x18,0x3C,0x7E,0xDB,0x99,0x18,0x18,0x18, //arrow : left
	0x18,0x18,0x18,0x99,0xDB,0x7E,0x3C,0x18, //arrow : right
	0x7F,0x7F,0x0F,0x1F,0x3B,0x73,0xE3,0x40, //arrow : up-left
	0x40,0xE3,0x73,0x3B,0x1F,0x0F,0x7F,0x7F, //arrow : up-right
	0xFE,0xFE,0xF0,0xF8,0xDC,0xCE,0xC7,0x02, //arrow : down-left
	0x02,0xC7,0xCE,0xDC,0xF8,0xF0,0xFE,0xFE, //arrow : down-right
	0x3C,0x42,0x81,0x99,0x99,0x81,0x42,0x3C, //arrow : point	//19 (115) (147)
};

#endif
