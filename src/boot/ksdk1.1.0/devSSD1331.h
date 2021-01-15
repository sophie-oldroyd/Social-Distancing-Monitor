#include <stdint.h>

#pragma once

/*
 *	Sets the screen settings
 *  Obtained from https://os.mbed.com/users/star297/code/ssd1331//file/4385fd242db0/ssd1331.cpp/
 */
#define width   96-1        // Max X axial direction in screen
#define height  64-1        // Max Y axial direction in screen
#define Set_Column_Address  0x15
#define Set_Row_Address     0x75
#define contrastA           0x81
#define contrastB           0x82
#define contrastC           0x83
#define display_on          0xAF
#define display_off         0xAE

/*
 *	Sets the internal font settings
 *  Obtained from https://os.mbed.com/users/star297/code/ssd1331//file/4385fd242db0/ssd1331.cpp/
 */
#define X_width 6
#define Y_height 16

/*
 *	Sets the GAC hardware acceleration commands
 *  Obtained from https://os.mbed.com/users/star297/code/ssd1331//file/4385fd242db0/ssd1331.cpp/
 */
#define GAC_DRAW_LINE           0x21    // Draw Line
#define GAC_FILL_ENABLE_DISABLE 0x26    // Enable Fill

/*
 *	Both enums are setting different commands for the OLED screen. 
 *  Obtained from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino 
 */

typedef enum
{
	kSSD1331ColororderRGB		= 1,
	kSSD1331DelaysHWFILL		= 3,
	kSSD1331DelaysHWLINE		= 1,
} SSD1331Constants;

typedef enum
{
	kSSD1331CommandDRAWLINE		= 0x21,
	kSSD1331CommandDRAWRECT		= 0x22,
	kSSD1331CommandCLEAR		= 0x25,
	kSSD1331CommandFILL		= 0x26,
	kSSD1331CommandSETCOLUMN	= 0x15,
	kSSD1331CommandSETROW		= 0x75,
	kSSD1331CommandCONTRASTA	= 0x81,
	kSSD1331CommandCONTRASTB	= 0x82,
	kSSD1331CommandCONTRASTC	= 0x83,
	kSSD1331CommandMASTERCURRENT	= 0x87,
	kSSD1331CommandSETREMAP		= 0xA0,
	kSSD1331CommandSTARTLINE	= 0xA1,
	kSSD1331CommandDISPLAYOFFSET	= 0xA2,
	kSSD1331CommandNORMALDISPLAY	= 0xA4,
	kSSD1331CommandDISPLAYALLON	= 0xA5,
	kSSD1331CommandDISPLAYALLOFF	= 0xA6,
	kSSD1331CommandINVERTDISPLAY	= 0xA7,
	kSSD1331CommandSETMULTIPLEX	= 0xA8,
	kSSD1331CommandSETMASTER	= 0xAD,
	kSSD1331CommandDISPLAYOFF	= 0xAE,
	kSSD1331CommandDISPLAYON	= 0xAF,
	kSSD1331CommandPOWERMODE	= 0xB0,
	kSSD1331CommandPRECHARGE	= 0xB1,
	kSSD1331CommandCLOCKDIV		= 0xB3,
	kSSD1331CommandPRECHARGEA	= 0x8A,
	kSSD1331CommandPRECHARGEB	= 0x8B,
	kSSD1331CommandPRECHARGEC	= 0x8C,
	kSSD1331CommandPRECHARGELEVEL	= 0xBB,
	kSSD1331CommandVCOMH		= 0xBE,
} SSD1331Commands;

/*
 *	Defining various variables and functions that are used within the devSSD1331.c file.
 *  Obtained from https://os.mbed.com/users/star297/code/ssd1331//file/4385fd242db0/ssd1331.cpp/ 
 * and https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino 
 */

int writeCommand(uint8_t commandByte);
int writeCommand_buf(uint8_t* commandByteBuf, uint8_t len);
int charactertoscreen(int character, uint8_t x);
void clearscreen(void);
int	devSSD1331init(void);
void line(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint16_t color); /* Draws line start x,y, end x,y, color */
void reset_cursor();
void PutChar(uint8_t x,uint8_t y,int a);
unsigned char* font;
uint16_t Char_Color;    /* Defines text colour */
uint16_t BGround_Color; /* Defines background colour */
/* Defines pixel location */
uint8_t _x;
uint8_t _y;
/* Defines window location */
uint8_t _x1;
uint8_t _x2;
uint8_t _y1;
uint8_t _y2;
uint8_t char_x;
uint8_t char_y;
uint8_t chr_size;
uint8_t cwidth;       /* Defines character's width */
uint8_t cvert;        /* Defines character's height */

