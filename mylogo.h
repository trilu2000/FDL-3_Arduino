/*- -----------------------------------------------------------------------------------------------------------------------
*  FDL-3 arduino implementation
*  2018-01-17 <trilu@gmx.de> Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
* - -----------------------------------------------------------------------------------------------------------------------
* - the logo for the 0.9inch oled display ---------------------------------------------------------------------------------
*   special thanks to Jesse Kovarovics http://www.projectfdl.com to make this happen
* - -----------------------------------------------------------------------------------------------------------------------
*/

#ifndef _MYLOGO_h
#define _MYLOGO_h

static const uint8_t testbmp_bits[] PROGMEM = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,	0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0xF8, 0x7F, 0xFC, 0x0F, 0x70,
	0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0xF8,	0x7F, 0xFC, 0x3F, 0x70, 0x00, 0x00, 0x00, 0xFE, 0x01, 0x00, 0x00, 0x80,
	0x01, 0x00, 0x00, 0xF8, 0x7F, 0xFC, 0x3F, 0x70, 0x00, 0x00, 0x00, 0xFF,	0x03, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x38, 0x00, 0x1C, 0x78, 0x70,
	0x00, 0x00, 0x00, 0x87, 0x03, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x38,	0x00, 0x1C, 0x70, 0x70, 0x00, 0x00, 0x00, 0x83, 0x03, 0x00, 0x00, 0x80,
	0x01, 0x00, 0x00, 0x38, 0x00, 0x1C, 0xE0, 0x70, 0x00, 0x00, 0x00, 0x80,	0x03, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x38, 0x00, 0x1C, 0xE0, 0x70,
	0x00, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x38,	0x00, 0x1C, 0xE0, 0x70, 0x00, 0x00, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x80,
	0x01, 0x00, 0x00, 0xF8, 0x3F, 0x1C, 0xE0, 0x70, 0x00, 0x00, 0x00, 0x70,	0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0xF8, 0x3F, 0x1C, 0xE0, 0x70,
	0x00, 0x00, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0xF8,	0x3F, 0x1C, 0xE0, 0x70, 0x00, 0xF8, 0x0F, 0x80, 0x03, 0x00, 0x00, 0x80,
	0x01, 0x00, 0x00, 0x38, 0x00, 0x1C, 0xE0, 0x70, 0x00, 0xF8, 0x0F, 0x00,	0x03, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x38, 0x00, 0x1C, 0xE0, 0x70,
	0x00, 0xF8, 0x0F, 0x00, 0x03, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x38,	0x00, 0x1C, 0xE0, 0x70, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x80,
	0x01, 0x00, 0x00, 0x38, 0x00, 0x1C, 0x70, 0x70, 0x00, 0x00, 0x80, 0x03,	0x03, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x38, 0x00, 0x1C, 0x78, 0x70,
	0x00, 0x00, 0x00, 0x87, 0x03, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x38,	0x00, 0xFC, 0x3F, 0xF0, 0xFF, 0x01, 0x00, 0xFF, 0x03, 0x00, 0x00, 0x80,
	0x01, 0x00, 0x00, 0x38, 0x00, 0xFC, 0x3F, 0xF0, 0xFF, 0x01, 0x00, 0xFE,	0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x38, 0x00, 0xFC, 0x0F, 0xF0,
	0xFF, 0x01, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x20, 0x00, 0x40,	0x00, 0x02, 0x00, 0x00, 0x06, 0x8C, 0x00, 0x08, 0xF8, 0x00, 0x00, 0x80,
	0x01, 0x20, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x06, 0x0C, 0x00, 0x00,	0x88, 0x01, 0x00, 0x80, 0x01, 0x50, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00,
	0x0A, 0x0A, 0x00, 0x00, 0x08, 0x01, 0x00, 0x80, 0x01, 0x50, 0x68, 0x5C,	0x42, 0xA2, 0x83, 0x03, 0x0A, 0x8A, 0xE8, 0x08, 0x08, 0x69, 0x38, 0x80,
	0x01, 0x88, 0x18, 0x62, 0x42, 0x62, 0x44, 0x04, 0x12, 0x89, 0x18, 0x09,	0x88, 0x19, 0x44, 0x80, 0x01, 0x88, 0x08, 0x42, 0x42, 0x22, 0x24, 0x08,
	0x12, 0x89, 0x08, 0x09, 0xF8, 0x08, 0x82, 0x80, 0x01, 0xF8, 0x08, 0x42,	0x42, 0x22, 0x24, 0x08, 0xA2, 0x88, 0x08, 0x09, 0x08, 0x08, 0x82, 0x80,
	0x01, 0x04, 0x09, 0x42, 0x42, 0x22, 0x24, 0x08, 0xA2, 0x88, 0x08, 0x09,	0x08, 0x08, 0x82, 0x80, 0x01, 0x04, 0x09, 0x62, 0x62, 0x22, 0x44, 0x04,
	0xA2, 0x88, 0x08, 0x09, 0x08, 0x08, 0x44, 0x80, 0x01, 0x02, 0x0A, 0x5C,	0x5C, 0x22, 0x84, 0x03, 0x42, 0x88, 0x08, 0x09, 0x08, 0x08, 0x38, 0x80,
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
	0x01, 0x00, 0x00, 0x20, 0x08, 0x00, 0x00, 0x02, 0x00, 0x00, 0x18, 0x80,	0x03, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x20, 0x08, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x16, 0x40, 0x04, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x20,	0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x20, 0x08, 0x00, 0x00, 0x80,
	0x01, 0x00, 0x00, 0x40, 0xC4, 0xA3, 0x71, 0x82, 0xA3, 0x03, 0x10, 0x20,	0x08, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x40, 0x24, 0x64, 0x88, 0x42,
	0x64, 0x04, 0x10, 0x20, 0x08, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x40,	0x24, 0x24, 0x08, 0x22, 0x28, 0x04, 0x10, 0x20, 0x08, 0x00, 0x00, 0x80,
	0x01, 0x00, 0x00, 0x80, 0xE2, 0x27, 0x70, 0x22, 0x28, 0x04, 0x10, 0x20,	0x08, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x22, 0x20, 0x80, 0x22,
	0x28, 0x04, 0x10, 0x20, 0x08, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80,	0x22, 0x24, 0x88, 0x42, 0x24, 0x04, 0x10, 0x42, 0x04, 0x00, 0x00, 0x80,
	0x01, 0x00, 0x00, 0x00, 0xC1, 0x23, 0x70, 0x82, 0x23, 0x04, 0x7E, 0x82,	0x03, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,	0xFF, 0xFF, 0xFF, 0xFF, };

static const uint8_t old_testbmp_bits[] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x0f, 0x00, 0x10, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
	0x00, 0x04, 0x04, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0xc4, 0x84, 0x0f, 0x10, 0xe0, 0x0f, 0x1f,
	0xff, 0xc3, 0x07, 0x00, 0xfc, 0x83, 0x0f, 0x00, 0x00, 0xa4, 0x44, 0x10, 0x10, 0x10, 0x88, 0x20, 0x66, 0x22, 0x08, 0x00, 0x10, 0x40, 0x10, 0x00,
	0x00, 0xa4, 0x24, 0x20, 0x10, 0x10, 0x48, 0x40, 0x22, 0x12, 0x10, 0x00, 0x10, 0x20, 0x20, 0x00, 0x00, 0xa8, 0xe4, 0x3f, 0x10, 0x10, 0x40, 0x40,
	0x22, 0xf2, 0x1f, 0x00, 0x10, 0x20, 0x20, 0x00, 0x00, 0x28, 0x23, 0x00, 0x10, 0x10, 0x40, 0x40, 0x22, 0x12, 0x00, 0x00, 0x10, 0x20, 0x20, 0x00,
	0x00, 0x18, 0x23, 0x00, 0x10, 0x10, 0x40, 0x40, 0x22, 0x12, 0x00, 0x00, 0x10, 0x20, 0x20, 0x00, 0x00, 0x18, 0x43, 0x20, 0x10, 0x10, 0x88, 0x20,
	0x22, 0x22, 0x10, 0x00, 0x10, 0x40, 0x10, 0x00, 0x00, 0x18, 0x82, 0x1f, 0xfe, 0xe1, 0x07, 0x1f, 0x67, 0xc6, 0x0f, 0x00, 0xe0, 0x87, 0x0f, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0xfc, 0xff, 0x7f, 0xf0, 0x07, 0x00, 0x80, 0x7f, 0x78, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xf0, 0x07,
	0x00, 0x80, 0xe3, 0x78, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0xc3, 0xc1, 0x01, 0x00, 0x80, 0xc1, 0x70, 0x1c, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x30, 0x30, 0x83, 0xc3, 0x01, 0x00, 0x80, 0xc0, 0xe0, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x33, 0x03, 0xc3, 0x01,
	0x00, 0x00, 0xe0, 0xc0, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x03, 0x03, 0xc3, 0x01, 0x00, 0x00, 0x70, 0xc0, 0x07, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0xf0, 0x03, 0x03, 0xc3, 0x01, 0xfc, 0x07, 0x38, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x03, 0x03, 0xc3, 0x41,
	0xfc, 0x07, 0x1c, 0xc0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x03, 0x03, 0xc3, 0xe1, 0x00, 0x00, 0x0e, 0xe0, 0x0e, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x30, 0x00, 0x83, 0xc3, 0xe1, 0x00, 0x00, 0x07, 0x60, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0xc3, 0xc1, 0xe1,
	0x00, 0xc0, 0x03, 0x70, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xc3, 0xff, 0xf0, 0xff, 0x00, 0xc0, 0xff, 0xfc, 0x7c, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0xfc, 0xc3, 0x7f, 0xf0, 0xff, 0x00, 0xc0, 0xff, 0x7c, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x18, 0x07, 0x00,
	0x00, 0x10, 0x00, 0x20, 0x00, 0x00, 0x60, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x18, 0x04, 0x00, 0x60, 0x10, 0x06, 0x21, 0x10, 0x06, 0x01, 0x10,
	0x06, 0x50, 0x00, 0x80, 0x40, 0x00, 0x04, 0x00, 0xb0, 0x10, 0xcb, 0xfb, 0x3c, 0xcf, 0x73, 0x3c, 0x0b, 0xf0, 0x64, 0xe0, 0xf3, 0x1c, 0x44, 0x02,
	0x90, 0x10, 0x49, 0x20, 0xe4, 0x59, 0x42, 0x04, 0x03, 0x90, 0x25, 0x80, 0x90, 0x11, 0x44, 0x02, 0xf0, 0x10, 0x4f, 0x20, 0x84, 0x59, 0x42, 0x04,
	0x0e, 0x90, 0x3d, 0x80, 0x10, 0x10, 0x44, 0x02, 0x10, 0x10, 0xc1, 0x20, 0x04, 0x49, 0x42, 0x0c, 0x08, 0x90, 0x18, 0x80, 0x10, 0x10, 0x44, 0x02,
	0xf0, 0x3c, 0x8f, 0xe3, 0x04, 0x4f, 0xf2, 0x38, 0x0f, 0xf0, 0x18, 0x80, 0x13, 0x3c, 0xcf, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

#endif
