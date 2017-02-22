///***************************************************************************/
///**
// * File: font.c
// *
// * Description: Characters display function.
// *
// ******************************************************************************/
//
//#include "main.h"
//#include <stdint.h>
//#include "font.h"
//
//const I8U font_content[] = {
//
//0x00,0xf0,0x08,0x04,0x04,0x08,0xf0,0x00,0x00,0x0f,0x10,0x20,0x20,0x10,0x0f,0x00, /*0*/
//0x00,0x00,0x10,0x08,0xfc,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0x00,0x00,0x00, /*1*/
//0x10,0x08,0x04,0x04,0x04,0x88,0x70,0x00,0x30,0x28,0x24,0x22,0x21,0x20,0x20,0x00, /*2*/
//0x08,0x04,0x04,0x84,0x84,0x48,0x30,0x00,0x10,0x20,0x20,0x20,0x20,0x11,0x0e,0x00, /*3*/
//0x00,0x80,0x60,0x18,0xc4,0x00,0x00,0x00,0x06,0x05,0x04,0x04,0x3f,0x04,0x04,0x00, /*4*/
//0x00,0xfc,0x44,0x44,0x44,0x84,0x04,0x00,0x00,0x10,0x20,0x20,0x20,0x10,0x0f,0x00, /*5*/
//0x80,0x60,0x90,0x88,0x84,0x00,0x00,0x00,0x0f,0x11,0x20,0x20,0x20,0x11,0x0e,0x00, /*6*/
//0x04,0x04,0x04,0x04,0xc4,0x34,0x0c,0x00,0x00,0x00,0x00,0x3e,0x01,0x00,0x00,0x00, /*7*/
//0x30,0x48,0x84,0x84,0x84,0x48,0x30,0x00,0x0e,0x11,0x20,0x20,0x20,0x11,0x0e,0x00, /*8*/
//0x70,0x88,0x04,0x04,0x04,0x88,0xf0,0x00,0x00,0x00,0x21,0x11,0x09,0x06,0x01,0x00, /*9*/
//0x3c,0x42,0x42,0xbc,0x60,0x18,0x06,0x00,0x60,0x18,0x06,0x3d,0x42,0x42,0x3c,0x00, /*%*/
//};
//
///**@brief Function for string encoding conversion.
// *
// * Description: UTF-8 code(3 B) converted to unicode code (2 B).
// */
//static I16U _encode_conversion_pro(I8U *utf_8)
//{
//	I16U unicode;
//
//	unicode  = (utf_8[0] & 0x0F) << 12;
//	unicode |= (utf_8[1] & 0x3F) << 6;
//	unicode |= (utf_8[2] & 0x3F);
//
//	return unicode;
//}
//
///**@brief Function for reading font.
// *
// * Description: Get one 5x7 size ascii string OLED dot matrix data(8 B).
// */
//static void _font_read_one_5x7_ascii(I8U ASCII_code, I16U len, I8U *dataBuf)
//{
//	I32U addr_in = FONT_ASCII_5X7_SPACE_START;;
//
//	addr_in += ((ASCII_code - 0x20) << 3);
//
//	NOR_readData(addr_in, len, dataBuf);
//}
//
///**@brief Function for reading font.
// *
// * Description: Get one 8x16 size ascii string OLED dot matrix data(16 B).
// */
//static void _font_read_one_8x16_ascii(I8U ASCII_code, I16U len, I8U *dataBuf)
//{
//	I32U addr_in = FONT_ASCII_8X16_SPACE_START;;
//
//	addr_in += ((ASCII_code - 0x20) << 4);
//
//	NOR_readData(addr_in, len, dataBuf);
//}
//
///**@brief Function for reading font.
// *
// * Description: Get one 15x16 size Chinese characters OLED dot matrix data(32 B).
// */
//static void _font_read_one_Chinese_characters(I8U *utf_8, I16U len, I8U *dataBuf)
//{
//	I32U addr_in = FONT_CHINESE_15X16_SPACE_START;
//  I16U unicode;
//	I8U  data[16];
//
//	unicode = _encode_conversion_pro(utf_8);
//
//	if(unicode < 0x4E00 || unicode > 0x9FA5)  {
//
//    N_SPRINTF("[FONTS] No search to the simple Chinese characters.");
//
//		memset(dataBuf, 0, 32);
//
//		// Use succedaneous ascii characters (' " ')in here.
//		_font_read_one_8x16_ascii('"',16,data);
//
//		memcpy(dataBuf+4,data,8);
//		memcpy(dataBuf+20,&data[8],8);
//
//    return;
//	}
//
//	addr_in += ((unicode - 0x4E00) << 5);
//	NOR_readData(addr_in, len, dataBuf);
//}
//
///**@brief Function for loading characters.
// *
// * Description: get OLED dot matrix data of a continuous string.
// *
// * height is 8: only need display 5x7 ascii sting.
// *
// * height is 16: display 8x16 ascii sting and 15x16 chinese characters.
// *
// * b_center : display in the middle(TRUE),display from the top(FALSE).
// */
//void FONT_load_characters(I8U *p_in, char *string, I8U height, I8U horizontal_len, BOOLEAN b_center)
//{
//	I8U  font_data[32];
//	I8U  *p0, *p1, *p2, *p3;
// 	I8U  pos=0, offset=0 ,curr_line_pos=0;
//	I8U  ptr, i;
//
//	p0 = p_in;
//
//	// When OTA is in progress,do not operate nflash.
//  if (OTA_if_enabled())
//    return;
//
//  // At present,can only display 128 bytes size string.
//	while(pos < 128)
//	{
//		if(string[pos] == '\0')
//		  break;
//
//	  if((string[pos] >= 0x20) && (string[pos] <= 0x7e)) {
//      // ASCII code encode format.
//			if (height == 8) {
//			  _font_read_one_5x7_ascii(string[pos], 6, font_data);
//        memcpy(p0 + offset, font_data, 6);
//				offset += 6;
//		  } else if (height == 16) {
//				if(offset > (horizontal_len - 8)){
//				  b_center = FALSE;
//					curr_line_pos++;
//					if(curr_line_pos >= 2)
//            return;
//          p0 += 256;
//          offset = 0;
//		    }
//				_font_read_one_8x16_ascii(string[pos], 16, font_data);
//				memcpy(p0 + offset, font_data, 8);
//			  memcpy(p0 + offset + 128, &font_data[8], 8);
//				offset += 8;
//		  } else 	{
//			}
//
//		  pos += 1;
//	  } else if (((string[pos]&0xF0) == 0xE0) && ((string[pos+1]&0xC0) == 0x80) && ((string[pos+2]&0xC0) == 0x80)) {
//			// Chinese characters Utf-8 encode format.
//			if(offset > (horizontal_len - 16)){
//				b_center = FALSE;
//				curr_line_pos++;
//				if(curr_line_pos >= 2)
//          return;
//				p0 += 256;
//				offset = 0;
//			}
//		  _font_read_one_Chinese_characters((I8U *)(string+pos), 32, font_data);
//		  memcpy(p0 + offset, font_data, 16);
//      memcpy(p0 + offset + 128, &font_data[16], 16);
//			offset += 16;
//		  pos += 3;
//
//	  } else {
//			// Is not within the scope of the can display,continue to read the next.
//			pos++;
//		}
//	}
//
//	// Shift all the display to the middle
//	if (b_center) {
//		p0 = p_in;
//		p1 = p0+128;
//		ptr = (128 - offset)>>1;
//
//		if (ptr > 0) {
//			if (height == 8) {
//				p0 += 127; p2 = p0 - ptr;
//			  for (i = 0; i < 128-ptr; i++) {
//				  *p0-- = *p2--;
//			  }
//			  for (; i < 128; i++) {
//				  *p0-- = 0;
//			  }
//			} else {
//		   	p0 += 127; p2 = p0 - ptr;
//			  p1 += 127; p3 = p1 - ptr;
//			  for (i = 0; i < 128-ptr; i++) {
//				  *p0-- = *p2--;
//				  *p1-- = *p3--;
//			  }
//			  for (; i < 128; i++) {
//				  *p0-- = 0;
//				  *p1-- = 0;
//			  }
//			}
//		}
//	}
//}
//
///**@brief Function for loading ota percent characters.
// *
// * Description: In order to solve the problem of resource conflicts.
// *              When OTA is in progress,do not operate nflash.
// * 1: Height is 16: only need display 8x16 ascii sting.
// * 2: Display in the middle.
// */
//void FONT_load_ota_percent(I8U *ptr, I8U percent)
//{
//  I8U	ten, single;
//	I8U pos_1, pos_2;
//  I8U *p0;
//	const I8U *pin;
//
//  p0 = ptr;
//	pin = font_content;
//
//	if(percent >= 99)
//		percent = 99;
//
//	if(percent <= 9) {
//		pos_1 = percent*16;
//		memcpy(p0 + 52, pin + pos_1, 8);
//		memcpy(p0 + 180, pin + pos_1 +8, 8);
//		memset(p0 + 60, 0, 8);
//		memset(p0 + 188, 0, 8);
//		memcpy(p0 + 68, pin + 160, 8);
//		memcpy(p0 + 196, pin + 168, 8);
//	} else {
//		ten = percent/10;
//		single = percent%10;
//		pos_1 = ten*16;
//		pos_2 = single*16;
//		memcpy(p0 + 48,pin + pos_1, 8);
//		memcpy(p0 + 176, pin + pos_1 +8, 8);
//		memcpy(p0 + 56, pin + pos_2, 8);
//		memcpy(p0 + 184, pin + pos_2 +8, 8);
//		memset(p0 + 64, 0, 8);
//		memset(p0 + 192, 0, 8);
//		memcpy(p0 + 72, pin + 160, 8);
//		memcpy(p0 + 200, pin + 168, 8);
//	}
//}
//
//I8U FONT_get_string_display_depth(char *string)
//{
//	I8U  curr_offset=0, curr_line_pos=0;
//	I8U  pos=0;
//  I8U  sting_index=0;
//
//	while(pos < 128)
//	{
//		if(string[pos] == '\0')
//		  break;
//
//		if((string[pos] >= 0x20)&&(string[pos] <= 0x7e)) {
//		  // ASCII code encode format.
//			if(curr_offset >= 112) {
//				curr_offset = 0;
//				curr_line_pos++;
//				if ((curr_line_pos%2) == 0) {
//			    cling.ui.string_pos_buf[sting_index++] = pos;
//		    }
//		  }
//      curr_offset += 8;
//			pos += 1;
//		} else if (((string[pos]&0xF0) == 0xE0)&&((string[pos+1]&0xC0) == 0x80)&&((string[pos+2]&0xC0) == 0x80)) {
//		  // Chinese characters Utf-8 encode format.
//			if(curr_offset >= 104) {
//				curr_offset = 0;
//				curr_line_pos++;
//				if ((curr_line_pos%2) == 0) {
//			    cling.ui.string_pos_buf[sting_index++] = pos;
//		    }
//		  }
//			curr_offset += 16;
//			pos += 3;
//		}
//		else {
//			// Is not within the scope of the can display,continue to read the next.
//			pos++;
//		}
//	}
//
//	if (curr_offset > 0)
//		curr_line_pos ++;
//
//	return ((curr_line_pos+1)>>1);
//}
//
//I16U FONT_get_string_display_len(char *string)
//{
//	I16U  offset=0;
//	I8U   pos=0;
//
//	while(pos < 128)
//	{
//		if (string[pos] == '\0')
//		  break;
//
//		if ((string[pos] >= 0x20)&&(string[pos] <= 0x7e)) {
//		  // ASCII code encode format.
//      offset += 8;
//			pos += 1;
//		} else if (((string[pos]&0xF0) == 0xE0)&&((string[pos+1]&0xC0) == 0x80)&&((string[pos+2]&0xC0) == 0x80)) {
//		  // Chinese characters Utf-8 encode format.
//			offset += 16;
//			pos += 3;
//		}
//		else {
//			// Is not within the scope of the can display,continue to read the next.
//			pos++;
//		}
//	}
//	return offset;
//}
