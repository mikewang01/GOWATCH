#ifndef __FONT__
#define __FONT__

#include <stdint.h>
#include "standard_types.h"

// 5x7 ascii space
#define FONT_ASCII_5X7_SPACE_START	       (376832)  //start addr:368k 	
#define FONT_ASCII_5X7_SPACE_SIZE	         (768)		 //occupy 768 bytes

// 8x16 ascii space
#define FONT_ASCII_8X16_SPACE_START	       (FONT_ASCII_5X7_SPACE_START+FONT_ASCII_5X7_SPACE_SIZE)		
#define FONT_ASCII_8X16_SPACE_SIZE		     (1536)

//15x16 chinese space
#define FONT_CHINESE_15X16_SPACE_START	   (FONT_ASCII_8X16_SPACE_START+FONT_ASCII_8X16_SPACE_SIZE)		
#define FONT_CHINESE_15X16_SPACE_SIZE		   (668864)

void FONT_load_characters(I8U *p_in, char *data, I8U height, I8U horizontal_len, BOOLEAN b_center);
I8U  FONT_get_string_display_depth(char *string);
void FONT_load_ota_percent(I8U *ptr, I8U percent);
I16U FONT_get_string_display_len(char *string);
#endif // __GT23L16U2Y__
/** @} */
