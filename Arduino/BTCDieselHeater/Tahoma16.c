#include "tahoma16.h"
// 
//  Font data for Tahoma 16pt
//
//  Generated by The Dot Factory:
//  http://www.eran.io/the-dot-factory-an-lcd-font-and-image-generator/
// 
/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Dot Factory Settings
// 
// Flip/Rotate      Padding Removal           Line Wrap         Descriptors
//   [X] Flip X       Height(Y): Tightest       (O) At column     [X] Generate descriptor array
//   [ ] Flip Y       Width(X):  Tightest       ( ) At bitmap     Char Width:  In Bits
//                                                                Char Height: In Bits
// Comments              Byte                                     Font Height: In Bits
//   [X] Variable Name     Bit layout: RowMajor                   [ ] Multiple descriptor arrays
//   [X] BMP visualise     Order:      MSBfirst
//   [X] Char descriptor   Format:     Hex                        Create new when exceeds [80]
//    Style: Cpp           Leading:    0x                         
//                                                                Image width:  In Bits
//  Variable name format                                          Image height: In Bits
//    Bitmaps:   const uint8_t PROGMEM {0}Bitmaps
//    Char Info: const FONT_CHAR_INFO PROGMEM {0}Descriptors    Space char generation
//    Font Info: const FONT_INFO {0}FontInfo                      [X] Generate space bitmap     
//    Width:     const uint8_t {0}Width                           [2] pixels for space char
//    Height:    const uint8_t {0}Height
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

// Character bitmaps for Tahoma 16pt
const uint8_t tahoma_16ptBitmaps[] PROGMEM =
{
	// @0 '.' (4 pixels wide)
	0x00, 0x0F, //             ####
	0x00, 0x0F, //             ####
	0x00, 0x0F, //             ####
	0x00, 0x0F, //             ####

	// @8 '0' (11 pixels wide)
	0x0F, 0xF0, //     ########    
	0x3F, 0xFC, //   ############  
	0x7F, 0xFE, //  ############## 
	0xFF, 0xFF, // ################
	0xF0, 0x0F, // ####        ####
	0xE0, 0x07, // ###          ###
	0xF0, 0x0F, // ####        ####
	0xFF, 0xFF, // ################
	0x7F, 0xFE, //  ############## 
	0x3F, 0xFC, //   ############  
	0x0F, 0xF0, //     ########    

	// @30 '1' (10 pixels wide)
	0x38, 0x07, //   ###        ###
	0x38, 0x07, //   ###        ###
	0x38, 0x07, //   ###        ###
	0xFF, 0xFF, // ################
	0xFF, 0xFF, // ################
	0xFF, 0xFF, // ################
	0xFF, 0xFF, // ################
	0x00, 0x07, //              ###
	0x00, 0x07, //              ###
	0x00, 0x07, //              ###

	// @50 '2' (11 pixels wide)
	0x70, 0x07, //  ###         ###
	0xE0, 0x0F, // ###         ####
	0xE0, 0x1F, // ###        #####
	0xE0, 0x3F, // ###       ######
	0xE0, 0x7F, // ###      #######
	0xF1, 0xF7, // ####   ##### ###
	0xFF, 0xE7, // ###########  ###
	0x7F, 0xC7, //  #########   ###
	0x7F, 0x87, //  ########    ###
	0x3E, 0x07, //   #####      ###
	0x00, 0x07, //              ###

	// @72 '3' (11 pixels wide)
	0x70, 0x0E, //  ###        ### 
	0xF0, 0x0F, // ####        ####
	0xE0, 0x07, // ###          ###
	0xE3, 0x87, // ###   ###    ###
	0xE3, 0x87, // ###   ###    ###
	0xE3, 0x87, // ###   ###    ###
	0xE7, 0xCF, // ###  #####  ####
	0xFF, 0xFF, // ################
	0x7E, 0xFE, //  ###### ####### 
	0x7E, 0xFE, //  ###### ####### 
	0x3C, 0x78, //   ####   ####   

	// @94 '4' (12 pixels wide)
	0x00, 0xF0, //         ####    
	0x01, 0xF0, //        #####    
	0x07, 0x70, //      ### ###    
	0x0E, 0x70, //     ###  ###    
	0x38, 0x70, //   ###    ###    
	0x70, 0x70, //  ###     ###    
	0xFF, 0xFF, // ################
	0xFF, 0xFF, // ################
	0xFF, 0xFF, // ################
	0xFF, 0xFF, // ################
	0x00, 0x70, //          ###    
	0x00, 0x70, //          ###    

	// @118 '5' (11 pixels wide)
	0x00, 0x0E, //             ### 
	0xFF, 0x87, // #########    ###
	0xFF, 0x87, // #########    ###
	0xFF, 0x87, // #########    ###
	0xFF, 0x87, // #########    ###
	0xE3, 0x87, // ###   ###    ###
	0xE3, 0xCF, // ###   ####  ####
	0xE3, 0xFF, // ###   ##########
	0xE3, 0xFE, // ###   ######### 
	0xE1, 0xFC, // ###    #######  
	0xE0, 0xF8, // ###     #####   

	// @140 '6' (11 pixels wide)
	0x07, 0xF0, //      #######    
	0x1F, 0xFC, //    ###########  
	0x3F, 0xFE, //   ############# 
	0x7F, 0xFF, //  ###############
	0xFB, 0x0F, // ##### ##    ####
	0xF7, 0x07, // #### ###     ###
	0xE7, 0x8F, // ###  ####   ####
	0xE7, 0xFF, // ###  ###########
	0xE7, 0xFE, // ###  ########## 
	0xE3, 0xFC, // ###   ########  
	0x01, 0xF8, //        ######   

	// @162 '7' (11 pixels wide)
	0xE0, 0x01, // ###            #
	0xE0, 0x07, // ###          ###
	0xE0, 0x1F, // ###        #####
	0xE0, 0x7F, // ###      #######
	0xE1, 0xFF, // ###    #########
	0xE7, 0xFC, // ###  #########  
	0xFF, 0xF0, // ############    
	0xFF, 0xC0, // ##########      
	0xFF, 0x00, // ########        
	0xFC, 0x00, // ######          
	0xF0, 0x00, // ####            

	// @184 '8' (11 pixels wide)
	0x3C, 0x3C, //   ####    ####  
	0x7E, 0xFE, //  ###### ####### 
	0x7F, 0xFE, //  ############## 
	0xFF, 0xFF, // ################
	0xE7, 0x8F, // ###  ####   ####
	0xE3, 0x87, // ###   ###    ###
	0xE3, 0xC7, // ###   ####   ###
	0xFF, 0xFF, // ################
	0x7F, 0xFE, //  ############## 
	0x7E, 0xFE, //  ###### ####### 
	0x3C, 0x7C, //   ####   #####  

	// @206 '9' (11 pixels wide)
	0x1F, 0x80, //    ######       
	0x3F, 0xC7, //   ########   ###
	0x7F, 0xE7, //  ##########  ###
	0xFF, 0xE7, // ###########  ###
	0xF1, 0xE7, // ####   ####  ###
	0xE0, 0xEF, // ###     ### ####
	0xF0, 0xDF, // ####    ## #####
	0xFF, 0xFE, // ############### 
	0x7F, 0xFC, //  #############  
	0x3F, 0xF8, //   ###########   
	0x0F, 0xE0, //     #######     

	// @228 '�' (8 pixels wide)
	0x3C, 0x00, //   ####          
	0x7E, 0x00, //  ######         
	0xE7, 0x00, // ###  ###        
	0xC3, 0x00, // ##    ##        
	0xC3, 0x00, // ##    ##        
	0xE7, 0x00, // ###  ###        
	0x7E, 0x00, //  ######         
	0x3C, 0x00, //   ####          

	// @244 'C' (12 pixels wide)
	0x07, 0xE0, //      ######     
	0x1F, 0xF8, //    ##########   
	0x3F, 0xFC, //   ############  
	0x7F, 0xFE, //  ############## 
	0xF8, 0x1F, // #####      #####
	0xF0, 0x0F, // ####        ####
	0xE0, 0x07, // ###          ###
	0xE0, 0x07, // ###          ###
	0xE0, 0x07, // ###          ###
	0xE0, 0x07, // ###          ###
	0x70, 0x0E, //  ###        ### 
	0x78, 0x1E, //  ####      #### 
};

// Character descriptors for Tahoma 16pt
// { [Char width in bits], [Char height in bits], [Offset into tahoma_16ptCharBitmaps in bytes] }
const FONT_CHAR_INFO tahoma_16ptDescriptors[] PROGMEM =
{
	{4, 16, 0},      // '.' 
	{0, 0, 0},       // '/' 
	{11, 16, 8},     // '0' 
	{10, 16, 30},    // '1' 
	{11, 16, 50},    // '2' 
	{11, 16, 72},    // '3' 
	{12, 16, 94},    // '4' 
	{11, 16, 118},   // '5' 
	{11, 16, 140},   // '6' 
	{11, 16, 162},   // '7' 
	{11, 16, 184},   // '8' 
	{11, 16, 206},   // '9' 
	{0, 0, 0},       // ':' 
	{0, 0, 0},       // ';' 
	{0, 0, 0},       // '<' 
	{0, 0, 0},       // '='
	{0, 0, 0},       // '>'
	{0, 0, 0},       // '?' 
	{0, 0, 0},       // '@' 
	{0, 0, 0},       // 'A' 
	{0, 0, 0},       // 'B' 
	{12, 16, 244},   // 'C' 
	{0, 0, 0},       // 'D' 
	{0, 0, 0},       // 'E' 
	{0, 0, 0},       // 'F' 
	{0, 0, 0},       // 'G' 
	{0, 0, 0},       // 'H' 
	{0, 0, 0},       // 'I' 
	{0, 0, 0},       // 'J' 
	{0, 0, 0},       // 'K' 
	{0, 0, 0},       // 'L' 
	{0, 0, 0},       // 'M' 
	{0, 0, 0},       // 'N' 
	{0, 0, 0},       // 'O' 
	{0, 0, 0},       // 'P' 
	{0, 0, 0},       // 'Q' 
	{0, 0, 0},       // 'R' 
	{0, 0, 0},       // 'S' 
	{0, 0, 0},       // 'T' 
	{0, 0, 0},       // 'U' 
	{0, 0, 0},       // 'V' 
	{0, 0, 0},       // 'W' 
	{0, 0, 0},       // 'X' 
	{0, 0, 0},       // 'Y' 
	{0, 0, 0},       // 'Z' 
	{0, 0, 0},       // '[' 
	{0, 0, 0},       // '\'
	{0, 0, 0},       // ']' 
	{0, 0, 0},       // '^' 
	{0, 0, 0},       // '_' 
	{8, 16, 228},    // '`'   use for degree symbol
};

// Font information for Tahoma 16pt
// easier to leave in RAM, not that big anyway
const FONT_INFO tahoma_16ptFontInfo =
{
	16,   //  Character height
	'.',  //  Start character
  '`',  // End character
	2,    //  Width, in pixels, of space character
	tahoma_16ptDescriptors, //  Character descriptor array
	tahoma_16ptBitmaps,     //  Character bitmap array
};

