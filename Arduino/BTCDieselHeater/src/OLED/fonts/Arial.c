// 
//  Font data for Arial 
//
//
//  Generated by The Dot Factory:
//  http://www.eran.io/the-dot-factory-an-lcd-font-and-image-generator/
// 
/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Dot Factory Settings
// 
// Flip/Rotate      Padding Removal           Line Wrap         Descriptors
//   [X] Flip X       Height(Y): None           (O) At column     [X] Generate descriptor array
//   [ ] Flip Y       Width(X):  Tightest       ( ) At bitmap       Char Width:  In Bits
//       90deg                                                      Char Height: In Bits
//                                                                  Font Height: In Bits
// Comments                  Byte                                     [ ] Multiple descriptor arrays
//   [X] Variable Name         Bit layout: RowMajor                   
//   [X] BMP visualise   [#]   Order:      MSBfirst                 Create new when exceeds [80]
//   [X] Char descriptor       Format:     Hex                        
//    Style: Cpp               Leading:    0x                       Image width:  In Bits
//                                                                  Image height: In Bits
//  Variable name format                                          
//    Bitmaps:   const uint8_t PROGMEM {0}Bitmaps               Space char generation
//    Char Info: const FONT_CHAR_INFO PROGMEM {0}Descriptors      [ ] Generate space bitmap
//    Font Info: const FONT_INFO {0}FontInfo                      [2] pixels for space char     
//    Width:     const uint8_t {0}Width                           
//    Height:    const uint8_t {0}Height
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Arial.h"

// 
//  Font data for Arial 8pt
// 

// Character bitmaps for Arial 8pt
const uint8_t PROGMEM arial_8ptBitmaps [] = 
{
	// @0 ' ' (2 pixels wide)
	0x00, 0x00, //               
	0x00, 0x00, //               

	// @4 '!' (1 pixels wide)
	0x1F, 0xA0, //    ###### #   

	// @6 '"' (3 pixels wide)
	0x1C, 0x00, //    ###        
	0x00, 0x00, //               
	0x1C, 0x00, //    ###        

	// @12 '#' (5 pixels wide)
	0x04, 0xE0, //      #  ###   
	0x07, 0x80, //      ####     
	0x1C, 0xE0, //    ###  ###   
	0x07, 0x80, //      ####     
	0x1C, 0x80, //    ###  #     

	// @22 '$' (5 pixels wide)
	0x0C, 0x40, //     ##   #    
	0x12, 0x20, //    #  #   #   
	0x1F, 0xF0, //    #########  
	0x12, 0x20, //    #  #   #   
	0x09, 0xC0, //     #  ###    

	// @32 '%' (9 pixels wide)
	0x0C, 0x00, //     ##        
	0x12, 0x00, //    #  #       
	0x12, 0x20, //    #  #   #   
	0x0C, 0xC0, //     ##  ##    
	0x03, 0x00, //       ##      
	0x0C, 0xC0, //     ##  ##    
	0x11, 0x20, //    #   #  #   
	0x01, 0x20, //        #  #   
	0x00, 0xC0, //         ##    

	// @50 '&' (6 pixels wide)
	0x00, 0xC0, //         ##    
	0x0D, 0x20, //     ## #  #   
	0x12, 0x20, //    #  #   #   
	0x13, 0x20, //    #  ##  #   
	0x0C, 0xC0, //     ##  ##    
	0x00, 0xA0, //         # #   

	// @62 ''' (1 pixels wide)
	0x1C, 0x00, //    ###        

	// @64 '(' (3 pixels wide)
	0x07, 0xE0, //      ######   
	0x08, 0x10, //     #      #  
	0x10, 0x08, //    #        # 

	// @70 ')' (3 pixels wide)
	0x10, 0x08, //    #        # 
	0x08, 0x10, //     #      #  
	0x07, 0xE0, //      ######   

	// @76 '*' (3 pixels wide)
	0x0A, 0x00, //     # #       
	0x1C, 0x00, //    ###        
	0x0A, 0x00, //     # #       

	// @82 '+' (5 pixels wide)
	0x01, 0x00, //        #      
	0x01, 0x00, //        #      
	0x07, 0xC0, //      #####    
	0x01, 0x00, //        #      
	0x01, 0x00, //        #      

	// @92 ',' (1 pixels wide)
	0x00, 0x38, //           ### 

	// @94 '-' (3 pixels wide)
	0x00, 0x80, //         #     
	0x00, 0x80, //         #     
	0x00, 0x80, //         #     

	// @100 '.' (1 pixels wide)
	0x00, 0x20, //           #   

	// @102 '/' (3 pixels wide)
	0x00, 0x60, //          ##   
	0x07, 0x80, //      ####     
	0x18, 0x00, //    ##         

	// @108 '0' (5 pixels wide)
	0x0F, 0xC0, //     ######    
	0x10, 0x20, //    #      #   
	0x10, 0x20, //    #      #   
	0x10, 0x20, //    #      #   
	0x0F, 0xC0, //     ######    

	// @118 '1' (3 pixels wide)
	0x04, 0x00, //      #        
	0x08, 0x00, //     #         
	0x1F, 0xE0, //    ########   

	// @124 '2' (5 pixels wide)
	0x08, 0x20, //     #     #   
	0x10, 0x60, //    #     ##   
	0x10, 0xA0, //    #    # #   
	0x11, 0x20, //    #   #  #   
	0x0E, 0x20, //     ###   #   

	// @134 '3' (5 pixels wide)
	0x08, 0x40, //     #    #    
	0x10, 0x20, //    #      #   
	0x12, 0x20, //    #  #   #   
	0x12, 0x20, //    #  #   #   
	0x0D, 0xC0, //     ## ###    

	// @144 '4' (5 pixels wide)
	0x01, 0x80, //        ##     
	0x06, 0x80, //      ## #     
	0x08, 0x80, //     #   #     
	0x1F, 0xE0, //    ########   
	0x00, 0x80, //         #     

	// @154 '5' (5 pixels wide)
	0x06, 0x40, //      ##  #    
	0x1A, 0x20, //    ## #   #   
	0x12, 0x20, //    #  #   #   
	0x12, 0x20, //    #  #   #   
	0x11, 0xC0, //    #   ###    

	// @164 '6' (5 pixels wide)
	0x0F, 0xC0, //     ######    
	0x12, 0x20, //    #  #   #   
	0x12, 0x20, //    #  #   #   
	0x12, 0x20, //    #  #   #   
	0x09, 0xC0, //     #  ###    

	// @174 '7' (5 pixels wide)
	0x10, 0x00, //    #          
	0x10, 0xE0, //    #    ###   
	0x13, 0x00, //    #  ##      
	0x1C, 0x00, //    ###        
	0x10, 0x00, //    #          

	// @184 '8' (5 pixels wide)
	0x0D, 0xC0, //     ## ###    
	0x12, 0x20, //    #  #   #   
	0x12, 0x20, //    #  #   #   
	0x12, 0x20, //    #  #   #   
	0x0D, 0xC0, //     ## ###    

	// @194 '9' (5 pixels wide)
	0x0E, 0x40, //     ###  #    
	0x11, 0x20, //    #   #  #   
	0x11, 0x20, //    #   #  #   
	0x11, 0x20, //    #   #  #   
	0x0F, 0xC0, //     ######    

	// @204 ':' (1 pixels wide)
	0x04, 0x20, //      #    #   

	// @206 ';' (1 pixels wide)
	0x04, 0x38, //      #    ### 

	// @208 '<' (5 pixels wide)
	0x01, 0x00, //        #      
	0x02, 0x80, //       # #     
	0x02, 0x80, //       # #     
	0x02, 0x80, //       # #     
	0x04, 0x40, //      #   #    

	// @218 '=' (5 pixels wide)
	0x02, 0x80, //       # #     
	0x02, 0x80, //       # #     
	0x02, 0x80, //       # #     
	0x02, 0x80, //       # #     
	0x02, 0x80, //       # #     

	// @228 '>' (5 pixels wide)
	0x04, 0x40, //      #   #    
	0x02, 0x80, //       # #     
	0x02, 0x80, //       # #     
	0x02, 0x80, //       # #     
	0x01, 0x00, //        #      

	// @238 '?' (5 pixels wide)
	0x08, 0x00, //     #         
	0x10, 0x00, //    #          
	0x11, 0xA0, //    #   ## #   
	0x12, 0x00, //    #  #       
	0x0C, 0x00, //     ##        

	// @248 '@' (10 pixels wide)
	0x03, 0xE0, //       #####   
	0x0C, 0x10, //     ##     #  
	0x09, 0xC8, //     #  ###  # 
	0x12, 0x28, //    #  #   # # 
	0x14, 0x28, //    # #    # # 
	0x14, 0x48, //    # #   #  # 
	0x13, 0xE8, //    #  ##### # 
	0x16, 0x28, //    # ##   # # 
	0x08, 0x48, //     #    #  # 
	0x07, 0x90, //      ####  #  

	// @268 'A' (7 pixels wide)
	0x00, 0x60, //          ##   
	0x01, 0x80, //        ##     
	0x0E, 0x80, //     ### #     
	0x10, 0x80, //    #    #     
	0x0E, 0x80, //     ### #     
	0x01, 0x80, //        ##     
	0x00, 0x60, //          ##   

	// @282 'B' (6 pixels wide)
	0x1F, 0xE0, //    ########   
	0x12, 0x20, //    #  #   #   
	0x12, 0x20, //    #  #   #   
	0x12, 0x20, //    #  #   #   
	0x12, 0x20, //    #  #   #   
	0x0F, 0xC0, //     ######    

	// @294 'C' (6 pixels wide)
	0x07, 0x80, //      ####     
	0x08, 0x40, //     #    #    
	0x10, 0x20, //    #      #   
	0x10, 0x20, //    #      #   
	0x10, 0x20, //    #      #   
	0x08, 0x40, //     #    #    

	// @306 'D' (6 pixels wide)
	0x1F, 0xE0, //    ########   
	0x10, 0x20, //    #      #   
	0x10, 0x20, //    #      #   
	0x10, 0x20, //    #      #   
	0x08, 0x40, //     #    #    
	0x07, 0x80, //      ####     

	// @318 'E' (5 pixels wide)
	0x1F, 0xE0, //    ########   
	0x12, 0x20, //    #  #   #   
	0x12, 0x20, //    #  #   #   
	0x12, 0x20, //    #  #   #   
	0x12, 0x20, //    #  #   #   

	// @328 'F' (5 pixels wide)
	0x1F, 0xE0, //    ########   
	0x12, 0x00, //    #  #       
	0x12, 0x00, //    #  #       
	0x12, 0x00, //    #  #       
	0x10, 0x00, //    #          

	// @338 'G' (7 pixels wide)
	0x07, 0x80, //      ####     
	0x08, 0x40, //     #    #    
	0x10, 0x20, //    #      #   
	0x10, 0x20, //    #      #   
	0x11, 0x20, //    #   #  #   
	0x09, 0x40, //     #  # #    
	0x05, 0x80, //      # ##     

	// @352 'H' (6 pixels wide)
	0x1F, 0xE0, //    ########   
	0x02, 0x00, //       #       
	0x02, 0x00, //       #       
	0x02, 0x00, //       #       
	0x02, 0x00, //       #       
	0x1F, 0xE0, //    ########   

	// @364 'I' (1 pixels wide)
	0x1F, 0xE0, //    ########   

	// @366 'J' (4 pixels wide)
	0x00, 0xC0, //         ##    
	0x00, 0x20, //           #   
	0x00, 0x20, //           #   
	0x1F, 0xC0, //    #######    

	// @374 'K' (6 pixels wide)
	0x1F, 0xE0, //    ########   
	0x01, 0x00, //        #      
	0x02, 0x00, //       #       
	0x07, 0x00, //      ###      
	0x08, 0xC0, //     #   ##    
	0x10, 0x20, //    #      #   

	// @386 'L' (5 pixels wide)
	0x1F, 0xE0, //    ########   
	0x00, 0x20, //           #   
	0x00, 0x20, //           #   
	0x00, 0x20, //           #   
	0x00, 0x20, //           #   

	// @396 'M' (7 pixels wide)
	0x1F, 0xE0, //    ########   
	0x0C, 0x00, //     ##        
	0x03, 0x80, //       ###     
	0x00, 0x60, //          ##   
	0x03, 0x80, //       ###     
	0x0C, 0x00, //     ##        
	0x1F, 0xE0, //    ########   

	// @410 'N' (6 pixels wide)
	0x1F, 0xE0, //    ########   
	0x08, 0x00, //     #         
	0x06, 0x00, //      ##       
	0x01, 0x80, //        ##     
	0x00, 0x40, //          #    
	0x1F, 0xE0, //    ########   

	// @422 'O' (7 pixels wide)
	0x07, 0x80, //      ####     
	0x08, 0x40, //     #    #    
	0x10, 0x20, //    #      #   
	0x10, 0x20, //    #      #   
	0x10, 0x20, //    #      #   
	0x08, 0x40, //     #    #    
	0x07, 0x80, //      ####     

	// @436 'P' (5 pixels wide)
	0x1F, 0xE0, //    ########   
	0x11, 0x00, //    #   #      
	0x11, 0x00, //    #   #      
	0x11, 0x00, //    #   #      
	0x0E, 0x00, //     ###       

	// @446 'Q' (7 pixels wide)
	0x07, 0x80, //      ####     
	0x08, 0x40, //     #    #    
	0x10, 0x20, //    #      #   
	0x10, 0xA0, //    #    # #   
	0x10, 0xA0, //    #    # #   
	0x08, 0x40, //     #    #    
	0x07, 0xA0, //      #### #   

	// @460 'R' (6 pixels wide)
	0x1F, 0xE0, //    ########   
	0x12, 0x00, //    #  #       
	0x12, 0x00, //    #  #       
	0x13, 0x00, //    #  ##      
	0x12, 0xC0, //    #  # ##    
	0x0C, 0x20, //     ##    #   

	// @472 'S' (6 pixels wide)
	0x0C, 0x40, //     ##   #    
	0x12, 0x20, //    #  #   #   
	0x12, 0x20, //    #  #   #   
	0x11, 0x20, //    #   #  #   
	0x11, 0x20, //    #   #  #   
	0x08, 0xC0, //     #   ##    

	// @484 'T' (5 pixels wide)
	0x10, 0x00, //    #          
	0x10, 0x00, //    #          
	0x1F, 0xE0, //    ########   
	0x10, 0x00, //    #          
	0x10, 0x00, //    #          

	// @494 'U' (6 pixels wide)
	0x1F, 0xC0, //    #######    
	0x00, 0x20, //           #   
	0x00, 0x20, //           #   
	0x00, 0x20, //           #   
	0x00, 0x20, //           #   
	0x1F, 0xC0, //    #######    

	// @506 'V' (7 pixels wide)
	0x18, 0x00, //    ##         
	0x06, 0x00, //      ##       
	0x01, 0x80, //        ##     
	0x00, 0x60, //          ##   
	0x01, 0x80, //        ##     
	0x06, 0x00, //      ##       
	0x18, 0x00, //    ##         

	// @520 'W' (11 pixels wide)
	0x18, 0x00, //    ##         
	0x07, 0x80, //      ####     
	0x00, 0x60, //          ##   
	0x01, 0x80, //        ##     
	0x0E, 0x00, //     ###       
	0x10, 0x00, //    #          
	0x0E, 0x00, //     ###       
	0x01, 0x80, //        ##     
	0x00, 0x60, //          ##   
	0x07, 0x80, //      ####     
	0x18, 0x00, //    ##         

	// @542 'X' (6 pixels wide)
	0x10, 0x20, //    #      #   
	0x0C, 0xC0, //     ##  ##    
	0x03, 0x00, //       ##      
	0x03, 0x00, //       ##      
	0x0C, 0xC0, //     ##  ##    
	0x10, 0x20, //    #      #   

	// @554 'Y' (7 pixels wide)
	0x10, 0x00, //    #          
	0x0C, 0x00, //     ##        
	0x02, 0x00, //       #       
	0x01, 0xE0, //        ####   
	0x02, 0x00, //       #       
	0x0C, 0x00, //     ##        
	0x10, 0x00, //    #          

	// @568 'Z' (6 pixels wide)
	0x00, 0x20, //           #   
	0x10, 0x60, //    #     ##   
	0x11, 0xA0, //    #   ## #   
	0x16, 0x20, //    # ##   #   
	0x18, 0x20, //    ##     #   
	0x10, 0x20, //    #      #   

	// @580 '[' (2 pixels wide)
	0x1F, 0xF8, //    ########## 
	0x10, 0x08, //    #        # 

	// @584 '\' (3 pixels wide)
	0x18, 0x00, //    ##         
	0x07, 0x80, //      ####     
	0x00, 0x60, //          ##   

	// @590 ']' (2 pixels wide)
	0x10, 0x08, //    #        # 
	0x1F, 0xF8, //    ########## 

	// @594 '^' (5 pixels wide)
	0x02, 0x00, //       #       
	0x0C, 0x00, //     ##        
	0x10, 0x00, //    #          
	0x0C, 0x00, //     ##        
	0x02, 0x00, //       #       

	// @604 '_' (6 pixels wide)
	0x00, 0x08, //             # 
	0x00, 0x08, //             # 
	0x00, 0x08, //             # 
	0x00, 0x08, //             # 
	0x00, 0x08, //             # 
	0x00, 0x08, //             # 

	// @616 '`' (2 pixels wide)
	0x10, 0x00, //    #          
	0x08, 0x00, //     #         

	// @620 'a' (5 pixels wide)
	0x02, 0xC0, //       # ##    
	0x05, 0x20, //      # #  #   
	0x05, 0x20, //      # #  #   
	0x05, 0x40, //      # # #    
	0x03, 0xE0, //       #####   

	// @630 'b' (5 pixels wide)
	0x1F, 0xE0, //    ########   
	0x02, 0x40, //       #  #    
	0x04, 0x20, //      #    #   
	0x04, 0x20, //      #    #   
	0x03, 0xC0, //       ####    

	// @640 'c' (5 pixels wide)
	0x03, 0xC0, //       ####    
	0x04, 0x20, //      #    #   
	0x04, 0x20, //      #    #   
	0x04, 0x20, //      #    #   
	0x02, 0x40, //       #  #    

	// @650 'd' (5 pixels wide)
	0x03, 0xC0, //       ####    
	0x04, 0x20, //      #    #   
	0x04, 0x20, //      #    #   
	0x02, 0x40, //       #  #    
	0x1F, 0xE0, //    ########   

	// @660 'e' (5 pixels wide)
	0x03, 0xC0, //       ####    
	0x05, 0x20, //      # #  #   
	0x05, 0x20, //      # #  #   
	0x05, 0x20, //      # #  #   
	0x03, 0x40, //       ## #    

	// @670 'f' (3 pixels wide)
	0x04, 0x00, //      #        
	0x0F, 0xE0, //     #######   
	0x14, 0x00, //    # #        

	// @676 'g' (5 pixels wide)
	0x03, 0xC8, //       ####  # 
	0x04, 0x28, //      #    # # 
	0x04, 0x28, //      #    # # 
	0x02, 0x48, //       #  #  # 
	0x07, 0xF0, //      #######  

	// @686 'h' (5 pixels wide)
	0x1F, 0xE0, //    ########   
	0x02, 0x00, //       #       
	0x04, 0x00, //      #        
	0x04, 0x00, //      #        
	0x03, 0xE0, //       #####   

	// @696 'i' (1 pixels wide)
	0x17, 0xE0, //    # ######   

	// @698 'j' (2 pixels wide)
	0x00, 0x08, //             # 
	0x17, 0xF0, //    # #######  

	// @702 'k' (4 pixels wide)
	0x1F, 0xE0, //    ########   
	0x01, 0x00, //        #      
	0x02, 0xC0, //       # ##    
	0x04, 0x20, //      #    #   

	// @710 'l' (1 pixels wide)
	0x1F, 0xE0, //    ########   

	// @712 'm' (7 pixels wide)
	0x07, 0xE0, //      ######   
	0x02, 0x00, //       #       
	0x04, 0x00, //      #        
	0x07, 0xE0, //      ######   
	0x04, 0x00, //      #        
	0x04, 0x00, //      #        
	0x03, 0xE0, //       #####   

	// @726 'n' (5 pixels wide)
	0x07, 0xE0, //      ######   
	0x04, 0x00, //      #        
	0x04, 0x00, //      #        
	0x04, 0x00, //      #        
	0x03, 0xE0, //       #####   

	// @736 'o' (5 pixels wide)
	0x03, 0xC0, //       ####    
	0x04, 0x20, //      #    #   
	0x04, 0x20, //      #    #   
	0x04, 0x20, //      #    #   
	0x03, 0xC0, //       ####    

	// @746 'p' (5 pixels wide)
	0x07, 0xF8, //      ######## 
	0x02, 0x40, //       #  #    
	0x04, 0x20, //      #    #   
	0x04, 0x20, //      #    #   
	0x03, 0xC0, //       ####    

	// @756 'q' (5 pixels wide)
	0x03, 0xC0, //       ####    
	0x04, 0x20, //      #    #   
	0x04, 0x20, //      #    #   
	0x02, 0x40, //       #  #    
	0x07, 0xF8, //      ######## 

	// @766 'r' (3 pixels wide)
	0x07, 0xE0, //      ######   
	0x02, 0x00, //       #       
	0x04, 0x00, //      #        

	// @772 's' (5 pixels wide)
	0x02, 0x40, //       #  #    
	0x05, 0x20, //      # #  #   
	0x05, 0x20, //      # #  #   
	0x04, 0xA0, //      #  # #   
	0x02, 0x40, //       #  #    

	// @782 't' (3 pixels wide)
	0x04, 0x00, //      #        
	0x1F, 0xE0, //    ########   
	0x04, 0x20, //      #    #   

	// @788 'u' (5 pixels wide)
	0x07, 0xC0, //      #####    
	0x00, 0x20, //           #   
	0x00, 0x20, //           #   
	0x00, 0x40, //          #    
	0x07, 0xE0, //      ######   

	// @798 'v' (5 pixels wide)
	0x06, 0x00, //      ##       
	0x01, 0x80, //        ##     
	0x00, 0x60, //          ##   
	0x01, 0x80, //        ##     
	0x06, 0x00, //      ##       

	// @808 'w' (9 pixels wide)
	0x06, 0x00, //      ##       
	0x01, 0x80, //        ##     
	0x00, 0x60, //          ##   
	0x03, 0x80, //       ###     
	0x04, 0x00, //      #        
	0x03, 0x80, //       ###     
	0x00, 0x60, //          ##   
	0x01, 0x80, //        ##     
	0x06, 0x00, //      ##       

	// @826 'x' (5 pixels wide)
	0x04, 0x20, //      #    #   
	0x02, 0x40, //       #  #    
	0x01, 0x80, //        ##     
	0x02, 0x40, //       #  #    
	0x04, 0x20, //      #    #   

	// @836 'y' (5 pixels wide)
	0x06, 0x00, //      ##       
	0x01, 0x88, //        ##   # 
	0x00, 0x70, //          ###  
	0x01, 0x80, //        ##     
	0x06, 0x00, //      ##       

	// @846 'z' (5 pixels wide)
	0x04, 0x20, //      #    #   
	0x04, 0x60, //      #   ##   
	0x05, 0xA0, //      # ## #   
	0x06, 0x20, //      ##   #   
	0x04, 0x20, //      #    #   

	// @856 '{' (3 pixels wide)
	0x01, 0x00, //        #      
	0x0E, 0xF0, //     ### ####  
	0x10, 0x08, //    #        # 

	// @862 '|' (1 pixels wide)
	0x1F, 0xF8, //    ########## 

	// @864 '}' (3 pixels wide)
	0x10, 0x08, //    #        # 
	0x0E, 0xF0, //     ### ####  
	0x01, 0x00, //        #      

	// @870 '~' (5 pixels wide)
	0x03, 0x00, //       ##      
	0x02, 0x00, //       #       
	0x03, 0x00, //       ##      
	0x01, 0x00, //        #      
	0x02, 0x00, //       #       
};

// Character descriptors for Arial 8pt
// { [Char width in bits], [Char height in bits], [Offset into arial_8ptCharBitmaps in bytes] }
const FONT_CHAR_INFO PROGMEM arial_8ptDescriptors[] = 
{
	{1, 14, 0}, 		// ' ' 
	{1, 14, 4}, 		// '!' 
	{3, 14, 6}, 		// '"' 
	{5, 14, 12}, 		// '#' 
	{5, 14, 22}, 		// '$' 
	{9, 14, 32}, 		// '%' 
	{6, 14, 50}, 		// '&' 
	{1, 14, 62}, 		// ''' 
	{3, 14, 64}, 		// '(' 
	{3, 14, 70}, 		// ')' 
	{3, 14, 76}, 		// '*' 
	{5, 14, 82}, 		// '+' 
	{1, 14, 92}, 		// ',' 
	{3, 14, 94}, 		// '-' 
	{1, 14, 100}, 		// '.' 
	{3, 14, 102}, 		// '/' 
	{5, 14, 108}, 		// '0' 
	{3, 14, 118}, 		// '1' 
	{5, 14, 124}, 		// '2' 
	{5, 14, 134}, 		// '3' 
	{5, 14, 144}, 		// '4' 
	{5, 14, 154}, 		// '5' 
	{5, 14, 164}, 		// '6' 
	{5, 14, 174}, 		// '7' 
	{5, 14, 184}, 		// '8' 
	{5, 14, 194}, 		// '9' 
	{1, 14, 204}, 		// ':' 
	{1, 14, 206}, 		// ';' 
	{5, 14, 208}, 		// '<' 
	{5, 14, 218}, 		// '=' 
	{5, 14, 228}, 		// '>' 
	{5, 14, 238}, 		// '?' 
	{10, 14, 248}, 		// '@' 
	{7, 14, 268}, 		// 'A' 
	{6, 14, 282}, 		// 'B' 
	{6, 14, 294}, 		// 'C' 
	{6, 14, 306}, 		// 'D' 
	{5, 14, 318}, 		// 'E' 
	{5, 14, 328}, 		// 'F' 
	{7, 14, 338}, 		// 'G' 
	{6, 14, 352}, 		// 'H' 
	{1, 14, 364}, 		// 'I' 
	{4, 14, 366}, 		// 'J' 
	{6, 14, 374}, 		// 'K' 
	{5, 14, 386}, 		// 'L' 
	{7, 14, 396}, 		// 'M' 
	{6, 14, 410}, 		// 'N' 
	{7, 14, 422}, 		// 'O' 
	{5, 14, 436}, 		// 'P' 
	{7, 14, 446}, 		// 'Q' 
	{6, 14, 460}, 		// 'R' 
	{6, 14, 472}, 		// 'S' 
	{5, 14, 484}, 		// 'T' 
	{6, 14, 494}, 		// 'U' 
	{7, 14, 506}, 		// 'V' 
	{11, 14, 520}, 		// 'W' 
	{6, 14, 542}, 		// 'X' 
	{7, 14, 554}, 		// 'Y' 
	{6, 14, 568}, 		// 'Z' 
	{2, 14, 580}, 		// '[' 
	{3, 14, 584}, 		// '\' 
	{2, 14, 590}, 		// ']' 
	{5, 14, 594}, 		// '^' 
	{6, 14, 604}, 		// '_' 
	{2, 14, 616}, 		// '`' 
	{5, 14, 620}, 		// 'a' 
	{5, 14, 630}, 		// 'b' 
	{5, 14, 640}, 		// 'c' 
	{5, 14, 650}, 		// 'd' 
	{5, 14, 660}, 		// 'e' 
	{3, 14, 670}, 		// 'f' 
	{5, 14, 676}, 		// 'g' 
	{5, 14, 686}, 		// 'h' 
	{1, 14, 696}, 		// 'i' 
	{2, 14, 698}, 		// 'j' 
	{4, 14, 702}, 		// 'k' 
	{1, 14, 710}, 		// 'l' 
	{7, 14, 712}, 		// 'm' 
	{5, 14, 726}, 		// 'n' 
	{5, 14, 736}, 		// 'o' 
	{5, 14, 746}, 		// 'p' 
	{5, 14, 756}, 		// 'q' 
	{3, 14, 766}, 		// 'r' 
	{5, 14, 772}, 		// 's' 
	{3, 14, 782}, 		// 't' 
	{5, 14, 788}, 		// 'u' 
	{5, 14, 798}, 		// 'v' 
	{9, 14, 808}, 		// 'w' 
	{5, 14, 826}, 		// 'x' 
	{5, 14, 836}, 		// 'y' 
	{5, 14, 846}, 		// 'z' 
	{3, 14, 856}, 		// '{' 
	{1, 14, 862}, 		// '|' 
	{3, 14, 864}, 		// '}' 
	{5, 14, 870}, 		// '~' 
};

// Font information for Arial 8pt
const FONT_INFO arial_8ptFontInfo =
{
	14, //  Character height
	' ', //  Start character
	'~', //  End character
	1,
	arial_8ptDescriptors, //  Character descriptor array
	arial_8ptBitmaps, //  Character bitmap array
};



// 
//  Font data for Arial 7pt
// 

// Character bitmaps for Arial 7pt
const uint8_t PROGMEM arial_7ptBitmaps [] = 
{
	// @0 ' ' (2 pixels wide)
	0x00, 0x00, //             
	0x00, 0x00, //             

	// @4 '!' (1 pixels wide)
	0x3E, 0x80, //   ##### #   

	// @6 '"' (3 pixels wide)
	0x30, 0x00, //   ##        
	0x00, 0x00, //             
	0x30, 0x00, //   ##        

	// @12 '#' (4 pixels wide)
	0x0B, 0x80, //     # ###   
	0x3E, 0x00, //   #####     
	0x0B, 0x80, //     # ###   
	0x3E, 0x00, //   #####     

	// @20 '$' (5 pixels wide)
	0x19, 0x00, //    ##  #    
	0x24, 0x80, //   #  #  #   
	0x3F, 0xC0, //   ########  
	0x24, 0x80, //   #  #  #   
	0x13, 0x00, //    #  ##    

	// @30 '%' (6 pixels wide)
	0x10, 0x00, //    #        
	0x28, 0x00, //   # #       
	0x13, 0x80, //    #  ###   
	0x1D, 0x00, //    ### #    
	0x22, 0x80, //   #   # #   
	0x01, 0x00, //        #    

	// @42 '&' (6 pixels wide)
	0x03, 0x00, //       ##    
	0x1C, 0x80, //    ###  #   
	0x24, 0x80, //   #  #  #   
	0x26, 0x80, //   #  ## #   
	0x39, 0x00, //   ###  #    
	0x02, 0x80, //       # #   

	// @54 ''' (1 pixels wide)
	0x30, 0x00, //   ##        

	// @56 '(' (3 pixels wide)
	0x0F, 0x80, //     #####   
	0x10, 0x40, //    #     #  
	0x20, 0x20, //   #       # 

	// @62 ')' (3 pixels wide)
	0x20, 0x20, //   #       # 
	0x10, 0x40, //    #     #  
	0x0F, 0x80, //     #####   

	// @68 '*' (3 pixels wide)
	0x28, 0x00, //   # #       
	0x30, 0x00, //   ##        
	0x28, 0x00, //   # #       

	// @74 '+' (5 pixels wide)
	0x02, 0x00, //       #     
	0x02, 0x00, //       #     
	0x0F, 0x80, //     #####   
	0x02, 0x00, //       #     
	0x02, 0x00, //       #     

	// @84 ',' (1 pixels wide)
	0x00, 0xC0, //         ##  

	// @86 '-' (2 pixels wide)
	0x02, 0x00, //       #     
	0x02, 0x00, //       #     

	// @90 '.' (1 pixels wide)
	0x00, 0x80, //         #   

	// @92 '/' (3 pixels wide)
	0x01, 0x80, //        ##   
	0x0E, 0x00, //     ###     
	0x30, 0x00, //   ##        

	// @98 '0' (4 pixels wide)
	0x1F, 0x80, //    ######   
	0x20, 0x80, //   #     #   
	0x20, 0x80, //   #     #   
	0x1F, 0x00, //    #####    

	// @106 '1' (2 pixels wide)
	0x10, 0x00, //    #        
	0x3F, 0x80, //   #######   

	// @110 '2' (4 pixels wide)
	0x10, 0x80, //    #    #   
	0x21, 0x80, //   #    ##   
	0x26, 0x80, //   #  ## #   
	0x18, 0x80, //    ##   #   

	// @118 '3' (4 pixels wide)
	0x11, 0x00, //    #   #    
	0x20, 0x80, //   #     #   
	0x24, 0x80, //   #  #  #   
	0x1B, 0x00, //    ## ##    

	// @126 '4' (5 pixels wide)
	0x06, 0x00, //      ##     
	0x0A, 0x00, //     # #     
	0x12, 0x00, //    #  #     
	0x3F, 0x80, //   #######   
	0x02, 0x00, //       #     

	// @136 '5' (4 pixels wide)
	0x0D, 0x00, //     ## #    
	0x38, 0x80, //   ###   #   
	0x28, 0x80, //   # #   #   
	0x27, 0x00, //   #  ###    

	// @144 '6' (4 pixels wide)
	0x1F, 0x00, //    #####    
	0x28, 0x80, //   # #   #   
	0x28, 0x80, //   # #   #   
	0x17, 0x00, //    # ###    

	// @152 '7' (4 pixels wide)
	0x20, 0x00, //   #         
	0x23, 0x80, //   #   ###   
	0x2C, 0x00, //   # ##      
	0x30, 0x00, //   ##        

	// @160 '8' (4 pixels wide)
	0x1B, 0x00, //    ## ##    
	0x24, 0x80, //   #  #  #   
	0x24, 0x80, //   #  #  #   
	0x1B, 0x00, //    ## ##    

	// @168 '9' (4 pixels wide)
	0x1D, 0x00, //    ### #    
	0x22, 0x80, //   #   # #   
	0x22, 0x80, //   #   # #   
	0x1F, 0x00, //    #####    

	// @176 ':' (1 pixels wide)
	0x08, 0x80, //     #   #   

	// @178 ';' (1 pixels wide)
	0x08, 0xC0, //     #   ##  

	// @180 '<' (3 pixels wide)
	0x04, 0x00, //      #      
	0x0A, 0x00, //     # #     
	0x11, 0x00, //    #   #    

	// @186 '=' (4 pixels wide)
	0x0A, 0x00, //     # #     
	0x0A, 0x00, //     # #     
	0x0A, 0x00, //     # #     
	0x0A, 0x00, //     # #     

	// @194 '>' (3 pixels wide)
	0x11, 0x00, //    #   #    
	0x0A, 0x00, //     # #     
	0x04, 0x00, //      #      

	// @200 '?' (5 pixels wide)
	0x10, 0x00, //    #        
	0x20, 0x00, //   #         
	0x26, 0x80, //   #  ## #   
	0x24, 0x00, //   #  #      
	0x18, 0x00, //    ##       

	// @210 '@' (8 pixels wide)
	0x0F, 0x80, //     #####   
	0x10, 0x40, //    #     #  
	0x27, 0xA0, //   #  #### # 
	0x28, 0xA0, //   # #   # # 
	0x27, 0xA0, //   #  #### # 
	0x2C, 0xA0, //   # ##  # # 
	0x11, 0x40, //    #   # #  
	0x0E, 0x40, //     ###  #  

	// @226 'A' (7 pixels wide)
	0x00, 0x80, //         #   
	0x07, 0x00, //      ###    
	0x1A, 0x00, //    ## #     
	0x22, 0x00, //   #   #     
	0x1A, 0x00, //    ## #     
	0x07, 0x00, //      ###    
	0x00, 0x80, //         #   

	// @240 'B' (4 pixels wide)
	0x3F, 0x80, //   #######   
	0x24, 0x80, //   #  #  #   
	0x24, 0x80, //   #  #  #   
	0x1F, 0x00, //    #####    

	// @248 'C' (5 pixels wide)
	0x1F, 0x00, //    #####    
	0x20, 0x80, //   #     #   
	0x20, 0x80, //   #     #   
	0x20, 0x80, //   #     #   
	0x11, 0x00, //    #   #    

	// @258 'D' (5 pixels wide)
	0x3F, 0x80, //   #######   
	0x20, 0x80, //   #     #   
	0x20, 0x80, //   #     #   
	0x20, 0x80, //   #     #   
	0x1F, 0x00, //    #####    

	// @268 'E' (5 pixels wide)
	0x3F, 0x80, //   #######   
	0x24, 0x80, //   #  #  #   
	0x24, 0x80, //   #  #  #   
	0x24, 0x80, //   #  #  #   
	0x24, 0x80, //   #  #  #   

	// @278 'F' (4 pixels wide)
	0x3F, 0x80, //   #######   
	0x24, 0x00, //   #  #      
	0x24, 0x00, //   #  #      
	0x20, 0x00, //   #         

	// @286 'G' (5 pixels wide)
	0x1F, 0x00, //    #####    
	0x20, 0x80, //   #     #   
	0x20, 0x80, //   #     #   
	0x24, 0x80, //   #  #  #   
	0x17, 0x00, //    # ###    

	// @296 'H' (5 pixels wide)
	0x3F, 0x80, //   #######   
	0x04, 0x00, //      #      
	0x04, 0x00, //      #      
	0x04, 0x00, //      #      
	0x3F, 0x80, //   #######   

	// @306 'I' (1 pixels wide)
	0x3F, 0x80, //   #######   

	// @308 'J' (4 pixels wide)
	0x01, 0x80, //        ##   
	0x00, 0x80, //         #   
	0x00, 0x80, //         #   
	0x3F, 0x00, //   ######    

	// @316 'K' (5 pixels wide)
	0x3F, 0x80, //   #######   
	0x04, 0x00, //      #      
	0x0A, 0x00, //     # #     
	0x11, 0x00, //    #   #    
	0x20, 0x80, //   #     #   

	// @326 'L' (4 pixels wide)
	0x3F, 0x80, //   #######   
	0x00, 0x80, //         #   
	0x00, 0x80, //         #   
	0x00, 0x80, //         #   

	// @334 'M' (7 pixels wide)
	0x3F, 0x80, //   #######   
	0x18, 0x00, //    ##       
	0x07, 0x00, //      ###    
	0x00, 0x80, //         #   
	0x07, 0x00, //      ###    
	0x18, 0x00, //    ##       
	0x3F, 0x80, //   #######   

	// @348 'N' (5 pixels wide)
	0x3F, 0x80, //   #######   
	0x18, 0x00, //    ##       
	0x04, 0x00, //      #      
	0x03, 0x00, //       ##    
	0x3F, 0x80, //   #######   

	// @358 'O' (5 pixels wide)
	0x1F, 0x00, //    #####    
	0x20, 0x80, //   #     #   
	0x20, 0x80, //   #     #   
	0x20, 0x80, //   #     #   
	0x1F, 0x00, //    #####    

	// @368 'P' (4 pixels wide)
	0x3F, 0x80, //   #######   
	0x24, 0x00, //   #  #      
	0x24, 0x00, //   #  #      
	0x3C, 0x00, //   ####      

	// @376 'Q' (5 pixels wide)
	0x1F, 0x00, //    #####    
	0x20, 0x80, //   #     #   
	0x20, 0x80, //   #     #   
	0x21, 0x80, //   #    ##   
	0x1E, 0x80, //    #### #   

	// @386 'R' (5 pixels wide)
	0x3F, 0x80, //   #######   
	0x24, 0x00, //   #  #      
	0x24, 0x00, //   #  #      
	0x26, 0x00, //   #  ##     
	0x19, 0x80, //    ##  ##   

	// @396 'S' (4 pixels wide)
	0x19, 0x00, //    ##  #    
	0x24, 0x80, //   #  #  #   
	0x24, 0x80, //   #  #  #   
	0x13, 0x00, //    #  ##    

	// @404 'T' (5 pixels wide)
	0x20, 0x00, //   #         
	0x20, 0x00, //   #         
	0x3F, 0x80, //   #######   
	0x20, 0x00, //   #         
	0x20, 0x00, //   #         

	// @414 'U' (5 pixels wide)
	0x3F, 0x00, //   ######    
	0x00, 0x80, //         #   
	0x00, 0x80, //         #   
	0x00, 0x80, //         #   
	0x3F, 0x00, //   ######    

	// @424 'V' (7 pixels wide)
	0x30, 0x00, //   ##        
	0x0C, 0x00, //     ##      
	0x03, 0x00, //       ##    
	0x00, 0x80, //         #   
	0x03, 0x00, //       ##    
	0x0C, 0x00, //     ##      
	0x30, 0x00, //   ##        

	// @438 'W' (9 pixels wide)
	0x30, 0x00, //   ##        
	0x0F, 0x00, //     ####    
	0x00, 0x80, //         #   
	0x1F, 0x00, //    #####    
	0x20, 0x00, //   #         
	0x1F, 0x00, //    #####    
	0x00, 0x80, //         #   
	0x0F, 0x00, //     ####    
	0x30, 0x00, //   ##        

	// @456 'X' (5 pixels wide)
	0x20, 0x80, //   #     #   
	0x1B, 0x00, //    ## ##    
	0x04, 0x00, //      #      
	0x1B, 0x00, //    ## ##    
	0x20, 0x80, //   #     #   

	// @466 'Y' (5 pixels wide)
	0x20, 0x00, //   #         
	0x18, 0x00, //    ##       
	0x07, 0x80, //      ####   
	0x18, 0x00, //    ##       
	0x20, 0x00, //   #         

	// @476 'Z' (6 pixels wide)
	0x21, 0x80, //   #    ##   
	0x22, 0x80, //   #   # #   
	0x24, 0x80, //   #  #  #   
	0x28, 0x80, //   # #   #   
	0x30, 0x80, //   ##    #   
	0x20, 0x80, //   #     #   

	// @488 '[' (2 pixels wide)
	0x3F, 0xE0, //   ######### 
	0x20, 0x20, //   #       # 

	// @492 '\' (3 pixels wide)
	0x30, 0x00, //   ##        
	0x0E, 0x00, //     ###     
	0x01, 0x80, //        ##   

	// @498 ']' (2 pixels wide)
	0x20, 0x20, //   #       # 
	0x3F, 0xE0, //   ######### 

	// @502 '^' (3 pixels wide)
	0x18, 0x00, //    ##       
	0x20, 0x00, //   #         
	0x18, 0x00, //    ##       

	// @508 '_' (5 pixels wide)
	0x00, 0x20, //           # 
	0x00, 0x20, //           # 
	0x00, 0x20, //           # 
	0x00, 0x20, //           # 
	0x00, 0x20, //           # 

	// @518 '`' (2 pixels wide)
	0x20, 0x00, //   #         
	0x10, 0x00, //    #        

	// @522 'a' (4 pixels wide)
	0x09, 0x80, //     #  ##   
	0x0A, 0x80, //     # # #   
	0x0A, 0x80, //     # # #   
	0x0F, 0x80, //     #####   

	// @530 'b' (4 pixels wide)
	0x3F, 0x80, //   #######   
	0x08, 0x80, //     #   #   
	0x08, 0x80, //     #   #   
	0x07, 0x00, //      ###    

	// @538 'c' (4 pixels wide)
	0x07, 0x00, //      ###    
	0x08, 0x80, //     #   #   
	0x08, 0x80, //     #   #   
	0x05, 0x00, //      # #    

	// @546 'd' (4 pixels wide)
	0x07, 0x00, //      ###    
	0x08, 0x80, //     #   #   
	0x08, 0x80, //     #   #   
	0x3F, 0x80, //   #######   

	// @554 'e' (4 pixels wide)
	0x07, 0x00, //      ###    
	0x0A, 0x80, //     # # #   
	0x0A, 0x80, //     # # #   
	0x06, 0x80, //      ## #   

	// @562 'f' (3 pixels wide)
	0x08, 0x00, //     #       
	0x1F, 0x80, //    ######   
	0x28, 0x00, //   # #       

	// @568 'g' (4 pixels wide)
	0x07, 0x40, //      ### #  
	0x08, 0xA0, //     #   # # 
	0x08, 0xA0, //     #   # # 
	0x0F, 0xC0, //     ######  

	// @576 'h' (4 pixels wide)
	0x3F, 0x80, //   #######   
	0x08, 0x00, //     #       
	0x08, 0x00, //     #       
	0x07, 0x80, //      ####   

	// @584 'i' (1 pixels wide)
	0x2F, 0x80, //   # #####   

	// @586 'j' (2 pixels wide)
	0x00, 0x20, //           # 
	0x2F, 0xC0, //   # ######  

	// @590 'k' (4 pixels wide)
	0x3F, 0x80, //   #######   
	0x02, 0x00, //       #     
	0x07, 0x00, //      ###    
	0x08, 0x80, //     #   #   

	// @598 'l' (1 pixels wide)
	0x3F, 0x80, //   #######   

	// @600 'm' (7 pixels wide)
	0x0F, 0x80, //     #####   
	0x08, 0x00, //     #       
	0x08, 0x00, //     #       
	0x0F, 0x80, //     #####   
	0x08, 0x00, //     #       
	0x08, 0x00, //     #       
	0x07, 0x80, //      ####   

	// @614 'n' (4 pixels wide)
	0x0F, 0x80, //     #####   
	0x08, 0x00, //     #       
	0x08, 0x00, //     #       
	0x07, 0x80, //      ####   

	// @622 'o' (4 pixels wide)
	0x07, 0x00, //      ###    
	0x08, 0x80, //     #   #   
	0x08, 0x80, //     #   #   
	0x07, 0x00, //      ###    

	// @630 'p' (4 pixels wide)
	0x0F, 0xE0, //     ####### 
	0x08, 0x80, //     #   #   
	0x08, 0x80, //     #   #   
	0x07, 0x00, //      ###    

	// @638 'q' (4 pixels wide)
	0x07, 0x00, //      ###    
	0x08, 0x80, //     #   #   
	0x08, 0x80, //     #   #   
	0x0F, 0xE0, //     ####### 

	// @646 'r' (3 pixels wide)
	0x0F, 0x80, //     #####   
	0x08, 0x00, //     #       
	0x08, 0x00, //     #       

	// @652 's' (4 pixels wide)
	0x04, 0x80, //      #  #   
	0x0A, 0x80, //     # # #   
	0x0A, 0x80, //     # # #   
	0x09, 0x00, //     #  #    

	// @660 't' (3 pixels wide)
	0x08, 0x00, //     #       
	0x1F, 0x80, //    ######   
	0x08, 0x80, //     #   #   

	// @666 'u' (4 pixels wide)
	0x0F, 0x00, //     ####    
	0x00, 0x80, //         #   
	0x00, 0x80, //         #   
	0x0F, 0x80, //     #####   

	// @674 'v' (5 pixels wide)
	0x08, 0x00, //     #       
	0x07, 0x00, //      ###    
	0x00, 0x80, //         #   
	0x07, 0x00, //      ###    
	0x08, 0x00, //     #       

	// @684 'w' (5 pixels wide)
	0x0F, 0x00, //     ####    
	0x00, 0x80, //         #   
	0x0F, 0x00, //     ####    
	0x00, 0x80, //         #   
	0x0F, 0x00, //     ####    

	// @694 'x' (4 pixels wide)
	0x08, 0x80, //     #   #   
	0x07, 0x00, //      ###    
	0x07, 0x00, //      ###    
	0x08, 0x80, //     #   #   

	// @702 'y' (5 pixels wide)
	0x08, 0x00, //     #       
	0x07, 0x20, //      ###  # 
	0x00, 0xC0, //         ##  
	0x07, 0x00, //      ###    
	0x08, 0x00, //     #       

	// @712 'z' (3 pixels wide)
	0x09, 0x80, //     #  ##   
	0x0A, 0x80, //     # # #   
	0x0C, 0x80, //     ##  #   

	// @718 '{' (3 pixels wide)
	0x02, 0x00, //       #     
	0x3D, 0xE0, //   #### #### 
	0x20, 0x20, //   #       # 

	// @724 '|' (1 pixels wide)
	0x3F, 0xC0, //   ########  

	// @726 '}' (3 pixels wide)
	0x20, 0x20, //   #       # 
	0x3D, 0xE0, //   #### #### 
	0x02, 0x00, //       #     

	// @732 '~' (4 pixels wide)
	0x06, 0x00, //      ##     
	0x04, 0x00, //      #      
	0x02, 0x00, //       #     
	0x06, 0x00, //      ##     
};

// Character descriptors for Arial 7pt
// { [Char width in bits], [Char height in bits], [Offset into arial_7ptCharBitmaps in bytes] }
const FONT_CHAR_INFO PROGMEM arial_7ptDescriptors[] = 
{
	{1, 12, 0}, 		// ' ' 
	{1, 12, 4}, 		// '!' 
	{3, 12, 6}, 		// '"' 
	{4, 12, 12}, 		// '#' 
	{5, 12, 20}, 		// '$' 
	{6, 12, 30}, 		// '%' 
	{6, 12, 42}, 		// '&' 
	{1, 12, 54}, 		// ''' 
	{3, 12, 56}, 		// '(' 
	{3, 12, 62}, 		// ')' 
	{3, 12, 68}, 		// '*' 
	{5, 12, 74}, 		// '+' 
	{1, 12, 84}, 		// ',' 
	{2, 12, 86}, 		// '-' 
	{1, 12, 90}, 		// '.' 
	{3, 12, 92}, 		// '/' 
	{4, 12, 98}, 		// '0' 
	{2, 12, 106}, 		// '1' 
	{4, 12, 110}, 		// '2' 
	{4, 12, 118}, 		// '3' 
	{5, 12, 126}, 		// '4' 
	{4, 12, 136}, 		// '5' 
	{4, 12, 144}, 		// '6' 
	{4, 12, 152}, 		// '7' 
	{4, 12, 160}, 		// '8' 
	{4, 12, 168}, 		// '9' 
	{1, 12, 176}, 		// ':' 
	{1, 12, 178}, 		// ';' 
	{3, 12, 180}, 		// '<' 
	{4, 12, 186}, 		// '=' 
	{3, 12, 194}, 		// '>' 
	{5, 12, 200}, 		// '?' 
	{8, 12, 210}, 		// '@' 
	{7, 12, 226}, 		// 'A' 
	{4, 12, 240}, 		// 'B' 
	{5, 12, 248}, 		// 'C' 
	{5, 12, 258}, 		// 'D' 
	{5, 12, 268}, 		// 'E' 
	{4, 12, 278}, 		// 'F' 
	{5, 12, 286}, 		// 'G' 
	{5, 12, 296}, 		// 'H' 
	{1, 12, 306}, 		// 'I' 
	{4, 12, 308}, 		// 'J' 
	{5, 12, 316}, 		// 'K' 
	{4, 12, 326}, 		// 'L' 
	{7, 12, 334}, 		// 'M' 
	{5, 12, 348}, 		// 'N' 
	{5, 12, 358}, 		// 'O' 
	{4, 12, 368}, 		// 'P' 
	{5, 12, 376}, 		// 'Q' 
	{5, 12, 386}, 		// 'R' 
	{4, 12, 396}, 		// 'S' 
	{5, 12, 404}, 		// 'T' 
	{5, 12, 414}, 		// 'U' 
	{7, 12, 424}, 		// 'V' 
	{9, 12, 438}, 		// 'W' 
	{5, 12, 456}, 		// 'X' 
	{5, 12, 466}, 		// 'Y' 
	{6, 12, 476}, 		// 'Z' 
	{2, 12, 488}, 		// '[' 
	{3, 12, 492}, 		// '\' 
	{2, 12, 498}, 		// ']' 
	{3, 12, 502}, 		// '^' 
	{5, 12, 508}, 		// '_' 
	{2, 12, 518}, 		// '`' 
	{4, 12, 522}, 		// 'a' 
	{4, 12, 530}, 		// 'b' 
	{4, 12, 538}, 		// 'c' 
	{4, 12, 546}, 		// 'd' 
	{4, 12, 554}, 		// 'e' 
	{3, 12, 562}, 		// 'f' 
	{4, 12, 568}, 		// 'g' 
	{4, 12, 576}, 		// 'h' 
	{1, 12, 584}, 		// 'i' 
	{2, 12, 586}, 		// 'j' 
	{4, 12, 590}, 		// 'k' 
	{1, 12, 598}, 		// 'l' 
	{7, 12, 600}, 		// 'm' 
	{4, 12, 614}, 		// 'n' 
	{4, 12, 622}, 		// 'o' 
	{4, 12, 630}, 		// 'p' 
	{4, 12, 638}, 		// 'q' 
	{3, 12, 646}, 		// 'r' 
	{4, 12, 652}, 		// 's' 
	{3, 12, 660}, 		// 't' 
	{4, 12, 666}, 		// 'u' 
	{5, 12, 674}, 		// 'v' 
	{5, 12, 684}, 		// 'w' 
	{4, 12, 694}, 		// 'x' 
	{5, 12, 702}, 		// 'y' 
	{3, 12, 712}, 		// 'z' 
	{3, 12, 718}, 		// '{' 
	{1, 12, 724}, 		// '|' 
	{3, 12, 726}, 		// '}' 
	{4, 12, 732}, 		// '~' 
};


// Font information for Arial 7pt
const FONT_INFO arial_7ptFontInfo =
{
	9, //  Character height
	' ', //  Start character
	'~', //  End character
	1, //  Width, in pixels, of space character
	arial_7ptDescriptors, //  Character descriptor array
	arial_7ptBitmaps, //  Character bitmap array
};
