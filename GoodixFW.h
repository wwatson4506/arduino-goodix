/*
 * License: Refer to LICENSE file in this directory.
 * Modified: For use with Teensy 4.x and MicroMod
 *       By: Warren Watson 2024.
*/

// Configuration data of the 911 chip. Should be sent to address 0x8047

uint8_t PROGMEM g911xFW[] = {
// ER-TFTM101-1 GT9271
0x63,0x00,0x04,0x58,0x02,0x0A,0x3D,0x00,
0x01,0x08,0x28,0x0F,0x50,0x32,0x03,0x05,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x17,
0x19,0x1D,0x14,0x90,0x2F,0x89,0x23,0x25,
0xD3,0x07,0x00,0x00,0x00,0x02,0x03,0x1D,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x19,0x32,0x94,0xD5,0x02,
0x07,0x00,0x00,0x04,0xA2,0x1A,0x00,0x90,
0x1E,0x00,0x80,0x23,0x00,0x73,0x28,0x00,
0x68,0x2E,0x00,0x68,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x16,0x15,0x14,0x11,0x10,0x0F,0x0E,0x0D,
0x0C,0x09,0x08,0x07,0x06,0x05,0x04,0x01,
0x00,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x29,0x28,
0x27,0x26,0x25,0x24,0x23,0x22,0x21,0x20,
0x1F,0x1E,0x1C,0x1B,0x19,0x14,0x13,0x12,
0x11,0x10,0x0F,0x0E,0x0D,0x0C,0x0A,0x08,
0x07,0x06,0x04,0x02,0x00,0xFF,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x71,0x01
        /*0x42, 0xD0, 0x02, 0x00, 0x05, 0x05, 0x75, 0x01, 0x01, 0x0F, 0x24,
        0x0F, 0x64, 0x3C, 0x03, 0x05, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 
        0x00, 0x16, 0x19, 0x1C, 0x14, 0x8C, 0x0E, 0x0E, 0x24, 0x00, 0x31, 
        0x0D, 0x00, 0x00, 0x00, 0x83, 0x33, 0x1D, 0x00, 0x41, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x08, 0x0A, 0x00, 0x2B, 0x1C, 0x3C, 0x94, 0xD5, 
        0x03, 0x08, 0x00, 0x00, 0x04, 0x93, 0x1E, 0x00, 0x82, 0x23, 0x00, 
        0x74, 0x29, 0x00, 0x69, 0x2F, 0x00, 0x5F, 0x37, 0x00, 0x5F, 0x20, 
        0x40, 0x60, 0x00, 0xF0, 0x40, 0x30, 0x55, 0x50, 0x27, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x19, 0x00, 0x00, 
        0x50, 0x50, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x12, 
        0x14, 0x16, 0x18, 0x1A, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1D, 
        0x1E, 0x1F, 0x20, 0x21, 0x22, 0x24, 0x26, 0x28, 0x29, 0x2A, 0x1C, 
        0x18, 0x16, 0x14, 0x13, 0x12, 0x10, 0x0F, 0x0C, 0x0A, 0x08, 0x06, 
        0x04, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9C, 0x01*/ //(186 bytes)
    
    // This configuration was sent with the display
    /*0x41, // Version letter in ASCII 'A' = 0x41, 'Z' = 0x5A - 0 will reset version to 'A'
    0x00, 0x05, 0x90, 0x01, 0x05, 0x0D, 0x00,
    0x01, 0x08, 0x28, 0x05, 0x50, 0x32, 0x03, 0x05,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x8B, 0x2B, 0x0A, 0x22, 0x24,
    0x31, 0x0D, 0x00, 0x00, 0x00, 0x01, 0x03, 0x2D,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x64, 0x32,
    0x00, 0x00, 0x00, 0x1A, 0x40, 0x94, 0xC5, 0x02,
    0x07, 0x00, 0x00, 0x04, 0x95, 0x1C, 0x00, 0x7F,
    0x22, 0x00, 0x71, 0x28, 0x00, 0x62, 0x31, 0x00,
    0x58, 0x3A, 0x00, 0x58, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10,
    0x12, 0x14, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x18,
    0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x24,
    0x26, 0x13, 0x12, 0x10, 0x0F, 0x0C, 0x0A, 0x08,
    0x06, 0x04, 0x02, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xE3, 0x01*/ // 186 bytes
    
    // This config is on the display from the factory:
/*    0x5B, // Version is [ - the character beyond Z.
    0x00, 0x05, 0x90, 0x01, 0x05, 0x0D, 0x00,
    0x01, 0x08, 0x28, 0x05, 0x50, 0x32, 0x30, 0x05,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x8B, 0x2B, 0x0B, 0x22, 0x24,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42,
    0x75, 0x09, 0x1B, 0x08, 0x8F, 0x09, 0x52, 0x09,
    0x44, 0x09, 0x5F, 0x09, 0x69, 0x09, 0x45, 0x09,
    0x37, 0x09, 0x5B, 0x09, 0x62, 0x09, 0x5F, 0x09,
    0x02, 0x08, 0x80, 0x09, 0x45, 0x09, 0x34, 0x09,
    0xD4, 0x08, 0x6C, 0x09, 0x28, 0x09, 0x17, 0x09,
    0x2D, 0x09, 0x3A, 0x09, 0x15, 0x09, 0x05, 0x09,
    0x28, 0x09, 0x2D, 0x09, 0x28, 0x08, 0xD2, 0x08,
    0x69, 0x09, 0x20, 0x09, 0x11, 0x09, 0x29, 0x09,
    0xFC, 0x09, 0x20, 0x09, 0x24, 0x09, 0x20, 0x08,
    0xD5, 0x09, 0x89, 0x09, 0x6A, 0x09, 0x51, 0x09,
    0x66, 0x09, 0x70, 0x09, 0x47, 0x09, 0x36, 0x09,
    0x58, 0x09*/
};

