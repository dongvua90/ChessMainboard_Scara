#ifndef AS5600_H
#define AS5600_H

#include "main.h"
#include "tim.h"

#define _AS5600Address       0x36
#define _ZMCOAddress         0x00
#define _ZPOSAddressLSB      0x02
#define _MPOSAddressMSB      0x03
#define _MPOSAddressLSB      0x04
#define _MANGAddressMSB      0x05
#define _MANGAddressLSB      0x06
#define _CONFAddressMSB      0x07
#define _CONFAddressLSB      0x08
#define _RAWANGLEAddressMSB  0x0C
#define _RAWANGLEAddressLSB  0x0D
#define _ANGLEAddressMSB     0x0E
#define _ANGLEAddressLSB     0x0F
#define _STATUSAddress       0x0B
#define _AGCAddress          0x1A
#define _MAGNITUDEAddressMSB 0x1B
#define _MAGNITUDEAddressLSB 0x1C
#define _BURNAddress         0xFF

void AS5600_M1_getPOS();
void AS5600_M2_getPOS();
char AS5600_M1_status();
char AS5600_M2_status();
void AS5600_Start_Update();
void AS5600_Start_Update_Low();
void AS5600_Start_Update_High();




#endif
