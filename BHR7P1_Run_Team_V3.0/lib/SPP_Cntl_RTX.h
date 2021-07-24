/***********************************************************************
 *  BHR BIPED CONTROL SOFTWARE UNDER Windows XP+Real_Time Extension    *  
 *	Filename:  SPP_Cntl_RTX.h                                          *
 *	 *	Jan.11, 2011,  Created for Windwos RTX8.1SP2                   *
 *	    (C) Biped Group,  Intelligent Robitics Institute               *
 *         Beijing Institute of Technology, China                      *
 *           2001-2010, All rights reserved                            *
 ***********************************************************************/
#ifndef __SPP_CNTL_H__
#define __SPP_CNTL_H__

/*Bits of Status Port(0x378):
Bit0(Pin2)
Bit1(Pin3)
.....
Bit7(Pin9)
===========================
Bits of Status Port(0x379):
Bits0-2: Not available
Bit3(Pin15): -Error
Bit4(Pin13): +Select in
Bit5(Pin12): +Paper out
Bit6(Pin10): -ACK
Bit8(Pin11): -BUSY(NOT)
=======================
Bits of Control Port(0x37A):
Bit0(pin1): +Strobe (NOT)
Bit1(pin14): +AUTO Linefeed (NOT)
Bit2(pin16): -INIT
Bit3(pin13?): +SLCT IN(NOT)
Bit4(pin10): +IRQ
Bits5-7:Not available
*/

//port: 0->Data port, 1->Status port, 2->Control port

void Init_SPP_RTX(void);
void SPP_RTX_Write(int port, unsigned char value);
unsigned char SPP_RTX_Read(int port);
unsigned char SPP_RTX_Read_DataPort(void);
unsigned char SPP_RTX_Read_StatusPort(void);
unsigned char SPP_RTX_Read_ControlPort(void);
void SPP_HighPin17(void);
void SPP_LowPin17(void);
void Shut_SPP_RTX(void);

#endif
