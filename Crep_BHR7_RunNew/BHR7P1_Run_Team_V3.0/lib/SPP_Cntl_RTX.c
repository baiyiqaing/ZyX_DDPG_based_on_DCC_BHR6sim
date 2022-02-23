/***********************************************************************
 *  BHR BIPED CONTROL SOFTWARE UNDER Windows XP+Real_Time Extension    *  
 *	Filename:  SPP_Cntl_RTX.c                                          *
 *	 *	Jan.11, 2011,  Created for Windwos RTX8.1SP2                   *
 *	    (C) Biped Group,  Intelligent Robitics Institute               *
 *         Beijing Institute of Technology, China                      *
 *           2001-2010, All rights reserved                            *
 ***********************************************************************/
#include "windows.h"
#include "stdio.h"
//#include "rtapi.h"

#define readw(addr) (*((volatile unsigned short *)((void *)(addr)))) //read memory 
#define readb(addr) (*((volatile unsigned char *)((void *)(addr))))
#define writel(dat,addr) (*((volatile unsigned int *)((void *)(addr)))=(dat))
#define writew(dat,addr) (*((volatile unsigned short *)((void *)(addr)))=(dat))


#pragma warning(disable : 4312)
#pragma warning(disable : 4311)

static unsigned int iobase=0;
			
void SPP_RTX_Write(int port, unsigned char value )
{
	RtWritePortUchar((PUCHAR)(iobase + port), value);
}

unsigned char SPP_RTX_Read(int port)
{

	UCHAR result = 0;
	//result = RtReadPortUchar((PUCHAR)(iobase + port));
	return result;
}

unsigned char SPP_RTX_Read_DataPort(void)
{
	UCHAR result = 0;
	//result = RtReadPortUchar((PUCHAR)(iobase)); //
	return	result;
}

unsigned char SPP_RTX_Read_StatusPort(void)
{
	UCHAR result = 0;
	//result = RtReadPortUchar((PUCHAR)(iobase + 1));
	return	result;
}

unsigned char SPP_RTX_Read_ControlPort(void)
{
	UCHAR result = 0;
	//result = RtReadPortUchar((PUCHAR)(iobase +2));
	return	result;
}

void SPP_HighPin17(void)
{
	//SPP_RTX_Write(2,0x08);
	UCHAR result = 0;
	//result = RtReadPortUchar((PUCHAR)(iobase +2));
	//RtWritePortUchar((PUCHAR)(iobase +2), result&(~(1<<3)));
}

void SPP_LowPin17(void)
{
	UCHAR result = 0;
	//result = RtReadPortUchar((PUCHAR)(iobase +2));
	//SPP_RTX_Write(2,result|(1<<3));
}

void Init_SPP_RTX(void)
{
	unsigned long BaseAddress = 0x00000408UL;
	//unsigned long MappedAddr;
	//LARGE_INTEGER  physaddrValue;

	//physaddrValue.QuadPart = BaseAddress;
	//MappedAddr = (ULONG)RtMapMemory( physaddrValue, 0x2L, FALSE);
	//iobase = readw(MappedAddr);	
//	iobase=0x0378;
	//printf("SPP Base Addr:0x %x\n",iobase);
	
	//if(!RtEnablePortIo( (PCHAR)iobase, 8))
			//printf("\n SPP IO permission enable Error");
}

void Shut_SPP_RTX(void)
{
	//if(!RtDisablePortIo( (PCHAR)iobase, 3))
			//printf("\n SPP IO permission disable Error");
}