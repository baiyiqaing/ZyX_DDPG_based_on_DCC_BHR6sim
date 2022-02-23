/***********************************************************************
 *  BHR BIPED CONTROL SOFTWARE UNDER Windows XP+Real_Time Extension    *  
 *	Filename:     rtx_process.h                                        *
 *	Sept.16, 2010, Originaly created for Windwos RTX8.1SP2 by Z.G. YU  *
 ***********************************************************************/
#ifndef RTX_PROCESS_H
#define RTX_PROCESS_H

#ifdef    RTX_PROCESS_C
#define   EXTERN    /*define*/
#else
#define   EXTERN    extern
#endif

#define RFDAT_BUF_NUM  5000000  //good for pingpong
EXTERN double Rfdat_Buf[RFDAT_BUF_NUM];
EXTERN void rtx_delayus(int us);
EXTERN int SendCmd(int channel, unsigned int msgID , unsigned  char *candata, unsigned char bytelength);
EXTERN float RtResponse_Buf[10000000L]; //buffer for realtime response
#undef EXTERN

#endif