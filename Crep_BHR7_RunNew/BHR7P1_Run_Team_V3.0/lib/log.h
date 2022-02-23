/***********************************************************************
 *  BHR BIPED CONTROL SOFTWARE UNDER Windows XP+Real_Time Extension    *  
 *	Filename:     log.h                                      *
 *	Jan.11, 2010, Originaly adapted for Windwos RTX8.1SP2 by Z.G. YU   *
 *  Save realtime and non_realtime response to file                    *
 *	    (C) Biped Group,  Intelligent Robitics Institute               *
 *         Beijing Institute of Technology, China                      *
 *           2001-2010, All rights reserved                            *
 ***********************************************************************/
#include <stdio.h>
#include <malloc.h>
#define True 1
#define False 0

int Log_Start(long size);
int Log_Data(double * dbuf,int dnum);
int Log_Stop(char *s,int col);

int Log_Start_RT(long size);
int Log_Data_RT(double * dbuf,int dnum);
int Log_Stop_RT(char *s,int col);

/*
int Rfdat_Start(long size);
int Read_Rfdata(char *s);
void Get_Rf_Data();
*/
 //End of the file log.h
 