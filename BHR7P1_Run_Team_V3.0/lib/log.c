/***********************************************************************
 *  BHR BIPED CONTROL SOFTWARE UNDER Windows XP+Real_Time Extension    *  
 *	Filename:     log.c                                      *
 *	Jan.11, 2010, Originaly adapted for Windwos RTX8.1SP2 by Z.G. YU   *
 *  Save realtime and non_realtime response to file                    *
 *	    (C) Biped Group,  Intelligent Robitics Institute               *
 *         Beijing Institute of Technology, China                      *
 *           2001-2010, All rights reserved                            *
 ***********************************************************************/

#include <string.h>
#include <stdio.h>
//#include <stdlib.h>
#include <errno.h>
#include "..\lib\log.h"

#pragma warning(disable : 4996)

double *log_buf = NULL;
long log_data_size,log_data_num;
int log_flag = False;

/******* Reserve result data ********/
int Log_Start(long size){

	if(log_flag == True )return -1;

	log_data_size = size ;
	log_buf = (double *)malloc( size * sizeof(double) );
		
	if(log_buf == NULL){
		printf("\n Sorry,we can't allocate [%zd]Kbytes\n", size * sizeof(double) /1000);
		log_flag = False;
		return -1;
	}

	log_flag = True;
	log_data_num = 0;
return 0;
}

int Log_Data(double * dbuf,int dnum){

	int i;

	if( log_flag == False)return(0);

	for( i = 0 ; i < dnum;i++){
		if( log_data_num > log_data_size)return i;
		*(log_buf + log_data_num) = *(dbuf + i);
		log_data_num ++;
	}

return i;
}


int Log_Stop(char *s,int col){
	
	FILE *fp;
	char filename[33];
	long l;
	int i;
	errno_t err;


	if(log_flag == False )return -1;
	if(log_buf == NULL )return -1;
	strcpy(filename,s);
//	strcpy_s(filename, _countof(s), s);
	if( (err  = fopen_s( &fp, filename, "wt" )) !=0 ){
		printf("\nError - file can't open.\n");
		free(log_buf);
		log_buf = NULL;
		return -1 ;
	}
	
	

	for( l = 0 ; l < log_data_num ; l += col)
	{

		for( i=0; (l + i) < log_data_num && i < col ;i++){
			if( i != 0)fprintf(fp,"\t");
			fprintf(fp,"%f", * (log_buf + l + i) );

		}
		//fprintf(fp, "\r\n"); //for linux, 
		fprintf(fp, "\n"); //for Microsoft C, '\n' is explained as \x0D and \x0A
	}
	fclose(fp);

	free(log_buf);
	log_buf = NULL;
	//log_flag = True;
	log_flag = False;
	return 0;	
}

/******* Reserve responese realtime ********/
int Log_Start_RT(long size){

	if(log_flag == True )return -1;

	log_data_size = size ;
	log_buf = (double *)malloc( size * sizeof(double) );
		
	if(log_buf == NULL){
		printf("\n Sorry,we can't allocate [%zd]Kbytes\n", size * sizeof(double) /1000);
		log_flag = False;
		return -1;
	}

	log_flag = True;
	log_data_num = 0;
return 0;
}

int Log_Data_RT(double * dbuf,int dnum){

	int i;

	if( log_flag == False)return(0);

	for( i = 0 ; i < dnum;i++){
		if( log_data_num > log_data_size)return i;
		*(log_buf + log_data_num) = *(dbuf + i);
		log_data_num ++;
	}

return i;
}


int Log_Stop_RT(char *s,int col){
	
	FILE *fp;
	char filename[33];
	long l;
	int i;
	errno_t err;


	if(log_flag == False )return -1;
	if(log_buf == NULL )return -1;
	strcpy(filename,s);
//	strcpy_s(filename, _countof(s), s);
	if( (err  = fopen_s( &fp, filename, "wt" )) !=0 ){
		printf("\nError - file can't open.\n");
		free(log_buf);
		log_buf = NULL;
		return -1 ;
	}
	
	

	for( l = 0 ; l < log_data_num ; l += col)
	{

		for( i=0; (l + i) < log_data_num && i < col ;i++){
			if( i != 0)fprintf(fp,"\t");
			fprintf(fp,"%f", * (log_buf + l + i) );

		}
		//fprintf(fp, "\r\n"); //for linux, 
		fprintf(fp, "\n"); //for Microsoft C, '\n' is explained as \x0D and \x0A
	}
	fclose(fp);

	free(log_buf);
	log_buf = NULL;
	//log_flag = True;
	log_flag = False;
return 0;	
}

//End of the file log.c



