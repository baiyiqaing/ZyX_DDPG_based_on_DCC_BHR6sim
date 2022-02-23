/***********************************************************************
 *  BHR BIPED CONTROL SOFTWARE UNDER Windows XP+Real_Time Extension    *  
 *	Filename:     robot_hmi.c                                          *
 *	Sept.16, 2010, Originaly created for Windwos RTX8.1SP2 by Z.G. YU  *
 *					CANopen DS402 for Elmo and kollmorgen drivers      *
 *   if any question, feel free to contact: yuzg75@gmail.com           *
 *                                                                     *
 *	Jan. 2010, modified for Pingpong-robot, CANopen protocol           *	
 * 			            	by Z.G. YU and X.C. CHEN                   *                        
 *	Jan. 2007, modified for BHR-3M distributed control systems         *
 *      based on CAN-bus (in linux)	by YU and CHEN                     *
 *	2004 March 28 modified by Peng Du  for BHR-2 in Linux              *
 *	2002 July 12  modified by Wang Guang for BHR-1 in Linux	           *
 *	Launch in 2001 by Prof. Huang, Enviroments: Linux+ RTLinux         *
 *	    (C) Biped Group,  Intelligent Robitics Institute               *
 *         Beijing Institute of Technology, China                      *
 *           2001-2010, All rights reserved                            *
 *%%%%%%%%%%%%%%%%%%%%%%% Function discription %%%%%%%%%%%%%%%%%%%%%%%%*
 * (1) Keyboard control                                                *
 * (2) Open shared Meomory and transmit offline trajectory file        *
 * (3) Communicate with rtx-process by  shared Meomory                 *
 *  -transfer plan data and control signal to rtx-process              *
 *  -recall control response from rtx-process                          *
 * (4) Display control response dynamically                            *
 * (5) save data to the File log.dat                                   * 
 *------------------Key board operation sequence-----------------------*
 *(1)General operation:                                                *
 *   9->servo power on,                                                *
 *   5->reset initial walking state,                                   *
 *   0->offset inertial sensors after robot standing on ground         *
 *   3->enable sensory reflex control                                  *
 *   7->execute planning motion                                        *
 *   1->suspend(pause) planning motion                                 *
 *   2->disable sensory reflex control                                 *
 *   8->servo power off                                                *
 *   P->Initialize Arm initial posture for rally Ping-pong             *
 *   L->Move Leg while striking ball                                   *
 *   m/f->record log file                                              *
 *   Space ->to start RTLinux or exit this routine                     *   
 *(2)Search joint home position:                                       *
 *   Press '6'                                                         *
 ***********************************************************************/
#include <windows.h>
#include <stdio.h>
#include <rtapi.h>
#include <stdlib.h> //system()
#include <conio.h>
#include <ctype.h>
#include "control.h"
#include "lib\hardware_conf.h"
#include "lib\log.h"
#include "lib\DCC_Run_ReadConfig.h"
#include <ReadRosLib\ReadRos.h>

//#define USE_ROS_EST
#define MAXCANBYTES 10 
//1个帧包含16个字节,接收视觉数据
//0x55,0xaa,1byte(Flag),2byte(Low byte-High byte),2byte,2byte,2byte,1byte(Check_Sum)

#define ROBOT_NETADDR "192.168.1.166"

#define VISION_NETADDR "192.168.1.116" //VISION_Computer wifi ip-address
#define NETPORT 7592//6150
//#pragma warning(disable : 4996)
#pragma warning(disable : 4244)
#pragma warning(disable : 4101)

#define NO_ERRORS		0
#define ERROR_OCCURED	-1
#define	ERROR_CODE		1

#define SAVE_FILE_BUF_NUM  	6000000L      /* Control data record area */
#define ADD_COUNT               38 /* newly buf added, after 85 */
#define LOG_FIELD               (85+ADD_COUNT+62) //50
int	DataLogStart = 0;

int Reflex_Control_On = 0x55;

extern VOID  GotoConsoleXY(int x, int y);  
extern void Send_Command( PCtrlMsg pCmdMsg, HANDLE mutex,int cmd, int arg1 , int arg2);
extern int Transfer_Trajectory(char *file_name, int FileIDN, HANDLE Mutex_Cntl, HANDLE Mutex_Data, PMSGSTR pMsg_Data,PCtrlMsg pMsg_Cntl);

void Display_Joint(double *f);

 /*
int WINAPI WinMain(
    HINSTANCE	hInstance,
    HINSTANCE	hPrevInstance,
    LPSTR	lpCmdLine,
    int		nCmdShow
    ) 
*/
typedef struct {
    long  Index;		//  flag
	long  arg2; 
    double	data[200];
} WIN_MEM_SHARE, *PWIN_MEM_SHARE;

static union{
	unsigned char cv[2]; //sizeof(char) is 1
	short int siv; //sizeof(short int) is 4
}siv2cv;
static int Byte2Short(unsigned char * const Byt); 
int Byte2Short(unsigned char * const Byt) 
 { 

	siv2cv.cv[0] = *Byt;
	siv2cv.cv[1] = *(Byt+1);
	return siv2cv.siv;
  }
  
/****************************************************************************
*Name:Byte2Long() 
*****************************************************************************/
static union{
	unsigned char cv[4]; //sizeof(char) is 1
	long lv; //sizeof(long) is 4
}lv2cv; 
static int Byte2Long(unsigned char * const Byt);
//enc_val[i][k] = Byte2Long(&canmsgfRX.data[0]);
static int Byte2Long(unsigned char * const Byt) 
{ 
	int i;
	for(i=0; i<4; i++)
		lv2cv.cv[i] = *(Byt+i);
	return lv2cv.lv;
}

	
int main(int argc, char **argv)
{
	HANDLE	hShm_Data, hShm_Cntl, hMutex_Data, hMutex_Cntl;
	//HANDLE	hRESP_Shm_Data; //response realtime
	HANDLE	hSemPost;
	PMSGSTR pMsg_Data;
	PRESP_MSGSTR pRESP_Msg_Data;
	PCtrlMsg pMsg_Cntl;
	HANDLE  mutex = NULL;
	long wifi_dat1, wifi_dat2;

	DWORD	dwMaximumSizeHigh = 0;
	LONG	lInitialCount = 0;
	LONG	lMaximumCount = 1;
	LONG	lReleaseCount = 1;

	SHORT keycmd =0;
	int i=0;
	int temp=0;
	int k = 0;
	LONG NonRtControlledTime = 0;
	int n = 0;
	double ftemp=0.0;
	int Response_Count_Last = 0;
	int Response_Count = 0;
	BOOL bFlag=FALSE;
	int Vertical_Initial = 0; //vertical leg
	int Bended_Initial = 0; //bend leg
	int Sin_Traject = 0;
	int update_n = 0; //display update
	int SoftReset_CAN = 0;
	int RESPONSE_RECORD_CYCLE = 1;
	LONG pre_index = -1;
	DWORD dw;
	
	int FileTrans[10]={0,0,0,0,0,0,0,0,0,0};  //whp, 防止误读离线文件
	int fnum=0;
	int ii=0;
	
	double cmd_arg[10]; //Send_Command()浮点参数
	FILE *fp;
	int logfile_ID = 0;
	char logfilename[30];
	char rt_logfilename[30];
	errno_t errno;
	BOOL auto_reset_elmo = FALSE;
	int Reset_Elmo_Save_Encoder = 0;
	
#if 1 //generate logfile ID 
	if((errno=fopen_s( &fp, "lib\\LogFile_ID.txt", "rt" ))==0){		
		rewind(fp);
		if(!feof(fp))
			fscanf_s(fp,"%d",&logfile_ID);
		fclose(fp);
		logfile_ID++;
		if(logfile_ID>5)
			logfile_ID = 0;
	} else {
		printf("\n Open file_id failed");
		logfile_ID = 0;
	}

	if((errno=fopen_s(&fp, "lib\\LogFile_ID.txt","wt"))==0)
	{
		fprintf_s(fp,"%d",logfile_ID);
		fclose(fp);
	} else	{
		printf("\n Creat file_id failed");
	}
	sprintf_s(logfilename,30,"log_%d.dat",logfile_ID);
	sprintf_s(rt_logfilename,30,"rtlog_%d.dat",logfile_ID);
#endif	
	system("cls"); //good
	GotoConsoleXY(1,1);

    // Create required RTX IPC objects -- mutex is last so that clients
    // can not start sending messages before the server is initialized.    
	SleepEx(1000, FALSE); //wait 1sec

	hMutex_Data = RtOpenMutex(SYNCHRONIZE, FALSE, (LPCWSTR)MSGSTR_MUTEX_DATA);
	if (hMutex_Data==NULL)
	{
		printf("Could not open Mutex DATA.  GetLastError = %d\n", GetLastError());
		return FALSE;
	}
	hMutex_Cntl = RtOpenMutex(SYNCHRONIZE, FALSE, (LPCWSTR)CTRLMSG_MUTEX_CNTL);
	if (hMutex_Cntl==NULL)
	{
		printf("Could not open Mutex DATA.  GetLastError = %d\n", GetLastError());
		return FALSE;
	}
	
	hShm_Data = RtOpenSharedMemory( PAGE_READWRITE, FALSE, (LPCWSTR)MSGSTR_SHM_DATA, (LPVOID) &pMsg_Data);
	if (hShm_Data==NULL)
	{
		printf("Could not open Shared Memory.  GetLastError = %d\n", GetLastError());
		return FALSE;
	}
	// hRESP_Shm_Data = RtOpenSharedMemory( PAGE_READWRITE, FALSE, (LPCWSTR)RT_MSGSTR_SHM_DATA, (LPVOID) &pRESP_Msg_Data);
	// if (hRESP_Shm_Data==NULL)
	// {
		// printf("Could not open Shared RT response Memory.  GetLastError = %d\n", GetLastError());
		// return FALSE;
	// }
	
	hShm_Cntl = RtOpenSharedMemory( PAGE_READWRITE, FALSE, (LPCWSTR)CTRLMSG_SHM_CNTL, (LPVOID) &pMsg_Cntl);
	if (hShm_Cntl==NULL)
	{
		printf("Could not open Shared Memory.  GetLastError = %d\n", GetLastError());
		return FALSE;
	}
	// hSemPost = RtOpenSemaphore( SYNCHRONIZE, FALSE,(LPCWSTR)MSGSTR_SEM_POST);
	// if (hSemPost==NULL)
	// {
			// printf("Could not open Semaphore.  GetLastError = %d\n", GetLastError());
			// RtCloseHandle(hShm_Data);
			// //RtCloseHandle(hRESP_Shm_Data);
			// RtCloseHandle(hShm_Cntl);
			// RtCloseHandle(hMutex_Data);
			// return FALSE;
	// }
	// printf("\n Now Windows_RTX init hardware, please wait patiently....");
// //wait here until rtx_process has completed its initialization
	// if(RtWaitForSingleObject(hSemPost, INFINITE)==WAIT_FAILED)
			// printf("RtWait hSemPost failed.");

	//读取参数文件
	NRT_ReadDCCRunParms(&dccRunParms,L"DCCRunConConfig.json");
	
//sent offline data to the rtx_process.rtss
						/****************Data Transfer*****************
						 *                                           *
						 *                                           *
						 *          Change directory Here!!!!        *
						 *                  ┌──┐                     *
						 *                  |  |                     *
						 *                  ┘  └                     *
						 *                  \  /                     * 
						 *                   \/                      *
						 ********************************************/
#if 1
	FileTrans[fnum++] = Transfer_Trajectory("data\\old_offline_3.0.dat",		 0, hMutex_Cntl, hMutex_Data, pMsg_Data, pMsg_Cntl);//打招呼
//	FileTrans[fnum++] = Transfer_Trajectory("data\\big_sep.dat",		 0, hMutex_Cntl, hMutex_Data, pMsg_Data, pMsg_Cntl);//打招呼
	FileTrans[fnum++] = Transfer_Trajectory("data\\crawl_down.dat",  1, hMutex_Cntl, hMutex_Data, pMsg_Data, pMsg_Cntl);//爬行下蹲
	FileTrans[fnum++] = Transfer_Trajectory("data\\crawl.dat",		 2, hMutex_Cntl, hMutex_Data, pMsg_Data, pMsg_Cntl);//爬行
	FileTrans[fnum++] = Transfer_Trajectory("data\\crawl_up.dat",	 3, hMutex_Cntl, hMutex_Data, pMsg_Data, pMsg_Cntl);//爬行起立
	FileTrans[fnum++] = Transfer_Trajectory("data\\shuai.dat",       4, hMutex_Cntl, hMutex_Data, pMsg_Data, pMsg_Cntl);//摔倒保护
	FileTrans[fnum++] = Transfer_Trajectory("data\\crouch.dat",      5, hMutex_Cntl, hMutex_Data, pMsg_Data, pMsg_Cntl);//摔倒起立
	FileTrans[fnum++] = Transfer_Trajectory("data\\button_press.dat",6, hMutex_Cntl, hMutex_Data, pMsg_Data, pMsg_Cntl);//按按钮
	
#endif


	for(ii=0;ii<fnum;ii++)  //防止误读离线文件 whp
	{
		if(FileTrans[ii]!=ii)
		{
			printf("\n*****,error occurs while transferring trajectory data!****");
			Send_Command(pMsg_Cntl, hMutex_Cntl, RT_START, 0, 0);
			goto exitroutine;
		}
	}
	
	Send_Command(pMsg_Cntl, hMutex_Cntl, OFFLINE_DATA_END, 0, 0); //end of tranfer	
	if ( Log_Start(SAVE_FILE_BUF_NUM) != 0) return -1;	

	
	printf("\t Press any key to continue");
	_getch();	
	SleepEx(300, FALSE); 
	
#ifdef USE_ROS_EST
	//初始化ROS_Estimation
	NRT_InitRosEstm("192.168.1.102");
	// 读取测试，成功
	for (int i = 0; i < 10; i++) {
		printf("Test: ");
		NRT_ReadRosEstm(&rosData); SleepEx(100, FALSE);
		NRT_TestRosEstm();
	}
	SleepEx(100, FALSE);
#endif
//------------- processing key stroke-------------------	
	while(keycmd!=' ')
    {
		if(_kbhit()){
			keycmd = _getch(); 			
		}else{
			keycmd = 0;
		}
		switch (keycmd){		

		//case '7':
		//	Send_Command(pMsg_Cntl, hMutex_Cntl, START_WALK, 1 , 0);
		//	//printf("\n Key press B");
		//break;
		case '5':		
			Send_Command(pMsg_Cntl, hMutex_Cntl, HOME_ON_LEG, 1 , 0);
		break;
		
		/*case '6':
			Send_Command(pMsg_Cntl, hMutex_Cntl, SEARCH_HOME, 1 , 0);
		break;*/
		case '9':
			Send_Command(pMsg_Cntl, hMutex_Cntl, POWER_ON, 1 , 0);
			RESPONSE_RECORD_CYCLE = 6;
		break;
		//case '8':
		//	Send_Command(pMsg_Cntl, hMutex_Cntl, POWER_ON, 0 , 0);
		//	//elmo_poweron_flag = TRUE;
		//break;
		//case '0':
		//	//Offset_On = TRUE; //offset inertial sensor, acc., gyro
		//	Send_Command(pMsg_Cntl, hMutex_Cntl, OFFSET_GYRO_ACC, 0 , 0);			
		//break;
			
		//case 'P':
		//case 'p':
		//	Send_Command(pMsg_Cntl, hMutex_Cntl, ARM_HOME_ON, 0 , 0);
		//	//home_on_arm_flag= TRUE;
		//break;
		//case 'L':
		//case 'l':
		//	Send_Command(pMsg_Cntl, hMutex_Cntl, WAIST_LEG_ONLINE, 0 , 0);		
		//break;
		//case 'J'://V
		//case 'j':

		//	if(++ Vertical_Initial == 3){
		//		Send_Command(pMsg_Cntl, hMutex_Cntl, HOME_VERTICAL_LEG, 1 , 0);
		//		Vertical_Initial = 0;
		//	}
		//break;
		//case 'U'://B
		//case 'u':
		//	if(++ Bended_Initial == 3){
		//		Send_Command(pMsg_Cntl, hMutex_Cntl, HOME_VERTICAL_LEG, 0 , 0);
		//		Bended_Initial = 0;
		//	}
		//break;	
				
		case 'M':
		case 'm':
			DataLogStart = 1; 			
			RESPONSE_RECORD_CYCLE = 1;
			Send_Command(pMsg_Cntl, hMutex_Cntl, RECORD_RT_DAT, 1 , 0);
		break;

		//case 'S':
		//case 's':
		//	if(++ Sin_Traject == 3){
		//		Send_Command(pMsg_Cntl, hMutex_Cntl, SIN_TRAJ_TEST, 1 , 0); 
		//	}
		//break;
		//case 'O':
		//case 'o':
		//	Send_Command(pMsg_Cntl, hMutex_Cntl, FALL_START_FLAG, 0 , 0); 
		//	DataLogStart = 0;
		//break;
		 case '/'://F
		 case '?':
			RESPONSE_RECORD_CYCLE = 6;
			Send_Command(pMsg_Cntl, hMutex_Cntl, RECORD_RT_DAT, 0 , 0); 
			DataLogStart = 0;
		break;
		
		case 'I': //Reset elmo driver, but reserve encoder value//R
		case 'i':

			if(++ Reset_Elmo_Save_Encoder == 3){
				Send_Command(pMsg_Cntl, hMutex_Cntl, RESET_ELMO_SAVE_ENCODER, 1 , 0);
				SoftReset_CAN = 0;
			}
		break;	
		case '8'://X
		case '*':

			if(++ SoftReset_CAN == 3){
				Send_Command(pMsg_Cntl, hMutex_Cntl, SOFTRESET_CANOPEN, 1 , 0);
				SoftReset_CAN = 0;
			}
		break;


		case ' ':
			Send_Command(pMsg_Cntl, hMutex_Cntl, RT_START, 0, 0);
		break;	

		/***************** FALL AND WALK LQQ - 20170529 state transition ******************/		
		//case '1':
		//	Send_Command(pMsg_Cntl, hMutex_Cntl, HELLO, 1, 0);
		//	break;
		//case '2':
		//	Send_Command(pMsg_Cntl, hMutex_Cntl, CRAWL, 1, 0);
		//	break;
		//case '3':
		//	Send_Command(pMsg_Cntl, hMutex_Cntl, FALL_DL, 1, 0);
		//	break;
		//case '4':
		//	Send_Command(pMsg_Cntl, hMutex_Cntl, ACTION, 1, 0);
		//	break;
		
		case 'k': //walk
		case 'K':
		    Send_Command(pMsg_Cntl, hMutex_Cntl, PRE_CON_MODE, 0, 0);
			break;
		
		
		
		/*********************** Move Keyboard Control by Previre Control ***********************/	
		/* Walk */
		case 'R':
		case 'r':
			Send_Command(pMsg_Cntl, hMutex_Cntl, PRECON_MOVE_FORWARD, 1, 0);
			break;
			
		///* Move Left Side */
		//case  '[':
		//	Send_Command(pMsg_Cntl, hMutex_Cntl, PRECON_MOVE_LEFT, 0, 0);
		//	break;
		//	
		///* Move right Side */
		//case  ']':
		//	Send_Command(pMsg_Cntl, hMutex_Cntl, PRECON_MOVE_RIGHT, 0, 0);
		//	break;	
		//	
		///* Walk backward */
		//case  ';':
		//	Send_Command(pMsg_Cntl, hMutex_Cntl, PRECON_MOVE_BACK, 0, 0);
		//	break;	
		
		default:
		
		break;	
	
		}
#ifdef USE_ROS_EST
		// 读取 Ros 信息
		NRT_ReadRosEstm(&rosData);
#endif
//----------end of processing key stroke----------------
//------------display response here -----------------
	
		Response_Count = pMsg_Data->Response_Index;
		if(Response_Count != Response_Count_Last){
			k = 0;
			//GotoConsoleXY(1,0); //first param: longitude, second param: latitude
			Display_Joint( (double *)pMsg_Data->Buffer);
			Response_Count_Last = Response_Count;	 //important
			
		if( DataLogStart == 1 && update_n == 0) 
		//if( DataLogStart == 1)			
				Log_Data(pMsg_Data->Buffer, LOG_FIELD);
			update_n = (update_n + 1) % RESPONSE_RECORD_CYCLE;
		}

    } //end while
	//------end of display response data --------------
	//Log_Stop("log.dat", LOG_FIELD); //
	 Log_Stop(logfilename, LOG_FIELD); //	
//---------------save realtime response--------
#if 0
	SleepEx(1000, FALSE);
	Send_Command(pMsg_Cntl, hMutex_Cntl, RT_DAT_SAVE, 0, 0);
	if ( Log_Start_RT(SAVE_FILE_BUF_NUM) != 0) return -1;
	SleepEx(60, FALSE);
	
	do {		
		dw = RtWaitForSingleObject(hMutex_Data, INFINITE );/*the handle of object to wait for ever*/
		if(dw==WAIT_FAILED){
			printf("\n Mutex data failed");
		}else if(dw==WAIT_ABANDONED){
			printf("\nMutex data abanoned");
		}else if (dw==WAIT_OBJECT_0) { //mutex was locked
		#if 1
			if( (pRESP_Msg_Data->Response_Index-pre_index==1)&& (pRESP_Msg_Data->flag ==NEW_FRAME) ){			
				Log_Data_RT(pRESP_Msg_Data->Buffer, MAX_RESP_MSGSTR_SIZE);				
				pre_index =pRESP_Msg_Data->Response_Index;
				pRESP_Msg_Data->flag =RECEIVE_OK;
			}
		#endif
			RtReleaseMutex(hMutex_Data);
		}		
	}while(pRESP_Msg_Data->flag != END_SEND);
//------------end saving realtime response--------	
#endif
	SleepEx(100, FALSE);
exitroutine:
	

	RtCloseHandle(hShm_Data);
	RtCloseHandle(hShm_Cntl);
	RtCloseHandle(hMutex_Data);
	// RtCloseHandle(hSemPost);
	//RtCloseHandle(hRESP_Shm_Data);

//	Log_Stop_RT("rtlog.dat", MAX_RESP_MSGSTR_SIZE); //
//	Log_Stop_RT(rt_logfilename, MAX_RESP_MSGSTR_SIZE); //
    return NO_ERRORS;
}

 
#if 1
void Display_Joint(double *f)
{
	int i;
	int k = 0, Nk = 0, MsgNum = 0;
	char * s;
	static long cls_screen = 0;
    //static int Send_Loss_Last=0,Receipt_Loss_Last=0; 	
	//static long time_out_delay=0;
		
	//clear screen only once at begin
	if(cls_screen==10) 
		system("cls"); //good
	if(cls_screen++>=100)
		cls_screen = 20;
		
	GotoConsoleXY(0,0);
	printf("-- BHR-6 Software(C)2001-2019,BIT,Biped Group, Walk by Preview Control (Pure Version) --");

	printf("\n [ Timer  ] [%9.3f sec]", f[k++] );
	printf( " [ Unit Deg ]");
	
	k=1;
	// MSG_RT [1] 程序当前时间; [2] 已占用时间; [3] 单周期实际运行时间
	MsgNum = (int)f[k++]; Nk = (int)f[k++]; for(i=0; i<MsgNum; i++) k++; 
	printf("\n [ Timer2 ] [%9.3f sec] [ Elapsed  ] [%5.3f] [ Period gap ] [%5.3f]", f[Nk + 1], f[Nk + 2], f[Nk + 3]);

	// MSG_CAN [1] CAN 总帧数; [2] CAN 错误帧数; [3] CAN 丢失帧数; [4] CAN 打开标志;
	MsgNum = (int)f[k++]; Nk = (int)f[k++]; for(i=0; i<MsgNum; i++) k++; 
	printf("\n [ CAN No ]: %ld", (long)f[Nk+1]);
	printf("          [ Send Loss ]: %ld", (long)f[Nk+2]);
	printf("       [ Receipt Loss ]: %ld  ", (long)f[Nk+3]);	
	if ((long)f[Nk + 4] == 0xaa)
		printf("[CAN:-Recv & Send-!] ");
	else if ((long)f[Nk + 4] == 0x55)
		printf("[CAN: OFF] ");
	else if ((long)f[Nk + 4] == 0x66)
		printf("[CAN: Recv & SetRef] ");
	else  if ((long)f[Nk + 4] == 0x99)
		printf("[CAN: Recv-Only] ");

	// MSG_ONOFF [1] 上电标志; [2] 清零标志; [3] 记录数据标志; 
	printf("\n");
	MsgNum = (int)f[k++]; Nk = (int)f[k++]; for(i=0; i<MsgNum; i++) k++; 
	if ((int)f[Nk + 1] == 0xaa)
		printf(" [ SRV Power ]: ! ON !");
	else
		printf(" [ SRV Power ]: ! OFF!");

	if ((long)f[Nk + 2])
		printf("        [ Sen_Ofs ]: ON ");
	else
		printf("        [ Sen_Ofs ]: OFF");
	
	if(DataLogStart == 1) { 
		printf("      [ Data Log ]: ON ");
	} else{
		printf("      [ Data Log ]: OFF");
	}

	printf("\n---------------------------------------------------------------------------");
	
	// MSG_WAIST_JOINT
	MsgNum = (int)f[k++]; Nk = (int)f[k++]; for (i = 0; i<MsgNum; i++) k++;
	printf("\n [  Waist  ]     [ Yaw     L_TuiGan     R_TuiGan ]");
	//[1-3] 腰关节码盘返回值：1-yaw；2-左推杆；3-右推杆
	printf("\n [   Real  ]     %7.3f, %7.3f, %7.3f", f[Nk + 1]*ToDeg, f[Nk + 2], f[Nk + 3]);
	//[4-6] 腰关节发送位置指令：4-yaw；5-左推杆；6-右推杆
	printf("\n Reference->     %7.3f, %7.3f, %7.3f", f[Nk + 4]*ToDeg, f[Nk + 5], f[Nk + 6]);
	
	printf("\n---------------------------------------------------------------------------");
	
	
	printf("\n [ ARM Joint ]");
	printf("\t   [ ARM_1    ARM_2    ARM_3    ARM_4    ARM_5    ARM_6 ]");	
	// MSG_RARM_JOINT
	MsgNum = (int)f[k++]; Nk = (int)f[k++]; for(i=0; i<MsgNum; i++) k++; 
	//[1-6] 右臂各关节码盘返回值：
	printf("\n [   R Real  ]      ");
	for(i=1; i<=6; i++) printf("%7.3f, ", f[Nk + i]*ToDeg);//
	//[7-12] 右臂各关节发送位置指令：
	printf("\n R Reference->      ");
	for(i=7; i<=12; i++) printf("%7.3f, ", f[Nk + i]*ToDeg);//
	
	// MSG_LARM_JOINT
	MsgNum = (int)f[k++]; Nk = (int)f[k++]; for(i=0; i<MsgNum; i++) k++; 
	//[1-6] 左臂各关节码盘返回值：
	printf("\n [   L Real  ]      ");
	for(i=1; i<=6; i++) printf("%7.3f, ", f[Nk + i]*ToDeg);//
	//[7-12] 左臂各关节发送位置指令：
	printf("\n L Reference->      ");
	for(i=7; i<=12; i++) printf("%7.3f, ", f[Nk + i]*ToDeg);//
	
	printf("\n---------------------------------------------------------------------------");
	
	// MSG_RLEG_JOINT
	printf("\n [ LEG Joint ]");
	printf("\t   [ HIP_Z    HIP_Y    HIP_X    KNEE_X   JOINT_L   JOINT_R ]");	
	MsgNum = (int)f[k++]; Nk = (int)f[k++]; for(i=0; i<MsgNum; i++) k++; 
	//[1-6] 右腿各关节码盘返回值：
	printf("\n [   R Real  ]      ");
	for(i=1; i<=6; i++) printf("%7.3f, ", f[Nk + i]*ToDeg);//
	//[7-12] 右腿各关节发送位置指令：
	printf("\n R Reference->      ");
	for(i=7; i<=12; i++) printf("%7.3f, ", f[Nk + i]*ToDeg);//

	// MSG_LLEG_JOINT
	//[1-6] 左腿各关节码盘返回值：
	MsgNum = (int)f[k++]; Nk = (int)f[k++]; for(i=0; i<MsgNum; i++) k++; 
	printf("\n [   L Real  ]      ");
	for(i=1; i<=6; i++) printf("%7.3f, ", f[Nk + i]*ToDeg);//
	//[7-12] 左腿各关节发送位置指令：
	printf("\n L Reference->      ");
	for(i=7; i<=12; i++) printf("%7.3f, ", f[Nk + i]*ToDeg);//
	
	printf("\n---------------------------------------------------------------------------");
	
	// MSG_FORCE_SENSOR
	printf("\n [Force Sensor]");	
	printf("   [ Fx       Fy       Fz ](N)");printf("  [ Tx       Ty       Tz ](Nm)");	
	//[1-6] 右脚力传感器信息
	MsgNum = (int)f[k++]; Nk = (int)f[k++]; for(i=0; i<MsgNum; i++) k++; 
	printf("\n [  R Sensed ]  ");
	for (i=1; i<=3; i++) printf("%7.3f, ", f[Nk + i]); printf("  ");
	for (i=4; i<=6; i++) printf("%7.3f, ", f[Nk + i]);
	//[7-12] 左脚力传感器信息
	printf("\n [  L Sensed ]  ");
	for (i=7; i<=9; i++) printf("%7.3f, ", f[Nk + i]); printf("  ");
	for (i=10; i<=12; i++) printf("%7.3f, ", f[Nk + i]);
	
	// MSG_ZMP
	MsgNum = (int)f[k++]; Nk = (int)f[k++]; for(i=0; i<MsgNum; i++) k++; 
	//[1-2] 理想ZMP位置XY; [3-4] 实际ZMP位置XY; [5-6] 右脚ZMP位置XY; [7-8] 左脚ZMP位置XY
	printf("\n [IZMPxy_Atcl]     [Total ZMPxy]     [ Right ZMPxy ]     [ Left ZMPxy ]");
	printf("\n (%5.3f, %5.3f)   (%5.3f, %5.3f)   (%5.3f, %5.3f)   (%5.3f, %5.3f)     ", f[Nk + 1], f[Nk + 2], f[Nk + 3], f[Nk + 4], f[Nk + 5], f[Nk + 6], f[Nk + 7], f[Nk + 8]);

	printf("\n---------------------------------------------------------------------------");

	//MSG_IMU
	MsgNum = (int)f[k++]; Nk = (int)f[k++]; for(i=0; i<MsgNum; i++) k++; 
	printf("\n [ IMU ] -> Pitch: [%5.2f]  Roll: [%5.2f]  AX: [%7.3f]  AY: [%7.3f]  AZ: [%7.3f]", f[Nk + 1], f[Nk + 2], f[Nk + 3], f[Nk + 4], f[Nk + 5]);
	
	//MSG_ADJUST_FORCE_SENSOR
	printf("\n [Force Sensor 2]");	
	printf("   [ Fx       Fy       Fz ](N)");printf("  [ Tx       Ty       Tz ](Nm)");	
	//[1-6] 右脚力传感器信息
	MsgNum = (int)f[k++]; Nk = (int)f[k++]; for(i=0; i<MsgNum; i++) k++; 
	printf("\n [  R Adjusted ]  ");
	for (i=1; i<=3; i++) printf("%7.3f, ", f[Nk + i]); printf("  ");
	for (i=4; i<=6; i++) printf("%7.3f, ", f[Nk + i]);
	//[7-12] 左脚力传感器信息
	printf("\n [  L Adjusted ]  ");
	for (i=7; i<=9; i++) printf("%7.3f, ", f[Nk + i]); printf("  ");
	for (i=10; i<=12; i++) printf("%7.3f, ", f[Nk + i]);
	
	printf("\n---------------------------------------------------------------------------");
	
	//MSG_MODE
	MsgNum = (int)f[k++]; Nk = (int)f[k++]; for(i=0; i<MsgNum; i++) k++; 
	printf("\n Walk Mode: ");
	switch((int)f[Nk + 1])
	{
		case        HELLO:  printf("[ HELLO  ]"); break;
		case        CRAWL:  printf("[ CRAWL  ]"); break;
		case      FALL_DL:  printf("[ FALL   ]"); break;
		case       ACTION:  printf("[ ACTION ]"); break;
		case PRE_CON_MODE:  printf("[ WALK   ]"); break;
		default:            printf("[ !EEROR ]"); break;
	}
	printf("    File ID: [%2d ]", (int)f[Nk + 2]);
	if((int)f[Nk + 3]==1 ) printf("    Fall Detect: [ ON ]" );
	else                   printf("    Fall Detect: [ OFF]" );
	if((int)f[Nk + 4]==11) printf("    Fall Trigger: [ Yes]" );
	else                   printf("    Fall Trigger: [ No ]" );
	
	printf("\n---------------------------------------------------------------------------");
	
	//MSG_PRECON_MODE
	MsgNum = (int)f[k++]; Nk = (int)f[k++]; for(i=0; i<MsgNum; i++) k++; 
	printf("\n [ Preview Control ] => ");
	switch((int)f[Nk + 1])
	{
		case    DISP_PRECON_MOVE_NONE:  printf("[   NONE  ]"); break;
		case DISP_PRECON_MOVE_FORWARD:  printf("[ FORWARD ]"); break;
		case    DISP_PRECON_MOVE_LEFT:  printf("[  LEFT   ]"); break;
		case   DISP_PRECON_MOVE_RIGHT:  printf("[  RIGHT  ]"); break;
		case    DISP_PRECON_MOVE_BACK:  printf("[   BACK  ]"); break;
		default:            printf("[ !EEROR ]"); break;
	}
	printf("     [ Walk on ]: [ %d]", (int)f[Nk + 2]);
	printf("     [ Now Step ]: [%3d]", (int)f[Nk + 3]);
	
	//MSG_PRECON_PARAMETERS
	MsgNum = (int)f[k++]; Nk = (int)f[k++]; for(i=0; i<MsgNum; i++) k++; 
	printf("\n [ Step Length ]:%5.2f    [ Step Period ]:%5.2f   [ Step Num ]:%3d", f[Nk + 1], f[Nk + 2], (int)f[Nk + 3]);
	
	printf("\n---------------------------------------------------------------------------");
	
	//MSG_TPC
	MsgNum = (int)f[k++]; Nk = (int)f[k++]; for(i=0; i<MsgNum; i++) k++; 
	printf("\n [Rel RZMP2RAnk] [ Rel LZMP2LAnk] [ Rel ZMP2Body]  [ Ref ZMP2Body]  [ Deta COM]");
	printf("\n (%5.3f, %5.3f) | (%5.3f, %5.3f) | (%5.3f, %5.3f) | (%5.3f, %5.3f) | (%5.3f, %5.3f)", f[Nk + 1], f[Nk + 2], f[Nk + 3], f[Nk + 4], f[Nk + 5], f[Nk + 6], f[Nk + 7], f[Nk + 8], f[Nk + 9], f[Nk + 10]);
	
	printf("\n---------------------------------------------------------------------------");
	//MSG_CURRENT
	MsgNum = (int)f[k++]; Nk = (int)f[k++]; for(i=0; i<MsgNum; i++) k++; 
	printf("\n [ I-R2   I-R3     I-R4   I-R5     I-R6  ][  I-L2    I-L3    I-L4    I-L5    I-L6]");
	printf("\n (%5.3f, %5.3f) | (%5.3f, %5.3f) | (%5.3f, %5.3f) | (%5.3f, %5.3f) | (%5.3f, %5.3f)", f[Nk + 1], f[Nk + 2], f[Nk + 3], f[Nk + 4], f[Nk + 5], f[Nk + 6], f[Nk + 7], f[Nk + 8], f[Nk + 9], f[Nk + 10]);

	// IMU
	MsgNum = (int)f[k++]; Nk = (int)f[k++]; for(i=0; i<MsgNum; i++) k++; 
	printf("\n <Pitch[deg]>  <Roll[deg]>  <ACC_x[ms^-2]>  <ACC_y[ms^-2]>  <ACC_z[ms^-2]>");
	printf("\n   [%5.3f]       [%5.3f]       [%5.3f]       [%5.3f]        [%5.3f]", f[Nk + 1] * 57.3, f[Nk + 2] * 57.3, f[Nk + 3], f[Nk + 4], f[Nk + 5]);
	printf("\n <GYR_x[rads^-1]>  <GYR_y[rads^-1]>  <GYR_z[rads^-1]>");
	printf("\n   [%5.3f]       [%5.3f]       [%5.3f]", f[Nk + 6], f[Nk + 7], f[Nk + 8]);
//	NRT_ReadRosEstm(&rosData);
	printf("\n[Recieve ROS Infor]: %d\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
		rosData.flag, rosData.dTime,
		rosData.dPosX, rosData.dPosY, rosData.dPosZ,
		rosData.dAngX, rosData.dAngY, rosData.dAngZ
	);
}
#endif
 

 //End of the file robot_hmi.c
 