/***********************************************************************
 *  BHR BIPED CONTROL SOFTWARE UNDER Windows XP+Real_Time Extension    *  
 *	Filename:     control.h                                            *
 *	Sept.16, 2010, Originaly created for Windwos RTX8.1SP2 by Z.G. YU  *
 ***********************************************************************/
#ifndef __CONTROL__H
#define __CONTROL__H

#define	MSGSTR_SHM_DATA		"Message_Data.Shm"
#define	CTRLMSG_SHM_CNTL	"MSG_Cntl.Shm"
#define MSGSTR_MUTEX_DATA	"Message_Data.Mutex"
#define CTRLMSG_MUTEX_CNTL	"MSG_CNTL.Mutex"
#define MSGSTR_SEM_POST		"Message.SemPost"//
#define RT_MSGSTR_SHM_DATA  "RT_Message_Data.Shm"
// Define message structure.
//
#define M_PI 3.14159265358979

#define	MAX_MSGSTR_SIZE	 300
#define TRAJ_TRANSFER_NUM 200
#define FORCE_SENSOR_CHANNEL THIRD


typedef struct {
    int	Response_Index;		//  flag
    double	Buffer[MAX_MSGSTR_SIZE];
} MSGSTR, *PMSGSTR;

typedef struct  {
	int command; //default 0, not available
	int arg1;  // for transfer traj, arg1->group ID, >=1
	int arg2; // number in each group
	double arg3[TRAJ_TRANSFER_NUM];
	int FileID; //File ID of the off-line file
} CtrlMsg, *PCtrlMsg;

//for transmit real-time response
#define MAX_RESP_MSGSTR_SIZE 400//200 /*buffer size of realtime response*/
#define NEW_FRAME 66
#define RECEIVE_OK 99
#define END_SEND -1

typedef struct {
    int	Response_Index;		
	int flag;
    double Buffer[MAX_RESP_MSGSTR_SIZE];
} RESP_MSGSTR, *PRESP_MSGSTR;

#define TRANFER_SUCCESS			0x66
#define RT_START				1
#define POWER_ON				2
#define OFFLINE_DATA_TRANSFER	3 //in processing
#define OFFLINE_DATA_END		4 //end process
#define TEST_KEY				5
#define START_WALK				6
#define HOME_ON_LEG				7
#define SEARCH_HOME				8
#define SENSORY_CONTROL			9
#define OFFSET_GYRO_ACC			10
#define ARM_HOME_ON				11
#define HOME_VERTICAL_LEG		12
#define SIN_TRAJ_TEST			13
#define WAIST_LEG_ONLINE		14
#define SOFTRESET_CANOPEN		15
#define RT_DAT_SAVE				16
#define RECORD_RT_DAT			17
#define RESET_ELMO_SAVE_ENCODER 18

#define FALL_START_FLAG			19
#define MOVE_UP					20
#define MOVE_STOP				21
#define MOVE_EXIT				22

#define HELLO					23//打招呼
#define	CRAWL					24//爬行
#define FALL_DL					25//摔倒
#define ACTION					26//执行功能
#define PRE_CON_MODE			27//预观控制行走

#define UNLIMIT_STEP 120 // !!! The BHR6 can walks no more than 'UNLIMIT_STEP' steps.

#define PRECON_MOVE_NONE    -1  
#define PRECON_MOVE_FORWARD 122 
#define	PRECON_MOVE_LEFT    123
#define	PRECON_MOVE_RIGHT   124
#define	PRECON_MOVE_BACK    125

#define DISP_PRECON_MOVE_NONE    126
#define DISP_PRECON_MOVE_FORWARD 127
#define DISP_PRECON_MOVE_LEFT    128
#define DISP_PRECON_MOVE_RIGHT   129
#define DISP_PRECON_MOVE_BACK    130

#endif //__CONTROL__H
//End of File

