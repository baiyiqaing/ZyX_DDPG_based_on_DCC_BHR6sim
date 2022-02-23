/***********************************************************************
*  BHR BIPED CONTROL SOFTWARE UNDER Windows XP+Real_Time Extension     *  
*	Filename:     rtx_process.c                                        *
*	Sept.16, 2010, Originaly adapted for Windwos RTX8.1SP2 by Z.G. YU  *
*					CANopen DS402 for Elmo and kollmorgen drivers      *
*	Jan. 2010, modified for Pingpong-robot, CANopen protocol           *	
* 			            	by Z.G. YU and X.C. CHEN                   *         
*	Jan. 2007, modified for BHR-3M distributed control systems         *
*      based on CAN-bus (in linux)	by YU and CHEN                     *
*	2004 March 28 modified by Peng Du  for BHR-2 in Linux              *
*	2002 July 12  modified by Wang Guang for BHR-1 in Linux	           *
*	Launch in 2001 by Prof. Huang, Enviroments: Linux+ RTLinux         *
*	    (C) Biped Group,  Intelligent Robitics Institute               *
*         Beijing Institute of Technology, China                       *
*           2001-2010, All rights reserved                             *
*                                                                      *
* Remarks: The Arm_Traject() was jointly developed with Nankai Univ.   * 
***********************************************************************/
#define RTX_PROCESS_C

#include <windows.h>
#include <stdio.h>
//#include <rtapi.h>
#ifdef UNDER_RTSS
#include "rtssapi.h"
#endif //UNDER_RTSS

#include <math.h>
#include <float.h>
#include "control.h"
#include "lib\Advtech_PCM3680I_Driver.h"
#include "lib\Resistant_Compliance.h"
#include "lib\Uneven_Trailblazer.h"
#include "lib\CoM_Stabilizer.h"
#include "lib\QP_balance.h"
#include "lib\DCC_RunCon.h"
#include "lib\DCC_Run_ReadConfig.h"

#include "lib\can_G_ds402.h"
#include "lib\adv_can.h"
#include "rtx_process.h"

#include "lib\SPP_Cntl_RTX.h"

#include "filepath.inc"

#define USE_IFS //jr3 force sensors//mg
//#define	FALL_DETECT

//#define NON_HARDWARE

//20170528 demo control interface
#include "lib\DemoControl.h"

/****** Walking Pattern Generation by Preview Control x zhouqq 2019/01/17 **********/
#define USE_XS_IMU300
double XS_Pitch=0.0,XS_Roll=0.0,XS_Yaw=0.0;
double XS_AccX=0.0, XS_AccY=0.0, XS_AccZ=0.0;
double XS_GyrX=0.0, XS_GyrY=0.0, XS_GyrZ=0.0;
int xsLossNum = 0;
#define XS_CHANNEL 4
double FootFT_temp[5][6];
#include "lib\Tra_Generate.h"
#include "lib\hardware_conf.h"
#pragma warning(disable : 4244)
#pragma warning(disable : 4101)
#pragma warning(disable : 4305)

#define USE_PREVIEW_CONTROL

extern double FzR_filtered;
extern double FzL_filtered;
extern Run_ConVal DCC_Run;
extern Run_ConVal FootCompliance_ConVal;
extern Run_ConVal FlyRot_ConVal;
extern Run_Horizontal LIPM_ConVal;
extern Run_Horizontal TPC_Run_ConVal;
extern Run_Rotational BodyRot_ConVal;
extern Run_Rotational delta_Rot_old;
extern Run_FS ADD_Trq_Ref;
extern double pitch_body;
extern double roll_body;

extern RE_NQP NQP_re;

extern state_IPSD State_CP;
extern Horizontal_Current Delta_COM;
extern Horizontal_Current GP_rel;
extern Horizontal_Current GP_ref;
extern Horizontal_Current delta_p_hat;

extern Run_ConVal Ref_RotAndPos_ConVal; // LandRot
extern double Moment_pitch_re;
extern double Moment_roll_re ;
extern double footpitch_re;
extern double footroll_re;

int PreCon_Mode = PRECON_MOVE_NONE;
int PreCon_ModeDisp = DISP_PRECON_MOVE_NONE;
extern RC_comp RC_Comp_Cont;

extern PreCon_Tra F_ext_re;
extern PreCon_Tra F_vir_re;
extern PreCon_Tra e_re;	
extern PreCon_Tra Tau_ext_re;
extern PreCon_Tra Tau_vir_re;
extern PreCon_Tra r_re;

extern Position P_ZMPRFoot_RAnkle;
extern Position P_ZMPLFoot_LAnkle;

extern int sense_on_flag;
extern int control_on_flag;

// re
extern Run_FS Rfoot_ref_re, Lfoot_ref_re, Rfoot_rel_re, Lfoot_rel_re;
extern Run_Horizontal zmp_rel_re, zmp_ref_re, delta_com_re, delta_vcom_re;
extern Position P_ZMPRel_B;
extern Run_ConVal ContactConVal;

// TPCMPC
extern double LIPM_ZMP_x;
extern double LIPM_ZMP_y;
extern double x_MPCTPC ;
extern double dx_MPCTPC;
extern double y_MPCTPC ;
extern double dy_MPCTPC;

extern double Rz, Lz;
/* int PS2_Wts=1; */
/**************** Walking Pattern Generation by Preview Control s ******************/


//re
extern double F_cal_x_re;
extern double F_resi_x_re;
extern double F_v_x_re;
extern double e_x_re;
extern double F_cal_y_re;
extern double F_resi_y_re;
extern double F_v_y_re;
extern double e_y_re;

extern double FzR_rel;
extern double FzL_rel;
extern double FzR_ref;
extern double FzL_ref;

extern double deltax_re ;
extern double deltay_re ;
extern double deltadx_re;
extern double deltady_re;

extern Run_ConVal FootCompliance_ConVal;
extern Run_ConVal TPCFoot_ConVal_re;

//Chz
extern double chz_log[40];
extern double chz_XZMP_filted, chz_YZMP_filted;
extern int chzrun_signal[30000][3];
double chz_IMU_log[40];
//Chz
extern double additor_pit;
extern double additor_rol;
extern double ref_pitch_re;
extern double rel_pitch_re;
extern double con_pitch_re;
extern double ref_roll_re;
extern double rel_roll_re;
extern double con_roll_re;
/************************** Walk-Craw-Fall Demo x **************************/
#define	WEIGHT	    (50*9.8)//48.2//47.5*9.8	/*Humanoid Weight*/
#define FZ_Mins	    (0.1*WEIGHT)//0.1
#define FZ_CONT		(1.0*WEIGHT)

int Mode_Flag = PRE_CON_MODE; // !!! Default Mode: Walk by preview Contol

double Ref_Waist_Pitch = 0;
double Ref_Waist_Roll = 0;
BOOL Fall_Start_flag = FALSE;
BOOL Fall_flag = FALSE;
void Detect_Falling4Sensor(void);
int Falling_Alarm = 0;
/************************** Walk-Craw-Fall Demo s **************************/



/************** Fibre-Optic IMU Sensor Pitch Roll Acc Gyro x********************/
//#define USE_GQ_IMU100 
double GQ_Pitch=0.0,GQ_Roll=0.0;
double GQ_AccX=0.0, GQ_AccY=0.0, GQ_AccZ=0.0;
void GQ_IMU100_GetData(unsigned char * data);
/************** Fibre-Optic IMU Sensor Pitch Roll Acc Gyro s********************/

int N_step_con= 6;// Initial walk steps
double L_step_con= 0.38;//0.35;//0.35;//0.25;//0.25;//0.25；

long send_loss=0;

BOOL LogResponse_RTSS(void);
//Motor Torque=CAN_OD_0x6078/1000*(StallCurrent_Motor/1.414)*Kt_Motor;
/*  TYPE , StallCurrent(Arms), PeakCurrent(Arms), Torque const(Kt(Sine)), Voltage Const. Ke(L-L) Vrms/krpm, Stall Torque(Tsp), peak torque(Tpp), Resitance(ohm, R, Lead-Lead)
Leg Joint 1: Parker K044025-EY1,2.53Arms ,7.96Arms, 0.045Nm/Arms, 2.749Vrms/krpm, 0.1147Nm,0.3625Nm, 3.788ohm
Leg Joint 2: Parker K032300-8Y1,1.68Arms ,5.3Arms, 0.178Nm/Arms,  10.782Vrms/krpm, 0.30Nm,  0.946Nm, 5.869ohm
Leg Joint 3: Parker K044100-8Y1,3.18Arms,10.06Arms ,0.115Nm/Arms, 6.931Vrms/krpm, 0.365Nm, 1.153Nm, 2.382ohm,
Leg Joint 4: Parker K044100-8Y1,3.18Arms,10.06Arms ,0.115Nm/Arms, 6.931Vrms/krpm, 0.365Nm, 1.153Nm, 2.382ohm 
Leg Joint 5: Parker K044050-EY1,2.3Arms,7.28Arms,0.091Nm/Arms,5.497Vrms/krpm, 0.21Nm, 0.66Nm, 2.545ohm
Leg Joint 6: Maxon EC-powermax30, 4.7A, 124A, 0.0276Nm/A, 0.346krpm/V, 0.12Nm, 3.43Nm,0.386ohm
*/
//RoboDrive TQ ILM7010, Star-Serial, 230g, 270W@48V,Phase currrent amplitude 7A,0.74Nm@rated Torque, rated 3500rpm@48V
static const double Kt_Motor[LEG_NUM+1][JOINT_NUM+1]={
	// Joints: N.A HipZ  HipY   HipX  KneeX  FootX  FootY
	{  0,    0,    0,    0,    0,    0,    0   },
	{  0,  0.011,  0.11,  0.11,  0.11,   0.11,  0.0276/1.414},  //Right LEG
	{  0,  0.011,  0.11,  0.11,  0.11,   0.11,  0.0276/1.414}  //Left LEG
};
//Stall Current Amplitude (equals to Elmo CL[1]),should be verified at the real robot by CL[1] command
//可用Elmo监控软件直接读出的结果
static const double StallCurrent_Motor[LEG_NUM+1][JOINT_NUM+1]={
	// Joints: N.A HipZ  HipY   HipX  KneeX  FootX  FootY
	{  0,    0,    0,    0,    0,    0,    0   },
	{  0,  18,  18,  18,  18,   4.0,  6.6},  //Right LEG
	{  0,  18,  18,  18,  18,  4.0,  6.6}  //Left LEG
};

double Leg_Torque[LEG_NUM+1][JOINT_NUM+1];

static const int MAX_CAN_CHANNEL = 6; //4 CAN channels


//
//**** Error Codes ****//
#define NO_ERRORS		0
#define ERROR_OCCURED	-1
//#define CONTROL_PERIOD 0.005 //unit sec,******
#define CONTROL_PERIOD 0.004//0.004 //unit sec,******
// Local data.
//
//#define bool BOOL
#define true TRUE
#define false FALSE
void Trans_IFS_Data_l( unsigned char* data);
void Trans_IFS_Data_r( unsigned char* data);
void IFS_Init();	
float Init_IFS[2][6];
BOOL IFS_Init_Flag = FALSE;
int IFS_Init_Count=0;
//#define printf RtPrintf

LONG       ClockResolution;     // Clock resolution (100 ns units)
LONG       ClockTimerPeriod;    // Clock timer period (100 ns units)
LONG       TimerPeriod;         // Actual timer period (100 ns units)
LONG       OutOfRangeCount;     // Number of counts beyond range
LONG       MaxLatency;          // Maximum latency (100 ns units)
LONG       BucketSize;          // Histogram bucket size
LARGE_INTEGER    Period;        // Timer period
LARGE_INTEGER    PreviousTime;  // Previous timer expiration time
LARGE_INTEGER    StartTime;     // Start time of sampling run
BOOL        Sound = FALSE;      // Generate sound (square wave)
BOOL        Hist = FALSE;       // Display the histogram
BOOL        Fastest = FALSE;    // Use the fastest available timer

int RTFCNDCL TimerHandler (PVOID unused);	

int RTFCNDCL Timer_TrajRecv_Handler (PVOID unused);	
//
// MAIN -- send some messages and exit.
//
PMSGSTR pMsg_Data;
PRESP_MSGSTR pRESP_Msg_Data;
PCtrlMsg pMsg_Cntl;
HANDLE hMutex_Data = NULL;
HANDLE hMutex_Cntl = NULL;
HANDLE	hSemPost;
BOOL Read_Pattern_Flag = TRUE;

LONG	lReleaseCount = 1;
BOOL	EndlessLoopFlag = TRUE;
can_msg_t pmsg;

int RecvCANData(int channel);
CAN_STRUCT can_struct_set;
int SendCmd(int channel, unsigned int msgID , unsigned  char *candata, unsigned char bytelength);

#define HOME_SEARCH_DELAY  (30/DTIME)  //85

#define TORQUE_CONSTANT_200W 0.0276 //27.6mNm 

static double con_val[LEG_NUM + 1][JOINT_NUM +1];
static double conv_val[LEG_NUM + 1][JOINT_NUM +1];

static double Con_Disp_Leg[LEG_NUM+1][JOINT_NUM+1];   //----For display in pid.c
static double Con_Disp_Arm[ARM_NUM+1][JOINT_ARM_NUM+1]; 
static double Con_Disp_Head[JOINT_Head_NUM+1];
static double Con_Disp_Waist[JOINT_WAIST_NUM+1];

static double Ref_Leg_Last[LEG_NUM+1][JOINT_NUM+1];   //----For display in pid.c
static double Ref_Leg_Last_2[LEG_NUM+1][JOINT_NUM+1];   //--For display in pid.c

static double Con_Temp_Leg[LEG_NUM+1][JOINT_NUM+1];   //----Save temporarily 
static double Con_Temp_Arm[ARM_NUM+1][JOINT_ARM_NUM+1]; 
static double Con_Temp_Head[JOINT_Head_NUM+1];
static double Con_Temp_Waist[JOINT_WAIST_NUM+1];

//----Plus or minus sign of control 
static const double Sign_Con_Leg[LEG_NUM+1][JOINT_NUM+1]={
	// Joints: N.A HipZ  HipY   HipX  KneeX  FootX  FootY
	{  0,    0,    0,    0,    0,    0,    0   },
	{  0,  1.0,  1.0,  1.0,  -1.0,   1.0,  -1.0},  //Right LEG
	{  0,  1.0,  1.0,  -1.0,  1.0,   1.0,  -1.0 }  //Left LEG
};  
static const double Sign_Con_Arm[ARM_NUM+1][JOINT_ARM_NUM+1]={
	// Joints: N.A.  Arm1  Arm2  Arm3   Arm4  Arm5  Arm6
	{  0,    0,    0,    0,    0,    0,   0,   0 },
	{  0,  1.0,  -1.0, -1.0,  1.0,  -1.0, 1.0 , -1.0}, //Right ARM
	{  0,  -1.0,  -1.0, -1.0, -1.0,  -1.0,  -1.0, 1.0} //Left ARM
}; 
static const double Sign_Con_Head[JOINT_Head_NUM+1]=
{
	//Joints: N.A.  Head1  Head2
	0,  1.0,  1.0  
};
static const double Sign_Con_Waist[JOINT_WAIST_NUM+1]=
{
	//Joints: N.A.  Yaw  Roll
	0,  -1.0,  1.0,  1.0   
};

#define K_L 1.0   //limit the range of joints


#if 1   // modify limit,07-09-11
static const double LIMIT_RLEG_MAX [JOINT_NUM ] = { 0.0*ToRad*K_L,  16.5*ToRad*K_L,  65.0*ToRad*K_L,  1.0*ToRad*K_L,	12.0*ToRad*K_L,  12.0*ToRad*K_L };
static const double LIMIT_LLEG_MAX [JOINT_NUM ] = { 0.0*ToRad*K_L,  16.5*ToRad*K_L,  65.0*ToRad*K_L,  1.0*ToRad*K_L,    12.0*ToRad*K_L,  12.0*ToRad*K_L };
static const double LIMIT_RLEG_MIN [JOINT_NUM ] = {-0.0*ToRad*K_L, -16.5*ToRad*K_L, -35.0*ToRad*K_L, -59.0*ToRad*K_L, -47.0*ToRad*K_L, -47.0*ToRad*K_L };
static const double LIMIT_LLEG_MIN [JOINT_NUM ] = {-0.0*ToRad*K_L, -16.5*ToRad*K_L, -35.0*ToRad*K_L, -59.0*ToRad*K_L, -47.0*ToRad*K_L, -47.0*ToRad*K_L };
#endif

#if 1 //pingpong, 7dof
static const double LIMIT_RARM_MAX [JOINT_ARM_NUM ] = { 120.0*ToRad,	 20.0*ToRad,	 180.0*ToRad,  135.0*ToRad,	 200.0*ToRad,	 91.0*ToRad , 71.0*ToRad };
static const double LIMIT_RARM_MIN [JOINT_ARM_NUM ] = { -120.0*ToRad,	-90.0*ToRad/*-80.0*/,	-90.0*ToRad,  -90.0*ToRad,	-90.0*ToRad,	-91.0*ToRad,  -20.0*ToRad };
static const double LIMIT_LARM_MAX [JOINT_ARM_NUM ] = { 120.0*ToRad,	 90.0*ToRad,	 95.0*ToRad,  135.0*ToRad,	 95.0*ToRad,	 60.0*ToRad,  0.0*ToRad  };
static const double LIMIT_LARM_MIN [JOINT_ARM_NUM ] = { -120.0*ToRad,	-20.0*ToRad,	-95.0*ToRad,  -90.0*ToRad,	-95.0*ToRad,	-60.0*ToRad , 0.0*ToRad };
#endif 

static const double LIMIT_HEAD_MAX [JOINT_Head_NUM ] = { 50.0*ToRad, 15.0*ToRad  };
static const double LIMIT_HEAD_MIN [JOINT_Head_NUM ] = {-50.0*ToRad, -15.0*ToRad  };

//20170107 TUIGAN										   Yaw		   Left_Tuigan	  Right_Tuigan
static const double LIMIT_WAIST_MAX [JOINT_WAIST_NUM ] = { 50.0/*25.0*/*ToRad, 97.0/*110.3*/, 97.0/*110.3*/  };
static const double LIMIT_WAIST_MIN [JOINT_WAIST_NUM ] = {-50.0/*25.0*/*ToRad, -20.0, -20.0  };

static const double Qbody_Offset[3] = {0.0, 0.5*M_PI/180.0, -0.4*M_PI/180.0};  

//---------------------------------------------------

void Get_Sensor_Data_New(void);
int CheckDSPControllerConfig2(void);

/*------------------global varibles-------------------------*/
static can_msg_t  canmsgfTx,canmsgfRX;
static unsigned int JCStates=0x0ffff;

static unsigned CanStart=0x55;

static int Reset_Elmo_Save_Encoder = 0;

extern void Init_Q_body(void);  //rtdata.c functions
extern void Get_Q_body(void);	 //rtdata.c fucntions
extern void Get_Mdf_Data(void ); //rtdata.c fucntions

long CANframeCount=0;    //to count how many CAN frame be sent or received  
int SendErrorCount=0;   //  
int CANReceiptLoss=0;

float Kp[16][2],Kd[16][2]; //NOT_USED_CHECK_2017
int ENC_Only_Flag=0x55;
int Set_Ref_Equ_ENC_Delay=0x00;
int Power_On_Flag = 0x55;
// to display Hall number
int Protect_forever_flag=0x0;//NOT_USED_CHECK_2017
/*--------------- invoked at every servo cycle------------------- */

void Get_Elmo_Encoder(void);
int  Elmo_Num =0; //number of Elmo connected to CAN bus

static union{
	unsigned char cv[4]; //sizeof(char) is 1
	long lv; //sizeof(long/int) is 4
}lv2cv;

static union{
	unsigned char cv[4]; //sizeof(char) is 1
	float flt; //sizeof(float) is 4
}flt2cv;

static union{
	unsigned char cv[2]; //sizeof(char) is 1
	short int siv; //sizeof(short int) is 2
}siv2cv;

void Long2Byte(unsigned char * const Byt, long ldat); 
int Byte2Long(unsigned char * const Byt); 
float Byte2Float(unsigned char * const Byt);
void Float2Byte(unsigned char * const Byt, float fdat);
float Byte2Float_(unsigned char * const Byt);

int Byte2ShortInt(unsigned char * const Byt); 

static void Set_Driver_Ref(void);
static void Elmo_Sync(void); 

static void copy_elmo_cmd(elmo_cmd *, elmo_cmd *);
static void  init_elmo_cmds(void);
static void start_search_home(void);
static void Clear_Encoder_Elmo(void);
static void elmo_send_home_offset(void);
unsigned int elmocmd;
static BOOL elmo_poweron_flag = FALSE;
static BOOL period_cntl = TRUE ; // flag, CAN control periodicly
static BOOL elmo_poweron = FALSE; //flag, elmo driver power on
static BOOL elmo_enc_clear = FALSE; //flag, clear elmo encoder
static BOOL elmo_search_home = FALSE;  //flag, search joint  home point
static BOOL elmo_setref = FALSE; //	
static BOOL send_timestamp = FALSE;
static int elmo_poweron_delay = 0; 
static int elmo_search_home_delay = 0;
static int elmo_px_eq0 = FALSE;
static double Iner_Sensor_Current[6];
static double Iner_Sensor_Offset[6];


int Home_Sequence=0;
int Arm_Sequence_Mov=0;  //whp
int Arm_Poweroff_Right4=0x55; //whp
int Arm_Poweroff_Right5=0x55;	//whp
int Arm_Poweroff_Right6=0x55; //whp
int Arm_Poweroff_Right7=0x55; //whp
int Arm_Poweroff_Right2=0x55; //whp

int Arm_Poweroff_Right7_ptr=0;
int Arm_Poweroff_Right6_ptr=0;
int Arm_Poweroff_Right5_ptr=0;
int Arm_Poweroff_Right2_ptr=0;

static double Joint_Arm_Old[3][8];
static double Ref_Arm_Val_Old[3][8];
///////////////////////////////

static long setref_cnt = 0;//cxc


short int temp_i=0;

double	Pitch_Angle_adis[1] = {0.0}; //
double	Roll_Angle_adis[1] = {0.0};
double	Pitch_Gyro_adis[1] = {0.0};
double	Roll_Gyro_adis[1] = {0.0};


BOOL Read_Offline_Data = TRUE;


void Update_Response(void);
void init_period_cntl(void);
void loop_period_cntl(void);
void period_end_cntl(void);

static double time_2_now = 0.0;
static double time_1_period = 0.0;
static double timegap_1_period = 0.0;

void SoftReset_via_CANopen(void);
static int SoftReset_CAN = 0;
static BOOL SoftReset_CAN_trigged = FALSE;
static double Feedback_Max[7][9]; //inside elmo limit 
static double Feedback_Min[7][9];


int ID_File = 0; //Start from 0
extern BOOL File_Read_End;

void Cal_Plan_File_Len(void);

void Reload_Encoder(void);

void rtx_delayus(int us)
{
	LARGE_INTEGER  stime, etime;
	long elapsed_t;
	if(!RtGetClockTime( CLOCK_FASTEST, &stime)) { //0.100us units, time.QuadPart ->Signed 64-bit integer
		printf("Error:  Could not get clock time.  GetLastError = %d\n", GetLastError());
		return;
	}
	do{
		RtGetClockTime( CLOCK_FASTEST, &etime);
		elapsed_t = (LONG) (etime.QuadPart-stime.QuadPart);
	}while(elapsed_t<us*10);	

}

static long Rt_Resp_Cnt = 0;
static long ControlledCycle = 0;
void Save_Rt_Response(void);
int RTFCNDCL Timer_SendResponse_Handler(PVOID unused);
BOOL Send_Response_Flag = TRUE;

/*Added on Dec.14, 2011 @UEC*/
/*Limit increment of joint reference within each control cycle, i.e. 4ms*/
#define REF_CHANGE_RATE  (40.0*M_PI/180.0) //10 degree
static BOOL Valid_Joint_Speed[7][9]; //Check joint velocity from command. if overspeed, equals to FALSE;
static double Ref_Val_at_Joint_Side[7][9]; //
static double Ref_Val_at_Joint_Side_previous[7][9]; 
static BOOL Motor_On_Flag = FALSE; //MO=1?
void Con_Data_Trans(void);


int main(int argc, char *argv[])
{

	HANDLE 	hTimer;
	
	LONG    sampleTime = 1000 * 15;
	PCHAR   msg = "ms";
	ULONG	stackSize = 0;
	LONG	usSize = 10;
	ULONG	nbytes = 1;
	LONG	decClockRes = 50;
	int		defaults = 1;
	int		sleepTime = 1000;
	DWORD	dwMaximumSizeHigh = 0;
	HANDLE	hShm_Data, hShm_Cntl;
	//HANDLE	hRESP_Shm_Data;
	LONG	lInitialCount = 0;
	LONG	lMaximumCount = 1;
	LONG	lReleaseCount = 1;
	int		Response_Count = 0;
	int itemp=0;
	PVOID pMallocAddr = NULL;
	int i=0;

	Fastest = TRUE;
	Sound   = TRUE;
	Hist    = TRUE;
	Period.QuadPart = (LONG)(CONTROL_PERIOD*10000000L); //unit 0.1usec
	
	hMutex_Data = RtCreateMutex( NULL, FALSE, (LPCWSTR)MSGSTR_MUTEX_DATA);
	if (hMutex_Data==NULL)
		printf("\nRtCreate Mutex Data failed.");	

	hMutex_Cntl = RtCreateMutex( NULL, FALSE, (LPCWSTR)CTRLMSG_MUTEX_CNTL);
	if (hMutex_Cntl==NULL)
		printf("\nRtCreate Mutex Cntl failed.");

	hShm_Data = RtCreateSharedMemory( PAGE_READWRITE, dwMaximumSizeHigh, sizeof(MSGSTR), (LPCWSTR)MSGSTR_SHM_DATA, (LPVOID) &pMsg_Data);
	if(GetLastError()==ERROR_ALREADY_EXISTS)
		printf("\nThe shared DATA memory already exist.The routine may already be running.");	
	if (hShm_Data==NULL)
		printf("RtCreate DATA SharedMemory failed.");

	//hRESP_Shm_Data = RtCreateSharedMemory( PAGE_READWRITE, dwMaximumSizeHigh, sizeof(MSGSTR), (LPCWSTR)RT_MSGSTR_SHM_DATA, (LPVOID) &pRESP_Msg_Data);
	// if(GetLastError()==ERROR_ALREADY_EXISTS)
		// printf("\nThe shared DATA memory already exist.The routine may already be running.");	
	// if (hRESP_Shm_Data==NULL)
		// printf("RtCreate DATA response SharedMemory failed.");
	hShm_Cntl = RtCreateSharedMemory( PAGE_READWRITE, dwMaximumSizeHigh, sizeof(CtrlMsg), (LPCWSTR)CTRLMSG_SHM_CNTL, (LPVOID) &pMsg_Cntl);
	if(GetLastError()==ERROR_ALREADY_EXISTS)
		printf("\nThe shared CONTROL memory already exist.The routine may already be running.");
	if (hShm_Cntl==NULL)
		printf("\nRtCreate CONTROL Shared Memory failed.");

	// hSemPost = RtCreateSemaphore( NULL, lInitialCount, lMaximumCount, (LPCWSTR)MSGSTR_SEM_POST);
	// if (hSemPost==NULL)
	// {
		// printf("Could not create Semaphore.  GetLastError = %d\n", GetLastError());
		// RtCloseHandle(hShm_Data);
		// RtCloseHandle(hShm_Cntl);
		// RtCloseHandle(hMutex_Data);
		// return FALSE;
	// }
	#ifdef JOINT_OFFSET
	joint_offset_init();
	#endif

	/* Move Contro by Preview Control */
	Init_Walk_Tra();                // !!! Initialize parameters in Preview Control Method.
	PreCon_Mode = PRECON_MOVE_NONE; // !!! Default Mode, NEVER delete this sentence.
	Mode_Flag = PRE_CON_MODE;       // !!! Default Mode, NEVER delete this sentence.
	
	init_period_cntl();
	//------------------------------------------------------------------------------------
	// Create timer to tranfer offline trajectory
	if (!(hTimer = RtCreateTimer(NULL, 	// Security - NULL is none
		stackSize,     // Stack size - 0 is use default
		Timer_TrajRecv_Handler,      // Timer handler
		NULL,  // NULL context (argument to handler)
		RT_PRIORITY_MAX,  // Priority
		CLOCK_FASTEST)))  // Always use fastest available clock
	{
		printf("Srtm: Error: Could not create the timer.  GetLastError = %d\n", GetLastError());
		return ERROR_OCCURED;
	}	

	SleepEx(50, FALSE); //wait 0.05sec

	// if(!RtReleaseSemaphore(hSemPost, lReleaseCount, NULL))  //release the block
	// {
		// printf("MsgClient: Error: Could not release semaphore.  GetLastError = %d\n", GetLastError());
		// return FALSE;
	// }

	// 初始化参数传输
	if (RT_InitDCCRunParms())
	{
		if (RT_LoadDCCRunParms(&dccRunParms))
		{
			;
		}
		else
		{
			RT_LoadDefalutDCCRunParms(&dccRunParms);
		}
	}
	RT_PrintfDCCRunParms(&dccRunParms);

	// Start timer	
	Period.QuadPart = (LONG)(1000); //unit 0.1usec
	if (!RtSetTimerRelative( hTimer, &Period, &Period))
	{
		printf("Could not set and start the timer.  GetLastError = %d\n", GetLastError());
		RtDeleteTimer( hTimer);
		return ERROR_OCCURED;
	}
	Read_Pattern_Flag = TRUE;
	//wait for transfer plan trajectory
	printf("\n waiting for read offline trajectory");
	do {
		SleepEx(100, FALSE); //wait 0.2sec
		//printf("\n in while waiting for read offline trajectory");
	}while(Read_Pattern_Flag);

	SleepEx(100, FALSE);
	if(!RtDeleteTimer( hTimer))
	{
		printf("Srtm: Error: Could not delete timer.  GetLastError = %d\n", GetLastError());
		return ERROR_OCCURED;
	}
	printf("\n Trajectory Timer ceased");
	//------------------------------------------------------------------------------------
	//    GetCANCard1680Addr_OneCard();

	//	init_period_cntl();
	// Create timer object
	if (!(hTimer = RtCreateTimer(NULL, 	// Security - NULL is none
		stackSize,     // Stack size - 0 is use default
		TimerHandler,      // Timer handler
		NULL,  // NULL context (argument to handler)
		RT_PRIORITY_MAX,  // Priority
		CLOCK_FASTEST)))  // Always use fastest available clock
	{
		printf("Srtm: Error: Could not create the timer.  GetLastError = %d\n", GetLastError());
		return ERROR_OCCURED;
	}	

	SleepEx(100, FALSE); //wait 0.1sec

	// Start timer	
	Period.QuadPart = (LONG)(CONTROL_PERIOD*10000000L); //unit 0.1usec
	if (!RtSetTimerRelative( hTimer, &Period, &Period))
	{
		printf("Could not set and start the timer.  GetLastError = %d\n", GetLastError());
		RtDeleteTimer( hTimer);
		return ERROR_OCCURED;
	}

	Cal_Plan_File_Len();
	/*do something for ever*/
	//	while(1)  //set flag for exit //cycle = 1sec,
	while(EndlessLoopFlag)
	{

		SleepEx(200, FALSE); //sleep 0.5sec

	}

	if(!RtDeleteTimer( hTimer))
	{
		printf("Srtm: Error: Could not delete timer.  GetLastError = %d\n", GetLastError());
		return ERROR_OCCURED;
	}
	
	period_end_cntl();
//----------Transfer real-time response here---------
#if 0  //Transfer real-time response here

	SleepEx(50, FALSE); //wait 0.05sec
	if (!(hTimer = RtCreateTimer(NULL, 	// Security - NULL is none
		stackSize,     // Stack size - 0 is use default
		Timer_SendResponse_Handler,      // Timer handler
		NULL,  // NULL context (argument to handler)
		RT_PRIORITY_MAX,  // Priority
		CLOCK_FASTEST)))  // Always use fastest available clock
	{
		printf("Srtm: Error: Could not create the timer.  GetLastError = %d\n", GetLastError());
		return ERROR_OCCURED;
	}	

	SleepEx(50, FALSE); //wait 0.05sec

	// Start timer	
	Period.QuadPart = (LONG)(1000); //unit 0.1usec
	if (!RtSetTimerRelative( hTimer, &Period, &Period))
	{
		printf("Could not set and start the timer.  GetLastError = %d\n", GetLastError());
		RtDeleteTimer( hTimer);
		return ERROR_OCCURED;
	}
	Send_Response_Flag = TRUE;

	printf("\n waiting for sending response");
	do{
		SleepEx(10, FALSE); //wait 0.2sec

	}while(Send_Response_Flag);

	SleepEx(100, FALSE);
	if(!RtDeleteTimer(hTimer))
	{
		printf("Srtm: Error: Could not delete timer.  GetLastError = %d\n", GetLastError());
		return ERROR_OCCURED;
	}
	printf("\n sending response Timer ceased");
#endif //end of transferring realtime response	
	
	SleepEx(300, FALSE); //sleep 0.5sec
	RtCloseHandle(hShm_Data);
	RtCloseHandle(hShm_Cntl);
	RtCloseHandle(hSemPost);
	//RtCloseHandle(hRESP_Shm_Data);
	//period_end_cntl();
	
	LogResponse_RTSS(); 
	
	printf("\n\nProgram terminated normally.");
	return NO_ERRORS;
}
//
void Cal_Plan_File_Len(void)
{
	int i=0;
	for(i=Actual_File_Num; i>=0; i--){
		if(i==Actual_File_Num)
			Last_File_Length[i]= Max_Num_Rfdat_All - iFileStartPoint[i];
		else
			Last_File_Length[i]	= iFileStartPoint[i+1]-iFileStartPoint[i];
		printf("\n Last_Len[%d]=%d, start[%d]=%d;\n",i,Last_File_Length[i],i,iFileStartPoint[i]);
	}

}
int RTFCNDCL Timer_TrajRecv_Handler (PVOID unused)
{
	static long group_id = 0;
	static long pre_group_id = 0;
	int cnt=0;
	DWORD dw1;
	int i=0;
	long temp_cnt=0;
	static int iFileID=0;
	static int Last_FileID = 0;

	dw1 = RtWaitForSingleObject(hMutex_Cntl, INFINITE );
	if(dw1==WAIT_OBJECT_0) {
		if( pMsg_Cntl->command ==OFFLINE_DATA_TRANSFER){//tranfer		
			group_id = pMsg_Cntl->arg1;
			if((group_id != pre_group_id )&& group_id>=1){
				//printf("\n group id %d", group_id);
				cnt = 1;
				iFileID = pMsg_Cntl->FileID;
				
				if(Last_FileID != iFileID){ //new file
					Max_Num_Rfdat_All--; //Important!
					iFileStartPoint[iFileID] = Max_Num_Rfdat_All;
				}
				
				while(cnt <= pMsg_Cntl->arg2){
					Rfdat_Buf[Max_Num_Rfdat_All++] = pMsg_Cntl->arg3[cnt-1];
					cnt++;
				}				

				Last_FileID = iFileID;
				Actual_File_Num = Last_FileID;	
				
				//printf("\n Total:%d, StartPoint[%d]=%d, ", Max_Num_Rfdat_All,iFileID,iFileStartPoint[iFileID]);
				//printf("Last_File_Length:%d\n", Last_File_Length[iFileID]);

				pMsg_Data->Buffer[0] =TRANFER_SUCCESS;			
				pMsg_Data->Buffer[1] =group_id;  //control period
				pre_group_id = group_id;	

			}

		}
	}else if(dw1==WAIT_ABANDONED){
		printf("\n  Mutex_CNTL abanoned");
	}else if(dw1==WAIT_FAILED){
		printf("\n Mutex_CNTL failed");
	}	
	RtReleaseMutex(hMutex_Cntl);

	if( pMsg_Cntl->command ==OFFLINE_DATA_END){
		/*
		printf("\n Exit tranfer");
		for(i=Last_FileID; i>=0; i--){
			if(i==Last_FileID)
				Last_File_Length[i]= Max_Num_Rfdat_All - iFileStartPoint[i];
			else
				Last_File_Length[i]	= iFileStartPoint[i+1]-iFileStartPoint[i];
			printf("\n Last_Len[%d]=%d, start[%d]=%d;\n",i,Last_File_Length[i],i,iFileStartPoint[i]);
		}
		*/
		
		Read_Pattern_Flag = FALSE;
		

	}
	/*
	for(i=0;i<10;i++){
		Max_Num_Rfdat[i] -=1;
	} */
	
	//printf("\n data: %f, 42385=%f, 42386=%f", Rfdat_Buf[20], Rfdat_Buf[42385+20], Rfdat_Buf[42386+20]);

	return NO_ERRORS ;
}

/*************For transmit realtime response ******************/
int RTFCNDCL Timer_SendResponse_Handler(PVOID unused)
{
	//static long group_id = 0;

	DWORD dw;
	int i=0;
	static long cycle_cnt = 0;
	static long response_group_cnt = 0;
	static LONG Send_Rtdat_Single_Cnt = 0;
	static BOOL rt_dat_save  = FALSE;
	if(pMsg_Cntl->command ==RT_DAT_SAVE){
		rt_dat_save = TRUE;	
	}
	if(rt_dat_save == TRUE){
		dw = RtWaitForSingleObject(hMutex_Data, INFINITE);/*the handle of object to wait for ever*/
		if(dw==WAIT_FAILED){
			printf("\n Mutex data failed");
		}else if(dw==WAIT_ABANDONED){
			printf("\nMutex data abanoned");
		}else if (dw==WAIT_OBJECT_0) { //mutex was locked
			if(cycle_cnt==0){ //first time
				pRESP_Msg_Data->Response_Index = 0;
				for(i=0;i<MAX_RESP_MSGSTR_SIZE && Send_Rtdat_Single_Cnt<Rt_Resp_Cnt;i++){
					pRESP_Msg_Data->Buffer[i] = RtResponse_Buf[Send_Rtdat_Single_Cnt++];
				}
				pRESP_Msg_Data->flag =NEW_FRAME;
			}
			if(cycle_cnt++>10) 
				cycle_cnt = 10;
			if(pRESP_Msg_Data->flag ==RECEIVE_OK){
				pRESP_Msg_Data->Response_Index = ++response_group_cnt;
				for(i=0;i<MAX_RESP_MSGSTR_SIZE && Send_Rtdat_Single_Cnt<Rt_Resp_Cnt;i++){
					pRESP_Msg_Data->Buffer[i] = RtResponse_Buf[Send_Rtdat_Single_Cnt++];
				}
				pRESP_Msg_Data->flag =NEW_FRAME;
			}
			if(Send_Rtdat_Single_Cnt+50 >= Rt_Resp_Cnt){
				pRESP_Msg_Data->flag = END_SEND;//data is over
				Send_Response_Flag = FALSE; //exit while
			}
			RtReleaseMutex(hMutex_Data);
		}
	}
	return NO_ERRORS ;
}
// 
int RTFCNDCL TimerHandler (PVOID unused)
{
	LARGE_INTEGER       stime, etime;
	LONGLONG           offset, offset_last;
	LONGLONG	time_from_start;
	static long cnt=0, cnt1=0;
	static LARGE_INTEGER  last_time;
	static LARGE_INTEGER  start_time;
	//static PMSGSTR 	pMsg;
	static short ii=0;
	DWORD 	dw;
	static int cntl_flag=1;
	static LONG LastControlledTime = 0;
	static int n=0;
	int i=0,j=0,k=0;
	static int Response_Count = 0;
	static BOOL record_rt_dat_flag;	
	static double temvx=0;
	static double temvz=0;
	int printftemvx,printftemvz;
	static double tempz=0;
	int printftempz;
	int printfgoutputballdelta;
	BOOL Key_Press_Walk_On= FALSE;
	long wifi_dat1, wifi_dat2;

	LastControlledTime++;
	ControlledCycle = LastControlledTime;
	if(LastControlledTime == 1){
		if(!RtGetClockTime( CLOCK_FASTEST, &start_time))  //0.100us units, time.QuadPart ->Signed 64-bit integer
		{
			printf("Error:  Could not get clock time.  GetLastError = %d\n", GetLastError());
			return ERROR_OCCURED;
		}
	}
	if(!RtGetClockTime( CLOCK_FASTEST, &stime))  //0.100us units, time.QuadPart ->Signed 64-bit integer
	{
		printf("Error:  Could not get clock time.  GetLastError = %d\n", GetLastError());
		return ERROR_OCCURED;
	}

	dw = RtWaitForSingleObject(hMutex_Cntl, INFINITE );/*the handle of object to wait for ever*/
	if(dw==WAIT_FAILED){
		printf("\n Mutex CNTL failed");
	}else if(dw==WAIT_ABANDONED){
		printf("\nMutex_CNTL abanoned");		
	}else if (dw==WAIT_OBJECT_0)	{ //mutex was locked
		//read data from non_realtime process
		//if(fabs(pMsg_Cntl->Buffer[RT_START])<0.1)
		//	EndlessLoopFlag = FALSE;
		switch (pMsg_Cntl->command)
		{
		case RT_START:
			EndlessLoopFlag = pMsg_Cntl->arg1;
			//period_end_cntl();
			printf("\n space pressed");
			break;
		case TEST_KEY:
			printf("\n key %c pressed",pMsg_Cntl->arg1);			
			break;			
		case START_WALK:		
		#if 1
			Key_Press_Walk_On = pMsg_Cntl->arg1;		
			ID_File = Demo_Get_ID_File(Mode_Flag, ID_File);
			printf("ID_File = %d\n",ID_File);
			if(Key_Press_Walk_On==TRUE && Walk_On==FALSE)
			{ 
				if(ID_File <= Actual_File_Num)
				{
					Walk_On = TRUE;
					if(File_Read_End==TRUE)
					{
						Actual_File_Data_Ptr = iFileStartPoint[ID_File];
					}
				}else
				{
					Walk_On = FALSE;
				}	
				
			}
			else
			{
				Walk_On=FALSE;
			}
		#endif	
			// Walk_On = pMsg_Cntl->arg1; // zqq comment 0212
			break;
		
		
		/***************** FALL AND WALK LQQ**************************/
		// 20170529 STATE TRANSITION
		case HELLO:
			Mode_Flag = HELLO;
			printf("\n Say hello to viewers !");
			break;
			
		case CRAWL:
			Mode_Flag = CRAWL;
			printf("\n Crawl !");
			break;
			
		case FALL_DL:
			Mode_Flag = FALL_DL;
			printf("\n Fall !");
			break;
			
		case ACTION:
			Mode_Flag = ACTION;
			printf("\n Press the button !");
			break;	
			
		case FALL_START_FLAG:
			Fall_Start_flag = TRUE;
			break;
			
		case PRE_CON_MODE:
			Mode_Flag = PRE_CON_MODE;
			printf("\n Preview Control Walk !");
			break;
		
	/***************** Move Control by Previre Control x *************/
		case PRECON_MOVE_FORWARD: // Move Forward			
			PreCon_Mode = PRECON_MOVE_FORWARD;
			PreCon_ModeDisp = DISP_PRECON_MOVE_FORWARD;					
			break;
			
		case PRECON_MOVE_LEFT:    // Move Left Side			
			PreCon_Mode = PRECON_MOVE_LEFT;
			PreCon_ModeDisp = DISP_PRECON_MOVE_LEFT;					
			break;
			
		case PRECON_MOVE_RIGHT:   // Move Right Side
			PreCon_Mode = PRECON_MOVE_RIGHT;
			PreCon_ModeDisp = DISP_PRECON_MOVE_RIGHT;					
			break;
		
		case PRECON_MOVE_BACK:    // Move Back
			PreCon_Mode = PRECON_MOVE_BACK;
			PreCon_ModeDisp = DISP_PRECON_MOVE_BACK;					
			break;		
	/***************** Move Control by Previre Control s *************/
	
	
		case HOME_ON_LEG:
			home_on_flag = pMsg_Cntl->arg1;
			home_on_arm_flag=TRUE;//手臂变直
			break;
		case SEARCH_HOME:
			elmo_search_home_flag = TRUE;
			break;
		case POWER_ON:
			elmo_poweron_flag = pMsg_Cntl->arg1;
			break;
		case SENSORY_CONTROL:
			Scntl_On = pMsg_Cntl->arg1;
			Parkstart = 1;
			break;
		case OFFSET_GYRO_ACC:
			Offset_On = TRUE;
			break;
		case ARM_HOME_ON:
			home_on_arm_flag= TRUE;
			//Home_Sequence++; //whp
			if(Home_Sequence==2)
			{	
				Home_Sequence = 3; 
				ctrl_wasit_leg_online_flag = FALSE;  //防止被'L'关闭Get_Plan()后无法再次启动 WHP
			}
			else 
				Home_Sequence = 1;
			break;
		case HOME_VERTICAL_LEG: 
			if(pMsg_Cntl->arg1==1)
				Leg_Init_Flag = 0xaa; //Initial vertical leg
			if(pMsg_Cntl->arg1==0)
				Leg_Init_Flag = 0x55; //bended leg
			break;	
		case SIN_TRAJ_TEST:
			Sin_Test_On = 0xaa;
			break;
		case SOFTRESET_CANOPEN:
			SoftReset_CAN = 0xaa;
			break;
		case RECORD_RT_DAT:
			record_rt_dat_flag = pMsg_Cntl->arg1;
			//RtPrintf("\n record %d",record_rt_dat_flag );
			break;
		case RESET_ELMO_SAVE_ENCODER:
			Reset_Elmo_Save_Encoder = 0xaa;
			break;	

		default:

			break;		
		} //end of switch
		//reset pMsg_Cntl->command =0, to be not available
		pMsg_Cntl->command = 0;
		RtReleaseMutex(hMutex_Cntl);
	}	
	/***************************fall************/

	loop_period_cntl();	 //evoke every cycle 
	if(record_rt_dat_flag){ //true
		Save_Rt_Response();
	}
	//printf("\n I am here");
	/*end cycle */
	if(!RtGetClockTime( CLOCK_FASTEST, &etime))  //0.100us units, time.QuadPart ->Signed 64-bit integer
	{
		printf("Error:  Could not get clock time.  GetLastError = %d\n", GetLastError());
		return ERROR_OCCURED;
	}
	offset = (LONGLONG) (etime.QuadPart-stime.QuadPart);
	time_1_period = offset*0.0001;
	offset_last = (LONGLONG) (stime.QuadPart-last_time.QuadPart);
	time_from_start = (LONGLONG) (etime.QuadPart-start_time.QuadPart);
	time_2_now = time_from_start*0.0001;
	time_2_now = time_2_now*0.001;
	timegap_1_period = offset_last*0.0001;

	if(LastControlledTime % 20==0) {
		//RtPrintf("\n Elapsed time : %d, period time: %d, endless flag", offset, offset_last, EndlessLoopFlag);
		//printf("\n %d", LastControlledTime);
	}
	last_time.QuadPart = stime.QuadPart;	
	//if( n == 0)
	{

		k=0;
		dw = RtWaitForSingleObject(hMutex_Data, INFINITE );/*the handle of object to wait for ever*/
		if(dw==WAIT_FAILED){
			printf("\n Mutex data failed");
		}else if(dw==WAIT_ABANDONED){
			printf("\nMutex data abanoned");
		}else if (dw==WAIT_OBJECT_0) { //mutex was locked
			pMsg_Data->Buffer[k++] =LastControlledTime * CONTROL_PERIOD;
			Update_Response();
			if(Response_Count++>6000)
				Response_Count = 0;
			pMsg_Data->Response_Index = Response_Count;
			//release the mutex
			RtReleaseMutex(hMutex_Data);
		}		
	}
	//	n = (n + 1) % RESPONSE_SAMPLE_PERIOD;

	return NO_ERRORS ;
}


void init_period_cntl(void)
{

	int i=0,j=0,k=0,n = 0;
	unsigned char heartbeatdat[8];
	CAN_STRUCT *canstructvar;
	canstructvar = &can_struct_set;
	printf("\n\n _____program Begining______");
	canstructvar->accode[0]=0x30; ////Elmo,receive ID: 0x18? and 0x08?      
	canstructvar->accode[1]=0x0;
	canstructvar->accode[2]=0x0;
	canstructvar->accode[3]=0x0;

	canstructvar->accmask[0]=0x01; //only receive 0x18x, //0x21; 
	canstructvar->accmask[1]=0xe0;
	canstructvar->accmask[2]=0xff;
	canstructvar->accmask[3]=0xff;
	canstructvar->speed=CAN_1000K; //CAN_800K;//CAN_1000K; baud rate 1Mbps 
	canstructvar->interruptmask=0xff;
	canstructvar->filtertype = 0; //0 for single filter*, 1 double filter	

	DTIME = CONTROL_PERIOD;
	
	RtPrintf("  \nNow initialize Ehanced Parallel Port.\n ");	
	Init_SPP_RTX();	
	SPP_LowPin17();
	RtPrintf("  Now initialize PCI-3680 CAN card.\n ");
	GetCANCard_PCM3680I();
	//GetCANCard1680();
	for(i=0; i<MAX_CAN_CHANNEL; i++){
		can_Init_set_param(i, canstructvar);
	}
	ctrl_wasit_leg_online_flag=FALSE;
	Plan_Count =0;
	FirstCANError=0;	

	//*************************	
	//added on Dec.14 2011 @UEC
	for(i=0;i<7;i++){
		for(j=0;j<9;j++){
			Valid_Joint_Speed[i][j] = TRUE;; //Check joint velocity from command. if overspeed, equals to FALSE;
			Ref_Val_at_Joint_Side[i][j] = 0.0; //
			Ref_Val_at_Joint_Side_previous[i][j] = 0.0;
		}
	}	
	Now_Rfdat_Num = 0; 
	for(i=0;i<10;i++) {
		iFileStartPoint[i] = 0; /*file length of each file*/
		Last_File_Length[i]=0;
	}
	Max_Num_Rfdat_All = 0; /*Total data num*/
	Max_Num_Rfdat_Sum = 0;
	Actual_File_Data_Ptr = 0;
	//ID_File = 0;
	//printf("ID_File=%d",ID_File);
	
	for(i=1;i<=15;i++) {  /*--yzg,init encoder-*/
		for(k=0;k<=1;k++)
		{ 
			enc_val[i][k]=0.0;
			enc_val_1[i][k]=0.0;
			enc_val_2[i][k]=0.0;
			CheckCANNode[i]=NCNT;
		}
	}
	for(i=0;i<7;i++){
		for(j=0;j<9;j++){
			MotorRateCurrent[i][j] = 0.0;
			MotorCurrent[i][j]=0.0;
		}
	}
	for(i=0;i<5;i++)
	{
		Heading_Angle[i]=0.0;
		Pitch_Angle[i]=0.0;
		Roll_Angle[i]=0.0;
	}
	AHRS_Offset[0]=0.0;
	AHRS_Offset[1]=0.0;
	AHRS_Offset[2]=0.0;
	AHRS_Offset[3]=0.0;
	Pitch_Gyro[0]=0.0;
	Roll_Gyro[0]=0.0;
	for(i=0; i<3; i++){
		y_posture[i][1] = 0.0;
		y_posture[i][2] = 0.0;
	}
	DFZ_Count = 0 ;	
	//int ttt=0;
	//pthread_setfp_np(pthread_self(),1);  /* floating point calculation*/ 

	Scntl_On = 0;
	Home_On  = 0;
	Walk_On  = 0;
	Offset_On = 0;
	Sin_Test_On=0x55; 
	Check_Joint_On=0;
	dsp_search_home=0x55;
	P_Ankfz_Switch = 0;
	P_Posfz_Switch = 0;
	P_Fotfz_Switch = 0;

	Parkstart = 0;//xt-11-30

	P_control = 0;//xt
	LEG_NO = 0;//xt
	COP_R_Count = 0; 

	Servomotor1 = 0;
	Ref_Feedback_Error=0.0;
	Test_Time=0;	
	Test_Time_Arm=0;	
	for(i=0;i<3;i++){  
		P_Count[i]=0;
		P_Count2[i]=0;
		Ball_Coordinate[i]=0.0;
		Hand_Coordinate[i]=0.0;
		Exec5[i] = 0;
		Exec6[i] = 0;
	}
	AHRS_ERR=0x55;
	Leg_Init_Flag = 0x55;
	Home_Sequence=0;
	//Arm_Read_Offline_Data=TRUE;//whp
	//----------------
	for( i=1;i<=LEG_NUM;i++)           /* reset planned data */
		for(k=1;k<=JOINT_NUM;k++)
			Ref_Val[i][k] =0; 

	for( i=1;i<=ARM_NUM;i++){
		for(k=1;k<=JOINT_ARM_NUM;k++){
			Ref_Arm_Val[i][k] =0;
		}
	}
	for(k=1;k<=JOINT_Head_NUM;k++){
		Ref_Head_Val[k] =0;
	}

	for(i=0;i<=2;i++){   /*--yzg,init encoder-*/
		for(k=0;k<8;k++){
			Joint_Arm_Old[i][k] = 0.0;
			Ref_Arm_Val_Old[i][k]= 0.0;
		}
	}

	for(k=1;k<=JOINT_WAIST_NUM;k++){ //waist ,modified data
		Ref_Waist_Val[k] =0;
		Rtf_Waist_Val[k] =0;
		Mdf_Waist_Val[k] =0;

	}
	for( i=1;i<=LEG_NUM;i++){           /* reset modified data */
		for(k=1;k<=JOINT_NUM;k++) {
			Mdf_Val[i][k] =0;
			Mdf_Val_Old[i][k] =0;
			Mdf_In[i][k]=0 ;   //
			Mdf_De[i][k]=0 ;   //
			Mdf_Val_Pos[i][k] = 0; //xt-07-10-5
		}
	}

	for(i = 0; i < 3; i++){ //for dz and ankle control by xt
		P_ZAnkle[i] = 0.0;
		Park[i] = 0;
	}

	for( i=0; i<=1; i++){
		Gsen_Offset[i] = 0.0;
		Gyro_Offset[i] = 0.0;
		Qbody_Init[i+1] = 0.0;
	}

	for( i=1;i<=LEG_NUM;i++){           /* reset control angle data */
		for(k=1;k<=JOINT_NUM;k++) 	{ 
			con_val[i][k] =0;
			Con_Val_Leg[i][k]=0.0;
			Ref_Leg_Last[i][k]=0.0;  //090620
			Ref_Leg_Last_2[i][k]=0.0;  //2010-01-12
		}
	}  
	for( i=1;i<=ARM_NUM;i++){   //---add by yzg
		for(k=1;k<=JOINT_ARM_NUM;k++){
			Con_Val_Arm[i][k]=0.0;
			Con_Disp_Arm[i][k]=0.0; //yzg
			Con_Temp_Arm[i][k]=0.0; 
		}
	}
	for (i=1;i<=JOINT_Head_NUM;i++) {
		Con_Val_Head[i]=0.0;
		Con_Disp_Head[i]=0.0; //yzg
		Con_Temp_Head[i]=0.0; 
	}
	for (i=1;i<=JOINT_WAIST_NUM;i++) {//waist
		Con_Val_Head[i]=0.0;
		Con_Disp_Head[i]=0.0; //yzg
		Con_Temp_Head[i]=0.0; 
	}

	for( i=1;i<=LEG_NUM;i++){       /* reset control velocity data */
		for(k=1;k<=JOINT_NUM;k++)
			conv_val[i][k] =0;
	}
	for( i=1;i<=LEG_NUM;i++){           /* reset body posture data */
		for(k=1;k<=JOINT_NUM;k++)
			Nowq_Val[i][k] = 0;
	}
	for(i = 1; i <= LEG_NUM ; i++){
		Dz_Val[i]  = 0.0;
		for(k = 0; k < 3; k++)
			D_ANKLE[i][k] = 0.0;		
	}

	for( i=1;i<=LEG_NUM;i++) {          /* reset feet position */
		Ref_Ankle[i][0] = 0.5*WIDTH_FOOT;
		Ref_Ankle[i][1] = 0.0;
		Ref_Ankle[i][2] =-2.0*LENG_LEG;
	}
	Ref_Ankle[2][0] = -0.5*WIDTH_FOOT;

	for( i=0;i<=2;i++) {          /* reset planned Qbody data */
		Ref_Qbody[i] =0.0;
		Ref_Qvbody[i] =0.0;
	}

	for( i=1;i<=2;i++)                  /* reset support signal */
	{      Ref_Cnt_Sgnl[i] = 1;
	}
	for( i=0;i<=5;i++)                  
		Cntl_Flag[i] = 0;  //trace executing route

	for(i = 1 ; i <= 2 ; i++) 
		Ref_Timecntl[i]  = 0;

	Ref_Yzmp[1] = -L_YBACK; //0.103;//-0.1;                 /* ZMP margin [1]:min; [2]:max*/
	Ref_Yzmp[2] =  L_YFRONT;                    //0.127;//0.13;
	Ref_Xzmp[1] = -0.5*WIDTH_FOOT-W_OUT;       //0.2;//-0.2;
	Ref_Xzmp[2] =  0.5*WIDTH_FOOT+W_OUT;       //0.2;//0.2;

	Boot_Robot();              /* Initialize robot system */	
	Init_Q_body();
	//----------------------------------------
	CheckDSPControllerConfig2(); //transfer DSP Config to HMI
	for( i=1;i<=ARM_NUM;i++){
		for(k=1;k<=JOINT_ARM_NUM;k++){       
			Init_Arm_Val[i][k] = Joint_Arm[i][k];
			Ref_Arm_Val[i][k] = Init_Arm_Val[i][k];
		}
	}
	for(k=1;k<=JOINT_Head_NUM;k++){      
		Init_Head_Val[k] = Joint_Head[k];
		Ref_Head_Val[k] = Init_Head_Val[k];
	} 
	for( i=1;i<=LEG_NUM;i++){
		for(k=1;k<=JOINT_NUM;k++){
			Init_Val[i][k] = Joint[i][k];
			Ref_Val[i][k] = Init_Val[i][k];
		}
	}
	//for waist 
	for(k=1;k<=JOINT_WAIST_NUM;k++){
		Init_Waist_Val[k] = Joint_Waist[1][k];   //---
	}
	LastControlledTime = 0;
	Power_On_Flag=0x55;
	for(i=0; i<6; i++){
		Iner_Sensor[i] = 0.0; //inertial sensor data
		Iner_Sensor_Current[i] = 0.0;
		Iner_Sensor_Offset[i] = 0.0;
	}
	Home_On_Again = FALSE;     
	Check_Elmo_Config(); //2010-1-6
	init_elmo_cmds();
#if 1
	heartbeatdat[0]=0x23;
	heartbeatdat[1]=0x07; //0x6007
	heartbeatdat[2]=0x60;
	heartbeatdat[3]=0x00; //sub-index
	heartbeatdat[4]=0x00;
	heartbeatdat[5]=0x00;
	heartbeatdat[6]=0x00;
	heartbeatdat[7]=0x00;
	
	for(i=0; i<MAX_CAN_CHANNEL; i++){
		for(j=1;j<=6;j++){
			if(CanCfg[i+1][j]){
				rtx_delayus(200);
				G_CANopen_NMT_Reset(i, j); 
				rtx_delayus(200);				
				//CANopen_Enter_Preoperation(i, 0); //int channel_num, int CAN_id,  CAN_id =0 ->broadcast
				//CANopen_NMT_Go_Opration(i, 0);//channel 0,broadcast	
				//CANopen_Clear_Cntl_Fault(i, 0); //clear ds402 fault
				G_CAN_MapDS402_IPSet(i, j, (int)(CONTROL_PERIOD*1000.0)); //int channel_num,int CAN_id, int ip_cycle_time			
				rtx_delayus(200);
				G_CAN_DS402_Elmo_Specific(i, j); //channel 0,broadcast
				rtx_delayus(200);
				//G_CAN_DIS_R_T_PDO234(i,j); //RPDO2 & TPDO2 are used as Binary Interpreter Commands for Elmo drivers	
				rtx_delayus(200);
				G_CANopen_NMT_Go_Opration(i, j);//channel 0,broadcast
				rtx_delayus(200);
				G_CAN_MapDS402_IP_Shutdown(i, j);	//channel 0,broadcast
				rtx_delayus(200);
				//LQQ //int SendCmd(int channel, unsigned int msgID , unsigned  char *candata, unsigned char bytelength);
				SendCmd(i,0x600+j,heartbeatdat,8);
				rtx_delayus(200);
			}
		}
	
	}
	

	
	elmo_send_home_offset();
#endif	


}

void period_end_cntl(void)
{
	int i=0;
	int j=0;
	CAN_STRUCT *canstructvar;
	canstructvar = &can_struct_set;
	//init_period_cntl();
	printf("\n\n __Terminate motor torque_____\n");
#if 1	
	elmocmd = MOEQ0; //MO=0, servo poweroff 
	for(j=1;j<=6;j++){
		rtx_delayus(300);
		for(i=0; i<MAX_CAN_CHANNEL; i++){			
			SendCmd( i, elmo_cmds[elmocmd].can_id+j, elmo_cmds[elmocmd].dat,elmo_cmds[elmocmd].len_cmd );
		}
	}
	for(i=0; i<MAX_CAN_CHANNEL; i++){	
		G_CAN_MapDS402_IP_Shutdown(i, 0);	//int channel_num, int CAN_id,	
		//CANopen_Clear_Cntl_Fault(i, 0); //clear ds402 fault
		G_CANopen_NMT_Reset(i, 0); 
	}
	
#endif

	printf(" Exit control loop.\n ");	
	for(i=0; i<MAX_CAN_CHANNEL; i++){
		can_Init_set_param(i, canstructvar);
	}
	SPP_LowPin17();
}	

void loop_period_cntl(void) 
{
	int i,j,k,n = 0;

	double *buf =NULL;	
	static LARGE_INTEGER  prev_time;
	static LARGE_INTEGER  start_time;
	double IfsData[3][6]; //
	static int reset_cnt=0;
	static unsigned int msgID = 0x0; //Network Management
	static char tmpdat[8];
	
	buf = pMsg_Data->Buffer;	
	LastControlledTime++; 
	rtx_delayus(100);
	if(LastControlledTime == 1){
		if(!RtGetClockTime( CLOCK_FASTEST, &start_time))  //0.100us units, time.QuadPart ->Signed 64-bit integer
		{
			printf("Error:  Could not get clock time.  GetLastError = %d\n", GetLastError());
			return ;
		}
	}
	//-----Soft reset
	if (SoftReset_CAN==0xaa){
		SoftReset_CAN = 0;
		if (elmo_setref == FALSE){
			SoftReset_CAN_trigged = TRUE;
			period_cntl = FALSE;
			SoftReset_via_CANopen();
		}
	}
	//---reset elmo but reserve encoder
	if (Reset_Elmo_Save_Encoder==0xaa){
		if(reset_cnt++>=60)
		{
			Reset_Elmo_Save_Encoder = 0;
			reset_cnt = 60;
			//period_cntl =  TRUE;
		}
		if (elmo_setref == FALSE){
			SoftReset_CAN_trigged = TRUE;
			period_cntl = FALSE;			
		}
		if(reset_cnt==2)
		{
			printf("\n **Reset driver**\n");
			SoftReset_via_CANopen();
		}
		
		if(reset_cnt==40)
		{
			tmpdat[0] = 0x82; //Reset node, perform full software reset
			tmpdat[1] = 0x00; //All nodes
			for(i=0; i<MAX_CAN_CHANNEL; i++){
				SendCmd(i, msgID, tmpdat, 2);	
			}
		}
		
		if(reset_cnt==42)
		{
			tmpdat[0] = 0x80; //Enter pre-operational
			tmpdat[1] = 0x00; //All nodes
			for(i=0; i<MAX_CAN_CHANNEL; i++){
				SendCmd(i, msgID, tmpdat, 2);	
			}	
		}
		if(reset_cnt==44)
		{
			tmpdat[0] = 0x01; //start remote node
			tmpdat[1] = 0x00; //All nodes
			for(i=0; i<MAX_CAN_CHANNEL; i++){
				SendCmd(i, msgID, tmpdat, 2);	
			}				
		}
			
		if(reset_cnt>=46)
		{
			Reload_Encoder();
		}
		rtx_delayus(200); //delay  100us
	}	
	//------- elmo power on, mo=1 -------
	if(elmo_poweron_flag){ //press key '9'
		elmo_poweron = TRUE;
		elmo_poweron_delay = 0;
		elmo_poweron_flag = FALSE; 		
		for( i=1;i<=LEG_NUM;i++){
			for(k=1;k<=JOINT_NUM;k++){
				Init_Val[i][k] = Joint[i][k];
				Ref_Val[i][k] = Init_Val[i][k];
			}
		}
		for( i=1;i<=ARM_NUM;i++){
			for(k=1;k<=JOINT_ARM_NUM;k++){       
				Init_Arm_Val[i][k] = Joint_Arm[i][k];
				Ref_Arm_Val[i][k] = Init_Arm_Val[i][k];
			}
		}
		for(k=1;k<=JOINT_Head_NUM;k++){      
			Init_Head_Val[k] = Joint_Head[k];
			Ref_Head_Val[k] = Init_Head_Val[k];
		}
		for(k=1;k<=JOINT_WAIST_NUM;k++){       
			Init_Waist_Val[k] = Joint_Waist[1][k];
			Ref_Waist_Val[k] = Init_Waist_Val[k];
		}
	}
	// elmo drive search joint home point, press key '6'	
	if(elmo_search_home_flag==TRUE){
		elmo_search_home_flag = FALSE;
		elmo_search_home = TRUE;
		elmo_search_home_delay = 0;

	}
	
	if(Arm_Sequence_Mov==1)  //whp 连续P手
	{
		Arm_Sequence_Mov = 0;
		Home_Sequence =2;
		home_on_arm_flag = TRUE;
		
	}
	
		if(Arm_Sequence_Mov==2)  //whp 连续回P手
	{
		Arm_Sequence_Mov = 0;
		Home_Sequence =4;
		home_on_arm_flag = TRUE;
		
	}
	
	//Go to home through sine curve, press key '5'
	if(home_on_flag == TRUE ) { 
		home_on_flag = FALSE;
		Home_On= TRUE;              
		Test_Time=0;	
		for( i=1;i<=LEG_NUM;i++){
			for(k=1;k<=JOINT_NUM;k++){
				Init_Val[i][k] = Joint[i][k];
				Ref_Val[i][k] = Init_Val[i][k];
			}
		}
		for( i=1;i<=ARM_NUM;i++){
			for(k=1;k<=JOINT_ARM_NUM;k++){       
				//	Init_Arm_Val[i][k] = Joint_Arm[i][k];
				//	Ref_Arm_Val[i][k] = Init_Arm_Val[i][k];
			}
		}
		for(k=1;k<=JOINT_Head_NUM;k++){      
			Init_Head_Val[k] = Joint_Head[k];
			Ref_Head_Val[k] = Init_Head_Val[k];
		}
		for(k=1;k<=JOINT_WAIST_NUM;k++){       
			Init_Waist_Val[k] = Joint_Waist[1][k];
			Ref_Waist_Val[k] = Init_Waist_Val[k];
		}
	}
	//home arm through press 'P'
	if(home_on_arm_flag==TRUE){
		home_on_arm_flag = FALSE;
		Home_On_Arm = TRUE;
		Home_On_Begin = TRUE;
		Test_Time_Arm = 0;
		for( i=1;i<=ARM_NUM;i++){
			for(k=1;k<=JOINT_ARM_NUM;k++){       
				Init_Arm_Val[i][k] = Joint_Arm[i][k];
				Ref_Arm_Val[i][k] = Init_Arm_Val[i][k];
			}
		}
	}
	// control waist, leg to move dummily
	if(ctrl_wasit_leg_online_flag ==TRUE){
		Read_Offline_Data = FALSE;
		//Arm_Beat_Down_Return = TRUE; //should be commented	 		
	}
	else{
		Read_Offline_Data = TRUE;
	}
	Set_Driver_Ref(); //Elmo,***********!!!********
	//------------elmo control state-----
	if((elmo_enc_clear == TRUE) || (elmo_poweron == TRUE)||(elmo_search_home == TRUE)||(SoftReset_CAN_trigged==TRUE)) //
		period_cntl = FALSE;
	else
		period_cntl = TRUE; 

	if(elmo_poweron && !elmo_enc_clear && !elmo_search_home) {
		//delay 5ms
		elmo_poweron_delay++;
#ifdef ELMO_CAN_INIT
		if(elmo_poweron_delay == 10){
			elmocmd = MOEQ1; //MO=1, servo power on 
			for(i=0; i< MAX_CAN_CHANNEL; i++)
				SendCmd( i, elmo_cmds[elmocmd].can_id, elmo_cmds[elmocmd].dat,elmo_cmds[elmocmd].len_cmd );
		}
#endif
		if(elmo_poweron_delay >= 50){
			elmo_poweron = FALSE;
			elmo_setref = TRUE;	
		}
	}

	//	clock_gettime(CLOCK_REALTIME,&pasttime); //yzg

	if(period_cntl && !elmo_poweron &&(LastControlledTime>=2)) {
		Get_Sensor_Data_New();  //Elmo***********!!!********
	}
	//triger protect by encoder feedback

	//------- clear elmo  Encoders-------
	if(elmo_enc_clear && !elmo_poweron && !elmo_search_home) {
		Clear_Encoder_Elmo();
		elmo_enc_clear = FALSE;     
	}
	//-------clear elmo by px=0, when servo amplifier off
	if(elmo_px_eq0) {
		elmo_px_eq0 = FALSE;
		elmocmd = PXEQ0;  
		for(i=0; i< MAX_CAN_CHANNEL; i++)
			SendCmd( i, elmo_cmds[elmocmd].can_id, elmo_cmds[elmocmd].dat,elmo_cmds[elmocmd].len_cmd );
	}
	//------- search home point-------
	if(elmo_search_home && !elmo_poweron && !elmo_enc_clear) {

		start_search_home();
		period_cntl = FALSE;
	}


	//---------send sync, and get encoder feedback ------------
	if(LastControlledTime % 2 ==0)
		send_timestamp = TRUE;
	else
		send_timestamp = FALSE;

	if(period_cntl) {
		Elmo_Sync();
	}

	for(i=0; i<6; i++){
		if(i>=0 && i<=2)
			Iner_Sensor[i] = FootFT[3][i]*M_PI/180.0;
		if(i>=3 && i<6)
			Iner_Sensor_Current[i]= FootFT[3][i];
	}

#define ADIS_ANG_LIMIT 0.1 //5degrees ->0.09, 10 degrees ->0.17, 15 degrees->0.25
	for (i = 3; i < 6; i++) {
		if(Iner_Sensor[i] >= ADIS_ANG_LIMIT  ) // limited within 10 degrees
			Gsens[i] = ADIS_ANG_LIMIT; 
		if(Iner_Sensor[i] <= -ADIS_ANG_LIMIT )
			Gsens[i] = -ADIS_ANG_LIMIT;
	}

	//LPF Filter: y(k)=(1-ALFA)*u(k)+ALFA*y(k-1)
#define  ALFA_A  0.9703  //1Hz, T=4.8ms
	//#define  ALFA  0.9415  //2Hz, T=4.8ms
#define  ALFA_G  0.8603  //5Hz, T=4.8ms
	//#define  ALFA  0.7413  //10Hz, T=4.8ms

	Iner_Sensor[3]=(1-ALFA_A)*(Iner_Sensor_Current[3]-Iner_Sensor_Offset[3])+ALFA_A*Iner_Sensor[3];
	Iner_Sensor[4]=(1-ALFA_A)*(Iner_Sensor_Current[4]-Iner_Sensor_Offset[4])+ALFA_A*Iner_Sensor[4];
	Iner_Sensor[0]=(1-ALFA_G)*(Iner_Sensor_Current[0]-Iner_Sensor_Offset[0])+ALFA_G*Iner_Sensor[0];
	Iner_Sensor[1]=(1-ALFA_G)*(Iner_Sensor_Current[1]-Iner_Sensor_Offset[1])+ALFA_G*Iner_Sensor[1];
#if 1 //01-09-20
	Roll_Gyro_adis[0] = Iner_Sensor[0];
	Pitch_Gyro_adis[0] = -1.0*Iner_Sensor[1];
	Pitch_Angle_adis[0] =-1.0*asin(Iner_Sensor[3]);
	Roll_Angle_adis[0] = asin(-1.0*Iner_Sensor[4]);
#endif

	Get_Q_body();   //Elmo

	// Get_Plan_Data();

	//------------Set Ref= current Encoder
	if (( CanStart==0x66)&&(ENC_Only_Flag==0xaa))
	{	

		for( i=1;i<=LEG_NUM;i++){
			for(k=1;k<=JOINT_NUM;k++){
				Init_Val[i][k] = Joint[i][k];
				Ref_Val[i][k] = Init_Val[i][k];
			}
		}
		for( i=1;i<=ARM_NUM;i++){
			for(k=1;k<=JOINT_ARM_NUM;k++){       
				Init_Arm_Val[i][k] = Joint_Arm[i][k];
				Ref_Arm_Val[i][k] = Init_Arm_Val[i][k];
			}
		}
		for(k=1;k<=JOINT_Head_NUM;k++){      
			Init_Head_Val[k] = Joint_Head[k];
			Ref_Head_Val[k] = Init_Head_Val[k];
		}
		for(k=1;k<=JOINT_WAIST_NUM;k++){       //waist
			Init_Waist_Val[k] = Joint_Waist[1][k];   //---
			Ref_Waist_Val[k] = Init_Waist_Val[k];
		}

		if(Set_Ref_Equ_ENC_Delay++==10)
			ENC_Only_Flag=0x55;

	} // end of if ( CanStart==0x66----)
	//-------------Send Driver Center to DSP------//
         
    /************************** Walk-Craw-Fall Demo **************************/
	Get_Plan_Data(ID_File, (unsigned long *)&Actual_File_Data_Ptr);   
	
/************************** Walking Pattern Generation by Preview Control x zhouqq 2019/01/16 **********************************/
#ifdef USE_PREVIEW_CONTROL
	
	switch (PreCon_Mode)
	{
		case PRECON_MOVE_NONE:  
			break;
		
		case PRECON_MOVE_FORWARD:  // WALK	Forward
			printf("\n\n key = # PreCon # Move Forward \n");	
				
			Init_Walk_Tra(); 
			
			Walk_On =1;
			
			PreCon_Mode = PRECON_MOVE_NONE;			
			break;
			
		case PRECON_MOVE_BACK:  // WALK	Back
			printf("\n\n key = # PreCon # Move Backward \n");				
				
			Init_Walk_Tra(); 
			
			Walk_On =1;
			
			PreCon_Mode = PRECON_MOVE_NONE;			
			break;

		case PRECON_MOVE_RIGHT:  // Move Right
			printf("\n\n key = # PreCon # Move Right \n");	
				
			Init_Walk_Tra(); 
			
			Walk_On =1;
			
			PreCon_Mode = PRECON_MOVE_NONE;			
			break;
			
		case PRECON_MOVE_LEFT:  // Move Left
			printf("\n\n key = # PreCon # Move Left \n");	
				
			Init_Walk_Tra(); 
			
			Walk_On =1;
			
			PreCon_Mode = PRECON_MOVE_NONE;			
			break;
			
			
		default:// Other
			PreCon_Mode = PRECON_MOVE_NONE;
			break;					
	}
	
	Get_Mdf_Data(); // In order to get the measured data of force sensor 
	
	for( i=1;i<=LEG_NUM;i++){
		for(k=1;k<=JOINT_NUM;k++){
			Rtf_Val[i][k] = Ref_Val[i][k]/Gear_Rate_Joint[i][k-1];
			Nowq_Val[i][k] = 0; // reset body posture data 
			Mdf_Val[i][k] = 0.0;
			Mdf_Val_Pos[i][k] = 0.0;                        
		}				
	}	// Some compensation values from 'Sensory Reflex Control' (Method of Prof. Huang) in 'rtdata.c'. But it is not used now. [Comment: zhouqq 2019/01/20]
	
	if(Walk_On == TRUE && Mode_Flag == PRE_CON_MODE)
	{
		F_RFoot.fx = Force_Grnd[1][0];
		F_RFoot.fy = Force_Grnd[1][1];
		F_RFoot.fz = Force_Grnd[1][2];
		F_RFoot.tx = Force_Grnd[1][3];
		F_RFoot.ty = Force_Grnd[1][4];
		F_RFoot.tz = Force_Grnd[1][5];
		//printf("%lf\n", Force_Grnd[1][2]);
		
		F_LFoot.fx = Force_Grnd[2][0];
		F_LFoot.fy = Force_Grnd[2][1];
		F_LFoot.fz = Force_Grnd[2][2];
		F_LFoot.tx = Force_Grnd[2][3];
		F_LFoot.ty = Force_Grnd[2][4];
		F_LFoot.tz = Force_Grnd[2][5];
		
		for(i=1;i<7;i++)
		{
			Real_LegJoint.qr[i] = Joint[1][i]/Gear_Rate_Joint[1][i-1];
			Real_LegJoint.ql[i] = Joint[2][i]/Gear_Rate_Joint[2][i-1];
		}
		
		/* Generate Walking Pattern */
		PreviewControl_Tra_Generate(); // Output: Ref_Leg_Joint[3][7], Ref_Arm_Joint[3][7]
		
		/***********************************************************************************************************
		NOTE : There are so many variables that are reletive to the leg joints in this programme.
			   In order to make it clear, the followings are some explanations of leg joints. So do the arm joints.
		
		Generate joint angle [rad] ---\  then multiple gear rate  ---\  then assign to 'con_val' ...
		Ref_Leg_Joint[3][7]        ---/  Ref_Val[3][7]            ---/  con_val[3][7]
		
		...then limit joint angle  ---\  then send it to display  ---\  then assign to 'Ref_Leg_Last'
		   con_val[3][7]           ---/  Con_Disp_Leg[3][7]       ---/  Ref_Leg_Last[3][7]
		************************************************************************************************************/
		
		for (i=1;i<3;i++)
		{
			for(j=1;j<7;j++)
			{
				Ref_Val[i][j] = Ref_Leg_Joint[i][j]*Gear_Rate_Joint[i][j-1]; // or use 'Joint_Info[i][j].Gear_Rate'
				
				Ref_Arm_Val[i][j] = Ref_Arm_Joint[i][j]*Gear_Arm_Rate_Joint[i][j-1]; 
				
				
			}	
		} 		
	}
 
#endif // end : USE_PREVIEW_CONTROL
	Detect_Falling4Sensor();
/********************************* Walking Pattern Generation by Preview Control s ******************************************/

	/* Tuigan Protect */
	if(fabs(Joint_Waist[1][2]/Gear_Waist_Rate_Joint[1]-Joint_Waist[1][3]/Gear_Waist_Rate_Joint[2])>15.0)
	{
		Ref_Waist_Val[2] = Joint_Waist[1][2];
		Ref_Waist_Val[3] = Joint_Waist[1][3];
	}
	
	/*** limit of joint angle and call pid***/
	for( i=1;i<=LEG_NUM;i++){
		for(k=1;k<=JOINT_NUM;k++){
			
			con_val[i][k] =  Ref_Val[i][k];
			
			#ifdef JOINT_OFFSET
				con_val[i][k] = con_val[i][k] + Joint_Offset[i-1][k-1]*Gear_Rate_Joint[i][k-1];
			#endif
			
			conv_val[i][k] = Refv_Val[i][k];

			if (i==1){
				if (con_val[i][k]/(Gear_Rate_Joint[i][k-1]) >= LIMIT_RLEG_MAX[k-1]){
					con_val[i][k] = LIMIT_RLEG_MAX[k-1]*Gear_Rate_Joint[i][k-1];
				}
				if (con_val[i][k]/(Gear_Rate_Joint[i][k-1]) <= LIMIT_RLEG_MIN[k-1]){
					con_val[i][k] = LIMIT_RLEG_MIN[k-1]*Gear_Rate_Joint[i][k-1];
				}

			}

			if (i==2){
				if (con_val[i][k]/(Gear_Rate_Joint[i][k-1]) >= LIMIT_LLEG_MAX[k-1]){
					con_val[i][k] = LIMIT_LLEG_MAX[k-1]*Gear_Rate_Joint[i][k-1];
				}
				if (con_val[i][k]/(Gear_Rate_Joint[i][k-1]) <= LIMIT_LLEG_MIN[k-1]){
					con_val[i][k] = LIMIT_LLEG_MIN[k-1]*Gear_Rate_Joint[i][k-1];
				}

			}

			conv_val[i][k] = Refv_Val[i][k];
			
			Con_Val_Leg[i][k]=Sign_Con_Leg[i][k]*con_val[i][k];//--yzg
	
			if(LastControlledTime<=1){ // --for display
				Con_Temp_Leg[i][k]=0.0;
				Con_Disp_Leg[i][k]=Sign_Con_Leg[i][k]*Con_Val_Leg[i][k];
			}
			else{
				Con_Disp_Leg[i][k]=Con_Temp_Leg[i][k]; 
				Con_Disp_Leg[i][k]=Sign_Con_Leg[i][k]*Con_Val_Leg[i][k];
			} //end else			
		}		
	}
	
	//Limit the control data range of the Arm joints
	for( i=1;i<=ARM_NUM;i++){
		for(k=1;k<=JOINT_ARM_NUM;k++){
			if (i==1){
				if (Ref_Arm_Val[i][k]/(Gear_Arm_Rate_Joint[i][k-1]) >= LIMIT_RARM_MAX[k-1]){
					Ref_Arm_Val[i][k] = LIMIT_RARM_MAX[k-1]*Gear_Arm_Rate_Joint[i][k-1];
				}
				if (Ref_Arm_Val[i][k]/(Gear_Arm_Rate_Joint[i][k-1]) <= LIMIT_RARM_MIN[k-1]){
					Ref_Arm_Val[i][k] = LIMIT_RARM_MIN[k-1]*Gear_Arm_Rate_Joint[i][k-1];
				}                                      

			}

			if (i==2){
				if (Ref_Arm_Val[i][k]/(Gear_Arm_Rate_Joint[i][k-1]) >= LIMIT_LARM_MAX[k-1]){
					Ref_Arm_Val[i][k] = LIMIT_LARM_MAX[k-1]*Gear_Arm_Rate_Joint[i][k-1];
				}
				if (Ref_Arm_Val[i][k]/(Gear_Arm_Rate_Joint[i][k-1]) <= LIMIT_LARM_MIN[k-1]){
					Ref_Arm_Val[i][k] = LIMIT_LARM_MIN[k-1]*Gear_Arm_Rate_Joint[i][k-1];
				}                                        
			} 

			Con_Val_Arm[i][k]=Sign_Con_Arm[i][k]*Ref_Arm_Val[i][k]; 

			if(LastControlledTime<=1){
				Con_Temp_Arm[i][k]=0.0;
				Con_Disp_Arm[i][k]=Sign_Con_Arm[i][k]*Con_Val_Arm[i][k];
			} else{
				Con_Disp_Arm[i][k]=Con_Temp_Arm[i][k]; 
				Con_Disp_Arm[i][k]=Sign_Con_Arm[i][k]*Con_Val_Arm[i][k];
			} //end else
		}
	}
	
	//Limit the control data range of the Waist joints
	for(k=1;k<=JOINT_WAIST_NUM;k++){
		if (Ref_Waist_Val[k]/(Gear_Waist_Rate_Joint[k-1]) >= LIMIT_WAIST_MAX[k-1]){
			Ref_Waist_Val[k] = LIMIT_WAIST_MAX[k-1]*Gear_Waist_Rate_Joint[k-1];
		}
		if (Ref_Waist_Val[k]/(Gear_Waist_Rate_Joint[k-1]) <= LIMIT_WAIST_MIN[k-1]){
			Ref_Waist_Val[k] = LIMIT_WAIST_MIN[k-1]*Gear_Waist_Rate_Joint[k-1];
		}                                      

		Con_Val_Waist[1][k]=Sign_Con_Waist[k]*Ref_Waist_Val[k]; //---yzg
		if(LastControlledTime<=1) {
			Con_Temp_Waist[k]=0.0;
			Con_Disp_Waist[k]=Sign_Con_Waist[k]*Con_Val_Waist[1][k];
		}
		else{
			Con_Disp_Waist[k]=Con_Temp_Waist[k]; 
			Con_Disp_Waist[k]=Sign_Con_Waist[k]*Con_Val_Waist[1][k];
		} //end else
	}
	
	for(k=1;k<=JOINT_Head_NUM;k++){
		if (Ref_Head_Val[k]/(Gear_Head_Rate_Joint[k-1]) >= LIMIT_HEAD_MAX[k-1]){
			Ref_Head_Val[k] = LIMIT_HEAD_MAX[k-1]*Gear_Head_Rate_Joint[k-1];
		}
		if (Ref_Head_Val[k]/(Gear_Head_Rate_Joint[k-1]) <= LIMIT_HEAD_MIN[k-1]){
			Ref_Head_Val[k] = LIMIT_HEAD_MIN[k-1]*Gear_Head_Rate_Joint[k-1];
		}
		Con_Val_Head[k]=Sign_Con_Head[k]*Ref_Head_Val[k];  //---yzg              
		if(LastControlledTime<=1) //-- for display
		{
			Con_Temp_Head[k]=0.0;
			Con_Disp_Head[k]=Sign_Con_Head[k]*Con_Val_Head[k];
		}
		else{
			Con_Disp_Head[k]=Con_Temp_Head[k]; 
			Con_Disp_Head[k]=Sign_Con_Head[k]*Con_Val_Head[k];
		} //end else
	}

	/*******************************************************************/           
	/* Send  response data to display and save */
	for(i=0;i<2;i++){	
		for(j=0;j<6;j++){  
			Ref_Leg_Last[i+1][j+1]=Con_Disp_Leg[i+1][j+1]; 
			Ref_Leg_Last_2[i+1][j+1]=Ref_Leg_Last[i+1][j+1];
		}
	}
	
	if(Arm_Poweroff_Right2 == 0xaa){
		Arm_Poweroff_Right2_ptr++;
		if(Arm_Poweroff_Right2_ptr==4){
			Arm_Poweroff_Right2 = 0x55;
			Arm_Poweroff_Right2_ptr = 0;
			elmocmd = MOEQ0;
			SendCmd( 2, elmo_cmds[elmocmd].can_id+2, elmo_cmds[elmocmd].dat,elmo_cmds[elmocmd].len_cmd ); //shutdown 1th joint
		}
	
	}
	
	if(Arm_Poweroff_Right7 == 0xaa){
		Arm_Poweroff_Right7_ptr++;
		if(Arm_Poweroff_Right7_ptr==4){
			Arm_Poweroff_Right7 = 0x55;
			Arm_Poweroff_Right7_ptr = 0;
			Arm_Poweroff_Right2 = 0xaa;
			elmocmd = MOEQ0;
			SendCmd( 2, elmo_cmds[elmocmd].can_id+1, elmo_cmds[elmocmd].dat,elmo_cmds[elmocmd].len_cmd ); //shutdown 1th joint
		}
	
	}
	
	if(Arm_Poweroff_Right6 == 0xaa){
		Arm_Poweroff_Right6_ptr++;
		if(Arm_Poweroff_Right6_ptr==4){
			Arm_Poweroff_Right6 = 0x55;
			Arm_Poweroff_Right6_ptr = 0;
			Arm_Poweroff_Right7 = 0xaa;
			elmocmd = MOEQ0;
			SendCmd( 2, elmo_cmds[elmocmd].can_id+6, elmo_cmds[elmocmd].dat,elmo_cmds[elmocmd].len_cmd ); //shutdown 1th joint
		}
	
	}
	
	if(Arm_Poweroff_Right5 == 0xaa){
		Arm_Poweroff_Right5_ptr++;
		if(Arm_Poweroff_Right5_ptr==4){
			Arm_Poweroff_Right5 = 0x55;
			Arm_Poweroff_Right5_ptr = 0;
			Arm_Poweroff_Right6 = 0xaa;
			elmocmd = MOEQ0;
			SendCmd( 2, elmo_cmds[elmocmd].can_id+5, elmo_cmds[elmocmd].dat,elmo_cmds[elmocmd].len_cmd ); //shutdown 1th joint
		}
	
	}
	
	
	if(Arm_Poweroff_Right4==0xaa){
		Arm_Poweroff_Right4 = 0x55;
		Arm_Poweroff_Right5 = 0xaa;
		elmocmd = MOEQ0;
		SendCmd( 2, elmo_cmds[elmocmd].can_id+4, elmo_cmds[elmocmd].dat,elmo_cmds[elmocmd].len_cmd ); //shutdown 4th joint	
	} 
	
}

/* Send  response data to display and save */	
void Update_Response(void)
{
	int i=0, k=0;
	double *buf =NULL;	
	buf = pMsg_Data->Buffer;
	
	int tp_k = 0;
	if(K_Preview_Con==0) tp_k = 0; 
	else tp_k = K_Preview_Con-1;

	//k=0;
	//buf[k++] = LastControlledTime*CONTROL_PERIOD;         

	k=1; 
	buf[k++] = 3;                      //[ ] 个数（需要显示的数据）
	buf[k++] = k;                      //[0] MSG_RT 开始位置
	buf[k++] = time_2_now;             //[1] 程序当前时间
	buf[k++] = time_1_period;          //[2] 已占用时间
	buf[k++] = timegap_1_period;       //[3] 单周期实际运行时间
	
	k=6;
	buf[k++] = 4;                      //[ ] 个数（需要显示的数据）
	buf[k++] = k;                      //[0] MSG_CAN 开始位置
	buf[k++] = CANframeCount; 	       //[1] CAN 总帧数
	buf[k++] = FirstCANError;	       //[2] CAN 错误帧数
	buf[k++] = CANReceiptLoss; 	       //[3] CAN 丢失帧数
	buf[k++] = CanStart;   		       //[4] CAN 打开标志
	
	k=12;
	buf[k++] = 2;                      //[ ] 个数（需要显示的数据）
	buf[k++] = k;                      //[0] MSG_ONOFF 开始位置
	buf[k++] = Power_On_Flag; 	       //[1] 上电标志
	buf[k++] = Offset_On; 		       //[2] 清零标志
 	
	k=16;
	buf[k++] = 6;                      //[ ] 个数（需要显示的数据）
	buf[k++] = k;         		       //[0] MSG_WAIST_JOINT 开始位置
	for (i = 0; i < 3; i++, k++) buf[k] = Joint_Waist[1][i + 1] / Gear_Waist_Rate_Joint[i];  //[1-3] 腰关节码盘返回值：1-yaw；2-左推杆；3-右推杆
	for (i = 0; i < 3; i++, k++) buf[k] = Con_Disp_Waist[i + 1] / Gear_Waist_Rate_Joint[i];  //[4-6] 腰关节发送位置指令：4-yaw；5-左推杆；6-右推杆
	
	k=24;
	buf[k++] = 12;                      //[ ] 个数（需要显示的数据）
	buf[k++] = k;         		       //[0] MSG_RARM_JOINT 开始位置
	for(i = 0 ; i < 6 ; i++,k++) buf[k] = Joint_Arm[1][i+1]/Gear_Arm_Rate_Joint[1][i];       //[1-6]  右臂各关节码盘返回值：
	for(i = 0 ; i < 6 ; i++,k++) buf[k] = Con_Disp_Arm[1][i+1]/Gear_Arm_Rate_Joint[1][i];	 //[7-12] 右臂各关节发送位置指令：

	k=38;
	buf[k++] = 12;                      //[ ] 个数（需要显示的数据）
	buf[k++] = k;         		       //[0] MSG_LARM_JOINT 开始位置
	for(i = 0 ; i < 6 ; i++,k++) buf[k] = Joint_Arm[2][i+1]/Gear_Arm_Rate_Joint[2][i]; 		 //[1-6]  左臂各关节码盘返回值：
	for(i = 0 ; i < 6 ; i++,k++) buf[k] = Con_Disp_Arm[2][i+1]/Gear_Arm_Rate_Joint[2][i]; 	 //[7-12] 左臂各关节发送位置指令：

	k=52;
	buf[k++] = 12;                      //[ ] 个数（需要显示的数据）
	buf[k++] = k;         		       //[0] MSG_RLEG_JOINT 开始位置
	for(i = 0 ; i < 6 ; i++,k++) buf[k] = Joint[1][i+1]/Gear_Rate_Joint[1][i];                //[1-6]  右腿各关节码盘返回值： 	
	for(i = 0 ; i < 6 ; i++,k++) buf[k] = Ref_Leg_Last[1][i+1]/Gear_Rate_Joint[1][i];         //[7-12] 右腿各关节发送位置指令： 

	k=66;
	buf[k++] = 12;                      //[ ] 个数（需要显示的数据）
	buf[k++] = k;         		       //[0] MSG_LLEG_JOINT 开始位置
	for(i = 0 ; i < 6 ; i++,k++) buf[k] = Joint[2][i+1]/Gear_Rate_Joint[2][i];                //[1-6]  左腿各关节码盘返回值：
	for(i = 0 ; i < 6 ; i++,k++) buf[k] = Ref_Leg_Last[2][i+1]/Gear_Rate_Joint[2][i];         //[7-12] 左腿各关节发送位置指令：
	
	k=80;
	buf[k++] = 12;                      //[ ] 个数（需要显示的数据）
	buf[k++] = k;         		       //[0] MSG_FORCE_SENSOR 开始位置
	for(i = 0 ; i < 6 ; i++,k++) buf[k] = FootFT[1][i];	                                      //[1-6]  右脚力传感器测量信息，先力后力矩
	for(i = 0 ; i < 6 ; i++,k++) buf[k] = FootFT[2][i];	                                      //[7-12] 左脚力传感器测量信息，先力后力矩
	
	k=94;
	buf[k++] = 8;                      //[ ] 个数（需要显示的数据）
	buf[k++] = k;         		       //[0] MSG_ZMP 开始位置
	for(i = 0 ; i < 2 ; i++,k++) buf[k] = IZMP_Actl[i+1];	                                   //[1-2] 理想ZMP位置                
	for(i = 0 ; i < 2 ; i++,k++) buf[k] = ZMP_Actl[0][i+1];                                    //[3-4] 实际ZMP位置
	for(i = 0 ; i < 2 ; i++,k++) buf[k] = ZMP_Actl[1][i+1]-Actl_Ankle_W[1][i];                 //[5-6] 右脚ZMP位置
	for(i = 0 ; i < 2 ; i++,k++) buf[k] = ZMP_Actl[2][i+1]-Actl_Ankle_W[2][i];                 //[7-8] 左脚ZMP位置	
	
	k=104;
	buf[k++] = 5;                      //[ ] 个数（需要显示的数据）
	buf[k++] = k;         		       //[0] MSG_IMU 开始位置
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = GQ_Pitch; //0.0;//	                                           //[1] Measured Pitch    
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = GQ_Roll;  //0.0;//	                                           //[2] Measured Roll   
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = GQ_AccX;  //0.0;//	                                           //[3] Measured Acceleration X   
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = GQ_AccY;  //0.0;//	                                           //[4] Measured Acceleration Y   
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = GQ_AccZ;  //0.0;//	                                           //[5] Measured Acceleration Z   
		
	k=111;
	buf[k++] = 12;                     //[ ] 个数（需要显示的数据）
	buf[k++] = k;         		       //[0] MSG_ADJUST_FORCE_SENSOR 开始位置
	for(i = 0 ; i < 6 ; i++,k++) buf[k] = Force_Grnd[1][i];	                                       //[1-6]  右脚力传感器调整后信息，先力后力矩
	for(i = 0 ; i < 6 ; i++,k++) buf[k] = Force_Grnd[2][i];	                                       //[7-12] 左脚力传感器调整后信息，先力后力矩

	k=125;
	buf[k++] = 4;                     //[ ] 个数（需要显示的数据）
	buf[k++] = k;         		      //[0] MSG_MODE 开始位置
	for(i = 0; i < 1 ; i++, k++) buf[k] = Mode_Flag;                                           //[1] 执行的运动模式
	for(i = 0; i < 1 ; i++, k++) buf[k] = ID_File;                                             //[2] 离线数据文件号
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = Fall_Start_flag;                                     //[3] 摔倒检测开关标志
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = Falling_Alarm;                                       //[4] 摔倒检测触发警告

	k=131;
	buf[k++] = 3;                     //[ ] 个数（需要显示的数据）
	buf[k++] = k;         		      //[0] MSG_PRECON_MODE 开始位置	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = PreCon_ModeDisp;	                                   //[1] 当前预观模式下行走的方向   
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = Walk_On;	                                           //[2] 开始执行标志位   
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = Signal_NowStep[tp_k];	                               //[3] 当前是第几步(1~Nstep)   
	
	k=136;
	buf[k++] = 3;                     //[ ] 个数（需要显示的数据）
	buf[k++] = k;         		      //[0] MSG_PRECON_PARAMETERS 开始位置	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = L_step;	                                           //[1] 步长   
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = T_step;	                                           //[2] 步行周期   
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = N_step;	                                           //[3] 步数   
	
	k=141;
	buf[k++] = 10;                     //[ ] 个数（需要显示的数据）
	buf[k++] = k;         		       //[0] MSG_TPC 开始位置	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRFoot_RAnkle.px;	                               //[1] 右腿测量ZMP在踝关节坐标系下的坐标 X   
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRFoot_RAnkle.py;	                               //[2] 右腿测量ZMP在踝关节坐标系下的坐标 Y   

	for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPLFoot_LAnkle.px;	                               //[3] 左腿测量ZMP在踝关节坐标系下的坐标 X   
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPLFoot_LAnkle.py;	                               //[4] 左腿测量ZMP在踝关节坐标系下的坐标 Y   

	for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRel_B.px;	                               //[5] 双腿测量ZMP在质心坐标系下的坐标 X   
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRel_B.py;	                               //[6] 双腿测量ZMP在质心坐标系下的坐标 Y   

	for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRef_B.px;	                               //[7] 双腿规划ZMP在质心坐标系下的坐标 X   
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRef_B.py;	                               //[8] 双腿规划ZMP在质心坐标系下的坐标 Y   

	for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_DetaCOM.px;	                                       //[9]  TPC质心补偿量 X   
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_DetaCOM.py;	                                       //[10] TPC质心补偿量 Y  

	k=153;
	buf[k++] = 10;                     //[ ] 个数（需要显示的数据）
	buf[k++] = k;         		       //[0] MSG_CURRENT 开始位置	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = Leg_Torque[1][2];	                               //[1]  右腿2關節電流為額定電流的百分比   
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = Leg_Torque[1][3];	                               //[2]  右腿3關節電流為額定電流的百分比   
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = Leg_Torque[1][4];	                               //[3]  右腿4關節電流為額定電流的百分比   
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = Leg_Torque[1][5];	                               //[4]  右腿5關節電流為額定電流的百分比   
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = Leg_Torque[1][6];	                               //[5]  右腿6關節電流為額定電流的百分比   
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = Leg_Torque[2][2];	                               //[6]  左腿2關節電流為額定電流的百分比   
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = Leg_Torque[2][3];	                               //[7]  左腿3關節電流為額定電流的百分比   
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = Leg_Torque[2][4];	                               //[8]  左腿4關節電流為額定電流的百分比   
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = Leg_Torque[2][5];	                               //[9]  左腿5關節電流為額定電流的百分比   
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = Leg_Torque[2][6];	                               //[10] 左腿6關節電流為額定電流的百分比   

	k = 165;
	buf[k++] = 8;
	buf[k++] = k;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = XS_Pitch;	 
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = XS_Roll;	 
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = XS_AccX;	 
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = XS_AccY;	 
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = XS_AccZ;	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = XS_GyrX;	 
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = XS_GyrY;	 
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = XS_GyrZ;	
	
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRFoot_RAnkle.px;	                               //[1] 右腿测量ZMP在踝关节坐标系下的坐标 X   
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRFoot_RAnkle.py;	                               //[2] 右腿测量ZMP在踝关节坐标系下的坐标 Y   
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRFoot_RAnkle.pz;	                               //[3] 右腿测量ZMP在踝关节坐标系下的坐标 Z  

	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPLFoot_LAnkle.px;	                               //[4] 左腿测量ZMP在踝关节坐标系下的坐标 X   
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPLFoot_LAnkle.py;	                               //[5] 左腿测量ZMP在踝关节坐标系下的坐标 Y   
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPLFoot_LAnkle.pz;	                               //[6] 左腿测量ZMP在踝关节坐标系下的坐标 Z 

	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRFoot_B.px;	                                    //[7] 右腿测量ZMP在质心坐标系下的坐标 X   
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRFoot_B.py;	                               //[8] 右腿测量ZMP在质心坐标系下的坐标 Y   
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRFoot_B.pz;	                               //[9] 右腿测量ZMP在质心坐标系下的坐标 Z  

	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPLFoot_B.px;	                               //[10] 左腿测量ZMP在质心坐标系下的坐标 X   
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPLFoot_B.py;	                               //[11] 左腿测量ZMP在质心坐标系下的坐标 Y   
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPLFoot_B.pz;	                               //[12] 左腿测量ZMP在质心坐标系下的坐标 Z 

	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRel_B.px;	                               //[13] 双腿测量ZMP在质心坐标系下的坐标 X   
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRel_B.py;	                               //[14] 双腿测量ZMP在质心坐标系下的坐标 Y   
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRel_B.pz;	                               //[15] 双腿测量ZMP在质心坐标系下的坐标 Z 

	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRef_B.px;	                               //[16] 双腿规划ZMP在质心坐标系下的坐标 X   
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRef_B.py;	                               //[17] 双腿规划ZMP在质心坐标系下的坐标 Y   
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRef_B.pz;	                               //[18] 双腿规划ZMP在质心坐标系下的坐标 Z 

	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_DetaCOM.px;	                                       //[19] TPC质心补偿量 X   
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_DetaCOM.py;	                                       //[20] TPC质心补偿量 Y   
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_DetaCOM.pz;	                                       //[21] TPC质心补偿量 Z 

}


void Detect_Falling4Sensor(void)
{
#define REAR_ZMP_BORDER 0.09
#define FRONT_ZMP_BORDER 0.12
#define FALLing_BODY_ANGLE 3.0 
#ifdef FALL_DETECT
	int i=0;
	double Pitch, Roll;
	
	// Pitch  = GQ_Pitch;
	// Roll = GQ_Roll;
	Falling_Alarm = 0;
	// if (
		// (Fall_Start_flag==TRUE) && 
		// (
		   // ( (ZMP_Actl[1][2]-Actl_Ankle_W[1][1]) > FRONT_ZMP_BORDER && (ZMP_Actl[2][2]-Actl_Ankle_W[2][1]) > 0.06/*FRONT_ZMP_BORDER*/ && fabs(Pitch) > FALLing_BODY_ANGLE ) 
		   // ||(fabs(Pitch) > 5.0*FALLing_BODY_ANGLE)
		 // )
	   // )	/* !!! MPU OK and Pitch detect */	
	if (
		(Fall_Start_flag==TRUE) && 
		(
		   ( (ZMP_Actl[1][2]-Actl_Ankle_W[1][1]) > FRONT_ZMP_BORDER && (ZMP_Actl[2][2]-Actl_Ankle_W[2][1]) > 0.06/*FRONT_ZMP_BORDER*/ ) 
		   ||(fabs(Pitch) > 5.0*FALLing_BODY_ANGLE)
		 )
	   )/* !!! MPU Wrong and No Pitch detect */		   
	{
		Falling_Alarm = 11;
		
		Fall_Start_flag = FALSE;
			
		Fall_flag = TRUE;
		ID_File = Demo_Get_ID_File(Mode_Flag, ID_File);
		if(ID_File <= Actual_File_Num)
		{
			Walk_On = TRUE;
			if(File_Read_End==TRUE)
			{
				Actual_File_Data_Ptr = iFileStartPoint[ID_File];
			}
		}	
		else
		{
			Walk_On = FALSE;
		}	
	}
	
	if (
		(
		   ( (ZMP_Actl[1][2]-Actl_Ankle_W[1][1]) > FRONT_ZMP_BORDER && (ZMP_Actl[2][2]-Actl_Ankle_W[2][1]) > FRONT_ZMP_BORDER && fabs(Pitch) > FALLing_BODY_ANGLE ) 
		   ||(fabs(Pitch) > 5.0*FALLing_BODY_ANGLE)
		 )
	   )				
	{
		Falling_Alarm = 11;					
	}
		
#endif

}

/*************************************************************************
*    Communcation with 14 DSP2407 joint controls(JCs) in dedicated protocol 
* of humanoid robot.Each JC controls two joints.Rountines for PCI-1680 
* produced by Advantech corp. Linux kernel is 2.4.20.
* (1)in standard Rata Frame or Remote Frame,i.e. 11 bits identifier
* (2)About the received ID : Bit 10 (MSB) must be 1.   About the send ID:
*     Bit 10(MSB) must be 0.
* (3)Baud rate: 1Mbps.
**************************************************************************/


/*************************************************************************
*Function:  Send standard CAN message, i.e. CAN 2.0A 
*if dspnode=0,then send Remote URGENT Realtime command,if not 0,then send Data Command
*candata[0]~candata[7]
**************************************************************************/

int SendCmd(int channel, unsigned int msgID , unsigned  char *candata, unsigned char bytelength)
{

#ifndef NON_HARDWARE

	int i;
	int ret_code=0;
	int wr_status = 0;
	LARGE_INTEGER       stime, etime;
	LONG                offset;

	/*  11 bits identifer of "CAN Card Sender"
	bit10  |bit9  bit8 bit7 bit6 bit5 |bit4  | bit3 bit2 bit1 bit0 
	mean:    1   |   command code           | 0/1  |DSP controlle(1-14)
	ID:       1 | x      x    x    x    x  | x    |DSP controlle(1-14) 
	*/
	//	CANframeCount++;
	canmsgfTx.rtr = 0;  // standard data frame
	canmsgfTx.dlen = bytelength;
	canmsgfTx.id=msgID;

	if(canmsgfTx.dlen){
		for(i=0;i<canmsgfTx.dlen;i++)
			canmsgfTx.data[i]=candata[i];
	}
	//As for IEI PICO8522 main industrial PC board,
	//the CAN 1th (not 0th) Channel is near the blackplane board.
	if(channel<0 || channel>=MAX_CAN_CHANNEL) //channel=0,1,...,5 
		return -1;
	else { 
		//else if(ChannelCfg[channel+1]) { //there is a node on this channel
#if 1 //@@ //for non-blocked can driver	

		if(!RtGetClockTime( CLOCK_FASTEST, &stime))  //0.100us units, time.QuadPart ->Signed 64-bit integer
		{
			printf("Error:  Could not get clock time.  GetLastError = %d\n", GetLastError());
			return -1;
		}
		do{
			wr_status = can_wrready(channel);

			RtGetClockTime( CLOCK_FASTEST, &etime);
			offset = (LONG) (etime.QuadPart-stime.QuadPart);
			if(offset>=1380) //210us
			{
				send_loss++;
				break;
			}
		}while(!wr_status);
#endif //@@	
		can_write_channel(channel,&canmsgfTx );
	} 
#endif //endof #ifndef NON_HARDWARE
	return 0;
}

/*************************************************************************
*Name:RecvCANData
*Function: Receive can data from DSP JCs
*if receive can data successfully,return 0xaa; return 0x55 (no data received)
**************************************************************************/
//int RecvCANData(int channel)
int RecvCANData(int channel)
{  
#ifndef NON_HARDWARE
	int ret;
	CANframeCount++;
	if(channel<0 ||channel>=MAX_CAN_CHANNEL)
		ret= -1;
	else {

		ret = can_read_channel(channel,&canmsgfRX);
		//ret = can_read_channel(3,&canmsgfRX);//mg
		if(ret !=0xaa)
		{ 
			#ifdef USE_XS_IMU300
			if(channel == (XS_CHANNEL-1))
			{
				if(xsLossNum++ > 10)
					CANReceiptLoss++;
			}
			else
			#endif
			CANReceiptLoss++; 
			ret = 0;
			//printf("chane=%d\n",channel);
		}
		#ifdef USE_XS_IMU300
		else if(channel == (XS_CHANNEL-1))
		{
			xsLossNum = 0;
		}
		#endif
	}
	return ret;
#endif // end of #ifndef NON_HARDWARE
#ifdef NON_HARDWARE
	return 0;
#endif
}
/***************Check configuration of DSP controller***********/
int CheckDSPControllerConfig2(void)
{ 
	int i;
	JCStates=0xffff;
	for(i=1;i<=15;i++)
	{
		if(CheckCANNode[i]==NCNT)
			JCStates&=~(1<<(i-1));
		else
			Elmo_Num ++; //number of Elmo connected to CAN bus
	} 
	JCStates&=~(1<<15);
	return 0;
}

static void Set_Driver_Ref(void)
{
	//	static long setref_cnt = 0;
	int i, j;
	Con_Data_Trans(); // Convert Driver Ref to  Con_Val_Snd_CAN[CH][ID]
	if(elmo_setref){ //period_cntl ,//&&&
		setref_cnt++;
		if(setref_cnt >= 100) setref_cnt = 100;

		for(j=1; j<=6; j++){ //external loop: Elmo ID //@@
			rtx_delayus(30);
			//rtl_usleep(150);
			// for(i=1; i<=MAX_CAN_CHANNEL; i++){ // internal loop:
			for(i=1; i<=6; i++){ // internal loop://20200710 dcc
				if( CanCfg[i][j] ){				
					if(setref_cnt==1)
						elmocmd = SET_IP_REF_MOTOROFF; // 
					if(setref_cnt>=2 && setref_cnt<=3)
						elmocmd = SET_IP_REF_SHUTDOWN; //
					if(setref_cnt>=4 && setref_cnt<=5){
						elmocmd = SET_IP_REF_READY; //
						Motor_On_Flag = TRUE;
					}	
					if(setref_cnt>=6 && setref_cnt<=7)
						elmocmd = SET_IP_REF_MOTORON; //
					if(setref_cnt>=8)
						elmocmd = SET_IP_REF_PEROIDIC;	

					if(!(_fpclass(Con_Val_Snd_CAN[i][j])&0x207)){ 
					
					Valid_Joint_Speed[3][1] = TRUE;
					Valid_Joint_Speed[3][2] = TRUE;
					Valid_Joint_Speed[3][4] = TRUE;
					
					Valid_Joint_Speed[4][1] = TRUE;
					Valid_Joint_Speed[4][2] = TRUE;
					Valid_Joint_Speed[4][4] = TRUE;					

						if(Valid_Joint_Speed[i][j]){
							Long2Byte( &elmo_cmds[elmocmd].dat[2], Con_Val_Snd_CAN[i][j]); //new motor should multiple electronic gear 
							//gain = PGEARO*2^PRBASE/PGEARI
							SendCmd(i-1, elmo_cmds[elmocmd].can_id+j,
								elmo_cmds[elmocmd].dat, elmo_cmds[elmocmd].len_cmd );
						}else{
							RtPrintf("\n Channel %d, Joint %d over speed!, suspending send data to this joint",i,j); 
						}	
					}else{
						RtPrintf("\n Not a Number occurs in File %s, Function %s, line %u, Channel %d,ID %d", \
							__FILE__, __FUNCDNAME__ ,__LINE__, i, j);
					}
					//					 if(i==1&&j==1)
					//					printf("\n%3d",Con_Val_Snd_CAN[i][j]);
				} //end of if(CanCfg[i][j])
			}
		} //@@	
	}//&&&

	for(i=1; i<= MAX_CAN_CHANNEL; i++){
		for(j=1; j<=7; j++){
			Con_Val_Snd_CAN_Last[i][j] = Con_Val_Snd_CAN[i][j];
		}
	}
}


void Get_Elmo_Encoder(void)
{
#ifndef NON_HARDWARE
#define MOTOR_RATE_CURRENT 6.6
	int i, j, k;
	int Vision_Flag=1;
	int Body_Flag=1;
static long iikk=0;	
/****************************
CANid for Force Sensor by Meng
*****************************/
#ifdef USE_IFS
	CanCfg[FORCE_SENSOR_CHANNEL][2]=TRUE;
	CanCfg[FORCE_SENSOR_CHANNEL][3]=TRUE;
#endif


/* 	CanCfg[6][1]=FALSE;//FALSE;//TRUE; //for WAIST ABSOLUTE sensors
	CanCfg[6][2]=FALSE;//FALSE;//TRUE;
	CanCfg[6][3]=FALSE;//FALSE;//TRUE;
	CanCfg[6][4]=FALSE;//FALSE;//TRUE;
	CanCfg[6][5]=FALSE;//FALSE;//TRUE;
	CanCfg[6][6]=FALSE;//FALSE;//TRUE; */
#ifdef USE_XS_IMU300
	CanCfg[XS_CHANNEL][1]=TRUE;//FALSE;//
	CanCfg[XS_CHANNEL][2]=TRUE;
	CanCfg[XS_CHANNEL][3]=TRUE;
	CanCfg[XS_CHANNEL][4]=TRUE;
	CanCfg[XS_CHANNEL][5]=FALSE;
	CanCfg[XS_CHANNEL][6]=FALSE;
	
#endif
	
#ifdef USE_GQ_IMU100
	#define GQ_CHANNEL 4
	CanCfg[GQ_CHANNEL][6]=FALSE;//FALSE;//
#endif
	
	for(j=1; j<7; j++){ //external loop: Elmo ID//mg j=7
	//	if(j == 7) break;
		//us_delay_foo(80); //match time requirements
		rtx_delayus(20); //150usec
		for(i=1; i<=MAX_CAN_CHANNEL; i++){ // CAN channel//mg
			if( CanCfg[i][j]){
				if(RecvCANData(i-1)==0xaa) {
					 for(k=1; k<7; k++){
						 if(canmsgfRX.id == (0x180+k))
						 {
							 if(i == 1 || i == 2 || i == 5 || i == 6)
							 {
								 enc_val[i][k] = Byte2Long(&canmsgfRX.data[0]); //encoder
								#if 1
								 //Read Current: -> 'Leg_Torque[][]' is the rate of the continue current
								 {
									 Leg_Torque[i][k]=Byte2ShortInt(&canmsgfRX.data[4])/1000.0;//*StallCurrent_Motor[i][j]/1.414*Kt_Motor[i][j]; //unit: Nm
									 iikk++;

								 }
							 }
								 
							#endif 

							/***************************************************************************************************** 
								Function: Add Other Equipment 
								Decrible: 1. i: Channel, k: ID
										  2. !!! Remeber to open the ID
								Example : 
									#ifdef USE_XXX
										CanCfg[channelX][IDX] = TRUE; // FALSE; // Open the ID, put this code befor this 'for'
									#endif
									
									#ifdef USE_XXX
										if((i==ChannelX)&&(k==IDX))
										{
											//printf("Can receive !\n"); // Test whether the equipment is added successfully.
											Get_Data_XXX(&canmsgfRX.data[0]);
										}
									#endif  
							******************************************************************************************************/
							#ifdef USE_GQ_IMU100
								if((i==GQ_CHANNEL)&&(k==6))
								{							
									GQ_IMU100_GetData(&canmsgfRX.data[0]);
								}
							#endif
							
							#ifdef USE_XS_IMU300
								
								if((i==XS_CHANNEL)&&(k==1))
								{			
									// printf("accx\t");
									// printf("accx\t");
									XS_AccX = (double)Byte2Float_(&canmsgfRX.data[0]);
									XS_GyrX = (double)Byte2Float_(&canmsgfRX.data[4]);
								}
								if((i==XS_CHANNEL)&&(k==2))
								{
									// printf("accy\t");
									XS_AccY = (double)Byte2Float_(&canmsgfRX.data[0]);
									XS_GyrY = (double)Byte2Float_(&canmsgfRX.data[4]);
								}
								if((i==XS_CHANNEL)&&(k==3))
								{
									// printf("accz\t");
									XS_AccZ = (double)Byte2Float_(&canmsgfRX.data[0]);
									XS_GyrZ = (double)Byte2Float_(&canmsgfRX.data[4]);
								}	
								if((i==XS_CHANNEL)&&(k==4))
								{
									// printf("roll\t");
									// XS_Roll = (double)Byte2Float_(&canmsgfRX.data[0]);
									// XS_Pitch = (double)Byte2Float_(&canmsgfRX.data[4]);
									XS_Pitch = (double)Byte2Float_(&canmsgfRX.data[0]) / 57.3;
									XS_Roll = (double)Byte2Float_(&canmsgfRX.data[4]) / 57.3;
								}
							#endif
						}
#ifdef USE_IFS
#if 1	
/****************************
Read Force Sensor data by Meng
*****************************/
						if(IFS_Init_Flag==FALSE)
						{
						IFS_Init();
						}
						if((i==FORCE_SENSOR_CHANNEL)&&(canmsgfRX.id == (0x180+1)))
						{
						
							Trans_IFS_Data_r( (unsigned char *)&canmsgfRX.data); // &canmsgfRX.data
							
							//enc_val[i][k]=0.0;
						}
						if((i==FORCE_SENSOR_CHANNEL)&&(canmsgfRX.id == (0x180+2)))
						{
						
							 Trans_IFS_Data_l( (unsigned char *)&canmsgfRX.data); //&canmsgfRX.data
							
						}
						
#endif
#endif
						if(canmsgfRX.id == (0x80+k))
							SendErrorCount++;   //
						if(canmsgfRX.id == (0x180+k)) break;
					}
				}
				//		break;
			}

			//	}while(rd_status!=0);

		}
	}
									// if((iii++%50==0)&&(k==4 ||k==3))
//	 printf("\ncurrent[1,3]=%f, [1,4]=%f",Leg_Torque[1][3],Leg_Torque[1][4]);
	// CanCfg[4][1]=FALSE;//for sensors
	// CanCfg[4][2]=FALSE;//
	// CanCfg[4][3]=FALSE;//
	// CanCfg[4][4]=FALSE;
	// CanCfg[4][5]=FALSE;
	// CanCfg[4][6]=FALSE;
#endif
// printf("\n");
}

/****************************************************************************
*Name:Byte2Long() 
*****************************************************************************/
int Byte2Long(unsigned char * const Byt) 
{ 
	int i;
	for(i=0; i<4; i++)
		lv2cv.cv[i] = *(Byt+i);
	return lv2cv.lv;
}
/****************************************************************************
*Name:Long2Byte() 
*****************************************************************************/
void Long2Byte(unsigned char * const Byt, long ldat) 
{
	int i;
	lv2cv.lv=ldat;
	for(i=0; i<4; i++)
		*(Byt+i) = lv2cv.cv[i];
} 
/****************************************************************************
*Name:Byte2float() 
*****************************************************************************/
float Byte2Float(unsigned char * const Byt)
{ 
	int i;
	for(i=0; i<4; i++)
		flt2cv.cv[i] = *(Byt+i);
	return flt2cv.flt;
}
float Byte2Float_(unsigned char * const Byt)
{ 
	int i;
	for(i=0; i<4; i++)
		flt2cv.cv[i] = *(Byt+3-i);
	return flt2cv.flt;
}
//cmd_ret_fvalue = Byte2Float(&canmsgfRX.data[4]);
/****************************************************************************
*Name:Float2Byte() 
*****************************************************************************/
void Float2Byte(unsigned char * const Byt, float fdat)
{ 
	int i;
	flt2cv.flt=fdat;
	for(i=0; i<4; i++)
		*(Byt+i) = flt2cv.cv[i];
}

/****************************************************************************
*Name:Byte2ShortInt() 
*****************************************************************************/
int Byte2ShortInt(unsigned char * const Byt) 
{ 
	short int i;
	for(i=0; i<2; i++)
		siv2cv.cv[i] = *(Byt+i);
	return siv2cv.siv;
}

void Get_Sensor_Data_New(void)
{
	Get_Elmo_Encoder();
	Get_Sensor_Data(); //--- have been modify with CAN
}


void copy_elmo_cmd(elmo_cmd *dst, elmo_cmd *src)
{   int i=0;
dst->cmd_name  = src->cmd_name;
dst->can_id    = src->can_id;
for(i=0;i<8;i++)
dst->dat[i]    = src->dat[i];
dst->len_cmd = src->len_cmd;
}

void  init_elmo_cmds(void)
{
	int i,j,k;
	for(i = 0; i<ELMO_CMD_LENGTH ; i++){

		for(k=0;k<ELMO_CMD_LENGTH;k++) {
			j= Elmo_DS301cmd[k].cmd_name;    

			if(i==j){
				copy_elmo_cmd(&elmo_cmds[i], &Elmo_DS301cmd[k]);
				break;
			}

		}
	}

}

void Clear_Encoder_Elmo(void)
{
	int i;	
	for(i=0; i<MAX_CAN_CHANNEL; i++){
		elmocmd = CLR_ENC_HM2EQ0;
		SendCmd( i, elmo_cmds[elmocmd].can_id, elmo_cmds[elmocmd].dat,elmo_cmds[elmocmd].len_cmd );
		rtx_delayus(150); //delay  100us

		elmocmd = CLR_ENC_HM3EQ0;
		SendCmd( i, elmo_cmds[elmocmd].can_id, elmo_cmds[elmocmd].dat,elmo_cmds[elmocmd].len_cmd );
		rtx_delayus(150); //delay  100us

		elmocmd = CLR_ENC_HM4EQ0;
		SendCmd( i, elmo_cmds[elmocmd].can_id, elmo_cmds[elmocmd].dat,elmo_cmds[elmocmd].len_cmd );
		rtx_delayus(150); //delay  100us

		elmocmd = CLR_ENC_HM5EQ0;
		SendCmd( i, elmo_cmds[elmocmd].can_id, elmo_cmds[elmocmd].dat,elmo_cmds[elmocmd].len_cmd );
		rtx_delayus(150); //delay  100us

		elmocmd = CLR_ENC_HM1EQ1;
		SendCmd( i, elmo_cmds[elmocmd].can_id, elmo_cmds[elmocmd].dat,elmo_cmds[elmocmd].len_cmd );
		rtx_delayus(150); //delay  100us
	}
}


static void Elmo_Sync_old(void)
{
	int i;
	unsigned char  dat[8];

	for(i=0; i<MAX_CAN_CHANNEL; i++){
		SendCmd(i, 0x80, dat, 0);  // broadcasting  
	}

}
static void Elmo_Sync(void)
{
	#ifndef NON_HARDWARE

	int i,j;
	int ret_code=0;
	int wr_status[6];
	int wr_finished[6];
	LARGE_INTEGER Duration;
	LARGE_INTEGER       stime, etime;
	LONG                offset;		
	int ch=0; //start from 0.
	int flag_exit=1; //finished transmission
	long delay_count=0;
	int channelconfigured[7]={0,0,0,0,0,0,0};

	for(j=1; j<=6; j++){  //Elmo ID //@@
		for(i=1; i<=MAX_CAN_CHANNEL; i++){ // internal loop:
			if( CanCfg[i][j] ){
				channelconfigured[i]=1; //if i-th connected with driver(s), =1
			}
		}	
	}
	//Duration.QuadPart = 1000L;  // 1000x0.1us=100us
	
	//RtSleepFt(&Duration);	//delay 1us
	Duration.QuadPart = 10L;  // 10x0.1us=1us
	/*  11 bits identifer of "CAN Card Sender"
	bit10  |bit9  bit8 bit7 bit6 bit5 |bit4  | bit3 bit2 bit1 bit0 
	mean:    1   |   command code           | 0/1  |DSP controlle(1-14)
	ID:       1 | x      x    x    x    x  | x    |DSP controlle(1-14) 
	*/
	//	CANframeCount++;
	canmsgfTx.rtr = 0;  // standard data frame
	canmsgfTx.dlen = 0;
	canmsgfTx.id=0x80;

	if(canmsgfTx.dlen){
		for(i=0;i<canmsgfTx.dlen;i++)
			canmsgfTx.data[i]=0;
	}
	for(i=0;i<MAX_CAN_CHANNEL;i++)
	{
		wr_status[i]=0;//initializated to false	
		wr_finished[i] = 0; 
	}
	//As for IEI PICO8522 main industrial PC board,
	//the CAN 1th (not 0th) Channel is near the blackplane board.

		//else if(ChannelCfg[channel+1]) { //there is a node on this channel


	if(!RtGetClockTime( CLOCK_FASTEST, &stime))  //0.100us units, time.QuadPart ->Signed 64-bit integer
	{
		printf("Error:  Could not get clock time.  GetLastError = %d\n", GetLastError());
		//return -1;
		return;
	}
	
	do{
		flag_exit =1;
		for(ch=0;ch<=MAX_CAN_CHANNEL-1;ch++)
		{
			if(wr_status[ch]==0)
			{
				wr_status[ch] = can_wrready(ch); //ready,return true;
				if(wr_status[ch]&& channelconfigured[ch+1]){
					can_write_channel(ch,&canmsgfTx ); //&canmsgfTx，需改为与通道对应的数组
					wr_finished[ch]=1; //writing finished
				}
			}
			flag_exit = flag_exit && wr_finished[ch];
			
		}
		//RtSleepFt(&Duration);	//delay 1us
		rtx_delayus(1);
		if(flag_exit) //每个通道完成了写入
			break;
			
		RtGetClockTime(CLOCK_FASTEST, &etime);
		offset = (LONG) (etime.QuadPart-stime.QuadPart);
		if(offset>=800) //60us
		{
			for(ch=0;ch<=MAX_CAN_CHANNEL-1;ch++)
			{
				if((wr_finished[ch]==0)&&(channelconfigured[ch+1]==1)) //未曾写入,强行写入
				{
					can_write_channel(ch,&canmsgfTx ); //&canmsgfTx，需改为与通道对应的数组	
					send_loss++;
				}
			}				
			break;
		}	
	
	}while(!flag_exit);
	
	return;
	#endif //endof #ifndef NON_HARDWARE
}

static void launch_elmo_home(int channel_num, int CAN_id ) //CAN_id =0, broadcast
{
	//Should set  HM2 according to specific data, before here 
	//send broadcasting command
	elmocmd = HOM_STEP1;
	SendCmd(channel_num, elmo_cmds[elmocmd].can_id+CAN_id, elmo_cmds[elmocmd].dat,elmo_cmds[elmocmd].len_cmd );
	elmocmd = HOM_STEP2;
	SendCmd(channel_num, elmo_cmds[elmocmd].can_id+CAN_id, elmo_cmds[elmocmd].dat,elmo_cmds[elmocmd].len_cmd );
	elmocmd = HOM_STEP3;
	SendCmd(channel_num, elmo_cmds[elmocmd].can_id+CAN_id, elmo_cmds[elmocmd].dat,elmo_cmds[elmocmd].len_cmd );
}

static void start_search_home(void)
{
	int i=0;
	elmo_search_home_delay++;
	if(elmo_search_home_delay == 2){
#if 0
	for(i=0; i<MAX_CAN_CHANNEL; i++){
			launch_elmo_home(i, 0); ////CAN_id =0, broadcast
		}
#endif

//		launch_elmo_home(0, 5);
//		launch_elmo_home(1, 0);// Channel 0 for right leg, and channel 1 for left leg. 
//		launch_elmo_home(0, 0);// Channel 0 for right leg, and channel 1 for left leg. 
//		launch_elmo_home(0, 0);
//		launch_elmo_home(0, 2);
		launch_elmo_home(0, 0);
//		printf("start searching home\n");
//		launch_elmo_home(1, 2);
//		launch_elmo_home(0, 4);
//		launch_elmo_home(0, 5);
//		launch_elmo_home(0, 6);
		launch_elmo_home(1, 0);
//		launch_elmo_home(3, 0);
//		launch_elmo_home(4, 0);
		//kollmorgan homing method

	}
	if(elmo_search_home_delay >= (int)(HOME_SEARCH_DELAY))
		elmo_search_home = FALSE;
	;
}

void SoftReset_via_CANopen(void)
{
	int i;	
	unsigned int msgID = 0x0; //Network Management
	char dat[8];
	dat[0] = 0x81; //Reset node, perform full software reset
	dat[1] = 0x0; //All nodes
	for(i=0; i<MAX_CAN_CHANNEL; i++){
		SendCmd(i, msgID, dat, 2);
		rtx_delayus(150); //delay  100us
	}
}

static void elmo_send_home_offset(void)
{
	int i, j;
	LARGE_INTEGER Duration;
	Duration.QuadPart = 4000L;  // 30ms
	Elmo_Home_Compen_Trans(); 
	for(j=1; j<=7; j++){ //external loop: Elmo ID //@@
		RtSleepFt(&Duration); //should be modified to match time requirements, 100 or 200us
		for(i=1; i<=MAX_CAN_CHANNEL; i++){ // internal loop: CAN channel
			if( CanCfg[i][j] ){
				elmocmd = HM2OFFSET; 
				Long2Byte( &elmo_cmds[elmocmd].dat[4],Elmo_Home_Compen[i][j]);
				SendCmd(i-1, elmo_cmds[elmocmd].can_id+j,
					elmo_cmds[elmocmd].dat, elmo_cmds[elmocmd].len_cmd );
				RtSleepFt(&Duration); // delay 400us
				elmocmd = QV01ASTIMEOUT; 
				Long2Byte( &elmo_cmds[elmocmd].dat[4], Elmo_Home_Timeout[i][j]);
				SendCmd(i-1, elmo_cmds[elmocmd].can_id+j,
					elmo_cmds[elmocmd].dat, elmo_cmds[elmocmd].len_cmd );
				RtSleepFt(&Duration); // delay 400us
				elmocmd = QV02ASVMOVE; 
				Long2Byte( &elmo_cmds[elmocmd].dat[4], Elmo_Home_Vmove[i][j]);
				SendCmd(i-1, elmo_cmds[elmocmd].can_id+j,
					elmo_cmds[elmocmd].dat, elmo_cmds[elmocmd].len_cmd );
				RtSleepFt(&Duration); // delay 400us
				elmocmd = QV03ASVSEARCH; 
				Long2Byte( &elmo_cmds[elmocmd].dat[4], Elmo_Home_Vsearch[i][j]);
				SendCmd(i-1, elmo_cmds[elmocmd].can_id+j,
					elmo_cmds[elmocmd].dat, elmo_cmds[elmocmd].len_cmd );
				RtSleepFt(&Duration); // delay 400us
				elmocmd = QV04ASMOVEBIAS; 
				Long2Byte( &elmo_cmds[elmocmd].dat[4], Elmo_Home_Bias[i][j]);
				SendCmd(i-1, elmo_cmds[elmocmd].can_id+j,
					elmo_cmds[elmocmd].dat, elmo_cmds[elmocmd].len_cmd );
			}
		}
	} //@@
}
  			 		


void Reload_Encoder(void)
{
	int i,j;
	unsigned char datm[8]; 	
	datm[0]=0x50;
	datm[1]=0x58;
	datm[2]=0;
	datm[3]=0;
	for(j=1; j<=6; j++){ //external loop: Elmo ID //@@
		rtx_delayus(200);
		//rtl_usleep(150);
		for(i=1; i<=MAX_CAN_CHANNEL; i++){ // internal loop:
			if( CanCfg[i][j] ){				
				Long2Byte( &datm[4], enc_val[i][j]); 
				SendCmd(i-1, 0x300+j,datm, 8);
			} 
		} //for i
	} //for j	

}

char LogName[MAX_RESP_MSGSTR_SIZE][30] = { 0 };
/* Save response data*/	

void SetLogName_default(int cnt, int k, char* logname, char mode[], int i)
{
	if (cnt != 1) return;
	strcat(LogName[k], logname);
	if (!strcmp(mode, "num"))
	{
		char stemp[2] = "1"; stemp[0] = (char)(i + '1');
		strcat(LogName[k], stemp);
	}
	else if (!strcmp(mode, "pos"))
	{
		char stemp[2] = "1"; stemp[0] = (char)(i + 'x');
		strcat(LogName[k], stemp);
	}
	else if (!strcmp(mode, "rot"))
	{
		if (i == 0) strcat(LogName[k], ".pitch");
		if (i == 1) strcat(LogName[k], ".row");
		if (i == 2) strcat(LogName[k], ".yaw");
	}
	else if (!strcmp(mode, "posrot"))
	{
		if (i == 0) strcat(LogName[k], ".x");
		if (i == 1) strcat(LogName[k], ".y");
		if (i == 2) strcat(LogName[k], ".z");
		if (i == 3) strcat(LogName[k], ".pitch");
		if (i == 4) strcat(LogName[k], ".row");
		if (i == 5) strcat(LogName[k], ".yaw");
	}
}

#define SetLogName(a, b, c) SetLogName_default(cycle_cnt, k, a, b, c)

void Save_Rt_Response(void)
{
	int i=0;
	float *buf = NULL;	
	static int cycle_cnt = 0;
	int k = 0;
	int tp_k = 0;
	
	if (cycle_cnt++ >= 10) cycle_cnt = 10;
	if (K_Preview_Con == 0) tp_k = 0; else tp_k = K_Preview_Con - 1;
	
	buf = &RtResponse_Buf[Rt_Resp_Cnt]; //important	
	
	SetLogName("time", "none", 0), buf[k++] = ControlledCycle * CONTROL_PERIOD;
	for (i = 0; i < 6; i++, k++) SetLogName("realleg_r", "num", i), buf[k] = 57.3 * Joint[1][i + 1] / Gear_Rate_Joint[1][i];
	for (i = 0; i < 6; i++, k++) SetLogName("desleg_r", "num", i), buf[k] = 57.3 * Ref_Leg_Last[1][i + 1] / Gear_Rate_Joint[1][i];//con_val[1][i+1]/Gear_Rate_Joint[i]; 

	for(i = 0 ; i < 6 ; i++,k++) SetLogName("realleg_l", "num", i), buf[k] = 57.3 * Joint[2][i+1]/Gear_Rate_Joint[2][i];
	for(i = 0 ; i < 6 ; i++,k++) SetLogName("desleg_l", "num", i), buf[k] = 57.3 * Ref_Leg_Last[2][i+1]/Gear_Rate_Joint[2][i];//con_val[1][i+1]/Gear_Rate_Joint[i]; 

	//for(i = 0 ; i < 3 ; i++,k++) buf[k] = Joint_Waist[1][i+1]/Gear_Waist_Rate_Joint[i];
	//for(i = 0 ; i < 3 ; i++,k++) buf[k] = Ref_Waist_Val[i+1]/(Gear_Waist_Rate_Joint[i]); 
	
	for (i = 0; i < 1; i++, k++) SetLogName("realarm_r", "num", i), buf[k] = Joint_Arm[1][i + 1] / Gear_Arm_Rate_Joint[1][i];
	for (i = 0; i < 1; i++, k++) SetLogName("desarm_r", "num", i), buf[k] = Con_Disp_Arm[1][i + 1] / Gear_Arm_Rate_Joint[1][i];
	
	for (i = 0; i < 1; i++, k++) SetLogName("realarm_l", "num", i), buf[k] = Joint_Arm[2][i + 1] / Gear_Arm_Rate_Joint[2][i];
	for (i = 0; i < 1; i++, k++) SetLogName("desarm_l", "num", i), buf[k] = Con_Disp_Arm[2][i + 1] / Gear_Arm_Rate_Joint[2][i];
	
	for (i = 0; i < 1; i++, k++) buf[k] = 111;
	for (i = 0; i < 2; i++, k++) SetLogName("ZMPAct_rel_r", "pos", i), buf[k] = ZMP_Actl[1][i + 1] - Actl_Ankle_W[1][i];//Ref_Ankle[1][i];
	for (i = 0; i < 2; i++, k++) SetLogName("ZMPAct_rel_l", "pos", i), buf[k] = ZMP_Actl[2][i + 1] - Actl_Ankle_W[2][i];//Ref_Ankle[2][i];

	for(i = 0 ; i < 1 ; i++,k++) buf[k] = 222; 
	for(i = 0 ; i < 6 ; i++,k++) SetLogName("AdjFT_r", "posrot", i), buf[k] = Force_Grnd[1][i];    // Adjusted Right Force Sensor 
	for(i = 0 ; i < 6 ; i++,k++) SetLogName("AdjFT_l", "posrot", i), buf[k] = Force_Grnd[2][i];    // Adjusted Left  Force Sensor
	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = 333; 
	for(i = 0 ; i < 6 ; i++,k++) SetLogName("SenFT_r", "posrot", i), buf[k] = FootFT[1][i];       // Sensed Right Force Sensor (No Adjusted) 
	for(i = 0 ; i < 6 ; i++,k++) SetLogName("SenFT_r", "posrot", i), buf[k] = FootFT[2][i];       // Sensed Left  Force Sensor (No Adjusted)

	for(i = 0 ; i < 1 ; i++,k++) buf[k] = 33333; 	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = FootFT[1][2] - 0.5 * micro_delta_fz;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = FootFT[2][2] + 0.5 * micro_delta_fz;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = micro_delta_fz;
	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = 444; 
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("Tra_ZMP.x", "none", 0), buf[k] = Tra_ZMP.x[tp_k];    // X ZMP Trajectory
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("Tra_ZMPCal.x", "none", 0), buf[k] = Tra_ZMPCal.x[tp_k];
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("Tra_ZMP.y", "none", 0), buf[k] = Tra_ZMP.y[tp_k];    // Y ZMP Trajectory
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("Tra_ZMPCal.y", "none", 0), buf[k] = Tra_ZMPCal.y[tp_k];
	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = 555; 
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("Tra_COM.x", "none", 0), buf[k] = Tra_COM.x[tp_k];    // X COM Trajectory
	for (i = 0; i < 1; i++, k++) SetLogName("Tra_VCOM.x", "none", 0), buf[k] = Tra_VCOM.x[tp_k];
	for (i = 0; i < 1; i++, k++) SetLogName("Tra_ACOM.x", "none", 0), buf[k] = Tra_ACOM.x[tp_k];
	for (i = 0; i < 1; i++, k++) SetLogName("Tra_COM.y", "none", 0), buf[k] = Tra_COM.y[tp_k];    // Y COM Trajectory
	for (i = 0; i < 1; i++, k++) SetLogName("Tra_VCOM.y", "none", 0), buf[k] = Tra_VCOM.y[tp_k];
	for (i = 0; i < 1; i++, k++) SetLogName("Tra_ACOM.y", "none", 0), buf[k] = Tra_ACOM.y[tp_k];
	for (i = 0; i < 1; i++, k++) SetLogName("Tra_COM.z", "none", 0), buf[k] = Tra_COM.z[tp_k];    // Z COM Trajectory
	for (i = 0; i < 1; i++, k++) SetLogName("Tra_VCOM.z", "none", 0), buf[k] = Tra_VCOM.z[tp_k];
	for (i = 0; i < 1; i++, k++) SetLogName("Tra_ACOM.z", "none", 0), buf[k] = Tra_ACOM.z[tp_k];
	for (i = 0; i < 1; i++, k++) SetLogName("pitch_body", "none", 0), buf[k] = pitch_body;
	for (i = 0; i < 1; i++, k++) SetLogName("roll_body", "none", 0), buf[k] = roll_body;

	for(i = 0 ; i < 1 ; i++,k++) buf[k] = 666; 
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("Tra_RAnkle.x", "none", 0), buf[k] = Tra_RAnkle.x[tp_k]; // Right Ankle
	for (i = 0; i < 1; i++, k++) SetLogName("Tra_RAnkle.y", "none", 0), buf[k] = Tra_RAnkle.y[tp_k];
	for (i = 0; i < 1; i++, k++) SetLogName("Tra_RAnkle.z", "none", 0), buf[k] = Tra_RAnkle.z[tp_k];
	for (i = 0; i < 1; i++, k++) SetLogName("Tra_LAnkle.x", "none", 0), buf[k] = Tra_LAnkle.x[tp_k]; // Left  Ankle
	for (i = 0; i < 1; i++, k++) SetLogName("Tra_LAnkle.y", "none", 0), buf[k] = Tra_LAnkle.y[tp_k];
	for (i = 0; i < 1; i++, k++) SetLogName("Tra_LAnkle.z", "none", 0), buf[k] = Tra_LAnkle.z[tp_k];
	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = 777;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("GQ_Pitch", "none", 0), buf[k] = GQ_Pitch; //0.0;//
	for (i = 0; i < 1; i++, k++) SetLogName("GQ_Roll", "none", 0), buf[k] = GQ_Roll;  //0.0;//
	for (i = 0; i < 1; i++, k++) SetLogName("GQ_AccX", "none", 0), buf[k] = GQ_AccX;  //0.0;//
	for (i = 0; i < 1; i++, k++) SetLogName("GQ_AccY", "none", 0), buf[k] = GQ_AccY;  //0.0;//
	for (i = 0; i < 1; i++, k++) SetLogName("GQ_AccZ", "none", 0), buf[k] = GQ_AccZ;  //0.0;//
	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = 888;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("K_Preview_Con", "none", 0), buf[k] = K_Preview_Con;
	for (i = 0; i < 1; i++, k++) SetLogName("Walk_On", "none", 0), buf[k] = Walk_On;
	for (i = 0; i < 1; i++, k++) SetLogName("Comp_Waist[tp_k]", "none", 0), buf[k] = Comp_Waist[tp_k];        // Comp_Waist
	for (i = 0; i < 1; i++, k++) SetLogName("Signal_SupportLeg", "none", 0), buf[k] = Signal_SupportLeg[tp_k]; // Signal_SupportLeg
	for (i = 0; i < 1; i++, k++) SetLogName("Signal_NowStep", "none", 0), buf[k] = Signal_NowStep[tp_k];    // Signal_NowStep
	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = 999;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("Ref_RForce_Z", "none", 0), buf[k] = Ref_RForce_Z;
	for (i = 0; i < 1; i++, k++) SetLogName("Ref_LForce_Z", "none", 0), buf[k] = Ref_LForce_Z;
	for (i = 0; i < 1; i++, k++) SetLogName("Rel_RForce_Z", "none", 0), buf[k] = Rel_RForce_Z;
	for (i = 0; i < 1; i++, k++) SetLogName("Rel_LForce_Z", "none", 0), buf[k] = Rel_LForce_Z;
	for (i = 0; i < 1; i++, k++) SetLogName("Deta_RAnkle_Z", "none", 0), buf[k] = Deta_RAnkle_Z;
	for (i = 0; i < 1; i++, k++) SetLogName("Deta_LAnkle_Z", "none", 0), buf[k] = Deta_LAnkle_Z;
	
	for (i = 0; i < 1 ; i++,k++) buf[k] = 1111;
	for (i = 0; i < 1 ; i++,k++) SetLogName("P_DetaCOM.px", "none", 0), buf[k] = P_DetaCOM.px;
	for (i = 0; i < 1; i++, k++) SetLogName("P_DetaCOM.py", "none", 0), buf[k] = P_DetaCOM.py;
	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = 2222; 
	for(i = 0 ; i < 6 ; i++,k++) SetLogName("Leg_Torque_r", "num", i), buf[k] = Leg_Torque[1][i + 1]; // Right Send Current 
	for (i = 0; i < 6; i++, k++) SetLogName("Leg_Torque_l", "num", i), buf[k] = Leg_Torque[2][i+1]; // Left Send Current 
	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = 433; 
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("Ref_Arm_Joint_r", "none", 0), buf[k] = 57.3*Ref_Arm_Joint[1][1];
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("Ref_Arm_Joint_l", "none", 0), buf[k] = 57.3*Ref_Arm_Joint[2][1];
	for(i = 0 ; i < 3 ; i++,k++) SetLogName("ZMP_Actl", "pos", i), buf[k] = ZMP_Actl[0][i];
	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = 12321; 
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("Comp_Waist", "none", 0), buf[k] = Comp_Waist[tp_k];
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("Comp_Waist_bhr7test", "none", 0), buf[k] = Comp_Waist_bhr7test[tp_k];
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("P_ZMPRel_B.px", "none", 0), buf[k] = P_ZMPRel_B.px;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("P_ZMPRel_B.py", "none", 0), buf[k] = P_ZMPRel_B.py;
	
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = 886; 
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = F_cal_x_re;
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = F_resi_x_re;
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = F_v_x_re;
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = e_x_re;
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = F_cal_y_re;
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = F_resi_y_re;
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = F_v_y_re;
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = e_y_re;	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = 886; 
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = F_ext_re.x[tp_k];
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = F_vir_re.x[tp_k];
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = e_re.x[tp_k];
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = Tau_ext_re.x[tp_k];
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = Tau_vir_re.x[tp_k];
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = r_re.x[tp_k];
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = F_ext_re.y[tp_k];
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = F_vir_re.y[tp_k];
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = e_re.y[tp_k];
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = Tau_ext_re.y[tp_k];
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = Tau_vir_re.y[tp_k];
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = r_re.y[tp_k];
	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = 500; 
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = Delta_COM.x;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = State_CP.roll;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = State_CP.droll;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = delta_p_hat.x;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = GP_rel.x - GP_ref.x;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = Delta_COM.y;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = State_CP.pitch;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = State_CP.dpitch;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = delta_p_hat.y;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = GP_rel.y - GP_ref.y;
	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = 600;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = NQP_re.e.x;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = NQP_re.de.x;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = NQP_re.dde.x;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = NQP_re.M_fall.roll;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = NQP_re.phi.roll * 57.3;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = NQP_re.del_zmp.x;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = NQP_re.del_the.roll * 57.3;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = NQP_re.M_flag.x;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = NQP_re.e.y;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = NQP_re.de.y;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = NQP_re.dde.y;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = NQP_re.M_fall.pitch;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = NQP_re.phi.pitch * 57.3;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = NQP_re.del_zmp.y;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = NQP_re.del_the.pitch * 57.3;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = NQP_re.M_flag.y;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRFoot_RAnkle.px;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRFoot_RAnkle.py;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPLFoot_LAnkle.px;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPLFoot_LAnkle.py;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRef_B.px;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRel_B.px;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRef_B.py;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = P_ZMPRel_B.py;
	
	// for(i = 0 ; i < 1 ; i++,k++) SetLogName("delx", "none", 0), buf[k] = deltax_re ;
	// for(i = 0 ; i < 1 ; i++,k++) SetLogName("dely", "none", 0), buf[k] = deltay_re ;
	// for(i = 0 ; i < 1 ; i++,k++) SetLogName("deldx", "none", 0), buf[k] =deltadx_re;
	// for(i = 0 ; i < 1 ; i++,k++) SetLogName("deldy", "none", 0), buf[k] =deltady_re;
	
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("TPC_Foot_Rx", "none", 0), buf[k] = TPCFoot_ConVal_re.RfootPos.x;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("TPC_Foot_Lx", "none", 0), buf[k] = TPCFoot_ConVal_re.LfootPos.x;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("TPC_Foot_Ry", "none", 0), buf[k] = TPCFoot_ConVal_re.RfootPos.y;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("TPC_Foot_Ly", "none", 0), buf[k] = TPCFoot_ConVal_re.LfootPos.y;
	
	
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("pit_ref", "none", 0), buf[k] = ref_pitch_re;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("pit_rel", "none", 0), buf[k] = rel_pitch_re;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("pit_con", "none", 0), buf[k] = con_pitch_re;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("rol_ref", "none", 0), buf[k] = ref_roll_re;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("rol_rel", "none", 0), buf[k] = rel_roll_re;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("rol_con", "none", 0), buf[k] = con_roll_re;
	
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("additor_pit", "none", 0), buf[k] = additor_pit;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("additor_rol", "none", 0), buf[k] = additor_rol;
																		  
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = 456;                            
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("TPC_x", "none", 0), buf[k] = TPC_Run_ConVal.x;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("TPC_y", "none", 0), buf[k] = TPC_Run_ConVal.y;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("dTPC_x", "none", 0), buf[k] = dx_MPCTPC;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("dTPC_y", "none", 0), buf[k] = dy_MPCTPC;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("delcom_x", "none", 0), buf[k] = delta_com_re.x;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("delcom_y", "none", 0), buf[k] = delta_com_re.y;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("delcomv_x", "none", 0), buf[k] = delta_vcom_re.x;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("delcomv_y", "none", 0), buf[k] = delta_vcom_re.y;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("LIPM_x", "none", 0), buf[k] = LIPM_ZMP_x;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("LIPM_y", "none", 0), buf[k] = LIPM_ZMP_y;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("zmp_rel_x", "none", 0), buf[k] = zmp_rel_re.x;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("zmp_ref_x", "none", 0), buf[k] = zmp_ref_re.x;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("zmp_rel_y", "none", 0), buf[k] = zmp_rel_re.y;
    for(i = 0 ; i < 1 ; i++,k++) SetLogName("zmp_ref_y", "none", 0), buf[k] = zmp_ref_re.y;
	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = 789;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("R_ref.pitch", "none", 0), buf[k] = Rfoot_ref_re.pitch;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("R_rel.pitch", "none", 0), buf[k] = Rfoot_rel_re.pitch;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("R_con.pitch", "none", 0), buf[k] = FootCompliance_ConVal.RfootRot.pitch;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("R_con.pitch", "none", 0), buf[k] = DCC_Run.RfootRot.pitch;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("R_ref.roll", "none", 0), buf[k] = Rfoot_ref_re.roll;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("R_rel.roll", "none", 0), buf[k] = Rfoot_rel_re.roll;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("R_con.roll", "none", 0), buf[k] = FootCompliance_ConVal.RfootRot.roll;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("R_con.pitch", "none", 0), buf[k] = DCC_Run.RfootRot.roll;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("L_ref.pitch", "none", 0), buf[k] = Lfoot_ref_re.pitch;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("L_rel.pitch", "none", 0), buf[k] = Lfoot_rel_re.pitch;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("L_con.pitch", "none", 0), buf[k] = FootCompliance_ConVal.LfootRot.pitch;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("R_con.pitch", "none", 0), buf[k] = DCC_Run.LfootRot.pitch;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("L_ref.roll", "none", 0), buf[k] = Lfoot_ref_re.roll;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("L_rel.roll", "none", 0), buf[k] = Lfoot_rel_re.roll;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("L_con.roll", "none", 0), buf[k] = FootCompliance_ConVal.LfootRot.roll;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("R_con.pitch", "none", 0), buf[k] = DCC_Run.LfootRot.roll;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("body.pitch", "none", 0), buf[k] = BodyRot_ConVal.pitch;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("body.roll", "none", 0), buf[k] = BodyRot_ConVal.roll;
	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = 2333;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("FzR_rel", "none", 0), buf[k] = FzR_rel;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("FzR_ref", "none", 0), buf[k] = FzR_ref;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("FzL_rel", "none", 0), buf[k] = FzL_rel;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("FzL_ref", "none", 0), buf[k] = FzL_ref;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("ccRz", "none", 0), buf[k] = ContactConVal.RfootPos.z;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("ccLz", "none", 0), buf[k] = ContactConVal.LfootPos.z;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("zRz", "none", 0), buf[k] = FootCompliance_ConVal.RfootPos.z;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("zLz", "none", 0), buf[k] = FootCompliance_ConVal.LfootPos.z;
	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = F_RFoot.fz;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = FzR_filtered;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = F_LFoot.fz;
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = FzL_filtered;	
	
	for (i = 0; i < 1; i++, k++) buf[k] = 4567;
	for (i = 0; i < 1; i++, k++) buf[k] = XS_Pitch;
	for (i = 0; i < 1; i++, k++) buf[k] = XS_Roll;
	for (i = 0; i < 1; i++, k++) buf[k] = XS_Yaw;
	for (i = 0; i < 1; i++, k++) buf[k] = XS_GyrX;
	for (i = 0; i < 1; i++, k++) buf[k] = XS_GyrY;
	for (i = 0; i < 1; i++, k++) buf[k] = XS_GyrZ;
	for (i = 0; i < 1; i++, k++) buf[k] = XS_AccX;
	for (i = 0; i < 1; i++, k++) buf[k] = XS_AccY;
	for (i = 0; i < 1; i++, k++) buf[k] = XS_AccZ;
	for (i = 0; i < 1; i++, k++) buf[k] = P_ZMPRel_B.px;
	for (i = 0; i < 1; i++, k++) buf[k] = P_ZMPRel_B.py;
	
	
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = 321;
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = Ref_RotAndPos_ConVal.BodyRot.pitch;
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = Ref_RotAndPos_ConVal.BodyRot.roll;
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = Moment_pitch_re;
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = Moment_roll_re;
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = footpitch_re;
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = footroll_re;
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = sense_on_flag;
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = control_on_flag;
	
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = FlyRot_ConVal.RfootPos.z;
	// for(i = 0 ; i < 1 ; i++,k++) buf[k] = FlyRot_ConVal.LfootPos.z;
	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = -123;
	for(i = 0 ; i < 1 ; i++,k++) SetLogName("chz_log", "none", 0), buf[k] = chzrun_signal[tp_k][2];
	for(i = 0 ; i < 40; i++,k++) SetLogName("chz_log", "num", i), buf[k] = chz_log[i];
	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = -886;
	for(i = 0 ; i < 6 ; i++,k++) buf[k] = FootFT_temp[1][i];
	for(i = 0 ; i < 6 ; i++,k++) buf[k] = FootFT_temp[2][i];
	
	for(i = 0 ; i < 1 ; i++,k++) buf[k] = -234;
	for(i = 0 ; i < 20 ; i++,k++) buf[k] = 1.0 * chz_IMU_log[i];

	/********************************************************************************************/
	if(cycle_cnt==1){
		RtPrintf("\n realtime response count: %d",k);
	}
	Rt_Resp_Cnt +=MAX_RESP_MSGSTR_SIZE; //MAX_RESP_MSGSTR_SIZE is defined in control.h
}

#undef SetLogname
/*****************************************************
* Convert servo control reference  to  uniform format
* available to Elmo CAN bus  
******************************************************/
void 	Con_Data_Trans(void)
{ 
	int i, j;
  	for(i = 1; i <= LEG_NUM ; i++){
		for(j = 1; j <= JOINT_NUM ;j++){
			Con_Val_Snd_CAN[Joint_Info[i][j].CanCH][Joint_Info[i][j].CanID] = 
					Con_Val_Leg[i][j]/Joint_Info[i][j].To_SI_Unit;
			Ref_Val_at_Joint_Side[Joint_Info[i][j].CanCH][Joint_Info[i][j].CanID]= 
					Con_Val_Leg[i][j]/Gear_Rate_Joint[i][j-1]; //		
		}
	}
	for(j = 1; j <= JOINT_Head_NUM ;j++){
		Con_Val_Snd_CAN[Joint_Head_Info[j].CanCH][Joint_Head_Info[3-j].CanID] = 
					0.5*Con_Val_Head[j]/Joint_Head_Info[j].To_SI_Unit;
		Ref_Val_at_Joint_Side[Joint_Head_Info[j].CanCH][Joint_Head_Info[3-j].CanID] = 
					0.5*Con_Val_Head[j]/Gear_Head_Rate_Joint[j-1];//			
	}
	for(i = 1; i <= ARM_NUM ; i++){
		for(j =1 ; j <= JOINT_ARM_NUM ;j++){
        	Con_Val_Snd_CAN[Joint_Arm_Info[i][j].CanCH][Joint_Arm_Info[i][j].CanID] = 
					Con_Val_Arm[i][j]/Joint_Arm_Info[i][j].To_SI_Unit;
			Ref_Val_at_Joint_Side[Joint_Arm_Info[i][j].CanCH][Joint_Arm_Info[i][j].CanID] = 
					Con_Val_Arm[i][j]/Gear_Arm_Rate_Joint[i][j-1];//			
		}
	}
        //for waist
	for(j =1 ; j <= JOINT_WAIST_NUM ;j++){
		Con_Val_Snd_CAN[Joint_Waist_Info[1][j].CanCH][Joint_Waist_Info[1][j].CanID] = 
					Con_Val_Waist[1][j]/Joint_Waist_Info[1][j].To_SI_Unit;
		Ref_Val_at_Joint_Side[Joint_Waist_Info[1][j].CanCH][Joint_Waist_Info[1][j].CanID] = 
			Con_Val_Waist[1][j]/Gear_Waist_Rate_Joint[j-1];	//	
	}
		//save value
	for(i=1;i<=MAX_CAN_CHANNEL;i++){
		for(j=1;j<7;j++){
			if(Motor_On_Flag==TRUE){ //Only check after MO=1
				if(fabs(Ref_Val_at_Joint_Side[i][j]- Ref_Val_at_Joint_Side_previous[i][j])>REF_CHANGE_RATE)
				{
					Valid_Joint_Speed[i][j] = FALSE; //FALSE -> over-speed
				}
				else
				{
					Valid_Joint_Speed[i][j] = TRUE;
				}
			}
			Ref_Val_at_Joint_Side_previous[i][j] = Ref_Val_at_Joint_Side[i][j];
		}
	}		

}

BOOL LogResponse_RTSS(void){	
	FILE *fp;
	char filename[33];
	int i,j;
	errno_t err;
	SYSTEMTIME systime; 
	//char szPath[100];
	char currenttime[20];
	
	GetLocalTime(&systime);
	memset(currenttime, 0, sizeof(currenttime));
	sprintf(currenttime,"\\RTLog%4d%02d%02d-%02d%02d.dat",systime.wYear, systime.wMonth, systime.wDay,systime.wHour, systime.wMinute);
	strncat(Workpath,currenttime,strlen(currenttime));
	printf("\nFile workpath:%s\n",Workpath);
	fp=fopen(Workpath,"w");
	if(fp==NULL)
	{
		printf("\nError - file can't open.\n");
		return FALSE ;
	}else
	{
		printf("\ncreate file");
		
	}
	
	// gai flag
	for (j = 0; j<MAX_RESP_MSGSTR_SIZE; j++)
		fprintf(fp, "%s\t", LogName[j]);
	fprintf(fp, "\n");

	for(i=0; i<Rt_Resp_Cnt/MAX_RESP_MSGSTR_SIZE;i++)
	{
		for(j=0;j<MAX_RESP_MSGSTR_SIZE;j++)
			fprintf(fp,"%f\t", RtResponse_Buf[i*MAX_RESP_MSGSTR_SIZE+j]);
		fprintf(fp, "\n"); //for Microsoft C, '\n' is explained as \x0D and \x0A	
	}
	fclose(fp);
	printf("\n****--Save realtime response successfully---***\n");
	return TRUE;	
}

/********************************************
*            Force Sensor by Meng           *
*********************************************/

void IFS_Init()

{
int i,j;
if (IFS_Init_Count==0)
		{
			for (i=1;i<=2;i++)
				{
					for (j=1;j<=6;j++)
						{
							Init_IFS[i-1][j-1]=0;	
						}
				}
		}
		if(IFS_Init_Count<=4000)
		{
		
		for (i=1;i<=2;i++)
				{
					for (j=1;j<=6;j++)
						{
							Init_IFS[i-1][j-1]=Init_IFS[i-1][j-1]+FootFT[i][j-1];	
						}
				}
		
			
		}
		else
		{
		for (i=1;i<=2;i++)
				{
					for (j=1;j<=6;j++)
						{
							
							Init_IFS[i-1][j-1]=Init_IFS[i-1][j-1]/4001;	
							if(j<=3)
							{Init_IFS[i-1][j-1]=floor(Init_IFS[i-1][j-1]);}
						}
						
				}
		IFS_Init_Flag=TRUE;
		}
		IFS_Init_Count++;
		
}
void Trans_IFS_Data_r( unsigned char* data)
{

         long temp_Fx=0, temp_Fy=0, temp_Fz=0, temp_Mx=0, temp_My=0, temp_Mz=0;
		 temp_Fz=data[2];
         temp_Fx=(data[0]<<3)|((data[1]>>5)&0x07);
         temp_Fy=((data[1]&0x1F)<<6)|((data[2]>>2)&0x3F);
         temp_Fz=((temp_Fz&0x03)<<10)|((data[3]<<2)|((data[4]>>6)&0x03));
         temp_Mx=((data[4]&0x3F)<<4)|((data[5]>>4)&0x0F);
         temp_My=((data[5]&0x0F)<<6)|((data[6]>>2)&0x3F);
         temp_Mz=((data[6]&0x03)<<8)|(data[7]);
        if (data[0]&0x80) FootFT[1][0]=(0-(temp_Fx&0x3FF));
        else FootFT[1][0]=(temp_Fx&0x3FF);
        if (data[1]&0x10) FootFT[1][1]=(0-(temp_Fy&0x3FF));
        else FootFT[1][1]=(temp_Fy&0x3FF);
        if (data[2]&0x02) FootFT[1][2]=((temp_Fz&0x7FF));
        else FootFT[1][2]=(0-(temp_Fz&0x7FF));
        if (data[4]&0x20) FootFT[1][3]=(0-(temp_Mx&0x1FF))*0.1;
        else FootFT[1][3]=(temp_Mx&0x1FF)*0.1;
        if (data[5]&0x08) FootFT[1][4]=(0-(temp_My&0x1FF))*0.1;
        else FootFT[1][4]=(temp_My&0x1FF)*0.1;
        if (data[6]&0x02) FootFT[1][5]=(0-(temp_Mz&0x1FF))*0.1;
        else FootFT[1][5]=(temp_Mz&0x1FF)*0.1;
		//IFS_Init_Flag=FALSE;
		if (IFS_Init_Flag==TRUE)
		{
			FootFT[1][0]=-(FootFT[1][0]-Init_IFS[0][0]);//
			FootFT[1][1]=(FootFT[1][1]-Init_IFS[0][1]);
			FootFT[1][2]=FootFT[1][2]-Init_IFS[0][2];
			FootFT[1][3]=(FootFT[1][3]-Init_IFS[0][3]);
			FootFT[1][4]=-(FootFT[1][4]-Init_IFS[0][4]);
			FootFT[1][5]=(FootFT[1][5]-Init_IFS[0][5]);
			for(int dcc = 0; dcc < 6; dcc++){
				FootFT_temp[1][dcc] = FootFT[1][dcc];
			}
		}
		


}
void Trans_IFS_Data_l( unsigned char* data)
{

         long temp_Fx=0, temp_Fy=0, temp_Fz=0, temp_Mx=0, temp_My=0, temp_Mz=0;
		 temp_Fz=data[2];
         temp_Fx=(data[0]<<3)|((data[1]>>5)&0x07);
         temp_Fy=((data[1]&0x1F)<<6)|((data[2]>>2)&0x3F);
         temp_Fz=((temp_Fz &0x03)<<10)|(data[3]<<2)|((data[4]>>6)&0x03);
         temp_Mx=((data[4]&0x3F)<<4)|((data[5]>>4)&0x0F);
         temp_My=((data[5]&0x0F)<<6)|((data[6]>>2)&0x3F);
         temp_Mz=((data[6]&0x03)<<8)|(data[7]);
        if (data[0]&0x80) FootFT[2][0]=(0-(temp_Fx&0x3FF));
        else FootFT[2][0]=(temp_Fx&0x3FF);
        if (data[1]&0x10) FootFT[2][1]=(0-(temp_Fy&0x3FF));
        else FootFT[2][1]=(temp_Fy&0x3FF);
        if (data[2]&0x02) FootFT[2][2]=((temp_Fz&0x7FF));
        else FootFT[2][2]=0-(temp_Fz&0x7FF);
        if (data[4]&0x20) FootFT[2][3]=(0-(temp_Mx&0x1FF))*0.1;
        else FootFT[2][3]=(temp_Mx&0x1FF)*0.1;
        if (data[5]&0x08) FootFT[2][4]=(0-(temp_My&0x1FF))*0.1;
        else FootFT[2][4]=(temp_My&0x1FF)*0.1;
        if (data[6]&0x02) FootFT[2][5]=(0-(temp_Mz&0x1FF))*0.1;
        else FootFT[2][5]=(temp_Mz&0x1FF)*0.1;
		if (IFS_Init_Flag==TRUE)
		{
			FootFT[2][0]=-(FootFT[2][0]-Init_IFS[1][0]);
			FootFT[2][1]=(FootFT[2][1]-Init_IFS[1][1]);
			FootFT[2][2]=FootFT[2][2]-Init_IFS[1][2];
			FootFT[2][3]=(FootFT[2][3]-Init_IFS[1][3]);
			FootFT[2][4]=-(FootFT[2][4]-Init_IFS[1][4]);
			FootFT[2][5]=(FootFT[2][5]-Init_IFS[1][5]); 
			for(int dcc = 0; dcc < 6; dcc++){
				FootFT_temp[2][dcc] = FootFT[2][dcc];
			}
		}

}
/*******************************************************************
* Function: Get Fibre Optic IMU sensor.                            *
* Note    : The installation of IMU should checked before use.     *
*   		Different direction matches different data.            *
********************************************************************/
#ifdef USE_GQ_IMU100
void GQ_IMU100_GetData(unsigned char * data)
{
	//double DEG2RAD=3.14159265358979/180.0;
	double GRA = 9.8;

	int temp1,temp2;
	int temp;
	int i,j,k;

	double IMU525_Pitch, IMU525_Roll, AccX_G, AccY_G, AccZ_G;
	//double AccX,AccY,AccZ;

	/* Pitch */
	temp1=data[0]&0x7f;	
	temp2=data[1]&0xfe;
	temp=((temp1<<8)|temp2)>>1;
	if((data[0]&0x80)==0x80)
	{
		// Pitch is Negtive '-' .
		IMU525_Pitch = -1.0*temp*0.01; // Measured pitch, Unit: deg
	}
	else
	{
		// Pitch is Positive '+' .
		IMU525_Pitch = 1.0*temp*0.01; // Unit: deg
	}
	
	/* Roll */
	temp1=data[2]&0xff;	
	temp2=data[3]&0xe0;
	temp=((temp1<<8)|temp2)>>5;	
	if((data[1]&0x01)==0x01)
	{
		// Roll is Negtive '-' .
		IMU525_Roll = -1.0*temp*0.1; // Measured roll, Unit: deg
	}
	else
	{
		// Roll is Positive '+' .
		IMU525_Roll = 1.0*temp*0.1; // Unit: deg
	}
	
	/* Acc X with Gravity*/
	temp1=data[3]&0x0f;	
	temp2=data[4]&0xfe;
	temp=((temp1<<8)|temp2)>>1;	
	if((data[3]&0x10)==0x10)
	{
		// Ax is Negtive '-' .
		AccX_G = -1.0*temp*0.001*GRA; // Unit: m/s^2
	}
	else
	{
		// Ax is Positive '+' .		
		AccX_G = 1.0*temp*0.001*GRA; // Unit: m/s^2
	}
	
	/* Acc Y with Gravity*/
	temp1=data[5]&0xff;	
	temp2=data[6]&0xe0;
	temp=((temp1<<8)|temp2)>>5;	
	if((data[4]&0x01)==0x01)
	{
		// Ay is Negtive '-' .	
		AccY_G = -1.0*temp*0.001*GRA; // Unit: m/s^2
	}
	else
	{
		// Ay is Positive '+' .	
		AccY_G = 1.0*temp*0.001*GRA; // Unit: m/s^2
		
	}
	
	/* Acc Z with Gravity*/
	temp1=data[6]&0x0f;	
	temp2=data[7]&0xfe;
	temp=((temp1<<8)|temp2)>>1;	
	if((data[6]&0x10)==0x10)
	{
		// Az is Negtive '-' .
		AccZ_G = -1.0*temp*0.001*GRA; // Unit: m/s^2
	}
	else
	{
		// Az is Positive '+' .
		AccZ_G = 1.0*temp*0.001*GRA; // Unit: m/s^2
	}
	
	//printf("Pitch = %lf, Roll = %lf, AccX = %lf, AccY = %lf, AccZ = %lf\n", IMU525_Pitch, IMU525_Roll, AccX_G, AccY_G, AccZ_G);	
	/* Translate to Robot Coordinate */
	GQ_Pitch =-IMU525_Roll;
	GQ_Roll = IMU525_Pitch;
	GQ_AccX =  -AccY_G;
	GQ_AccY =  AccX_G;
	GQ_AccZ =  AccZ_G;	
	
}
#endif
//-----------End of File-------
