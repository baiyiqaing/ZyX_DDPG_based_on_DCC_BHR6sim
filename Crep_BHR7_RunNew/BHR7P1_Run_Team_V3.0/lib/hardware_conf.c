/***********************************************************************
 *  BHR BIPED CONTROL SOFTWARE UNDER Windows XP+Real_Time Extension    *  
 *	Filename:     hardware_conf.c                                      *
 *	Sept.16, 2010, Originaly adapted for Windwos RTX8.1SP2 by Z.G. YU  *
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
 ***********************************************************************/
#define HARDWARE_CONFIG_C
#define GLOBAL_DEFINE

#include <windows.h>

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
//#include <rtapi.h>
#include "..\lib\hardware_conf.h"
#include "..\rtx_process.h"
#include "..\control.h"

#pragma warning(disable : 4244)
//#pragma comment(linker,"/align:0x20")

#define AUTO_CHECK_JOINT_MOVE  //yzg
#define JWBRI_USE
//#define SIGN_Test
//////////////////////////////////////
#define DA_OUT volt
#define M_PI 3.14159265358979
//#define  WANQ   //Home point calibration manually

//#include "../lib/ifs.h"

//#include "../lib/RibPci.h"

/****** Preview Control Walking Pattern Generation x**********/
#include "Tra_Generate.h"
/****** Preview Control Walking Pattern Generation s**********/



#define CLKWS   1  //clockwise turn
#define NCLKWS -1  //anticlockwise turn
#define PLUS    1  //DSP driver sign, positive sign,+ 	
#define DIS     0  //Disable
#define MINUS  -1 //DSP driver sign, minus sign, -
#define EN_PRT  1  //enable DSP driver safeguard
#define DISEN_PRT 0 //disable DSP driver safeguard

extern int Home_Sequence;
extern int Arm_Sequence_Mov;  //whp
Extern BOOL ctrl_wasit_leg_online_flag; //whp
extern int Arm_Poweroff_Right4;
const double Kfltr =1.0;//0.9;           /* Low pass filter for computing velocity */
extern double *Rf_Buf;

#define AMP_TEST  2.5*M_PI/180.0  //20//4 degree
#define WM_TEST   70.0*M_PI*DTIME//50.0*M_PI*DTIME    //0.2//0.04
#define WM_TEST_JTST  25.0*M_PI*DTIME    // Joint Self_test


//for inv_leg uses
#define W_waist   0.16//0.17//0.16//0.15
double Xwaist[LEG_NUM+1]={0.0,0.5*WIDTH_FOOT,-0.5*WIDTH_FOOT};
//double Xwaist[LEG_NUM+1]={0.0,0.5*W_waist,-0.5*W_waist};

//for SIN_TEST
#define AMP_TEST1  2.0*5.0*3.14157/180.0/2.0  //30,25
//#define WM_TEST1   0.2*3.14157*2.0*DTIME/0.0006    //0.5
    // New by yzg
#define WM_TEST1   10/*10 for 1Hz*/*25.0*3.14157*2.0*DTIME    //0.5

/* Joint Enc Direction */    
//Leg Joint                               1    2    3   4   5   6
static const int ENC_Leg_Gain[JOINT_NUM+1]={0,  1 ,  -1 ,  -1,  -1,  1,  1 }; //no modifying!!
                                              //Waist: Yaw, Roll
static const int ENC_Waist_Gain[JOINT_WAIST_NUM+1]={0, 1 , 1 , 1}; //no modifying !!

static const int ENC_Arm_Gain[ARM_NUM][JOINT_ARM_NUM+1]= {{0, 1, 1, 1, 1, 1, 1, 1}, {0, 1, 1, 1, 1, 1, 1, 1}}; //no modifying!!
//Arm    <R> 1  2  3  4  5  6 <L> 1, 2, 3, 4, 5, 6
  
#ifndef DEBUG_ELMO
double Initbais_Arm_Val[3][10] ={ \
{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},\
{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},\
{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}\
};
#define DEG2PI M_PI/180.0
  double Initbais_Val[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};     
#else /////////////////////
double Initbais_Arm_Val[3][10] ={ \
{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, \
{0.0, 3.0*DEG2PI, 3.0*DEG2PI, 3.0*DEG2PI, 3.0*DEG2PI, 3.0*DEG2PI, 3.0*DEG2PI, 3.0*DEG2PI, 0.0, 0.0},\
{0.0, 3.0*DEG2PI, 3.0*DEG2PI, 3.0*DEG2PI, 3.0*DEG2PI, 3.0*DEG2PI, 3.0*DEG2PI, 3.0*DEG2PI, 0.0, 0.0}
};

  double Initbais_Val[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};     
#endif

double 	Initbais_Waist_Val[3] = {0.0, 0.0*M_PI/180.0, 0.0*M_PI/180.0};



Joint_Information RARM_Config[] = {
/***     jointNO  Torq2Volt Test To_SI_Unit    Gear_Rate  Driver_Center   CanID     Kp     Kd    CanCH  Home_compensation   Limit_min  Limit_max ,HALL_Direction, HALL_Target,Driver_Sign,Protect_Enable,Reserved_temp ***/
 { ARM_9 ,  1.5/1.27, 0 , 2.0*M_PI/(4.0*16.0),  1119*2.0,  2000,NCNT,  -1,    0.1,  0.0001,   0/*ch*/, 0.10 , 0.0, 0.0, CLKWS, 5, MINUS, EN_PRT, 0},
 { ARM_8 ,  1.5/1.27, 0 , 2.0*M_PI/(4.0*16.0),  1119*2.0, 2000, NCNT,  -1,    0.1,  0.0001,   0/*ch*/,  0.30 ,0.0, 0.0, CLKWS, 5, MINUS, EN_PRT, 0},
 { ARM_7 ,  1.5/1.27, 0 , 2.0*M_PI/(4.0*512.0), 100*K_AMP_TJA, 0.005236, NCNT/*id,0x06,NCNT*/ , -1, 0.3,  0.002, THIRD/*ch*/, 0.0/*offset*/, -60.0, 60.0, 60/*timeout sec*/, -0/*vmove*/, -0/*vsrch*/, -0.0/*hbias,deg*/, 0 },// 
 { ARM_6 ,  1.5/1.27, 0 , 2.0*M_PI/(4.0*512.0), 100*K_AMP_TJA, 0.005236, NCNT  /*id,0x06,NCNT*/ , 1, 0.3,  0.002, THIRD/*ch*/, 8.9-5.75+0.093+0.032/*offset*/, -60.0, 60.0, 60/*timeout sec*/, -3000/*vmove*/, 1000/*vsrch*/, -5.0/*hbias,deg*/, 0 },// 
  { ARM_5 ,  1.5/1.27, 0 , 2.0*M_PI/(4.0*512.0), 100*K_AMP_TJA, 0.005236, NCNT/*id,0x05,NCNT*/, -1, 0.3,  0.0023, THIRD/*ch*/, -189.559+360.0+1.5-3.24-0.605/*offset*/, -90.0, 90.0, 60/*timeout*/, -3000/*vmove*/, 1000/*vsrch*/, -10.0/*hbias,deg*/, 0 },//1.0/
  { ARM_4 ,  1.5/1.27, 0 , 2.0*M_PI/(4.0*1000.0), 150*K_AMP_TJA, 0.005236, NCNT/*id,0x04,NCNT*/, 1, 0.3,   0.001,  THIRD/*ch*/, 6.655-0.4+0.615-0.197/*offset*/, -15.0, 80.0, 60/*timeout*/, 3000/*vmove*/, -1000/*vsrch*/, 18.0/*hbias deg*/, 0 },//1.0
  { ARM_3 ,  1.5/1.27, 0 , 2.0*M_PI/(4.0*512.0), 100*K_AMP_TJA, 0.005236, NCNT/*id,0x03,NCNT*/,  -1, 0.3,  0.001,   THIRD/*ch*/ , -12.792+0.8-0.622/*offset*/, -90.0, 90.0, 60/*timeout*/, -3000/*vmove*/, 1000/*vsrch*/, -10.0/*hbias deg*/, 0},//3.5
 { ARM_2 ,  3.0/1.27, 0 , 2.0*M_PI/(4.0*1024.0), 20*K_AMP_TJA, 0.005236, NCNT/*id,0x04,NCNT*/,  -1,  0.3,  0.001 ,  THIRD/*ch*/, -0.725+0.721+0.455/*offset*/, -90.0, 10.0, 60/*timeout*/, 3000/*vmove*/, -1000/*vsrch*/, 12.0/*hbias deg*/, 0},//6.0
 { ARM_1 ,  1.0, 0 , 360.0 / 262144.0, 50.0 * 57.3,	0.005236, 0x01/*id,0x01,NCNT*/,  1, 0.3,  0.001 ,FIFTH/*ch*/, 11.168-0.5 +0.520/*offset*/, -50.0, 110.0, 60/*timeout*/,-3000/*vmove*/, 1000/*vsrch*/, -1.0/*hbias deg*/, 0},
	 
	{ EOJ   ,  EOJ,     EOJ,    EOJ,               EOJ,   EOJ,    EOJ, EOJ, EOJ,  EOJ,   EOJ, EOJ , EOJ, EOJ, EOJ,	EOJ,  EOJ, EOJ, EOJ}  // up to 19 EOJs
	};

Joint_Information LARM_Config[] = {
/***     jointNO  Torq2Volt Test To_SI_Unit          Gear_Rate  CanID  chGain   Kp     Kd     CanCH***/
  { ARM_9 ,  1.5/1.27, 0 , 2.0*M_PI/(4.0*16.0),  261, 2000 , NCNT,   -1,     0.1,  0.0001,  0, 0.139, 0.0, 0.0, CLKWS, 5, MINUS, EN_PRT, 0 },
  { ARM_8 ,  1.5/1.27, 0 , 2.0*M_PI/(4.0*12.0),  256, 2000,  NCNT,   1,  0.3,  0.0001,  0, 0.234, 0.0, 0.0, CLKWS, 5, MINUS, EN_PRT, 0 },
  { ARM_7 ,  1.5/1.27, 0 , 2.0*M_PI/(4.0*12.0),  256,2000,    NCNT,   1,  0.1,  0.0001,   0, 0.1, 0.0,  0.0, CLKWS, 5, MINUS, EN_PRT, 0},
  { ARM_6 ,  1.5/1.27, 0 , 2.0*M_PI/(4.0*512.0), 100*K_AMP_TJA,0.005236, NCNT/*id,0x06,NCNT*/ ,  -1,  0.3,  0.001,FOURTH/*ch*/,-34.8/*offset*/, -60.0, 60.0, 60/*time sec*/, 3000/*vmove,cnt*/, 1000/*vsrch,cnt*/, 23.0/*hbias deg*/, 0 },//2.0
  { ARM_5 ,  1.5/1.27, 0 , 2.0*M_PI/(4.0*512.0), 100*K_AMP_TJA,0.005236, NCNT/*id,0x05,NCNT*/  , -1,  0.3,  0.001 ,  FOURTH/*ch*/, -4.85+2.013+6.0/*offset*/, -90.0, 90.0, 60/*time sec*/, -3000/*vmove*/, 1000/*vsrch*/, -20.0/*hbias deg*/, 0 },//2.0
  { ARM_4 ,  1.5/1.27, 0 , 2.0*M_PI/(4.0*1000.0), 150*K_AMP_TJA,0.005236, NCNT/*id,0x04,NCNT*/, -1, 0.3,  0.0013,  FOURTH/*ch*/, -0.1+0.896/*offset*/, -15.0, 75.0, 60/*time sec*/, -3000/*vmove*/, 1000/*vsrch*/, -13.0/*hbias deg*/, 0 },
  { ARM_3 ,  1.5/1.27, 0 , 2.0*M_PI/(4.0*512.0), 100*K_AMP_TJA,0.005236, NCNT/*id,0x03,NCNT*/,  -1, 0.3 ,  0.0013,  FOURTH/*ch*/, -9.0+6.1-0.438/*offset*/, -90.0, 90.0, 60/*time sec*/, -3000/*vmove*/, 1000/*vsrch*/, -15.0/*hbias deg*/, 0 },//2.0//4.5
 { ARM_2 ,  3.0/1.27, 0 , 2.0*M_PI/(4.0*1024.0), 20*K_AMP_TJA, 0.005236, NCNT/*id,0x02,NCNT*/, 1,   0.3, 0.0013 , FIFTH/*ch*/, 3.1+0.5/*offset*/, -10.0, 90.0, 60/*time sec*/, -3000/*vmove*/, 2000/*vsrch*/,-10.0/*hbias deg*/, 0 }, //5.0 
 { ARM_1 ,  1.0, 0 , 360.0 / 262144.0, 50.0 * 57.3, -0.005236, 0x01/*id,0x01,NCNT*/, -1,  0.3, 0.0013 , SIXTH/*ch*/ , 11.7/*offset*/, -50.0, 110.0, 60/*time sec*/, 3000/*vmove*/, -1000/*vsrch*/, 1.0/*hbias deg*/, 0},
	{ EOJ   ,  EOJ,     EOJ,    EOJ,  EOJ,      EOJ,        EOJ, EOJ, EOJ,  EOJ  ,  EOJ,  EOJ , EOJ , EOJ , EOJ, EOJ, EOJ, EOJ, EOJ} //up to 19 EOJs
	};

Joint_Information Head_Config[] = {
/***     jointNO  Torq2Volt Test To_SI_Unit    Gear_Rate  Driver_Center  CanID  chGain Kp     Kd      CanCH***/
	{ Head_Yaw , 1.5/1.27, 0 , 2.0*M_PI/(4.0*512.0), 100, 0.005326 , NCNT/*MPU6000 AccGyro---yaw,0x02,NCNT id=2,0x04,NCNT*/ , 1,   8.8,  0.001, SIXTH/*ch FIRST*/, 0.0,/*home off*/ -20.0, 20.0, 20/*time,sec*/, 0.0/*vmove*/, 0.0/*vsrch*/, 0.0/*hbias, deg*/, 0 },//-1,1.0
	{ Head_Pitch , 1.5/1.27, 0 , 2.0*M_PI/(4.0*512.0), 100, 0.005326, NCNT/*MPU6000 PitchRoll---pitch,0x01, NCNT id=1,0x03,NCNT*/,   1,   8.8,  0.001, SIXTH/*ch FIRST*/, 0.0,/*home off*/ -20.0, -20.0, 20/*time,sec*/, 0.0/*vmove*/, 0.0/*vsrch*/, 0.0/*hbias,deg*/, 0 },//-1,
	{ EOJ   ,  EOJ,     EOJ,    EOJ,    EOJ,   EOJ,   EOJ, EOJ,  EOJ,  EOJ,    EOJ,  EOJ,  EOJ, EOJ}
	};   
    //On Huitong-3 : DSP channel 0: actual +Y, Channel 1: +Z
    //     but in this data structure : just inversely!!, i.e. DSP-fd=0-->+Z
    /*           ENC-code   DSP-channel    Sign_Con_Head
          Head2:     1        0                -1
          Head1:    -1        1                1
      */ 
    //Guangdong  Head motor: ENC: 512, Gear-rate :455

   	/****Waist joint, ****/
Joint_Information WAIST_Config[] = {
/***     		jointNO  Torq2Volt Test To_SI_Unit              Gear_Rate CanID chGain Kp  Kd     CanCH, limit_min, joint_max, Hall_Direction, Hall_Target, Driver_Sign, Protect_Enable, Reserved_tmp***/
				{ WAIST_YAW,  3.0/1.27, 0 , 2.0*M_PI/(4.0*65536.0), 160, 0.005236 , NCNT /*id,0x05,NCNT*/, -1, 0.5, 0.002, THIRD/*ch*//*ch*/ , 0.0,/*home off*/ -30.0, 30.0, 30/*timeout,s*/, 0/*vmove*/, 0/*vsrch*/, 9.0/*hbias,deg*/, 0 }, 
/*L_Tuigan*/	{ WAIST_ROLL,  3.0/1.27, 0 , 2.0*M_PI/(4.0*512.0), 2.0*M_PI/1.5, 0.005236 , NCNT /*id,0x06,NCNT*/,  1, 0.4, 0.002, FOURTH/*ch*/ , 0.0,/*home off*/ -30.0, 30.0, 30/*timeout,s*/, 0/*vmove*/, 0/*vsrch*/, 9.0/*hbias,deg*/, 0 }, 
/*R_Tuigan*/	{ WAIST_PITCH,  3.0/1.27, 0 , 2.0*M_PI/(4.0*512.0), 2.0*M_PI/1.5, 0.005236 , NCNT /*id,0x06,NCNT*/,  1, 0.4, 0.002, THIRD/*ch*/ , 0.0,/*home off*/ -30.0, 30.0, 30/*timeout,s*/, 0/*vmove*/, 0/*vsrch*/, 9.0/*hbias,deg*/, 0 }, 
				{ EOJ   ,  EOJ,     EOJ,    EOJ,    EOJ,   EOJ,   EOJ, EOJ,  EOJ,  EOJ,    EOJ,  EOJ,  EOJ, EOJ}
	};   
	
#define OFFSET_HOME  0.0//0.1 //
/*For debug arm only*/
#if 1  //new home point for PK
/*Zero position, + 减掉当前值，- 加上当前值*/
Joint_Information RLEG_Config[] = {
/***     jointNO  Torq2Volt Test To_SI_Unit    Gear_Rate CanID chGain Kp  Kd    CanCH, limit_min, joint_max, hall_timeout(s), vmove(cnt), vsearch(cnt), home bias(deg) , Reserved_tmp
***/
//FIRST
	{ 1,  1.0,       0,    360.0/262144.0,  1.0,   80,      NCNT,        1,       1.0, 1.0,  FIRST, 0,      0,   0,        0,          0,          0,            0,               0},
	{ 2,  1.0,       0,    360.0/262144.0,  160.0 * 57.3, 80,      0x02,        -1,       1.0, 1.0,  FIRST, 0,      0,   0,        0,          0,          0,            0,               0},
	{ 3,  1.0,       0,    360.0/262144.0,  50.0 * 57.3,  80,      0x03,        -1,       1.0, 1.0,  FIRST, 0,      0,   0,        0,          0,          0,            0,               0}, //11.0,
	{ 4,  1.0,       0,    360.0/262144.0,  50.0 * 57.3,  80,      0x04,         1,       1.0, 1.0,  FIRST, 0,      0,   0,        0,          0,          0,            0,               0}, //12.5,
	{ 5,  1.0,       0,    360.0/262144.0,  50.0 * 57.3,  80,      0x05,         1,       1.0, 1.0,  FIRST, 0,      0,   0,        0,          0,          0,            0,               0}, //12.5,
	{ 6,  1.0,       0,    360.0/262144.0,  50.0 * 57.3,  80,      0x06,        -1,       1.0, 1.0,  FIRST, 0,      0,   0,        0,          0,          0,            0,               0}, //11.0,
	{ EOJ   ,  EOJ, EOJ, EOJ,    EOJ,   EOJ,      EOJ, EOJ, EOJ,  EOJ,    EOJ,  EOJ,  EOJ,  EOJ ,EOJ, EOJ, EOJ, EOJ, EOJ} //up to 19 EOJs
	};


Joint_Information LLEG_Config[] = {
/***     jointNO  Torq2Volt Test To_SI_Unit     Gear_Rate CanID chGain Kp  Kd     CanCH ***/
//SECOND 
	{ 1,  1.0,       0,    360.0/262144.0,  1.0,   80,      NCNT,        1,       1.0, 1.0,  SECOND/*SECOND,THIRD*/, 0,      0,   0,        0,          0,          0,            0,               0},
	{ 2,  1.0,       0,    360.0/262144.0,  160.0 * 57.3, 80,      0x02,        -1,       1.0, 1.0,  SECOND/*SECOND,THIRD*/, 0,      0,   0,        0,          0,          0,            0,               0},
	{ 3,  1.0,       0,    360.0/262144.0,  50.0 * 57.3,  80,      0x03,         1,       1.0, 1.0,  SECOND/*SECOND,THIRD*/, 0,      0,   0,        0,          0,          0,            0,               0}, //11.0,
	{ 4,  1.0,       0,    360.0/262144.0,  50.0 * 57.3,  80,      0x04,        -1,       1.0, 1.0,  SECOND/*SECOND,THIRD*/, 0,      0,   0,        0,          0,          0,            0,               0}, //12.5,
	{ 5,  1.0,       0,    360.0/262144.0,  50.0 * 57.3,  80,      0x05,         1,       1.0, 1.0,  SECOND/*SECOND,THIRD*/, 0,      0,   0,        0,          0,          0,            0,               0}, //12.5,
	{ 6,  1.0,       0,    360.0/262144.0,  50.0 * 57.3,  80,      0x06,        -1,       1.0, 1.0,  SECOND/*SECOND,THIRD*/, 0,      0,   0,        0,          0,          0,            0,               0}, //12.5,
	{EOJ   ,  EOJ, EOJ, EOJ,    EOJ,   EOJ,      EOJ, EOJ, EOJ,  EOJ,    EOJ,  EOJ,  EOJ,  EOJ ,EOJ, EOJ, EOJ, EOJ, EOJ} //up to 19 EOJs
	};
#endif  //end of new home point for PK
/*end of debug arm only*/

/* for sorting */
void CopyConfig(Joint_Information *dst, Joint_Information *src)
{
	dst->Torq2Volt  = src->Torq2Volt;
	dst->Test       = src->Test;
	dst->To_SI_Unit = src->To_SI_Unit;
        dst->Gear_Rate  = src->Gear_Rate;
        dst->Driver_Center =src->Driver_Center;
	dst->CanID      = src->CanID; //
	dst->DA_Gain    = src->DA_Gain;
        dst->Kp         = src->Kp;
        dst->Kd         = src->Kd;
        dst->CanCH         = src->CanCH;
        dst->Offset_Home_Main =src->Offset_Home_Main;
        dst->Joint_Limit_Min =src->Joint_Limit_Min;
        dst->Joint_Limit_Max =src->Joint_Limit_Max;
        dst->HALL_Timeout =src->HALL_Timeout;
        dst->HALL_Vmove =src->HALL_Vmove;
        dst->HALL_Vsearch =src->HALL_Vsearch;
        dst->HALL_Homebias =src->HALL_Homebias;
        dst->Reserved_Temp =src->Reserved_Temp;
}


void Initialize_Values()
{
	int i,j;

	/* sort joint control information */
	for(i = 0; RARM_Config[i].JointNo != EOJ ; i++){
		j = RARM_Config[i].JointNo;
		Joint_Arm_Info[RIGHT][j].JointNo  = j;
		CopyConfig(&Joint_Arm_Info[RIGHT][j], &RARM_Config[i]);
	}
	for(i = 0; LARM_Config[i].JointNo != EOJ ; i++){
		j = LARM_Config[i].JointNo;
		Joint_Arm_Info[LEFT][j].JointNo  = j;
		CopyConfig(&Joint_Arm_Info[LEFT][j], &LARM_Config[i]);
	}
	for(i = 0; Head_Config[i].JointNo != EOJ ; i++){
		j = Head_Config[i].JointNo;
		Joint_Head_Info[j].JointNo  = j;
		CopyConfig(&Joint_Head_Info[j], &Head_Config[i]);
	}
         //added for waist
	for(i = 0; WAIST_Config[i].JointNo != EOJ ; i++){
		j = WAIST_Config[i].JointNo;
		Joint_Waist_Info[1][j].JointNo  = j;
		CopyConfig(&Joint_Waist_Info[1][j], &WAIST_Config[i]);
	}

	for(i = 0; RLEG_Config[i].JointNo != EOJ ; i++){
		j = RLEG_Config[i].JointNo;
		Joint_Info[RIGHT][j].JointNo  = j;
		CopyConfig(&Joint_Info[RIGHT][j], &RLEG_Config[i]);
	}

	for(i = 0; LLEG_Config[i].JointNo != EOJ ; i++){
		j = LLEG_Config[i].JointNo;
		Joint_Info[LEFT][j].JointNo  = j;
		CopyConfig(&Joint_Info[LEFT][j], &LLEG_Config[i]);
	}


	// Initialize value
	for(i = 1 ; i <= ARM_NUM ;i++){
		for(j = 1; j <= JOINT_ARM_NUM ;j++){
			Joint_Arm[i][j] 	= 0;
			Cjoint_Arm[i][j]	= 0;
			Vjoint_Arm[i][j] 	= 0;
			Out_Torque_Arm[i][j]	= 0;
                        Joint_Arm_O[i][j]	= Joint_Arm[i][j];
		}
	}

	for(i = 1 ; i <= JOINT_Head_NUM ;i++){
			Joint_Head[i] 		= 0;
			Cjoint_Head[i] 		= 0;
			Vjoint_Head[i] 		= 0;
			Out_Torque_Head[i]	= 0;
                        Joint_Head_O[i] 	= Joint_Head[i];
		}

	for(i = 1 ; i <= WAIST_NUM ;i++){
		for(j = 1; j <= JOINT_WAIST_NUM ;j++){
			Joint_Waist[i][j] 	= 0;
			Cjoint_Waist[i][j]	= 0;
			Vjoint_Waist[i][j] 	= 0;
			Out_Torque_Waist[i][j]	= 0;
                        Joint_Waist_O[i][j]	= Joint_Waist[i][j];
		}
	}
	for(i = 1 ; i <= LEG_NUM ;i++){
		for(j = 1; j <= JOINT_NUM ;j++){
			Joint[i][j] 		= 0;
			Cjoint[i][j] 		= 0;
			Vjoint[i][j] 		= 0;
			Out_Torque[i][j]	= 0;
                        Joint_O[i][j] 		= Joint[i][j];
		}
	}

	// Foot Force Torque sensors
	for(i = 1; i <= LEG_NUM ; i++){
		for(j = 0 ; j < 6 ; j++) FootFT[i][j] = 0;
	}

	// Gyro and Accelaration sensors
	for(i = 0; i < 3 ; i++){
		Gsens[i] = 0;
                Gyro[i]  = 0;
	}

	LastControlledTime = 0;
	MinTick = NOT_INITIALIZED;
	MaxTick = NOT_INITIALIZED;
	CtrlMissCount = 0;
	ControlFlag = CONTROL_OFF;

	//to intergrate all the items about gear rate in pid_process.c and rtdata.c
	//2010-1-9
	
	//leg config	
	for(i = 1; i <= LEG_NUM ; i++){
      		for(j = 1; j <= JOINT_NUM; j++){
			Gear_Rate_Joint[i][j-1] = Joint_Info[i][j].Gear_Rate;	
			Gear_Leg[i][j-1] = Joint_Info[i][j].Gear_Rate;
		}
	}

	//arm 

	for(i = 1; i <= ARM_NUM ; i++){
      		for(j = 1; j <= JOINT_ARM_NUM; j++){
			Gear_Arm_Rate_Joint[i][j-1] = Joint_Arm_Info[i][j].Gear_Rate;
		}
	}

	// Head Config
	for(i = 1; i <= JOINT_Head_NUM ; i++){
		Gear_Head_Rate_Joint[i-1] = Joint_Head_Info[i].Gear_Rate;
	}

	//Waist  Config
	for(j = 1; j <= JOINT_WAIST_NUM ; j++){
		Gear_Waist_Rate_Joint[j-1] = Joint_Waist_Info[1][j].Gear_Rate;
	}




}

int Boot_Robot(){
	Initialize_Values();
	Home_On_Begin = FALSE;
	Home_Count_Arm =0;
	ControlFlag = CONTROL_ON;
	return 0;
}



double CTR_Read_Ctr_1(int ch, int node )
{
	return(enc_val[ch][node]);

}

// read from hardware and transfer to [rad] , [m] , [N]
void Get_Sensor_Data(void )
{
	int i,j;
	double val;
	for(i = 1; i <= LEG_NUM ; i++){
		for(j = 1; j <= JOINT_NUM ;j++){
	
        val=(double)(CTR_Read_Ctr_1(Joint_Info[i][j].CanCH,Joint_Info[i][j].CanID)) * ENC_Leg_Gain[j] * Joint_Info[i][j].To_SI_Unit*Joint_Info[i][j].DA_Gain;
//	if((i==1)&&(j==1)) val = -val;
        TEST_A[i][j] = (double)(CTR_Read_Ctr_1(Joint_Info[i][j].CanCH,Joint_Info[i][j].CanID)) * ENC_Leg_Gain[j] * Joint_Info[i][j].DA_Gain;

	Vjoint[i][j]= Kfltr*FREQ * (val - Joint[i][j] )+
                      (1.0-Kfltr)*FREQ * (Joint[i][j] - Joint_O[i][j] );
        Joint_O[i][j]= Joint[i][j];
        #ifdef JOINT_OFFSET
        Joint[i][j] = val-Joint_Offset[i-1][j-1]*Gear_Rate_Joint[i][j-1]; 
		#else
		Joint[i][j] = val;
		#endif
  
		}
	}

	//Get Head encoder data 
for (i=1;i<=JOINT_Head_NUM;i++) {
	val=(double)(CTR_Read_Ctr_1(Joint_Head_Info[i].CanCH,Joint_Head_Info[i].CanID));
        if (i==1) 
			val=val*Joint_Head_Info[i].To_SI_Unit*Joint_Head_Info[i].DA_Gain;
        else
			val=val*Joint_Head_Info[i].To_SI_Unit*Joint_Head_Info[i].DA_Gain;
	     	
        Vjoint_Head[i]=       Kfltr*FREQ * (val - Joint_Head[i] )+
                      (1.0-Kfltr)*FREQ * (Joint_Head[i] - Joint_Head_O[i] );
        Joint_Head_O[i]= Joint_Head[i];
        Joint_Head[i] = val;    
}
     
	val = Joint_Head[1];
	Joint_Head[1] = Joint_Head[2];
	Joint_Head[2] = val;  

 // Get ARM encoder position from RIB_borad 
for (i=1;i<=ARM_NUM;i++){	
	for (j=1;j<=JOINT_ARM_NUM;j++) {
	val=(double)(CTR_Read_Ctr_1(Joint_Arm_Info[i][j].CanCH,Joint_Arm_Info[i][j].CanID));
        val=val*ENC_Arm_Gain[i-1][j]*Joint_Arm_Info[i][j].To_SI_Unit*Joint_Arm_Info[i][j].DA_Gain;
        
        if (j==2&&i==2) val = -val;

	 Vjoint_Arm[i][j]=       Kfltr*FREQ * (val - Joint_Arm[i][j] )+
                      (1.0-Kfltr)*FREQ * (Joint_Arm[i][j] - Joint_Arm_O[i][j] );
        Joint_Arm_O[i][j]= Joint_Arm[i][j];
        Joint_Arm[i][j] = val;  
	}
} 
	//added for waist 
for (i=1;i<=WAIST_NUM;i++){	
	for (j=1;j<=JOINT_WAIST_NUM;j++) {
	val=(double)(CTR_Read_Ctr_1(Joint_Waist_Info[i][j].CanCH,Joint_Waist_Info[i][j].CanID));
        val=val*ENC_Waist_Gain[j]*Joint_Waist_Info[i][j].To_SI_Unit*Joint_Waist_Info[i][j].DA_Gain;
        
	 Vjoint_Waist[i][j]=  Kfltr*FREQ * (val - Joint_Waist[i][j] )+                     (1.0-Kfltr)*FREQ * (Joint_Waist[i][j] - Joint_Waist_O[i][j] );
       Joint_Waist_O[i][j]= Joint_Waist[i][j];
        Joint_Waist[i][j] = val;  
	}
} 

}

extern int ID_File; 
BOOL File_Read_End = TRUE;

void Get_Plan_Data(int offline_FileID, unsigned long *DataPtr )
{
	int i=0,k=0;
	
	//add for multiple off-line file
	if (offline_FileID > 10) goto Exitlabel;
	//printf("\n Current File ID=%d",offline_FileID);
	Max_Num_Rfdat_Sum = iFileStartPoint[offline_FileID] + Last_File_Length[offline_FileID];
	Now_Rfdat_Num = *DataPtr;
	//end for multiple off-line file
		
	if (Walk_On == TRUE_1 && Mode_Flag != PRE_CON_MODE) //Walk_On
	{
		File_Read_End = FALSE;
		Test_Time =Test_Time+DTIME;
		Now_Rfdat_Num ++;
		for(i = 0 ; i < 6; i++,Now_Rfdat_Num ++)  Ref_Val[1][i+1] = Rfdat_Buf[Now_Rfdat_Num]*Joint_Info[1][i+1].Gear_Rate; 
		//printf("%f\n",Ref_Val[1][4]);
		Now_Rfdat_Num ++;     
        for(i = 0 ; i < 6; i++,Now_Rfdat_Num ++)  Ref_Val[2][i+1] = Rfdat_Buf[Now_Rfdat_Num]*Joint_Info[2][i+1].Gear_Rate; 
		Now_Rfdat_Num ++;
		for(i = 0 ; i < 3; i++,Now_Rfdat_Num ++) Ref_Waist_Val[i+1]  = Rfdat_Buf[Now_Rfdat_Num]* Joint_Waist_Info[1][i+1].Gear_Rate;
		Now_Rfdat_Num = Now_Rfdat_Num + 2;
		for(i = 0 ; i < 4; i++,Now_Rfdat_Num ++) Ref_Arm_Val[1][i+1]  = Rfdat_Buf[Now_Rfdat_Num]* Joint_Arm_Info[1][i+1].Gear_Rate;
		Now_Rfdat_Num ++;
		for(i = 0 ; i < 4; i++,Now_Rfdat_Num ++) Ref_Arm_Val[2][i+1]  = Rfdat_Buf[Now_Rfdat_Num]* Joint_Arm_Info[2][i+1].Gear_Rate;
		Now_Rfdat_Num ++;
	
	//20170107 tuigan
		Ref_Waist_Pitch = Ref_Waist_Val[2];
		Ref_Waist_Roll = Ref_Waist_Val[3];
		get_tuigan_length(Ref_Waist_Val[3]/Joint_Waist_Info[1][3].Gear_Rate, Ref_Waist_Val[2]/Joint_Waist_Info[1][2].Gear_Rate, &Ref_Waist_Val[2]);
		Ref_Waist_Val[2] = Ref_Waist_Val[2]*Joint_Waist_Info[1][2].Gear_Rate;
		Ref_Waist_Val[3] = Ref_Waist_Val[3]*Joint_Waist_Info[1][3].Gear_Rate;

        Plan_Count++;
        if (Now_Rfdat_Num >= (Max_Num_Rfdat_Sum-12) ){   /*****add by xt*****/

			Walk_On=0;// comment by yzg,for invoked continously
			
			ID_File++;
			File_Read_End = TRUE;
            //Now_Rfdat_Num= Max_Num_Rfdat_Sum; 
			//printf("\nCurrent Ptr=%d, next start point=%d", Now_Rfdat_Num, iFileStartPoint[offline_FileID+1]);
        }
		
		*DataPtr = Now_Rfdat_Num;
                
        }//end of "if(Walk_On==TRUE_1)"


  /**********************SINE_TEST,rewrite yzg,****************************/
	if (Sin_Test_On == 0xaa ){ //'S''I''B'
        	Test_Time =Test_Time+DTIME;
   		//------HEAD Sin test  -------
		for( k=1;k<=JOINT_Head_NUM;k++){
			Ref_Head_Val[k]=0.5*AMP_TEST1*sin(WM_TEST1*Test_Time)*Joint_Head_Info[k].Gear_Rate;
		}

		//-------------ARM  Sin test---------
		for( i=1;i<=ARM_NUM;i++){
        		for(k=1;k<=JOINT_ARM_NUM;k++){
     				Ref_Arm_Val[i][k]  = AMP_TEST1*Joint_Arm_Info[i][k].Gear_Rate*sin(WM_TEST1*Test_Time);
			}
		}
    		//----------------LEG  Sin test------
		for( i=1;i<=LEG_NUM;i++){
			for(k=1;k<=JOINT_NUM;k++){
				Ref_Val[i][k]  = AMP_TEST1*Joint_Info[i][k].Gear_Rate*sin(WM_TEST*Test_Time);
			}
		}
        	for(k=1;k<=JOINT_WAIST_NUM;k++){
			Ref_Waist_Val[1]  = 0.5* AMP_TEST1*Joint_Waist_Info[1][k].Gear_Rate*sin(WM_TEST1*Test_Time);
			Ref_Waist_Val[2]  = 0.5* AMP_TEST1*Joint_Waist_Info[1][k].Gear_Rate*sin(WM_TEST1*Test_Time);
		}
		if (Now_Rfdat_Num >= (Max_Num_Rfdat_Sum - 12) ){   
                	Sin_Test_On=0;// comment by yzg,for invoked continously
                	Now_Rfdat_Num=0; 
										
        	}
                
        }//end of "if(Sin_Test_On==TRUE_1)"


        if ( Home_On  == TRUE_1 ) //Go the desired initial position for legs
        {
		
		for (i=1;i<3;i++)
		{
			for(k=1;k<7;k++)
			{
				Ref_Joint_detaTPC[i][k]= 0.0; 
				Mdf_Val[i][k] = 0.0;
				Mdf_Val_Pos[i][k] = 0.0;
			}	
		}

	if(Leg_Init_Flag == 0xaa){
		Initbais_Val[1] = 0.0*M_PI/180.0;
		Initbais_Val[2] = 0.0*M_PI/180.0;
		Initbais_Val[3] = 0.0*M_PI/180.0;
		Initbais_Val[4] = 0.0*M_PI/180.0;
		Initbais_Val[5] = 0.0*M_PI/180.0;
		Initbais_Val[6] = 0.0*M_PI/180.0;
	} 
	else {
		Initbais_Val[1] = 0.0*M_PI/180.0;
		Initbais_Val[2] = 0.0*M_PI/180.0;
		Initbais_Val[3] = 0.0*M_PI/180.0;
		Initbais_Val[4] = 0.0*M_PI/180.0;
		Initbais_Val[5] = 0.0*M_PI/180.0;
		Initbais_Val[6] = 0.0*M_PI/180.0;
	}

        Test_Time =Test_Time+DTIME;//0.003;
                if( (WM_TEST*Test_Time) >= M_PI ){
                        Test_Time = M_PI/(WM_TEST);
//			Home_Count = 0;
                        Home_On = 0;  //commented for no stopping
#ifdef DEBUG_ELMO
			Home_On_Again = TRUE;
#endif
	     //Additional initial Variables for Sensory Control

			for( i=1;i<=LEG_NUM;i++) {          /* reset feet position: Must be computed by kinemataics*/
			  Ref_Ankle[i][0] = 0.5*WIDTH_FOOT;
			  Ref_Ankle[i][1] = 0.0;
			  Ref_Ankle[i][2] =-2.0*LENG_LEG*cos(Initbais_Val[3]);
			}
			Ref_Ankle[2][0] = -0.5*WIDTH_FOOT;


			for( i=0;i<=2;i++) {          /* reset planned Qbody data */
			  Ref_Qbody[i] =0.0;
			  Ref_Qvbody[i] =0.0;
			}


			Ref_Yzmp[1] = -L_YBACK;                    //0.103;//-0.1;                 /* ZMP margin [1]:min; [2]:max*/
			Ref_Yzmp[2] =  L_YFRONT;                    //0.127;//0.13;
			Ref_Xzmp[1] = -0.5*WIDTH_FOOT-W_OUT;       //0.2;//-0.2;
			Ref_Xzmp[2] =  0.5*WIDTH_FOOT+W_OUT;       //0.2;//0.2;


			for( i=1;i<=2;i++)                  /* reset support signal */
			  Ref_Cnt_Sgnl[i] = 1;

			for(i = 1 ; i <= 2 ; i++) 
			  Ref_Timecntl[i]  = 0;

              
                }

		for( k=1;k<=JOINT_Head_NUM;k++){
                                Ref_Head_Val[k]  =0.5*Init_Head_Val[k]*(1.0+cos(WM_TEST*Test_Time));
                                Refv_Head_Val[k] = 0.0;//0.5*Init_Head_Val[k]*sin(WM_TEST*Test_Time);
                        }

                for( i=1;i<=ARM_NUM;i++){
                        for(k=1;k<=JOINT_ARM_NUM;k++){
//                                Ref_Arm_Val[i][k]  = 0.5*Init_Arm_Val[i][k]*(1.0+cos(WM_TEST*Test_Time));
//			Ref_Arm_Val[i][k]  = 0.5*Init_Arm_Val[i][k]* (1.0+cos(WM_TEST*Test_Time))+ Initbais_Arm_Val[i][k]*Joint_Arm_Info[i][k].Gear_Rate* sin(0.5*WM_TEST*Test_Time);

// 				Ref_Arm_Val[i][k]  = Init_Arm_Val[i][k]-Home_K[i][k]*WM_TEST*Test_Time;
                   		Refv_Arm_Val[i][k] = 0.0;//0.5*Init_Arm_Val[i][k]*sin(WM_TEST*Test_Time);
                        }
                }
/////////////////////////////////////////////////////////////              
#if 1  
		for( i=1;i<=LEG_NUM;i++){
                        for(k=1;k<=JOINT_NUM;k++){
                          Ref_Val[i][k]  = 0.5*Init_Val[i][k]*
						(1.0+cos(WM_TEST*Test_Time))-
						 Initbais_Val[k]*Joint_Info[i][k].Gear_Rate*
						sin(0.5*WM_TEST*Test_Time);
                                Refv_Val[i][k] = 0.0;
                        }
                }
               // for waist   // TUIGAN
			   //20170108
                for(k=1;k<=JOINT_WAIST_NUM;k++){
                           Ref_Waist_Val[k]  = 0.5*Init_Waist_Val[k]*
				(1.0+cos(WM_TEST*Test_Time))+ Initbais_Waist_Val[k]*Joint_Waist_Info[1][k].Gear_Rate* 
				sin(0.5*WM_TEST*Test_Time);
				
				//Ref_Waist_Val[k]  = Init_Waist_Val[k]; //TUIGAN
                        }
#endif             
		if (Home_On == 0){
                        Test_Time = 0;
						Leg_Init_Flag = 0x55;
			}
        }// end of if ( Home_On  == TRUE_1 )

	Arm_Home();
	
	Exitlabel:
		;
	
}

double Saturate(double x, double max_x)
{
	if(x > max_x) return(max_x);
	else if(x < -max_x) return(-max_x);
	else return(x);
}

#if  1 
void Arm_Home(void)
{

	int i = 0, k = 0;
	if (Home_On_Arm == TRUE)
	{
		if (Leg_Init_Flag == 0xaa)//3v5进此支路whp
		{
			Initbais_Arm_Val[1][1] = 0.0*DEG2PI;
			Initbais_Arm_Val[1][2] = 0.0*DEG2PI;
			Initbais_Arm_Val[1][3] = 0.0*DEG2PI;
			Initbais_Arm_Val[1][4] = 0.0*DEG2PI;
			Initbais_Arm_Val[1][5] = 0.0*DEG2PI;
			Initbais_Arm_Val[1][6] = 0.0*DEG2PI;
			Initbais_Arm_Val[1][7] = 0.0*DEG2PI;

			Initbais_Arm_Val[2][1] = 0.0*DEG2PI;
			Initbais_Arm_Val[2][2] = 0.0*DEG2PI;
			Initbais_Arm_Val[2][3] = 0.0*DEG2PI;
			Initbais_Arm_Val[2][4] = 0.0*DEG2PI;
			Initbais_Arm_Val[2][5] = 0.0*DEG2PI;
			Initbais_Arm_Val[2][6] = 0.0*DEG2PI;
		}
		else//5复位时进此支路whp
		{
			Initbais_Arm_Val[1][1] = 0.0*DEG2PI;
			Initbais_Arm_Val[1][2] = 0.0*DEG2PI;
			Initbais_Arm_Val[1][3] = 0.0*DEG2PI;
			Initbais_Arm_Val[1][4] = 50.0*DEG2PI;
			Initbais_Arm_Val[1][5] = 0.0*DEG2PI;
			Initbais_Arm_Val[1][6] = 0.0*DEG2PI;
			Initbais_Arm_Val[1][7] = 0.0*DEG2PI;

			Initbais_Arm_Val[2][1] = 0.0*DEG2PI;
			Initbais_Arm_Val[2][2] = 0.0*DEG2PI;
			Initbais_Arm_Val[2][3] = 0.0*DEG2PI;
			Initbais_Arm_Val[2][4] = 50.0*DEG2PI;
			Initbais_Arm_Val[2][5] = 0.0*DEG2PI;
			Initbais_Arm_Val[2][6] = 0.0*DEG2PI;
		}

		if (Home_On_Begin) {
			Home_On_Begin = FALSE;
			for (i = 1; i <= ARM_NUM; i++) {
				for (k = 1; k <= JOINT_ARM_NUM; k++) {
					Home_K[i][k] = (Init_Arm_Val[i][k] - Initbais_Arm_Val[i][k] * Joint_Arm_Info[i][k].Gear_Rate) / M_PI;
				}
			}
		}

		Test_Time_Arm = Test_Time_Arm + DTIME;//0.003;
		if ((WM_TEST*Test_Time_Arm) >= M_PI)
		{
			Test_Time_Arm = M_PI / (WM_TEST);
			Home_Count_Arm = 0;
			Home_On_Arm = FALSE;  //commented for no stopping
		}

		for (i = 1; i <= ARM_NUM; i++)
		{
			for (k = 1; k <= JOINT_ARM_NUM; k++)
			{
				//Ref_Arm_Val[i][k]  = 0.5*Init_Arm_Val[i][k]*(1.0+cos(WM_TEST*Test_Time));
				//Ref_Arm_Val[i][k]  = 0.5*Init_Arm_Val[i][k]* (1.0+cos(WM_TEST*Test_Time))+ Initbais_Arm_Val[i][k]*Joint_Arm_Info[i][k].Gear_Rate* sin(0.5*WM_TEST*Test_Time);
				Ref_Arm_Val[i][k] = Init_Arm_Val[i][k] - Home_K[i][k] * WM_TEST*Test_Time_Arm;
				Refv_Arm_Val[i][k] = 0.0;//0.5*Init_Arm_Val[i][k]*sin(WM_TEST*Test_Time);
			}
		}
		if (Home_On_Arm == FALSE)
		{
			Test_Time_Arm = 0;
			Home_Count_Arm = 0;
			Leg_Init_Flag = 0x55;
		}
	}
}
#endif


void Check_Elmo_Config(void) //2010-1-6
{
	int i;
	int j;
	int kkk=0;
#define NODE_ON_EACH_CHANNEL 6 //	
	elmo_node_count = 0;
	for(i=0; i<=6; i++){ //i->can channel
		ChannelCfg[i] = FALSE;
		for(j=0; j<=7; j++){ //j -> can node on each channel
			CanCfg[i][j] = FALSE;	
			Elmo_Configed_Low = 0;
			Elmo_Configed_High = 0;
			Elmo_Configed_MSB = 0;
		}
	}

	//leg config	
	for(i = 1; i <= LEG_NUM ; i++){
      		for(j = 1; j <= JOINT_NUM; j++){
			if(Joint_Info[i][j].CanID != NCNT){ 
				CanCfg[Joint_Info[i][j].CanCH][Joint_Info[i][j].CanID] = TRUE;
				elmo_node_count++;
				kkk = (Joint_Info[i][j].CanCH -1)*NODE_ON_EACH_CHANNEL + Joint_Info[i][j].CanID ;
				if(kkk>=1 && kkk<=16) Elmo_Configed_Low   |= 1<< (kkk-1);
				if(kkk>=17 && kkk<=32) Elmo_Configed_High |= 1<< (kkk-17);
				if(kkk>=33 && kkk<=48) Elmo_Configed_MSB  |= 1<< (kkk-33);

			}
		}
	}

	//arm 

	for(i = 1; i <= ARM_NUM ; i++){
      		for(j = 1; j <= JOINT_ARM_NUM; j++){
			if(Joint_Arm_Info[i][j].CanID != NCNT){ 
				CanCfg[Joint_Arm_Info[i][j].CanCH][Joint_Arm_Info[i][j].CanID] = TRUE;
				elmo_node_count++;
				kkk = (Joint_Arm_Info[i][j].CanCH -1)*NODE_ON_EACH_CHANNEL + Joint_Arm_Info[i][j].CanID ;
				if(kkk>=1 && kkk<=16) Elmo_Configed_Low   |= 1<< (kkk-1);
				if(kkk>=17 && kkk<=32) Elmo_Configed_High |= 1<< (kkk-17);
				if(kkk>=33 && kkk<=48) Elmo_Configed_MSB  |= 1<< (kkk-33);
			}
		}
	}

	// Head Config
	for(i = 1; i <= JOINT_Head_NUM ; i++){
			if(Joint_Head_Info[i].CanID != NCNT){ 
				CanCfg[Joint_Head_Info[i].CanCH][Joint_Head_Info[i].CanID] = TRUE;
				elmo_node_count++;
				kkk = (Joint_Head_Info[i].CanCH -1)*NODE_ON_EACH_CHANNEL + Joint_Head_Info[i].CanID ;
				if(kkk>=1 && kkk<=16) Elmo_Configed_Low   |= 1<< (kkk-1);
				if(kkk>=17 && kkk<=32) Elmo_Configed_High |= 1<< (kkk-17);
				if(kkk>=33 && kkk<=48) Elmo_Configed_MSB  |= 1<< (kkk-33);
			}
	}

	//Waist  Config
	for(j = 1; j <= JOINT_WAIST_NUM ; j++){
			if(Joint_Waist_Info[1][j].CanID != NCNT){ 
				CanCfg[Joint_Waist_Info[1][j].CanCH][Joint_Waist_Info[1][j].CanID] = TRUE;
				elmo_node_count++;
				kkk = (Joint_Waist_Info[1][j].CanCH -1)*NODE_ON_EACH_CHANNEL + Joint_Waist_Info[1][j].CanID ;
				if(kkk>=1 && kkk<=16) Elmo_Configed_Low   |= 1<< (kkk-1);
				if(kkk>=17 && kkk<=32) Elmo_Configed_High |= 1<< (kkk-17);
				if(kkk>=33 && kkk<=48) Elmo_Configed_MSB  |= 1<< (kkk-33);
			}
	}

	for(i=0; i<=6; i++){
		for(j=0; j<=7; j++){
			ChannelCfg[i] |=CanCfg[i][j];	
		}
	}

}

/////----------------------------
double Sign_Comp_Waist[JOINT_WAIST_NUM+1]=
        {
//Joints: NC  Yaw  Roll
          0, -1.0,  -1.0 
	}; 
double Sign_Comp_Head[JOINT_Head_NUM+1]=
        {
//Joints: NC  Head1  Head2
          0, -1.0,  -1.0  
         };
/*
 double Sign_Comp_Leg[LEG_NUM+1][JOINT_NUM+1]={
 // Joints: NC  HipZ  HipY   HipX  KneeX  FootX  FootY
        {  0,    0,    0,    0,    0,    0,    0   },
        {  0,  -1.0,  1.0,  1.0,  1.0,   -1.0,  -1.0}, //Right LEG
        {  0,  -1.0,  1.0,  -1.0,  -1.0,   1.0,   -1.0} //Left LEG
};*/  
 
 double Sign_Comp_Leg[LEG_NUM+1][JOINT_NUM+1]={
 // Joints: NC  HipZ  HipY   HipX  KneeX  FootX  FootY
        {  0,    0,    0,    0,    0,    0,    0   },
        {  0,  -1.0,  1.0,  -1.0,  -1.0,   1.0,  1.0}, //Right LEG
        {  0,  -1.0,  1.0,  1.0,  -1.0,   -1.0,   1.0} //Left LEG
};  
double Sign_Comp_Arm[ARM_NUM+1][JOINT_ARM_NUM+1]={
// Joints: NC  Arm1  Arm2  Arm3   Arm4  Arm5  Arm6
        {  0,    0,    0,    0,    0,    0,   0   },
        {  0,  1.0,   -1.0,  -1.0,  1.0,   -1.0,  1.0, 1.0},  //Right ARM
        {  0,  -1.0,   -1.0,  -1.0,  -1.0,   -1.0,  -1.0, 1.0}  //Left ARM
};
//---------Transfer Joint Home Point compensation to Elmo  controller-----

void Elmo_Home_Compen_Trans(void) //modified for Elmo, 2010-01-06
{ 
	int i,j;
	for(i=0; i<7; i++){ //initialize to zeros.
		for(j=0; j<9; j++){
			Elmo_Home_Compen[i][j] = 0;
			Elmo_Home_Timeout[i][j] = 0;
			Elmo_Home_Vmove[i][j] = 0;
			Elmo_Home_Vsearch[i][j] = 0;
			Elmo_Home_Bias[i][j] = 0;
		}
	}

        //for LEG 
  	for(i = 1; i <= LEG_NUM ; i++)
	    for(j = 1; j <= JOINT_NUM ;j++){
               Elmo_Home_Compen[Joint_Info[i][j].CanCH][Joint_Info[i][j].CanID]=Joint_Info[i][j].Offset_Home_Main*Joint_Info[i][j].Gear_Rate*Sign_Comp_Leg[i][j]*(M_PI/180.0/Joint_Info[i][j].To_SI_Unit);
               Elmo_Home_Timeout[Joint_Info[i][j].CanCH][Joint_Info[i][j].CanID]=1000*Joint_Info[i][j].HALL_Timeout;
               Elmo_Home_Vmove[Joint_Info[i][j].CanCH][Joint_Info[i][j].CanID]=Joint_Info[i][j].HALL_Vmove;
               Elmo_Home_Vsearch[Joint_Info[i][j].CanCH][Joint_Info[i][j].CanID]=Joint_Info[i][j].HALL_Vsearch;
               Elmo_Home_Bias[Joint_Info[i][j].CanCH][Joint_Info[i][j].CanID]=Joint_Info[i][j].HALL_Homebias*Joint_Info[i][j].Gear_Rate*(M_PI/180.0/Joint_Info[i][j].To_SI_Unit);
                      }
        //for HEAD
	for(j = 1; j <= JOINT_Head_NUM ;j++){
  	     Elmo_Home_Compen[Joint_Head_Info[j].CanCH][Joint_Head_Info[j].CanID]=Joint_Head_Info[j].Offset_Home_Main*Joint_Head_Info[j].Gear_Rate*Sign_Comp_Head[j]*(M_PI/180.0/Joint_Head_Info[j].To_SI_Unit);
  	     Elmo_Home_Timeout[Joint_Head_Info[j].CanCH][Joint_Head_Info[j].CanID]=1000*Joint_Head_Info[j].HALL_Timeout;
  	     Elmo_Home_Vmove[Joint_Head_Info[j].CanCH][Joint_Head_Info[j].CanID]=Joint_Head_Info[j].HALL_Vmove;
  	     Elmo_Home_Vsearch[Joint_Head_Info[j].CanCH][Joint_Head_Info[j].CanID]=Joint_Head_Info[j].HALL_Vsearch;
  	     Elmo_Home_Bias[Joint_Head_Info[j].CanCH][Joint_Head_Info[j].CanID]=Joint_Head_Info[j].HALL_Homebias*Joint_Head_Info[j].Gear_Rate*(M_PI/180.0/Joint_Head_Info[j].To_SI_Unit);
                      }
          //for ARM
        for(i = 1; i <= ARM_NUM ; i++)
            for(j = 1; j <= JOINT_ARM_NUM ;j++){
  	     Elmo_Home_Compen[Joint_Arm_Info[i][j].CanCH][Joint_Arm_Info[i][j].CanID]=Joint_Arm_Info[i][j].Offset_Home_Main*Joint_Arm_Info[i][j].Gear_Rate*Sign_Comp_Arm[i][j]*(M_PI/180.0/Joint_Arm_Info[i][j].To_SI_Unit);
  	     Elmo_Home_Timeout[Joint_Arm_Info[i][j].CanCH][Joint_Arm_Info[i][j].CanID]=1000*Joint_Arm_Info[i][j].HALL_Timeout;
  	     Elmo_Home_Vmove[Joint_Arm_Info[i][j].CanCH][Joint_Arm_Info[i][j].CanID]=Joint_Arm_Info[i][j].HALL_Vmove;
  	     Elmo_Home_Vsearch[Joint_Arm_Info[i][j].CanCH][Joint_Arm_Info[i][j].CanID]=Joint_Arm_Info[i][j].HALL_Vsearch;
  	     Elmo_Home_Bias[Joint_Arm_Info[i][j].CanCH][Joint_Arm_Info[i][j].CanID]=Joint_Arm_Info[i][j].HALL_Homebias*Joint_Arm_Info[i][j].Gear_Rate*(M_PI/180.0/Joint_Arm_Info[i][j].To_SI_Unit);
                       }
       //for waist     
	for(j = 1; j <= JOINT_WAIST_NUM ;j++){
  	     Elmo_Home_Compen[Joint_Waist_Info[1][j].CanCH][Joint_Waist_Info[1][j].CanID]=Joint_Waist_Info[1][j].Offset_Home_Main*Joint_Waist_Info[1][j].Gear_Rate*Sign_Comp_Waist[j]*(M_PI/180.0/Joint_Waist_Info[1][j].To_SI_Unit);
  	     Elmo_Home_Timeout[Joint_Waist_Info[1][j].CanCH][Joint_Waist_Info[1][j].CanID]=1000*Joint_Waist_Info[1][j].HALL_Timeout;
  	     Elmo_Home_Vmove[Joint_Waist_Info[1][j].CanCH][Joint_Waist_Info[1][j].CanID]=Joint_Waist_Info[1][j].HALL_Vmove;
  	     Elmo_Home_Vsearch[Joint_Waist_Info[1][j].CanCH][Joint_Waist_Info[1][j].CanID]=Joint_Waist_Info[1][j].HALL_Vsearch;
  	     Elmo_Home_Bias[Joint_Waist_Info[1][j].CanCH][Joint_Waist_Info[1][j].CanID]=Joint_Waist_Info[1][j].HALL_Homebias*Joint_Waist_Info[1][j].Gear_Rate*(M_PI/180.0/Joint_Waist_Info[1][j].To_SI_Unit);
                       }
}

//20170107 tuigan
//int get_tuigan_length(double roll, double pitch, double *tuigan_L)
int get_tuigan_length(double roll,double pitch,double tuigan_L[2])
{
	const double	ax = 156.5;
	const double	ay = 62.0;
	const double	az = 213.5;
	const double	bx = 120.0;
	const double	by = 37.0;
	const double	L_Zero = 218.035548;
	double	AB_1[3];
	double	AB_2[3];
	
	pitch = -pitch;

	AB_1[0] = bx - ax*cos(pitch) + az*cos(roll)*sin(pitch) - ay*sin(pitch)*sin(roll);
	AB_1[1] = by - ay*cos(roll) - az*sin(roll);
	AB_1[2] = ax*sin(pitch) + az*cos(pitch)*cos(roll) - ay*cos(pitch)*sin(roll);

	AB_2[0] = bx - ax*cos(pitch) + az*cos(roll)*sin(pitch) + ay*sin(pitch)*sin(roll);
	AB_2[1] = ay*cos(roll) - by - az*sin(roll);
	AB_2[2] = ax*sin(pitch) + az*cos(pitch)*cos(roll) + ay*cos(pitch)*sin(roll);

	tuigan_L[0] = sqrt(AB_1[0] * AB_1[0] + AB_1[1] * AB_1[1] + AB_1[2] * AB_1[2]) - L_Zero;//Right
	tuigan_L[1] = sqrt(AB_2[0] * AB_2[0] + AB_2[1] * AB_2[1] + AB_2[2] * AB_2[2]) - L_Zero;//Left

	//printf("pitch = %f\troll = %f\tL_Right = %f\tL_Left = %f\n",pitch*ToDeg,roll*ToDeg,tuigan_L[0],tuigan_L[1]);
	return 1;
}

#ifdef JOINT_OFFSET
void joint_offset_init()
{
	int i,j;
	
	Joint_Offset[0][0] = 0.0;
	Joint_Offset[0][1] = 0.0;
	Joint_Offset[0][2] = 0.0;
	Joint_Offset[0][3] = 0.0;
	Joint_Offset[0][4] = 0.0;
	Joint_Offset[0][5] = 0.0;
	
	Joint_Offset[1][0] = 0.0;
	Joint_Offset[1][1] = 0.0;
	Joint_Offset[1][2] = 0.0;
	Joint_Offset[1][3] = 0.0;
	Joint_Offset[1][4] = 0.0;
	Joint_Offset[1][5] = 0.0;

	printf("\n[Offset]\tLeg1\tLeg2\tLeg3\tLeg4\tLeg5\tLeg6\n");
	for(i=0;i<2;i++)
	{
		if(i==0)
			printf("Right:\t");
		else
			printf(" Left:\t");
		for(j=0;j<6;j++)
		{
			printf("%3.1f\t",Joint_Offset[i][j]*57.3);
		}
		printf("Deg\n");
	}
}
#endif
//END OF THE FILE


