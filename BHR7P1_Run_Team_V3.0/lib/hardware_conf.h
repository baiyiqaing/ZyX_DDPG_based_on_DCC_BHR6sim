/***********************************************************************
 *  BHR BIPED CONTROL SOFTWARE UNDER Windows XP+Real_Time Extension    *  
 *	Filename:  hardware_conf.h                                              *
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
#ifndef H_HARDWARE_CONFIG_H
#define H_HARDWARE_CONFIG_H
#include <windows.h>
/***********/
#define G  9.8                		/* Gravity constant [m/s^2] */
#define ToRad  (M_PI  /  180.0) 	/* from degree to radian    */
#define ToDeg  (180.0 /  M_PI)  	/* from radian to degree    */

#define ToKgf     (1.0  / 9.8)      	/* from [N] to [kgf]     */
#define ToKgfcm  (100.0 / 9.8)      	/* from [Nm] to [kgf.cm] */
#define INT16  signed short
#define UINT16 unsigned short
#define INT32  signed long int
#define UINT32 unsigned long int

/************/
//#include <stdbool.h>  //bool variables  supported in c++
#define RIGHT 		1
#define LEFT  		2
#define LEG_NUM 	2
#define JOINT_NUM	6
#define ARM_NUM 	2
#define WAIST_NUM 	1
#define JOINT_ARM_NUM   7	
#define JOINT_Head_NUM  2	
#define JOINT_WAIST_NUM  3	
#define EOJ		0xFF

//#define RFDAT_BUF_NUM  5000000//20000000 = 160M, 9000000=72M
//#define RFDAT_BUF_NUM  8000000  //good for pingpong
//#define RFDAT_BUF_NUM    10000000	//=160M

//#define DTIME	0.00432//move to the after, but its value is determined by CONTROL_PERIOD in rtx_process.c
	/* servo sampling time [s] */
#define FREQ	(1.0 / DTIME)		/*  [1/s]                  */

#define TIME_INI	0.6             /*Time for initial posture*/  
#define NUM_STEP	6.0             /*Number of walking step*/
#define TIME_STEP	0.9             /*Time of one step*/  
#define NUM_RFDATA	36.0*(TIME_INI+(NUM_STEP+0.4)*TIME_STEP)*FREQ  /*Total data num*/

#define LENG_LEG	0.35//0.312//0.285//pingpong//0.285//0.315             /* Leg Length */  
#define FOOT_FORSNR	0.0             /* Foot Height*/
#define WIDTH_FOOT	0.16//0.17//0.16//0.15             /* Foot width */  
#define L           0.35//  0.312 //0.285 //0.285//0.315            /*inv_leg uses*/


#define L_YBACK 	0.12       //0.102      /* Foot back length */
#define L_YFRONT 	0.15      //0.128       /* Foot front length */
#define W_OUT   	0.0825//0.09             /* Foot out length */
#define W_IN    	0.0675//0.06             /* Foot in length */


#define K_AMP_TJL 1.0                 /*Motion AMP*/
#define K_AMP_TJA 1.0
#define K_AMP_TJH 1.0


// logical joint No
#if 0
enum JointNo{
	FOOT_Y = 1,    // 1
	FOOT_X,        // 2
    KNEE_X,        // 3
	HIP_X,	       // 4
	HIP_Y,	       // 5
	HIP_Z	       // 6
}
#endif

enum JointNo_ARM{
	ARM_1 = 1,	// 1
  	ARM_2,         	// 2
  	ARM_3,         	// 3
  	ARM_4,         	// 4
  	ARM_5,         	// 5
  	ARM_6,         	// 6
  	ARM_7,         	// 7
  	ARM_8,         	// 8
  	ARM_9,         	// 9
};

enum JointNo_Head{
	Head_Pitch = 1,	// 1
	Head_Yaw,  	// 2
	Head_3, 	// 3
};
enum JointNo_Waist{   //TUIGAN
	WAIST_YAW = 1,	// 1
	WAIST_ROLL,  	// 2
	WAIST_PITCH,
};

enum JointNo{
	HIP_Z = 1,	// 1
  	HIP_Y,         	// 2
  	HIP_X,          // 3
    KNEE_X_,         // 4
  	FOOT_X,         // 5
  	FOOT_Y          // 6
};


/* for status controll */
#define NOT_INITIALIZED	0xffff
#define CONTROL_ON	1
#define CONTROL_OFF	0


#define TRUE_1          1
#define FALSE_0         0


#define NCNT  8 //8  
#define FIRST   1 //CAN channel
#define SECOND  2 
#define THIRD   3
#define FOURTH  4
#define FIFTH   5
#define SIXTH   6


/****************************************
   Global variables for robot control
 ***************************************/

#ifdef	GLOBAL_DEFINE
	#define	Extern	/* define actual body */
#else
	#define	Extern	extern
#endif


Extern double DTIME;

Extern long   Now_Rfdat_Num;                        /* Planned data adress*/
Extern long   Max_Num_Rfdat_All; /*Total data num*/
Extern long   Max_Num_Rfdat_Sum;
Extern long   iFileStartPoint[10];
Extern long   Actual_File_Num;
Extern long   Last_File_Length[10];
Extern unsigned long Actual_File_Data_Ptr;

Extern double Ref_Head_Val[JOINT_Head_NUM + 1]; 

Extern double Refv_Head_Val[JOINT_Head_NUM + 1];

Extern double Ref_Waist_Val[JOINT_WAIST_NUM +1];
Extern double Rtf_Waist_Val[JOINT_WAIST_NUM +1];
Extern double Mdf_Waist_Val[JOINT_WAIST_NUM +1];

Extern double Ref_Arm_Val[ARM_NUM + 1][JOINT_ARM_NUM +1];
Extern double Rtf_Arm_Val[ARM_NUM + 1][JOINT_ARM_NUM +1];
Extern double Mdf_Arm_Val[ARM_NUM + 1][JOINT_ARM_NUM +1];

Extern double Refv_Arm_Val[ARM_NUM + 1][JOINT_ARM_NUM +1];

Extern double Ref_Val[LEG_NUM + 1][JOINT_NUM +1];  /* Planned joint angle */
Extern double Refv_Val[LEG_NUM + 1][JOINT_NUM +1]; /* Planned joint speed */
Extern double Ref_Ankle[LEG_NUM + 1][3];           /* Planned ankle position*/
Extern double Ref_Yzmp[3],Ref_Xzmp[3];         /* ZMP margin [1]:min; [2]:max*/
Extern int    Ref_Cnt_Sgnl[3];                     /* Planned support singal*/
Extern int    Ref_Timecntl[3];


Extern int Home_Count_Arm ;
Extern double Home_K[ARM_NUM + 1][JOINT_ARM_NUM +1];  /*for home on*/
//xt-07-5-21
//Extern double P1x,P1y,P1z,P2x,P2y,P2z,DPx,DPy,DPz;

Extern double Ref_Ankle_D2S[3][3];
Extern double D_Ref_Ankle[3];//delta of Ref_Ankle_D2S with Ref_Ankle
Extern double TIME_D2S;
Extern double dTHETA[3][6];//6-15
Extern double DTHETA[3][6];//6-15
Extern double D_ANKLE[3][3];//6-19
Extern int Park[3];//for add 
Extern int LEG_NO;
Extern int PM;
#define Nmax 70//70 // 5 //4  //20
#define TIME_D2S_MIN 0.01
#define K_DTHETA 0.0003 
Extern double T_Ankle[LEG_NUM + 1][3];               //defined by xt
Extern double TAD_Ankle[LEG_NUM + 1][3];               //defined by xt
Extern double Qhw;                               
Extern double Ref_Qendx[LEG_NUM+1];                             
Extern double Ref_Val_1[LEG_NUM + 1][JOINT_NUM +1];  
Extern double Rtf_Val_1[LEG_NUM + 1][JOINT_NUM +1];  
Extern double Mdf_Val_Pos[LEG_NUM + 1][JOINT_NUM + 1];
Extern double P_ZAnkle[3];                               
Extern int P_control;                               
Extern int Parkstart;



Extern double Init_Val[LEG_NUM + 1][JOINT_NUM +1];  /*Initial value*/
Extern double Init_Arm_Val[ARM_NUM + 1][JOINT_ARM_NUM +1];  /*Initial value*/
Extern double Init_Head_Val[JOINT_Head_NUM +1];  /*Initial value*/
Extern double Init_Waist_Val[JOINT_WAIST_NUM +1];  /*Initial value*/

Extern double Rtf_Val[LEG_NUM + 1][JOINT_NUM +1];  /* Real-time joint angle */
Extern double Mdf_Val[LEG_NUM + 1][JOINT_NUM +1];  /* Modified joint angle increase*/
Extern double Mdfv_Val[LEG_NUM + 1][JOINT_NUM +1]; /* Modified joint speed */
Extern double Dz_Val[LEG_NUM + 1];                 /* Foot change of Z-axiz */

Extern double Test_dz[LEG_NUM + 1]; //for test dz_foot
Extern double Test_now[LEG_NUM + 1][JOINT_NUM + 1];
Extern double Test_dankle[3][3];//07-9-16

Extern double Mdf_In[LEG_NUM+1][3];
Extern double Mdf_De[LEG_NUM+1][3];
Extern double Mdf_Val_Old[LEG_NUM + 1][JOINT_NUM +1];  /* Modified joint angle increase*/

Extern double	Ref_Qbody[3], Ref_Qvbody[3];
Extern double	Ref_Qybody[3], Ref_Qyvbody[3];

Extern double Nowq_Val[LEG_NUM + 1][JOINT_NUM +1];  /* Real-time joint angle */

Extern long   LastControlledTime;

Extern long   Cjoint_Arm[ ARM_NUM+1 ][ JOINT_ARM_NUM + 1];  /* 32bit encoder counter */
Extern double Joint_Arm[  ARM_NUM+1 ][ JOINT_ARM_NUM + 1];   /* joint angle [rad]or[m] */
Extern double Joint_Arm_O[  ARM_NUM+1 ][ JOINT_ARM_NUM + 1];   /* old joint angle [rad]or[m] */
Extern double Vjoint_Arm[ ARM_NUM+1 ][ JOINT_ARM_NUM + 1];  /* joint speed [rad/s]or[m/s] */
Extern double Out_Torque_Arm[ ARM_NUM+1 ][ JOINT_ARM_NUM + 1];   /* Torque */

Extern long   Cjoint_Head[ JOINT_Head_NUM + 1];  /* 32bit encoder counter */
Extern double Joint_Head[ JOINT_Head_NUM + 1];   /* joint angle [rad]or[m] */
Extern double Joint_Head_O[ JOINT_Head_NUM + 1];   /* old joint angle [rad]or[m] */
Extern double Vjoint_Head[ JOINT_Head_NUM + 1];  /* joint speed [rad/s]or[m/s] */
Extern double Out_Torque_Head[ JOINT_Head_NUM + 1];   /* Torque */
//added for waist

Extern long   Cjoint_Waist[WAIST_NUM+1][ JOINT_WAIST_NUM + 1];  /* 32bit encoder counter */
Extern double Joint_Waist[WAIST_NUM+1][ JOINT_WAIST_NUM + 1];   /* joint angle [rad]or[m] */
Extern double Joint_Waist_O[WAIST_NUM+1][ JOINT_WAIST_NUM + 1];   /* old joint angle [rad]or[m] */
Extern double Vjoint_Waist[WAIST_NUM][ JOINT_WAIST_NUM + 1];  /* joint speed [rad/s]or[m/s] */
Extern double Out_Torque_Waist[WAIST_NUM+1][ JOINT_WAIST_NUM + 1];   /* Torque */

Extern long   Cjoint[ LEG_NUM+1 ][ JOINT_NUM + 1];   /*32bit encoder counter*/
Extern double Joint[ LEG_NUM+1 ][ JOINT_NUM + 1];    /* joint angle */
Extern double Joint_O[ LEG_NUM+1 ][ JOINT_NUM + 1];  /* old joint angle */
Extern double Vjoint[ LEG_NUM+1 ][ JOINT_NUM + 1];    /* joint speed */
Extern double Out_Torque[ LEG_NUM+1 ][ JOINT_NUM +1]; /* Torque */

Extern double FootFT[ LEG_NUM + 1 + 2 ][6];   
Extern double FootFT_j[ LEG_NUM + 1 + 2 ][6];   /* Foot Force [N] & Torque [Nm] */
Extern double Gyro[3];                     /* Gyro sensor [rad/s]  */
Extern double Gsens[3];                    /* Acceleration sensor [rad] */
Extern double HFsens[3];                /* Hybrid Fibre sensor [mV] */
Extern double dPosition[6];

Extern double Gsen_Offset[3];		/* Gsens Offset just before walking [rad]  */
Extern double Gyro_Offset[3];		/* Gyro Offset just before walking [rad/s] */
Extern double Tr_Gsen_Offset[3];    //xt-07-9-7
Extern double Tr_Gyro_Offset[3];
Extern double Q_body_Ref[3];
Extern double Gyro_old[3];
Extern double Gyro_Filter[3];
Extern double Gyro1_Rec[3];


Extern double Q_body_old[6];
Extern double Q_body[6];        /* Body posture: [1] for qx, [2] for qy */
Extern double Qbody_Init[3];


//Extern double Q_Gyro[2][3];     /* Angle from gyro sensor [rad/s],[0][]:t-1;[1][]:t  */
//Extern double Q_Gsens[2][3];    /* Angle from Acceleration sensor [rad],[0][]:t-1;[1][]:t */
Extern double Q_foot[3][3],Cos_Qft[3][3],Sin_Qft[3][3];   /*  foot posture;[][1] for qx,[][2] for qy */

Extern double Force_Grnd[ LEG_NUM + 1 ][6];   /* Force and Torque in ground [1][]:right,[2][]:left*/
Extern double ZMP_Actl[LEG_NUM + 1][6];       /* [0][]:both feet; [][1]:xzmp;[][2]:yzmp */
Extern long Joint_Cur;


/// New parameters 
Extern double	Actl_Hip[3][3], Actl_Knee[3][3], Actl_Ankle[3][3]; 
//Actl_Ankle[1][0, 1, 2],: right foot position(x,y,z), Actl_Ankle[2][0, 1, 2],: left foot position
Extern double	Actl_Hip_B[3][3], Actl_Knee_B[3][3], Actl_Ankle_B[3][3];
Extern double	Actl_Hip_W[3][3], Actl_Knee_W[3][3], Actl_Ankle_W[3][3];

Extern double	IZMP_Actl[3];
Extern double	IZMP_ymargin[3][3], IZMP_xmargin[3][3];

Extern double COG_Body_W[3];
Extern double COGx_Link[LEG_NUM + 1][JOINT_NUM +1];  // Link center
Extern double COGy_Link[LEG_NUM + 1][JOINT_NUM +1];  // Link center
Extern double COGz_Link[LEG_NUM + 1][JOINT_NUM +1];  // Link center

Extern double	COP_Ymargin_W[LEG_NUM+1][3];
Extern double	COP_Xmargin_W[LEG_NUM+1][3];

Extern double TEST_A[3][7];

Extern double	Test_Time;
Extern double	Test_Time_Arm;

Extern double MinProcessTime,MaxProcessTime; /* spent time for calcuration[s]*/
Extern long CtrlMissCount;

Extern double Test_All[8];

//Extern int fdq; //device number

Extern int Scntl_On;
Extern int Home_On;
Extern BOOL home_on_flag;
Extern BOOL home_on_arm_flag;
Extern BOOL Home_On_Arm;
Extern BOOL Home_On_Begin; 
Extern int Walk_On;
Extern int Offset_On;
Extern int Sin_Test_On;
Extern int Check_Joint_On; //auto check joint

Extern int P_Ankfz_Switch;
Extern int P_Posfz_Switch;
Extern int P_Fotfz_Switch;

Extern int Servomotor1;     //Sevomotor1: 1 for on; 0 for off
Extern int P_Count[3];
Extern int P_Count2[3];


Extern int ControlFlag;
Extern int MinTick,MaxTick,BeginTick;
Extern double BASE_TICK;             /* [s] */

Extern int CheckCANNode[16];
Extern double Con_Val_Leg[LEG_NUM+1][JOINT_NUM+1]; //----send to DSP for pid
Extern double Con_Val_Arm[ARM_NUM+1][JOINT_ARM_NUM+1];
Extern double Con_Val_Waist[WAIST_NUM+1][JOINT_WAIST_NUM+1];
Extern double Con_Val_Head[JOINT_Head_NUM+1];

#if 1 //for elmo driver
Extern BOOL   CanCfg[7][9]; //[CanCH][CanID], actually used from[1][1] to [6][7]
Extern BOOL   ChannelCfg[7]; //maxim 6 chs
Extern double enc_val[7][9]; //current time
Extern double enc_val_1[7][9]; //last time
Extern double enc_val_2[7][9]; //last last time
Extern double Con_Val_Snd_CAN[7][9];
Extern double Con_Val_Snd_CAN_Last[7][9];

Extern double MotorRateCurrent[7][9]; //motor continous current
Extern double MotorCurrent[7][9];

Extern unsigned char dsp_search_home;
Extern int CANNodeCount;
Extern double DSP_PID_Kp[7][9]; //PID paramters of Joint 
Extern double DSP_PID_Kd[7][9];
Extern int Elmo_Home_Compen[7][9]; //Compensation of Joint Home Point 
Extern int Elmo_Home_Timeout[7][9]; // 
Extern int Elmo_Home_Vmove[7][9]; // 
Extern int Elmo_Home_Vsearch[7][9]; // 
Extern int Elmo_Home_Bias[7][9]; // 
Extern double Ref_Feedback_Error;
Extern int  Driver_CPara_DSP[7][9];
Extern int  elmo_node_count ;
Extern unsigned int Elmo_Configed_Low;  //sequence number 1-16
Extern unsigned int Elmo_Configed_High; //sequence number 17-32
Extern unsigned int Elmo_Configed_MSB; //sequence number 33-48
Extern BOOL elmo_search_home_flag;
Extern unsigned int Elmo_Linked_Low;  //sequence number 1-16
//Extern unsigned int Elmo_Linked_High; //sequence number 17-32
Extern unsigned int Elmo_Linked_High; //sequence number 33-48
Extern double Iner_Sensor[6];
#endif

Extern BOOL ctrl_wasit_leg_online_flag;

Extern double Gear_Rate_Joint[3][JOINT_NUM];  //moved from pid_process.c 2010-01-09
Extern double Gear_Arm_Rate_Joint[3][JOINT_ARM_NUM];
Extern double Gear_Head_Rate_Joint[JOINT_Head_NUM];
Extern double Gear_Waist_Rate_Joint[JOINT_WAIST_NUM];
Extern double Gear_Leg[3][6]; //moved from rtdata.c,2010-01-09

Extern int Plan_Count;
Extern int FirstCANError;  //temp
Extern int Land_Flag[3];  //temp
Extern double Joint_Min[20][2];
Extern double Joint_Max[20][2];
Extern float  Heading_Angle[5]; //AHRS
Extern float  Pitch_Angle[5];
Extern float  Roll_Angle[5];
Extern int   AHRS_ERR;
Extern float  Pitch_Gyro[1];
Extern float  Roll_Gyro[1];
Extern double AHRS_Offset[4];  //[0]->pitch, [1]->Roll
Extern double Q_body_Last[3];  //[1]->pitch, [2]->Roll
Extern int  DFZ_Count ;	
Extern double now_val[LEG_NUM + 1][JOINT_NUM +1];  /*current change of joint angle */
Extern float  Ball_Coordinate[3];
Extern float  Hand_Coordinate[3];
Extern unsigned int Cntl_Flag[5]; //control_flag for rtdata.c
Extern unsigned int Exec5[3]; //rtdata.c joint 5
Extern unsigned int Exec6[3]; //rtdata.c  joint 6
Extern int    COP_R_Count; 
Extern int    Leg_Init_Flag ;
Extern float y_posture[3][3];
Extern BOOL Home_On_Again;
typedef struct {
	int  JointNo;
	double Torq2Volt;   //Torque gain [(Torque)/V]
	int Test;           // No use
	double To_SI_Unit;  // Unit conversion constant
        double Gear_Rate;   // Reductin rate
        double Driver_Center;  //Voltage Center of Driver 
	int CanID;   //Number of DSPs 
	int DA_Gain;        // Direction of joint motion
        double Kp;          // PID 
        double Kd;          // PID
        int    CanCH;          //channel of can cards 
        double Offset_Home_Main;  //Offset of Joint Home Point; //---yzg
        double Joint_Limit_Min;  
        double Joint_Limit_Max;
 	int HALL_Timeout; // 
	int HALL_Vmove; //
	int HALL_Vsearch; //
	double HALL_Homebias ; //move joint to leave the target hall, unit:deg 
        int Reserved_Temp;   
        
} Joint_Information;

/* sorted joint information */
Extern Joint_Information Joint_Info[ LEG_NUM + 1 ][ JOINT_NUM + 1];
//Extern Joint_Information Joint_Arm_Info[ ARM_NUM + 1 ][ JOINT_ARM_NUM + 1];
Extern Joint_Information Joint_Arm_Info[ ARM_NUM + 1 ][ JOINT_ARM_NUM + 1];
Extern Joint_Information Joint_Head_Info[ JOINT_Head_NUM + 1];
Extern Joint_Information Joint_Waist_Info[WAIST_NUM+1][JOINT_WAIST_NUM + 1]; //added for waist



#endif


/*
#ifdef   HARDWARE_CONFIG_C
#define   EXTERN    //define
#else
#define   EXTERN    extern
#endif
*/
#ifdef	GLOBAL_DEFINE
#undef GLOBAL_DEFINE
#endif

Extern  void Initialize_Values(void);
Extern  int  Boot_Robot(void);
Extern  void Get_Sensor_Data( void );
//Extern  void Get_Plan_Data( void );
Extern  void Get_Plan_Data(int offline_FileID, unsigned long *DataPtr);
Extern  void Arm_Home(void);

//---yzg
Extern  void Con_Data_Trans(void);
//void CheckCANConfig(void);
Extern  void Check_Elmo_Config(void); //2010-1-6
Extern  void PID_Trans(void);
Extern  void Elmo_Home_Compen_Trans(void);
//void DSP_Driver_Center_Trans(void);
Extern  void Error_Check(int , int  , double ,double );
Extern  void Error_Check1( int , double , double );
Extern  void Error_Check3( int , double , double );  //for waist
Extern  void Joint_Limit_Trans(void);
Extern  void DSP_Driver_Parameters_Trans(void);
Extern  void Expand32( INT32 * pLVal ,  /* 32bit counter  */
	       UINT16 new_pos   /* new encoder data (16bit)  */
	       );

/*********** TUIGAN LQQ***************/
Extern int get_tuigan_length ( double roll,double pitch,double tuigan_L[2]);
Extern int Mode_Flag;
Extern double Ref_Joint_detaTPC[3][7];
Extern double Ref_Waist_Pitch;
Extern double Ref_Waist_Roll;

#define JOINT_OFFSET
#ifdef JOINT_OFFSET
Extern double Joint_Offset[2][6];
Extern void joint_offset_init();
#endif
//-----------------------
#if 0
/* clean up compiler control variables */
#ifdef	GLOBAL_DEFINE
#undef GLOBAL_DEFINE
#endif

#undef Extern

#endif


