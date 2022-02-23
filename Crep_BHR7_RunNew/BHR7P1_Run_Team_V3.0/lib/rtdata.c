/****************************************************
melhrp1.c:  library to initialization, drive_motor, shutdown, etc.

Modication history
01b, 06June99, Huang Modified.
01May03, Modified by Huang


Description
This is module provides routines that can

--Base from /cntlwg_tt_batt/lib/rtdata.c
--Change Point:
 (1) Let Force_Grnd[1][2] + Force_Grnd[2][2] == Weight of humanoid, contact more effective;
 (2) Redefine:	IZMP_xmargin[3][3], IZMP_ymargin[3][3] are actual margin on line for ZMP control,
		COP_xmarginR[3], COP_xmarginL[3], COP_ymarginR[3]COP_ymarginL[3] for compliance control;
 (3) Compute IZMP_Actl[3];
 (4) Investigate ZMP_control for double support;
 (5) Experiment: Taiji, Walk
 (6) Abosulate Coordinate System O-XYZ fixed at Hip (0, 0, 0: Center of the line between the two cross points of waist joints, and )
 (7) Ankle compliance is related to ankle position, Force/Torque sensor data don't need rotaion, so Let Q_foot[i][i] = 0.0; 
 (8) Real time position adjustment like initial position must be added.
 (9) ZMP_Actl=Ref_Ankle+M/F and COP_margin must be corrected



 Suggestion:	(1) Let Joint[][]/gear_rate, then all gear_rate only appear at encolder reading;

 Note: IZMP_Act[2] : The relationship between IZMP_Actl and XZMP_Margin and YZMP_Margin is key to IZMP control, especially at double support phase: more investigate;


********************************************************/


#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "..\lib\hardware_conf.h"
//#include "..\control.h"
#include "..\lib\Tra_Generate.h"
#include "..\control.h"

#define  M_PI 3.14159265358979
#pragma warning(disable : 4244)
#pragma warning(disable : 4305)

//#define USE_ZMP_CONTROL
//#define USE_ANKLE_POSTURE_CONTROL
//#define USE_HIP_POSTURE_CONTROL
//#define USE_DZ_CONTROL
//#define USE_DANKLE_CONTROL

#define	WEIGHT	        40.7*9.8//43.0*9.8//47.5*9.8	/*Humanoid Weight*/
                        //battery for Huitong3 is 9.0 kg
#define FZ_Weight       0.99*WEIGHT
//if calibrate IFS sensor, we set FZ_Mins as 0.01.

#define FZ_Mins	      0.06*WEIGHT//0.05//80.0//30.0//100.0             /* Into supprot 9M3D:120*/

#define FZ_MINS_DZ      0.2*WEIGHT//2.0//80.0//120.0//200.0             /* Into supprot for dz*/
#define FZ_MAX_DZ      0.3*WEIGHT//2.0//80.0//120.0//200.0             /* Into supprot for dz*/
#define FZ_Maxs	       0.7*WEIGHT//0.8//600.0//550//600.0//450.0//500.0//580//500.0             /* Into single_support*/

#define FZ_CONT		1.0*WEIGHT

#define	FZ_DZ	        0.0//0.05*WEIGHT//0.0//
#define	DFZ_ANK	        -0.5*WEIGHT //-0.1*WEIGHT//0.0//80.0
#define	DFZ_BDY	        0.05*WEIGHT// 0.1*WEIGHT//0.0//80.0
#define DFZ_SWITCH	0.1*WEIGHT

#define K_RETURN	-2.0*0.5*0.1*0.005//-0.2*2.0*2.0*2.0*4.0*0.003*2.5//0.048 /* Return Planned pattern */
#define KS_RETURN	1.5//2.0*4.0*0.2//0.2//0.4//0.5//
#define KTMP		1.5//2.0//1.0

#define MAX_MDF_LIMIT   0.05 // about 3 degree,09-07-27
#define QBODY_LIMIT    8.0*M_PI/180.0// about 3 degree,09-07-27 meng

////////////////////////////////////////
/************xt-07-6-19**************/
#define K_ANKLE         -0.04//2.0*4.0*5.0*0.001     //-0.0003
#define K_RETURN_ANKLE  -0.04//2.0*4.0*5.0*0.001     //-0.0003,-0.001 
#define K_Mdf_Fif_Joint  2.0*1.0//1.0  //07-10-3
#define K_Mdf_Six_Joint  2.0*1.0//1.0  //07-10-3
#define K_Now_Val        3.0*1.0//3.0    //07-10-5
#define Max_dankle	 2.0*0.00002
double dz_ref_ankle[3];
double d_ankle[3][3];   //postion error of ankle between Ref and Actl
double now_val_pos[LEG_NUM + 1][JOINT_NUM +1];  /*change of joint angle 5,6 by 2,3 */
/////////////////////////////////////////


#define KCOMP_DZ	0.0001//0.00005//0.00015//0.00001         /* Coefficient of foot compliance */
#define K_RETURN_DZ	-2.0*0.05//2.0*1.5*3.0*0.006 //-0.002//-0.001//-0.01//-0.005//-0.002             /* Return Planned pattern */
#define KS_RETURN_DZ	 3.0*0.2

#define K_RETURN_PS	-0.1//2.0*2.0*8.0*0.003//-0.002//-0.001//-0.01//-0.005//-0.002             /* Return Planned pattern */
#define Max_Dz_Step	0.001//0.00004//0.00002//0.0001
#define Min_Dz_Step	-0.0001//-0.000015//-0.000005//-0.00006// -0.000007//or -0.000015 for land fast
#define K_05	        0.5             /* 0.5 */
#define KCOM_ANKL	2.0*K_05*0.00125//5.0*0.25*0.001//0.0005//     /* Cofficent of ankle compliance */
#define KCOM_ANKL_F	10.0//5.0*2.0//0.0005//     /* Cofficent of ankle compliance */
#define	KCOM_ANKL_I	0.0//(for Test)/0.000005(Normal)//0.00001//-0.001//0.0006//0.000025
#define	KCOM_ANKL_TT	0.0//-0.001//0.0006//0.000025

#define R_YZMP	        0.01             /* Stable margin of YZMP */
#define R_XZMP	        0.01 // 0.01             /* Stable margin of XZMP */
//#define RM_XZMP 0.05             /*Stability Margin*/

#define QGSEN_ER        0.2*3.14157/180.0             /*Gsens tolerate error*/
#define Q_MAX           0.05
#define OMEGA	        1.0//2.0//2.5//0.5//0.1//12.0             /* Posture estimation filter cut off[Hz]  */

#define OFF_XZMP        0.0//0.005         
#define OFF_YZMP        0.0//0.01
#define	Maxpos_Step	0.01//2.0*2.0*5.0*0.00005//0.00002
#define	Maxank_Step	2.0*5.0*0.0001//0.00005

#define K_Amp_ZMP_R	1.0
#define K_Amp_ZMP_L	1.0


double BVL_X[2] = {0.00, 0.00};            // Int 0.005
double BVL_Y[2] = {0.00, 0.00};            // Int 5.0


double K_HIPX_PI[2] = {0.0, 0.0}; //{0.8, 0.8}; 	
double K_HIPY_PI[2] = {0.0, 0.0};//{0.9, 0.9};
double K_HIPX_IN[2] = {0.00005, 0.00005};
double K_HIPY_IN[2] = {0.000, 0.000};
double K_HIPX_DE1[2] = {0.0, 0.0};//{0.1, 0.1};
double K_HIPY_DE1[2] = {0.0, 0.0};
double K_HIPX_DE2[2] = {0.0, 0.0};//{0.08, 0.08};
double K_HIPY_DE2[2] = {0.0, 0.0};

#define K_RETURN_BPS 	-0.001


double nowv_val[LEG_NUM + 1][JOINT_NUM +1];  /*current change of joint angle */

double now_knee[LEG_NUM+1][3];             /*desired current knee position*/
double dz_foot[LEG_NUM+1];                 /* current foot change in Z-axis */
double xwaist[LEG_NUM+1] = { 0.0, 0.5*WIDTH_FOOT, -0.5*WIDTH_FOOT };
double q_body_real[4];		/* estimated body posture, used at rtdata.c */


double	KFORCE_AD[3] = {0.0, 1.0, 1.0};


/*  new variable */
//#define	WEIGHT		76.0*9.8	/*Humanoid Weight*/
#define	WIDTH_WAIST	0.16//0.16//0.15//0.18
#define	WIDTH_IN_FT	0.06//0.072//0.725//0.0675//0.06//       /* Foot sole width inside */
#define	WIDTH_OT_FT	0.10//0.098//0.775//0.0825//0.09//      /* Foot sole width outside */
#define	LENGTH_FR_FT	0.1540//0.128//0.127             /* Foot sole width inside */
#define	LENGTH_BK_FT	0.114//0.102//0.103             /* Foot sole width outside */    
#define	HIGH_FOOT	0.129//0.126//0.115//0.15             /* Foot Height*/

#define	LFOOT_IN		sqrt(HIGH_FOOT*HIGH_FOOT+ WIDTH_IN_FT*WIDTH_IN_FT)
#define	LFOOT_OT	sqrt(HIGH_FOOT*HIGH_FOOT+ WIDTH_OT_FT*WIDTH_OT_FT)
#define	LFOOT_FR	sqrt(HIGH_FOOT*HIGH_FOOT+ LENGTH_FR_FT* LENGTH_FR_FT)
#define	LFOOT_BK	sqrt(HIGH_FOOT*HIGH_FOOT+ LENGTH_BK_FT* LENGTH_BK_FT)

#define	Q_SOLE_FR	atan(LENGTH_FR_FT/HIGH_FOOT);
#define	Q_SOLE_BK	atan(LENGTH_FR_BK/HIGH_FOOT);
#define	Q_SOLE_IN	atan(WIDTH_IN_FT/HIGH_FOOT);
#define	Q_SOLE_OT	atan(WIDTH_OT_FT/HIGH_FOOT);

#define LFT_FB          0.05//0.03

#define	IZMP_YMR	0.02//0.045//0.035//0.02
#define	IZMP_XMR_IN	0.01//0.02//0.02//0.015   10-12
#define	IZMP_XMR_OT	0.02//0.045//0.035//0.015  10-12
#define IZMP_MR_RTN     0.0
#define COP_R       	0.01//0.01

double	Sin_Joint[LEG_NUM+1][JOINT_NUM+1];
double	Cos_Joint[LEG_NUM+1][JOINT_NUM+1];
double	Sin_Joint34[LEG_NUM+1], Cos_Joint34[LEG_NUM+1];
double	Sin_Joint345[LEG_NUM+1], Cos_Joint345[LEG_NUM+1];
double	Sin_Qbody[3], Cos_Qbody[3];
double	Sin_RfQbody[3], Cos_RfQbody[3];


double	COP_ymarin[3] = {0.0, -0.08, 0.10};//;{0.0, -0.2, 0.2}
double	COP_xmarginR[3] = {0.0, -0.05, 0.03};//{0.0, -0.05, 0.08};	//Is the COP_margin is related to ankle joint?
double	COP_xmarginL[3] = {0.0, -0.03, 0.05};//{0.0, -0.08, 0.05};
double	COP_xmargin[LEG_NUM+1][3];
/////////////////////////////////////////////////////////////////////

#if 1  // Pingpong Huitong
double	MASS_ARM[8] ={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0};
//double	MASS_ARM[8] ={0.0, 1.083, 0.703, 1.267, 0.416, 0.819, 0.094,0.0};
//double	IXX_ARM[7]  ={0.0, 0.003377075, 0.001249151, 0.00388922,                            0.000447455, 0.002348308, 0.00007669};
//double	IYY_ARM[7]  ={0.0, 0.002250385, 0.00102095, 0.003552,                            0.0004823,0.002018,0.0000598};
double	XP_ARM[8] = {0.0, -0.05589, 0.00204, 0.000391, 0.00415, 0.014631, 0.01434};
double	YP_ARM[8] = {0.0,  -0.025772, -0.009391,-0.002451,0.0,0.0000086,-0.000029};
double	ZP_ARM[8] = {0.0,  0.00765,-0.019434, -0.127483,-0.02377, -0.12796, -0.004592 };

double	MASS_LEG[7]={0.0,   1.26,  1.739,  6.8,  3.437, 0.927, 1.622};
//double	IXX_LEG[7] ={0.65276, 0.01327, 0.0016317, 0.044644, 0.055585, 0.0039129, 0.01562};
//double	IYY_LEG[7] ={0.74351, 0.01104,0.001889, 0.05079, 0.06007, 0.003309, 0.01315};
double  MASS_BODY[2] ={0.0, 18.7};// {0.0, 31.0};        //no battery 18.5+8.5(arm and head)
//double	COG_BODY[3] ={0.000011, 0.024995, 0.4435};// {0.0, -0.01, 0.40};  
double	COG_BODY[3] ={0.000011, 0.0, 0.50};//{0.000011, -0.024995, 0.4435};// {0.0, -0.01, 0.40};  
double	XP_LEG[7] = {0.0,   -0.0,       0.008020, 0.011757, 0.008457,     0.003172,  0.003872};
double	YP_LEG[7] = {0.0,  -0.000758, -0.003959, 0.021050, -0.016899,   -0.000463, -0.009141};
double	ZP_LEG[7] = {0.0,   0.073040,  -0.000039,-0.161002, -0.134713,   -0.000138, -0.077005};

#endif


double rtq_body[3];

double COGx_Link_W[LEG_NUM + 1][JOINT_NUM +1];  // Link center
double COGy_Link_W[LEG_NUM + 1][JOINT_NUM +1];  // Link center
double COGz_Link_W[LEG_NUM + 1][JOINT_NUM +1];  // Link center


double Db_Cmp_Margin[5];


double XP_LEG_Link[LEG_NUM + 1][JOINT_NUM +1];

double Joint_Hip_Inv[LEG_NUM + 1][JOINT_NUM +1];


double Wq_Mtrc[3];
double Wi_Matric[3][3];

#define  ALFA  0.8603  //5Hz, T=4.8ms//cxc

void Init_Q_body(void);  //call interface between files
void Get_Q_body(void);	 //call interface between files
void Get_Mdf_Data(void ); //call interface between files
void Imag_ZMP(void);
void Ankle_Cmpl_Db(void);
void Actl_Ankle_Position(void);
void Matric_Wq(void);

void Init_Q_body(void)
{
        int i;

        for(i = 0; i < 3; i++){

	  Q_body[i] = 0.0;//Qbody_Init[i];//0.0;
	  Q_body_Last[i] = 0.0;//Qbody_Init[i];//0.0;
	  rtq_body[i] = 0.0;
	  Q_body_old[i] = 0.0;//Qbody_Init[i];//0.0;

	  Q_body_Ref[i] = 0.0;  
      Gyro_Filter[i] = 0.0;
	  Gyro_old[i] = 0.0;
       Gyro1_Rec[i]=0.0;
	}        
}

#if 1  //AHRS
void Get_Q_body(void)
{
     int i = 0;

//     Pitch_Angle[1]=(1-ALFA)*Pitch_Angle[0]+ALFA*Pitch_Angle[1];
//     Roll_Angle[1]=(1-ALFA)*Roll_Angle[0]+ALFA*Roll_Angle[1]; 

//    Pitch_Angle[1]=(1.0-ALFA)*(Pitch_Angle[0]-AHRS_Offset[0])+ALFA*Pitch_Angle[1];
    //Roll_Angle[1]=(1.0-ALFA)*(Roll_Angle[0]-AHRS_Offset[1])+ALFA*Roll_Angle[1];

//   Pitch_Angle[1]=(1.0-ALFA)*(Pitch_Angle[0])+ALFA*Pitch_Angle[1];
	Pitch_Angle[1]=Pitch_Angle[0]-AHRS_Offset[0];
	Roll_Angle[1]=Roll_Angle[0]-AHRS_Offset[1];
//	printf("\nRoll_Angle[1]=%f",Roll_Angle[1]);
     
     Q_body[1]=Pitch_Angle[1]-Qbody_Init[1];
     Q_body[2]=Roll_Angle[1]-Qbody_Init[2];

     for(i=1;i<=2;i++)
	{
	 if(Q_body[i] < -QBODY_LIMIT)
		{
	 	Q_body[i] = -QBODY_LIMIT;
		Pitch_Angle[1] = -QBODY_LIMIT;//cxc
		Roll_Angle[1] = -QBODY_LIMIT;
		}
	 if(Q_body[i] > QBODY_LIMIT)
		{
	 	Q_body[i] = QBODY_LIMIT;
		Pitch_Angle[1] = QBODY_LIMIT;
		Roll_Angle[1] = QBODY_LIMIT;
		}
	}
//comment out
//     Q_body[1]=-Qbody_Init[1];
//     Q_body[2]=-Qbody_Init[2];
//cxc
    Gyro_Filter[1] =(1-ALFA)*(Pitch_Gyro[0]-AHRS_Offset[2])+ALFA*Gyro_Filter[1] ; //pitch
    Gyro_Filter[2] =(1-ALFA)*(Roll_Gyro[0]-AHRS_Offset[3])+ALFA*Gyro_Filter[2] ; //roll
}
#endif

void Ankle_Cmpl_Db(void)
{
	int i;
	double ky_comp[3], kx_comp[3];
	//double Db_Cmp_Margin[5];
		
  if (Actl_Ankle_W[1][1] > (Actl_Ankle_W[2][1]+LFT_FB) ) {

	if ( IZMP_Actl[1] >= Db_Cmp_Margin[1] ){
	   for(i = 1; i <= LEG_NUM ; i++){
	        ky_comp[i] = 1.0;
		kx_comp[i] = 1.0;
	        if ( ZMP_Actl[i][2] > COP_Ymargin_W[i][2] ) {		//compliance control
		        if( i == 1 ){
			      ky_comp[i] = 2.0*(IZMP_ymargin[i][2]-IZMP_Actl[2])/(IZMP_ymargin[i][2]-IZMP_ymargin[i][1]);
			      ky_comp[i] = ky_comp[i]*ky_comp[i];
			}
			now_val[i][5] = ky_comp[i]*KCOM_ANKL*(ZMP_Actl[i][2]-COP_Ymargin_W[i][2]);
		}
		else if (ZMP_Actl[i][2] < COP_Ymargin_W[i][1] ) {		//compliance control
                        now_val[i][5] = ky_comp[i]*KCOM_ANKL*(ZMP_Actl[i][2]-COP_Ymargin_W[i][1]);
		}
		else {					//return to planned pattern
			now_val[i][5] = K_RETURN*KS_RETURN*Mdf_Val[i][5];
		}

		if ( ZMP_Actl[i][1] > COP_Xmargin_W[i][2] ) {		//compliance control
		        if( i == 1 ){
			      kx_comp[i] = 2.0*(IZMP_xmargin[i][2]-IZMP_Actl[1])/(IZMP_xmargin[i][2]-IZMP_xmargin[i][1]);
			      kx_comp[i] = kx_comp[i]*kx_comp[i];
			}
			now_val[i][6] = -kx_comp[i]*KCOM_ANKL*(ZMP_Actl[i][1]-COP_Xmargin_W[i][2]);
		} 
		else if ( ZMP_Actl[i][1] < COP_Xmargin_W[i][1] ) {	//compliance control
			now_val[i][6] = -kx_comp[i]*KCOM_ANKL*(ZMP_Actl[i][1]-COP_Xmargin_W[i][1]);
		}
		else {					//return to planned pattern
			now_val[i][6] = K_RETURN*KS_RETURN*Mdf_Val[i][6];
		}
	   }
	}

	else if ( IZMP_Actl[1] <= Db_Cmp_Margin[3] ){

	   for(i = 1; i <= LEG_NUM ; i++){
	        ky_comp[i] = 1.0;
		kx_comp[i] = 1.0;
	        if ( ZMP_Actl[i][2] > COP_Ymargin_W[i][2] ) {		//compliance control
			now_val[i][5] = ky_comp[i]*KCOM_ANKL*(ZMP_Actl[i][2]-COP_Ymargin_W[i][2]);
		}
		else if (ZMP_Actl[i][2] < COP_Ymargin_W[i][1] ) {		//compliance control
		        if( i == 2 ){
			      ky_comp[i] = 2.0*(IZMP_Actl[2]-IZMP_ymargin[i][1])/(IZMP_ymargin[i][2]-IZMP_ymargin[i][1]);
			      ky_comp[i] = ky_comp[i]*ky_comp[i];
			}
                        now_val[i][5] = ky_comp[i]*KCOM_ANKL*(ZMP_Actl[i][2]-COP_Ymargin_W[i][1]);
		}
		else {					//return to planned pattern
			now_val[i][5] = K_RETURN*KS_RETURN*Mdf_Val[i][5];
		}

		if ( ZMP_Actl[i][1] > COP_Xmargin_W[i][2] ) {		//compliance control
			now_val[i][6] = -kx_comp[i]*KCOM_ANKL*(ZMP_Actl[i][1]-COP_Xmargin_W[i][2]);
		} 
		else if ( ZMP_Actl[i][1] < COP_Xmargin_W[i][1] ) {	//compliance control
		        if( i == 2 ){
			      kx_comp[i] = 2.0*(IZMP_Actl[1]-IZMP_xmargin[i][1])/(IZMP_xmargin[i][2]-IZMP_xmargin[i][1]);
			      kx_comp[i] = kx_comp[i]*kx_comp[i];
			}
			now_val[i][6] = -kx_comp[i]*KCOM_ANKL*(ZMP_Actl[i][1]-COP_Xmargin_W[i][1]);
		}
		else {					//return to planned pattern
			now_val[i][6] = K_RETURN*KS_RETURN*Mdf_Val[i][6];
		}
	   }
	}

	else {

	   for(i = 1; i <= LEG_NUM ; i++){
	       if ( ZMP_Actl[i][2] > COP_Ymargin_W[i][2] ) {		//compliance control
		        ky_comp[i] = 2.0*(IZMP_Actl[1]-Db_Cmp_Margin[2])/(Db_Cmp_Margin[4]-Db_Cmp_Margin[2]);
			ky_comp[i] = ky_comp[i]*ky_comp[i];
			now_val[i][5] = ky_comp[i]*KCOM_ANKL*(ZMP_Actl[i][2]-COP_Ymargin_W[i][2]);
		}
		else if (ZMP_Actl[i][2] < COP_Ymargin_W[i][1] ) {		//compliance control
		        ky_comp[i] = 2.0*(Db_Cmp_Margin[4]-IZMP_Actl[1])/(Db_Cmp_Margin[4]-Db_Cmp_Margin[2]);
			ky_comp[i] = ky_comp[i]*ky_comp[i];
                        now_val[i][5] = ky_comp[i]*KCOM_ANKL*(ZMP_Actl[i][2]-COP_Ymargin_W[i][1]);
		}
		else {					//return to planned pattern
			now_val[i][5] = K_RETURN*KS_RETURN*Mdf_Val[i][5];
		}

		if ( ZMP_Actl[i][1] > COP_Xmargin_W[i][2] ) {		//compliance control
		        kx_comp[i] = 2.0*(Db_Cmp_Margin[4]-IZMP_Actl[1])/(Db_Cmp_Margin[4]-Db_Cmp_Margin[2]);
			kx_comp[i] = kx_comp[i]*kx_comp[i];
			now_val[i][6] = -kx_comp[i]*KCOM_ANKL*(ZMP_Actl[i][1]-COP_Xmargin_W[i][2]);
		} 
		else if ( ZMP_Actl[i][1] < COP_Xmargin_W[i][1] ) {	//compliance control
		        kx_comp[i] = 2.0*(IZMP_Actl[1]-Db_Cmp_Margin[2])/(Db_Cmp_Margin[4]-Db_Cmp_Margin[2]);
			kx_comp[i] = kx_comp[i]*kx_comp[i];
			now_val[i][6] = -kx_comp[i]*KCOM_ANKL*(ZMP_Actl[i][1]-COP_Xmargin_W[i][1]);
		}
		else {					//return to planned pattern
			now_val[i][6] = K_RETURN*KS_RETURN*Mdf_Val[i][6];
		}
	   }
	}
   }

  
  if ( Actl_Ankle_W[1][1] < (Actl_Ankle_W[2][1]-LFT_FB) ){

	if ( IZMP_Actl[1] >= Db_Cmp_Margin[1] ){
	   for(i = 1; i <= LEG_NUM ; i++){
	        ky_comp[i] = 1.0;
		kx_comp[i] = 1.0;
	        if ( ZMP_Actl[i][2] > COP_Ymargin_W[i][2] ) {		//compliance control
			now_val[i][5] = ky_comp[i]*KCOM_ANKL*(ZMP_Actl[i][2]-COP_Ymargin_W[i][2]);
		}
		else if (ZMP_Actl[i][2] < COP_Ymargin_W[i][1] ) {		//compliance control
		        if( i == 1 ){
			      ky_comp[i] = 2.0*(IZMP_Actl[2]-IZMP_ymargin[i][1])/(IZMP_ymargin[i][2]-IZMP_ymargin[i][1]);
			      ky_comp[i] = ky_comp[i]*ky_comp[i];
			}
                        now_val[i][5] = ky_comp[i]*KCOM_ANKL*(ZMP_Actl[i][2]-COP_Ymargin_W[i][1]);
		}
		else {					//return to planned pattern
			now_val[i][5] = K_RETURN*KS_RETURN*Mdf_Val[i][5];
		}

		if ( ZMP_Actl[i][1] > COP_Xmargin_W[i][2] ) {		//compliance control
		        if( i == 1 ){
			      kx_comp[i] = 2.0*(IZMP_xmargin[i][2]-IZMP_Actl[1])/(IZMP_xmargin[i][2]-IZMP_xmargin[i][1]);
			      kx_comp[i] = kx_comp[i]*kx_comp[i];
			}
			now_val[i][6] = -kx_comp[i]*KCOM_ANKL*(ZMP_Actl[i][1]-COP_Xmargin_W[i][2]);
		} 
		else if ( ZMP_Actl[i][1] < COP_Xmargin_W[i][1] ) {	//compliance control
			now_val[i][6] = -kx_comp[i]*KCOM_ANKL*(ZMP_Actl[i][1]-COP_Xmargin_W[i][1]);
		}
		else {					//return to planned pattern
			now_val[i][6] = K_RETURN*KS_RETURN*Mdf_Val[i][6];
		}
	   }
	}

	else if ( IZMP_Actl[1] <= Db_Cmp_Margin[3] ){

	   for(i = 1; i <= LEG_NUM ; i++){
	        ky_comp[i] = 1.0;
		kx_comp[i] = 1.0;
	        if ( ZMP_Actl[i][2] > COP_Ymargin_W[i][2] ) {		//compliance control
		        if( i == 2 ){
			      ky_comp[i] = 2.0*(IZMP_ymargin[i][2]-IZMP_Actl[2])/(IZMP_ymargin[i][2]-IZMP_ymargin[i][1]);
			      ky_comp[i] = ky_comp[i]*ky_comp[i];
			}
			now_val[i][5] = ky_comp[i]*KCOM_ANKL*(ZMP_Actl[i][2]-COP_Ymargin_W[i][2]);
		}
		else if (ZMP_Actl[i][2] < COP_Ymargin_W[i][1] ) {		//compliance control
                        now_val[i][5] = ky_comp[i]*KCOM_ANKL*(ZMP_Actl[i][2]-COP_Ymargin_W[i][1]);
		}
		else {					//return to planned pattern
			now_val[i][5] = K_RETURN*KS_RETURN*Mdf_Val[i][5];
		}

		if ( ZMP_Actl[i][1] > COP_Xmargin_W[i][2] ) {		//compliance control
			now_val[i][6] = -kx_comp[i]*KCOM_ANKL*(ZMP_Actl[i][1]-COP_Xmargin_W[i][2]);
		} 
		else if ( ZMP_Actl[i][1] < COP_Xmargin_W[i][1] ) {	//compliance control
		        if( i == 2 ){
			      kx_comp[i] = 2.0*(IZMP_Actl[1]-IZMP_xmargin[i][1])/(IZMP_xmargin[i][2]-IZMP_xmargin[i][1]);
			      kx_comp[i] = kx_comp[i]*kx_comp[i];
			}
			now_val[i][6] = -kx_comp[i]*KCOM_ANKL*(ZMP_Actl[i][1]-COP_Xmargin_W[i][1]);
		}
		else {					//return to planned pattern
			now_val[i][6] = K_RETURN*KS_RETURN*Mdf_Val[i][6];
		}
	   }
	}

	else {
	   for(i = 1; i <= LEG_NUM ; i++){
	       if ( ZMP_Actl[i][2] > COP_Ymargin_W[i][2] ) {		//compliance control
		        ky_comp[i] = 2.0*(Db_Cmp_Margin[2]-IZMP_Actl[1])/(Db_Cmp_Margin[2]-Db_Cmp_Margin[4]);
			ky_comp[i] = ky_comp[i]*ky_comp[i];
			now_val[i][5] = ky_comp[i]*KCOM_ANKL*(ZMP_Actl[i][2]-COP_Ymargin_W[i][2]);
		}
		else if (ZMP_Actl[i][2] < COP_Ymargin_W[i][1] ) {		//compliance control
		        ky_comp[i] = 2.0*(IZMP_Actl[1]-Db_Cmp_Margin[4])/(Db_Cmp_Margin[2]-Db_Cmp_Margin[4]);
			ky_comp[i] = ky_comp[i]*ky_comp[i];
                        now_val[i][5] = ky_comp[i]*KCOM_ANKL*(ZMP_Actl[i][2]-COP_Ymargin_W[i][1]);
		}
		else {					//return to planned pattern
			now_val[i][5] = K_RETURN*KS_RETURN*Mdf_Val[i][5];
		}

		if ( ZMP_Actl[i][1] > COP_Xmargin_W[i][2] ) {		//compliance control
		        kx_comp[i] = 2.0*(Db_Cmp_Margin[2]-IZMP_Actl[1])/(Db_Cmp_Margin[2]-Db_Cmp_Margin[4]);
			kx_comp[i] = kx_comp[i]*kx_comp[i];
			now_val[i][6] = -kx_comp[i]*KCOM_ANKL*(ZMP_Actl[i][1]-COP_Xmargin_W[i][2]);
		} 
		else if ( ZMP_Actl[i][1] < COP_Xmargin_W[i][1] ) {	//compliance control
		        kx_comp[i] = 2.0*(IZMP_Actl[1]-Db_Cmp_Margin[4])/(Db_Cmp_Margin[2]-Db_Cmp_Margin[4]);
			kx_comp[i] = kx_comp[i]*kx_comp[i];
			now_val[i][6] = -kx_comp[i]*KCOM_ANKL*(ZMP_Actl[i][1]-COP_Xmargin_W[i][1]);
		}
		else {					//return to planned pattern
			now_val[i][6] = K_RETURN*KS_RETURN*Mdf_Val[i][6];
		}
	   }
	}
   }

  
  if ( Actl_Ankle_W[1][1] >= (Actl_Ankle_W[2][1]-LFT_FB) && Actl_Ankle_W[1][1] <= (Actl_Ankle_W[2][1]+LFT_FB) ) {

	if ( IZMP_Actl[1] > IZMP_xmargin[1][1] ){
	   for(i = 1; i <= LEG_NUM ; i++){
	        ky_comp[i] = 1.0;
		kx_comp[i] = 1.0;
	        if ( ZMP_Actl[i][2] > COP_Ymargin_W[i][2] ) {		//compliance control
		        if( i == 1 ){
			      ky_comp[i] = 2.0*(IZMP_ymargin[i][2]-IZMP_Actl[2])/(IZMP_ymargin[i][2]-IZMP_ymargin[i][1]);
			      ky_comp[i] = ky_comp[i]*ky_comp[i];
			}

			now_val[i][5] = ky_comp[i]*KCOM_ANKL*(ZMP_Actl[i][2]-COP_Ymargin_W[i][2]);
		}
		else if (ZMP_Actl[i][2] < COP_Ymargin_W[i][1] ) {		//compliance control
		        if( i == 1 ){
			      ky_comp[i] = 2.0*(IZMP_Actl[2]-IZMP_ymargin[i][1])/(IZMP_ymargin[i][2]-IZMP_ymargin[i][1]);
			      ky_comp[i] = ky_comp[i]*ky_comp[i];
			}
                        now_val[i][5] = ky_comp[i]*KCOM_ANKL*(ZMP_Actl[i][2]-COP_Ymargin_W[i][1]);
		}
		else {					//return to planned pattern
			now_val[i][5] = K_RETURN*KS_RETURN*Mdf_Val[i][5];
		}

		if ( ZMP_Actl[i][1] > COP_Xmargin_W[i][2] ) {		//compliance control
Test_All[0]=81;
		        if( i == 1 ){
			  //			      kx_comp[i] = 2.0*(IZMP_xmargin[i][2]-IZMP_Actl[1])/(IZMP_xmargin[i][2]-IZMP_xmargin[i][1]);
			      kx_comp[i] = 2.0*(IZMP_xmargin[1][2]-IZMP_Actl[1])/(IZMP_xmargin[1][2]-IZMP_xmargin[2][2]);
			      kx_comp[i] = kx_comp[i]*kx_comp[i];
			}
			now_val[i][6] = -kx_comp[i]*KCOM_ANKL*(ZMP_Actl[i][1]-COP_Xmargin_W[i][2]);
		} 
		else if ( ZMP_Actl[i][1] < COP_Xmargin_W[i][1] ) {	//compliance control
Test_All[0]=82;
			now_val[i][6] = -kx_comp[i]*KCOM_ANKL*(ZMP_Actl[i][1]-COP_Xmargin_W[i][1]);
		}
		else {					//return to planned pattern
			now_val[i][6] = K_RETURN*KS_RETURN*Mdf_Val[i][6];
		}
	   }
	}

	else if ( IZMP_Actl[1] <= IZMP_xmargin[2][2] ){
	   for(i = 1; i <= LEG_NUM ; i++){
	        ky_comp[i] = 1.0;
		kx_comp[i] = 1.0;
	        if ( ZMP_Actl[i][2] > COP_Ymargin_W[i][2] ) {		//compliance control
		        if( i == 2 ){
			      ky_comp[i] = 2.0*(IZMP_ymargin[i][2]-IZMP_Actl[2])/(IZMP_ymargin[i][2]-IZMP_ymargin[i][1]);
			      ky_comp[i] = ky_comp[i]*ky_comp[i];
			}
			now_val[i][5] = ky_comp[i]*KCOM_ANKL*(ZMP_Actl[i][2]-COP_Ymargin_W[i][2]);
		}
		else if (ZMP_Actl[i][2] < COP_Ymargin_W[i][1] ) {		//compliance control
		        if( i == 2 ){
			      ky_comp[i] = 2.0*(IZMP_Actl[2]-IZMP_ymargin[i][1])/(IZMP_ymargin[i][2]-IZMP_ymargin[i][1]);
			      ky_comp[i] = ky_comp[i]*ky_comp[i];
			}
                        now_val[i][5] = ky_comp[i]*KCOM_ANKL*(ZMP_Actl[i][2]-COP_Ymargin_W[i][1]);
		}
		else {					//return to planned pattern
			now_val[i][5] = K_RETURN*KS_RETURN*Mdf_Val[i][5];
		}

		if ( ZMP_Actl[i][1] > COP_Xmargin_W[i][2] ) {		//compliance control
Test_All[0]=83;

			now_val[i][6] = -kx_comp[i]*KCOM_ANKL*(ZMP_Actl[i][1]-COP_Xmargin_W[i][2]);
		} 
		else if ( ZMP_Actl[i][1] < COP_Xmargin_W[i][1] ) {	//compliance control
Test_All[0]=84;

	        if( i == 2 ){
			  //			      kx_comp[i] = 2.0*(IZMP_Actl[1]-IZMP_xmargin[i][1])/(IZMP_xmargin[i][2]-IZMP_xmargin[i][1]);
			      kx_comp[i] = 2.0*(IZMP_Actl[1]-IZMP_xmargin[2][1])/(IZMP_xmargin[2][1]-IZMP_xmargin[1][1]);
			      kx_comp[i] = kx_comp[i]*kx_comp[i];
			}
			now_val[i][6] = -kx_comp[i]*KCOM_ANKL*(ZMP_Actl[i][1]-COP_Xmargin_W[i][1]);
		}
		else {					//return to planned pattern
			now_val[i][6] = K_RETURN*KS_RETURN*Mdf_Val[i][6];
		}
	   }
	}

	else {
	   for(i = 1; i <= LEG_NUM ; i++){
		ky_comp[i]=1.0;
		kx_comp[i]=1.0;

	       if ( ZMP_Actl[i][2] > COP_Ymargin_W[i][2] ) {		//compliance control
		        ky_comp[i] = 2.0*(IZMP_ymargin[i][2]-IZMP_Actl[2])/(IZMP_ymargin[i][2]-IZMP_ymargin[i][1]);
			ky_comp[i] = ky_comp[i]*ky_comp[i];
			now_val[i][5] = ky_comp[i]*KCOM_ANKL*(ZMP_Actl[i][2]-COP_Ymargin_W[i][2]);
		}
		else if (ZMP_Actl[i][2] < COP_Ymargin_W[i][1] ) {		//compliance control
			ky_comp[i] = 2.0*(IZMP_Actl[2]-IZMP_ymargin[i][1])/(IZMP_ymargin[i][2]-IZMP_ymargin[i][1]);
			ky_comp[i] = ky_comp[i]*ky_comp[i];
                        now_val[i][5] = ky_comp[i]*KCOM_ANKL*(ZMP_Actl[i][2]-COP_Ymargin_W[i][1]);
		}
		else {					//return to planned pattern
			now_val[i][5] = K_RETURN*KS_RETURN*Mdf_Val[i][5];
		}

		if ( ZMP_Actl[i][1] > COP_Xmargin_W[i][2] ) {		//compliance control
			Test_All[0]=85;
		        if( i == 1 ){
			  //			      kx_comp[i] = 2.0*(IZMP_xmargin[i][2]-IZMP_Actl[1])/(IZMP_xmargin[i][2]-IZMP_xmargin[i][1]);
			      kx_comp[i] = 2.0*(IZMP_xmargin[1][2]-IZMP_Actl[1])/(IZMP_xmargin[1][2]-IZMP_xmargin[2][2]);
			      kx_comp[i] = kx_comp[i]*kx_comp[i];
			}
			now_val[i][6] = -kx_comp[i]*KCOM_ANKL*(ZMP_Actl[i][1]-COP_Xmargin_W[i][2]);
		} 
		else if ( ZMP_Actl[i][1] < COP_Xmargin_W[i][1] ) {	//compliance control
			Test_All[0]=86;
		       if( i == 2 ){
			  //			      kx_comp[i] = 2.0*(IZMP_Actl[1]-IZMP_xmargin[i][1])/(IZMP_xmargin[i][2]-IZMP_xmargin[i][1]);
			      kx_comp[i] = 2.0*(IZMP_Actl[1]-IZMP_xmargin[2][1])/(IZMP_xmargin[2][1]-IZMP_xmargin[1][1]);
			      kx_comp[i] = kx_comp[i]*kx_comp[i];
			}
			now_val[i][6] = -kx_comp[i]*KCOM_ANKL*(ZMP_Actl[i][1]-COP_Xmargin_W[i][1]);
		}
		else {	
		//return to planned pattern
			now_val[i][6] = K_RETURN*KS_RETURN*Mdf_Val[i][6];
		}
	   }
	}
   }
  

}

void Matric_Wq(void)
{
        double sin_wqt[3], cos_wqt[3];
	double sin_wq0, cos_wq0;
	double l_sincos, lsincos;
	double k_ne, k_mu;
	int i, j;

	if( Wq_Mtrc[1] == 0 && Wq_Mtrc[2] == 0 ){
	  	for(i = 0; i < 3; i++){
		        for(j = 0; j < 3; j++){
			        Wi_Matric[i][j] = 0.0;
			}
		}
		Wi_Matric[0][0] = 1.0;
		Wi_Matric[1][1] = 1.0;
		Wi_Matric[2][2] = 1.0;
	}

	else{

	        for(i = 1; i < 3; i++){
		        sin_wqt[i] = sin(Wq_Mtrc[i]);
	                cos_wqt[i] = cos(Wq_Mtrc[i]);
		}

		l_sincos = sqrt(1-sin_wqt[1]*sin_wqt[1]*sin_wqt[2]*sin_wqt[2]);
		lsincos = sqrt(sin_wqt[1]*sin_wqt[1]*cos_wqt[2]*cos_wqt[2]+
	                       cos_wqt[1]*cos_wqt[1]*sin_wqt[2]*sin_wqt[2]);

		k_ne = sin_wqt[1]*cos_wqt[2]/lsincos;
		k_mu = cos_wqt[1]*sin_wqt[2]/lsincos;

		sin_wq0 = lsincos/l_sincos;
		cos_wq0 = cos_wqt[1]*cos_wqt[2]/l_sincos;

		Wi_Matric[0][0] = cos_wq0+k_ne*k_ne*(1-cos_wq0);
		Wi_Matric[1][1] = cos_wq0+k_mu*k_mu*(1-cos_wq0);
		Wi_Matric[2][2] = cos_wq0;

		Wi_Matric[0][1] = k_ne*k_mu*(1-cos_wq0);
		Wi_Matric[0][2] = k_mu*sin_wq0;

		Wi_Matric[1][0] = k_ne*k_mu*(1-cos_wq0);
		Wi_Matric[1][2] =-k_ne*sin_wq0;

		Wi_Matric[2][0] =-k_mu*sin_wq0;
		Wi_Matric[2][1] = k_ne*sin_wq0;
	}
}


#if 1
void Actl_Ankle_Position(void)//new kinematic (peng)
{
/// Revised points:	(1) Link center must consder Tq (the rotation body-angle (qx_waist), not actual Q-body)
///		(2) Link position must consider Tq

/// Note: If consider link position at the coordinate sytem O'-X'Y'Z': after rotation (Tq) from Abosulate Coordinate System O-XYZ
//////// We can compute link positions without Tq and Sin_Qbody[i], Cos_Qbody[i]; 


///Coordinate Sytem O'-X'Y'Z': Hip (0, 0, 0) Center of the line between the two cross points of waist joints,
///X-axis:  the line between the two cross points of waist joints 


	int i, j, k;

	for(i = 0; i <= 2; i++){
	       for(j = 0; j <= 2; j++){
	            Actl_Hip_W[i][j] = 0.0;
		    Actl_Knee_W[i][j] = 0.0;
		    Actl_Ankle_W[i][j] = 0.0;
	       }
	}

#if 0
	Joint_Hip_Inv[1][3]= Joint[1][3]+Ref_Qbody[1]*Gear_Leg[2];  //Joint[i][3] including the body-angle, hip joint for kinemataics 
	Joint_Hip_Inv[2][3]= Joint[2][3]+Ref_Qbody[1]*Gear_Leg[2];
#endif

#if 1   //Temp_test
	Joint_Hip_Inv[1][3]= Joint[1][3];//+Ref_Qbody[1]*Gear_Leg[2];  //Joint[i][3] including the body-angle, hip joint for kinemataics 
	Joint_Hip_Inv[2][3]= Joint[2][3];//+Ref_Qbody[1]*Gear_Leg[2];
#endif
/*	Joint_Hip_Inv[1][3]= Joint[1][3];//+Ref_Qbody[1]*Gear_Leg[2];  //Joint[i][3] including the body-angle, hip joint for kinemataics 
	Joint_Hip_Inv[2][3]= Joint[2][3];//+Ref_Qbody[1]*Gear_Leg[2];
*/

	for(i = 1; i <= LEG_NUM; i++){
		for(j = 1; j <= JOINT_NUM; j++){
		  if (j==3){
		    Sin_Joint[i][j] = sin(Joint_Hip_Inv[i][j]/Gear_Leg[i][j-1]);
		    Cos_Joint[i][j] = cos(Joint_Hip_Inv[i][j]/Gear_Leg[i][j-1]);
		  }
		  else{
			Sin_Joint[i][j] = sin(Joint[i][j]/Gear_Leg[i][j-1]);
			Cos_Joint[i][j] = cos(Joint[i][j]/Gear_Leg[i][j-1]);
		  }
		}
		Sin_Joint34[i] = sin(Joint_Hip_Inv[i][3]/Gear_Leg[i][2]+Joint[i][4]/Gear_Leg[i][3]);
		Cos_Joint34[i] = cos(Joint_Hip_Inv[i][3]/Gear_Leg[i][2]+Joint[i][4]/Gear_Leg[i][3]);
		Sin_Joint345[i] = sin(Joint_Hip_Inv[i][3]/Gear_Leg[i][2]+Joint[i][4]/Gear_Leg[i][3]+Joint[i][5]/Gear_Leg[i][4]);
		Cos_Joint345[i] = cos(Joint_Hip_Inv[i][3]/Gear_Leg[i][2]+Joint[i][4]/Gear_Leg[i][3]+Joint[i][5]/Gear_Leg[i][4]);
#if 0
		Sin_Qbody[i] = sin(Q_body[i]);		// The body-angle of planned pattern should be used
		Cos_Qbody[i] = cos(Q_body[i]);
		Sin_RfQbody[i] = sin(Ref_Qbody[i]);		// The body-angle of planned pattern should be used
		Cos_RfQbody[i] = cos(Ref_Qbody[i]);
#endif
#if 1 //Temp_test
		Sin_Qbody[i] =0.0;// sin(Q_body[i]);		// The body-angle of planned pattern should be used
		Cos_Qbody[i] =1.0;// cos(Q_body[i]);
		Sin_RfQbody[i] =0.0;// sin(Ref_Qbody[i]);		// The body-angle of planned pattern should be used
		Cos_RfQbody[i] =1.0;// cos(Ref_Qbody[i]);
#endif
	}

	Actl_Hip[1][0] =  0.5*WIDTH_WAIST;//*cos(Qhw);		//Is Q_body[2] body angel in frontl plane?
	Actl_Hip[2][0] = -0.5*WIDTH_WAIST;//*cos(Qhw);
	Actl_Hip[1][1] =  0.0;//-0.5*WIDTH_WAIST*sin(Qhw);
	Actl_Hip[2][1] =  0.0;//+0.5*WIDTH_WAIST*sin(Qhw);
	Actl_Hip[1][2] =  0.0;
	Actl_Hip[2][2] =  0.0;


	for(i = 1 ; i <= LEG_NUM ;i++){
		Actl_Knee[i][0] = Actl_Hip[i][0]-LENG_LEG*Sin_Joint[i][1]*Sin_Joint[i][3]-LENG_LEG*Cos_Joint[i][1]*Sin_Joint[i][2]*Cos_Joint[i][3];
		Actl_Knee[i][1] = Actl_Hip[i][1]+LENG_LEG*Cos_Joint[i][1]*Sin_Joint[i][3]-LENG_LEG*Sin_Joint[i][1]*Sin_Joint[i][2]*Cos_Joint[i][3];
		Actl_Knee[i][2] = Actl_Hip[i][2]-LENG_LEG*Cos_Joint[i][2]*Cos_Joint[i][3];	
	}

	for(i = 1 ; i <= LEG_NUM ;i++){
		Actl_Ankle[i][0] = Actl_Hip[i][0]+LENG_LEG*(-Sin_Joint[i][1]*Sin_Joint[i][3]-Cos_Joint[i][1]*Sin_Joint[i][2]*Cos_Joint[i][3]
			               +Cos_Joint[i][1]*Sin_Joint[i][2]*Sin_Joint[i][3]*Sin_Joint[i][4]
						   -Sin_Joint[i][1]*Cos_Joint[i][3]*Sin_Joint[i][4]
						   -Sin_Joint[i][1]*Sin_Joint[i][3]*Cos_Joint[i][4]
						   -Cos_Joint[i][1]*Sin_Joint[i][2]*Cos_Joint[i][3]*Cos_Joint[i][4]);
		Actl_Ankle[i][1] = Actl_Hip[i][1]+LENG_LEG*(Cos_Joint[i][1]*Sin_Joint[i][3]-Sin_Joint[i][1]*Sin_Joint[i][2]*Cos_Joint[i][3]
			               +Sin_Joint[i][1]*Sin_Joint[i][2]*Sin_Joint[i][3]*Sin_Joint[i][4]
						   +Cos_Joint[i][1]*Cos_Joint[i][3]*Sin_Joint[i][4]
						   +Cos_Joint[i][1]*Sin_Joint[i][3]*Cos_Joint[i][4]
						   -Sin_Joint[i][1]*Sin_Joint[i][2]*Cos_Joint[i][3]*Cos_Joint[i][4]);
		Actl_Ankle[i][2] = Actl_Hip[i][2]-LENG_LEG*Cos_Joint[i][2]*(Cos_Joint[i][3]+
			           Cos_Joint[i][3]*Cos_Joint[i][4]-Sin_Joint[i][3]*Sin_Joint[i][4]);	
	}

	for(i = 1; i <= LEG_NUM; i++){			//Position at Body Coordiante System
		Actl_Hip_B[i][0] =  Actl_Hip[i][0]*Cos_RfQbody[2]+Actl_Hip[i][2]*Sin_RfQbody[2];
		Actl_Hip_B[i][1] =  Actl_Hip[i][1];
		Actl_Hip_B[i][2] = -Actl_Hip[i][0]*Sin_RfQbody[2]+Actl_Hip[i][2]*Cos_RfQbody[2];

		Actl_Knee_B[i][0] =  Actl_Knee[i][0]*Cos_RfQbody[2]+Actl_Knee[i][2]*Sin_RfQbody[2];
		Actl_Knee_B[i][1] =  Actl_Knee[i][1];
		Actl_Knee_B[i][2] = -Actl_Knee[i][0]*Sin_RfQbody[2]+Actl_Knee[i][2]*Cos_RfQbody[2];

		Actl_Ankle_B[i][0] =  Actl_Ankle[i][0]*Cos_RfQbody[2]+Actl_Ankle[i][2]*Sin_RfQbody[2];
		Actl_Ankle_B[i][1] =  Actl_Ankle[i][1];
		Actl_Ankle_B[i][2] = -Actl_Ankle[i][0]*Sin_RfQbody[2]+Actl_Ankle[i][2]*Cos_RfQbody[2];		
	}

	//	Q_body[1]=0.0;
	//Q_body[2]=0.0;
	Wq_Mtrc[1] = 0.0;//Q_body[1]-Ref_Qbody[1];
	Wq_Mtrc[2] = 0.0;//Q_body[2]-Ref_Qbody[2];
	Matric_Wq();	
        for(i = 1; i <= LEG_NUM; i++){			//Position at World Coordiante System
	        for(k = 0; k <= 2; k++){
		       for(j = 0; j <= 2; j++){
		            Actl_Hip_W[i][k]   = Actl_Hip_W[i][k]  + Wi_Matric[k][j]*Actl_Hip_B[i][j];
			    Actl_Knee_W[i][k]  = Actl_Knee_W[i][k] + Wi_Matric[k][j]*Actl_Knee_B[i][j];
			    Actl_Ankle_W[i][k] = Actl_Ankle_W[i][k]+ Wi_Matric[k][j]*Actl_Ankle_B[i][j];
		      }
		}
	}


/*
	IZMP_xmargin[1][1] = Actl_Ankle[1][0]-LFOOT_IN*sin( Q_SOLE_IN+Joint[1][6]/Gear_Leg[5])+IZMP_MR;
	IZMP_xmargin[1][2] = Actl_Ankle[1][0]+LFOOT_OT*sin(Q_SOLE_OT-Joint[1][6]/Gear_Leg[5])-IZMP_MR;
	IZMP_xmargin[2][1] = Actl_Ankle[2][0]-LFOOT_OT*sin( Q_SOLE_OT+Joint[2][6]/Gear_Leg[5])+IZMP_MR;
	IZMP_xmargin[2][2] = Actl_Ankle[2][0]+LFOOT_IN*sin( Q_SOLE_IN-Joint[2][6]/Gear_Leg[5])-IZMP_MR;
  	for(i = 1; i <= LEG_NUM ; i++){
		IZMP_ymargin[i][1] = Actl_Ankle[i][1]-LFOOT_BK*sin( Q_SOLE_FR-Joint[i][5]/Gear_Leg[4])+IZMP_MR;
		IZMP_ymargin[i][2] = Actl_Ankle[i][1]+LFOOT_FR*sin( Q_SOLE_FR+Joint[i][5]/Gear_Leg[4])-IZMP_MR;
		COP_xmargin[1][i] = COP_xmarginR[i];
		COP_xmargin[2][i] = COP_xmarginL[i];
   	}
*/
	IZMP_xmargin[1][1] = Actl_Ankle_W[1][0]-WIDTH_IN_FT+IZMP_XMR_IN;
	IZMP_xmargin[1][2] = Actl_Ankle_W[1][0]+WIDTH_OT_FT-IZMP_XMR_OT;
	IZMP_xmargin[2][1] = Actl_Ankle_W[2][0]-WIDTH_OT_FT+IZMP_XMR_OT;
	IZMP_xmargin[2][2] = Actl_Ankle_W[2][0]+WIDTH_IN_FT-IZMP_XMR_IN;
  	for(i = 1; i <= LEG_NUM ; i++){
		IZMP_ymargin[i][1] = Actl_Ankle_W[i][1]-LENGTH_BK_FT+IZMP_YMR;
		IZMP_ymargin[i][2] = Actl_Ankle_W[i][1]+LENGTH_FR_FT-IZMP_YMR;
		COP_xmargin[1][i] = COP_xmarginR[i];
		COP_xmargin[2][i] = COP_xmarginL[i]; 
  	}

	for(i = 1; i <= LEG_NUM ; i++){
		for(j = 1; j <= 2; j++){
		  COP_Xmargin_W[i][j] = Actl_Ankle_W[i][0]+COP_xmargin[i][j];		
		  COP_Ymargin_W[i][j] = Actl_Ankle_W[i][1]+COP_ymarin[j];
		}
	}

}
#endif 

void Imag_ZMP(void)
{

/// Revised points:	(1) Link center must consder Tq (the rotation body-angle (qx_waist), not actual Q-body)
///		(2) Link position must consider Tq 
	int i, j;
	double izmp[3], izmp_b[3], izmp_w[3];
	double weight_leg;

	for(i = 1; i <= JOINT_NUM ;i++){
		XP_LEG_Link[1][i] =  XP_LEG[i];
		XP_LEG_Link[2][i] = -XP_LEG[i];
	}

	izmp[0] = 0.0;
	izmp[1] = 0.0;
	izmp[2] = 0.0;
	weight_leg = 0.0;

	for(i = 0; i <= 2; i++){
           	COG_Body_W[i]  = 0.0;
		izmp_w[i] = 0.0;
	}


	/* body CoG at O_XYZ World Coordinate System*/

	Wq_Mtrc[1] = 0.0;//Q_body[1];
	Wq_Mtrc[2] = 0.0;//Q_body[2];
	Matric_Wq();	
        for(i = 0; i <= 2; i++){			//Position at World Coordiante System
              for(j = 0; j <= 2; j++){
		COG_Body_W[i]   = COG_Body_W[i] + Wi_Matric[i][j]*COG_BODY[j];
	      }
		 
	}

	/* leg link parameter */

	for(i = 1 ; i <= LEG_NUM ;i++){
		COGx_Link[i][1] = Actl_Hip[i][0]+XP_LEG_Link[i][1];
		COGy_Link[i][1] = Actl_Hip[i][1]+YP_LEG[1];
		COGz_Link[i][1] = Actl_Hip[i][2]+ZP_LEG[1];

		COGx_Link[i][2] = Actl_Hip[i][0]+XP_LEG_Link[i][2]*Cos_Joint[i][2]+ZP_LEG[2]*Sin_Joint[i][2];
		COGy_Link[i][2] = Actl_Hip[i][1]+YP_LEG[2];
		COGz_Link[i][2] = Actl_Hip[i][2]-XP_LEG_Link[i][2]*Sin_Joint[i][2]+ZP_LEG[2]*Cos_Joint[i][2];

		COGx_Link[i][3] = Actl_Hip[i][0]+XP_LEG_Link[i][3]*Cos_Joint[i][2]+
			          YP_LEG[3]*Sin_Joint[i][2]*Sin_Joint[i][3]+ZP_LEG[3]*Sin_Joint[i][2]*Cos_Joint[i][3];
		COGy_Link[i][3] = Actl_Hip[i][1]+YP_LEG[3]*Cos_Joint[i][3]-ZP_LEG[3]*Sin_Joint[i][3];
		COGz_Link[i][3] = Actl_Hip[i][2]-XP_LEG_Link[i][3]*Sin_Joint[i][2]+
			         YP_LEG[3]*Cos_Joint[i][2]*Sin_Joint[i][3]+ZP_LEG[3]*Cos_Joint[i][2]*Cos_Joint[i][3];

		COGx_Link[i][4] = Actl_Knee[i][0]+XP_LEG_Link[i][4]*Cos_Joint[i][2]+
			          YP_LEG[4]*Sin_Joint[i][2]*Sin_Joint34[i]+ZP_LEG[4]*Sin_Joint[i][2]*Cos_Joint34[i];
		COGy_Link[i][4] = Actl_Knee[i][1]+YP_LEG[4]*Cos_Joint34[i]-ZP_LEG[4]*Sin_Joint34[i];
		COGz_Link[i][4] = Actl_Knee[i][2]-XP_LEG_Link[i][4]*Sin_Joint[i][2]+
			         YP_LEG[4]*Cos_Joint[i][2]*Sin_Joint34[i]+ZP_LEG[4]*Cos_Joint[i][2]*Cos_Joint34[i];

		COGx_Link[i][5] = Actl_Ankle[i][0]+XP_LEG_Link[i][5]*Cos_Joint[i][2]+
			          YP_LEG[5]*Sin_Joint[i][2]*Sin_Joint345[i]+ZP_LEG[5]*Sin_Joint[i][2]*Cos_Joint345[i];
		COGy_Link[i][5] = Actl_Ankle[i][1]+YP_LEG[5]*Cos_Joint345[i]-ZP_LEG[5]*Sin_Joint345[i];
		COGz_Link[i][5] = Actl_Ankle[i][2]-XP_LEG_Link[i][5]*Sin_Joint[i][2]+
			         YP_LEG[5]*Cos_Joint[i][2]*Sin_Joint345[i]+ZP_LEG[5]*Cos_Joint[i][2]*Cos_Joint345[i];

		COGx_Link[i][6] = Actl_Ankle[i][0]+
			          XP_LEG_Link[i][6]*(Cos_Joint[i][2]*Cos_Joint[i][6]-Sin_Joint[i][2]*Cos_Joint345[i]*Sin_Joint[i][6])+
			          YP_LEG[6]*Sin_Joint[i][2]*Sin_Joint345[i]+
			          ZP_LEG[6]*(Cos_Joint[i][2]*Sin_Joint[i][6]+Sin_Joint[i][2]*Cos_Joint345[i]*Cos_Joint[i][6]);
		COGy_Link[i][6] = Actl_Ankle[i][1]+XP_LEG_Link[i][6]*Sin_Joint345[i]*Sin_Joint[i][6]+
			          YP_LEG[6]*Cos_Joint345[i]-ZP_LEG[6]*Sin_Joint345[i]*Cos_Joint[i][6];
		COGz_Link[i][6] = Actl_Ankle[i][2]-
			          XP_LEG_Link[i][6]*(Sin_Joint[i][2]*Cos_Joint[i][6]+Cos_Joint[i][2]*Cos_Joint345[i]*Sin_Joint[i][6])+
			          YP_LEG[6]*Cos_Joint[i][2]*Sin_Joint345[i]+
			          ZP_LEG[6]*(-Sin_Joint[i][2]*Sin_Joint[i][6]+Cos_Joint[i][2]*Cos_Joint345[i]*Cos_Joint[i][6]);
	}

#if 1 //new COG position (peng)
for(i = 1 ; i <= LEG_NUM ;i++){
	for(j = 1; j <= JOINT_NUM ;j++){
		COGx_Link[i][j]=Cos_Joint[i][1]*(COGx_Link[i][j])-Sin_Joint[i][1]*(COGy_Link[i][j]);
		COGy_Link[i][j]=Sin_Joint[i][1]*(COGx_Link[i][j])+Cos_Joint[i][1]*(COGy_Link[i][j]);
		COGz_Link[i][j]=COGz_Link[i][j];
	}
}
#endif
	
	for(i = 1 ; i <= LEG_NUM ;i++){			//Leg CoP at O_X'Y'Z' Coordinate System
		for(j = 1; j <= JOINT_NUM ;j++){
			izmp[0] = izmp[0]+COGx_Link[i][j]*MASS_LEG[j];
			izmp[1] = izmp[1]+COGy_Link[i][j]*MASS_LEG[j];
			izmp[2] = izmp[2]+COGz_Link[i][j]*MASS_LEG[j];
			weight_leg  = weight_leg+ MASS_LEG[j];
		}
	}

	izmp_b[0] = izmp[0]*Cos_RfQbody[2]+izmp[2]*Sin_RfQbody[2];	//Leg CoP at O_XYZ World Coordinate System
	izmp_b[1] = izmp[1];
	izmp_b[2] =-izmp[0]*Sin_RfQbody[2]+izmp[2]*Cos_RfQbody[2];	//Leg CoP at O_XYZ World Coordinate System



	//	Q_body[1]=0.0;
	//Q_body[2]=0.0;
	Wq_Mtrc[1] = 0.0;//Q_body[1]-Ref_Qbody[1];
	Wq_Mtrc[2] = 0.0;//Q_body[2]-Ref_Qbody[2];
	Matric_Wq();	
        for(i = 0; i <= 2; i++){			//Position at World Coordiante System
              for(j = 0; j <= 2; j++){
	           	izmp_w[i]  = izmp_w[i] + Wi_Matric[i][j]*izmp_b[j];
	      }
	}

	//Leg CoP at O_XYZ World Coordinate System

	IZMP_Actl[1] = (COG_Body_W[0]*MASS_BODY[1]+izmp_w[0])/(weight_leg+MASS_BODY[1]);
	IZMP_Actl[2] = (COG_Body_W[1]*MASS_BODY[1]+izmp_w[1])/(weight_leg+MASS_BODY[1]);
	IZMP_Actl[0] = (COG_Body_W[2]*MASS_BODY[1]+izmp_w[2])/(weight_leg+MASS_BODY[1]);
	
	

}

        
void Get_Mdf_Data(void )
{
	int i,k;
	double f1_qxy[3];
	double ldashi[LEG_NUM+1][2];
	//double t_force;
	double xmargin_lb_rb, xmargin_lf_rf, xmargin_rf_lf, xmargin_rb_lb;
	double	t_ankle[3][3];
	double a,b,c,n4,n2,d2,n3,d3;
	double	tad_ankle[3][3];
	double SIN_Q1,COS_Q1,SIN_Q2,COS_Q2,SIN_Q4,COS_Q4;
	
	double BVL_T[2]	= {0.002, 0.002};//{0.0006, 0.0006};//{0.002, 0.002};
	double K_HIPX[2] ={0.005, 0.005};//{0.0015, 0.0015}; /* Cofficent of hip compliance (Int 1.0)*/
	double K_HIPY[2] ={0.005, 0.005};//{0.002, 0.002};   /* Cofficent of hip compliance (Int 1.0)*/

	Land_Flag[0]=0;
	Land_Flag[1]=0;
	Land_Flag[2]=0;
	
#if 1  //while adjust ZMP, enable it
	Q_body[1]=0.0;  //for adjust ZMP
	Q_body[2]=0.0;  //for adjust ZMP
#endif
	Actl_Ankle_Position();
	Imag_ZMP();

        
        /*** Actual foot posture computation  ***/
        /*X-axis rotation*/
        for(i = 1; i <= LEG_NUM ; i++){
               Q_foot[i][1]  = Q_body[1]+Joint[i][HIP_X]/(Gear_Leg[i][HIP_X-1])+
               Joint[i][KNEE_X]/(Gear_Leg[i][KNEE_X-1])+Joint[i][FOOT_X]/(Gear_Leg[i][FOOT_X-1]);
		Q_foot[i][1] = 0.0; //test
		Cos_Qft[i][1] = cos(Q_foot[i][1]);
                Sin_Qft[i][1] = sin(Q_foot[i][1]);

        }

        /* Y-axis rotaton*/
        for(i = 1; i <= LEG_NUM ; i++){
             Q_foot[i][2]  = Q_body[2]+Joint[i][HIP_Y-1]/(Gear_Leg[i][HIP_Y-1])+
                       Joint[i][FOOT_Y]/(Gear_Leg[i][FOOT_Y-1]);
                Q_foot[i][2] = 0.0; //test
		Cos_Qft[i][2] = cos(Q_foot[i][2]);
                Sin_Qft[i][2] = sin(Q_foot[i][2]);
        }
        
#if 1	

        /*** Force/Moment computation for ZMP */
        f1_qxy[1] = sqrt(1.0-pow((Sin_Qft[1][1]*Sin_Qft[1][2]),2.0));
        f1_qxy[2] = sqrt(1.0-pow((Sin_Qft[2][1]*Sin_Qft[2][2]),2.0));
        
        for(i = 1; i <= LEG_NUM ; i++){
						/*  for(k =0; k<=6; k++)
						{
						Force_Grnd[i][k]=FootFT[i-1][k];
						}  */
                Force_Grnd[i][0] =  KFORCE_AD[i]*(FootFT[i][0]*Cos_Qft[i][2]+FootFT[i][2]*Sin_Qft[i][2]);
                Force_Grnd[i][1] =  KFORCE_AD[i]*(FootFT[i][1]*Cos_Qft[i][1]+FootFT[i][2]*Sin_Qft[i][1]);
                Force_Grnd[i][2] =  KFORCE_AD[i]*(-FootFT[i][0]*Cos_Qft[i][1]*Sin_Qft[i][2]-
                                    FootFT[i][1]*Sin_Qft[i][1]*Cos_Qft[i][2]+
                                    FootFT[i][2]*Cos_Qft[i][1]*Cos_Qft[i][2])/f1_qxy[i];

                Force_Grnd[i][3] = -KFORCE_AD[i]*(FootFT[i][3]*Cos_Qft[i][2]+FootFT[i][5]*Sin_Qft[i][2]);
                Force_Grnd[i][4] = -KFORCE_AD[i]*(FootFT[i][4]*Cos_Qft[i][1]+FootFT[i][5]*Sin_Qft[i][1]);
                Force_Grnd[i][5] =  KFORCE_AD[i]*(-FootFT[i][3]*Cos_Qft[i][1]*Sin_Qft[i][2]-
                                    FootFT[i][4]*Sin_Qft[i][1]*Cos_Qft[i][2]+
                                    FootFT[i][5]*Cos_Qft[i][1]*Cos_Qft[i][2])/f1_qxy[i];  
                
        }
		// last_real_Fz[0] = real_Fz[0];
		// last_real_Fz[1] = real_Fz[1];
		// real_Fz[0] = Force_Grnd[1][2];
		// real_Fz[1] = Force_Grnd[2][2];
		// real_dFz[0] = (real_Fz[0]-last_real_Fz[0])/0.004;
		// real_dFz[1] = (real_Fz[1]-last_real_Fz[1])/0.004;
		
	Rel_RForce_Z = Force_Grnd[1][2];
	Rel_LForce_Z = Force_Grnd[2][2];
		
#endif
        /***  Actual ZMP  ***/        ///// ZMP_Actl = Act_Ankle+M/F, before ZMP_Actl = Ref_Ankle+M/F is not suggested
        /* each foot */
        for(i = 1; i <= LEG_NUM ; i++){
                if ( Force_Grnd[i][2] < FZ_Mins ){         /* no contact */
                        ZMP_Actl[i][1] = Actl_Ankle_W[i][0];
                        ZMP_Actl[i][2] = Actl_Ankle_W[i][1];
                        
                        Force_Grnd[i][0] = 0.0;
                        Force_Grnd[i][1] = 0.0;
                        Force_Grnd[i][2] = 0.0;            /* case of no contact */
                }
                else {
			if(i==1){
//                                   Force_Grnd[i][2]+Ref_Ankle[i][0];              /*x-ZMP of one-foot */
                        ZMP_Actl[i][1] = -1.0*K_Amp_ZMP_R*(Force_Grnd[i][4])/
                                   Force_Grnd[i][2]+Actl_Ankle_W[i][0];         
		//	if( i == 2 ) {
		//	ZMP_Actl[2][1] = 1.16*ZMP_Actl[2][1];
		//	}
	/*y-ZMP of one-foot*/
                        ZMP_Actl[i][2] = 1.0*K_Amp_ZMP_R*(Force_Grnd[i][3])/
                                   Force_Grnd[i][2]+Actl_Ankle_W[i][1];
				}
             
			else{  //1.2 ,1.1
                        ZMP_Actl[i][1] = -1.0*K_Amp_ZMP_L*(Force_Grnd[i][4])/
                                   Force_Grnd[i][2]+Actl_Ankle_W[i][0];        
                        ZMP_Actl[i][2] =  1.0*K_Amp_ZMP_L*(Force_Grnd[i][3])/
                                   Force_Grnd[i][2]+Actl_Ankle_W[i][1]; 
			     }
				}
	}		
		

		//***modification of the Actual ZMP***/

/*Total zmp*/
        if ( (Force_Grnd[1][2]+ Force_Grnd[2][2]) < FZ_Mins ){
                        ZMP_Actl[0][1] = 0.0;
                        ZMP_Actl[0][2] = 0.0;
                }

        else{
       /* y-ZMP of two_feet*/

//
                ZMP_Actl[0][1] =  (Force_Grnd[1][2]*ZMP_Actl[1][1]+
                                   Force_Grnd[2][2]*ZMP_Actl[2][1])/
                                  (Force_Grnd[1][2]+Force_Grnd[2][2]);            /* x-ZMP of two_feet*/
                ZMP_Actl[0][2] = (Force_Grnd[1][2]*ZMP_Actl[1][2]+
                                   Force_Grnd[2][2]*ZMP_Actl[2][2])/
                                  (Force_Grnd[1][2]+Force_Grnd[2][2]);         
                
        }

#if 1  //yuzg Z force limitation
     // 	t_force = Force_Grnd[1][2]+Force_Grnd[2][2];		/* Let reaction force be humanoid weight */

	
	// if ( t_force > 0 ){
	  // Force_Grnd[1][2] = Force_Grnd[1][2]*WEIGHT/t_force;
	  // Force_Grnd[2][2] = Force_Grnd[2][2]*WEIGHT/t_force;
	// }
	
#endif
        /****** Ankle compliance ******/


//ZMP control & Compliance control & Return original pattern

//#ifdef 5
   if ( (Force_Grnd[1][2]-FZ_Mins)*(Force_Grnd[2][2]-FZ_Mins)<0 ){	//Actual single support
  	for(i = 1; i <= LEG_NUM ; i++){
		if ( (Force_Grnd[i][2] > FZ_Mins) ){

/* sigattal plane */
			if  ( IZMP_Actl[2] < IZMP_ymargin[i][1] ) {			// ZMP control 
				now_val[i][5] = -KCOM_ANKL_I*(IZMP_Actl[2]- IZMP_ymargin[i][1]-IZMP_MR_RTN);
			}
			else if  ( IZMP_Actl[2] > IZMP_ymargin[i][2] ) {		// ZMP control 
				now_val[i][5] = -KCOM_ANKL_I*(IZMP_Actl[2]- IZMP_ymargin[i][2]+IZMP_MR_RTN);
			}

			/* Compliance control or return to planned pattern */
			else {	
			  		
				if ( ZMP_Actl[i][2] > COP_Ymargin_W[i][2] ) {		//compliance control
					now_val[i][5] = 2.0*(IZMP_ymargin[i][2]-IZMP_Actl[2])/(IZMP_ymargin[i][2]-IZMP_ymargin[i][1])*
					                2.0*(IZMP_ymargin[i][2]-IZMP_Actl[2])/(IZMP_ymargin[i][2]-IZMP_ymargin[i][1])*
					                     KCOM_ANKL*(ZMP_Actl[i][2]-COP_Ymargin_W[i][2]);
				}
				else if (ZMP_Actl[i][2] < COP_Ymargin_W[i][1] ) {		//compliance control
                        	        now_val[i][5] = 2.0*(IZMP_Actl[2]-IZMP_ymargin[i][1])/(IZMP_ymargin[i][2]-IZMP_ymargin[i][1])*
					                2.0*(IZMP_Actl[2]-IZMP_ymargin[i][1])/(IZMP_ymargin[i][2]-IZMP_ymargin[i][1])*
					                     KCOM_ANKL*(ZMP_Actl[i][2]-COP_Ymargin_W[i][1]);
				}
				else {					//return to planned pattern
					now_val[i][5] = K_RETURN*KS_RETURN*Mdf_Val[i][5];
				}
			  
			}

/* frontal plane */
			if  ( IZMP_Actl[1] < IZMP_xmargin[i][1] ) {			// ZMP control 
				now_val[i][6] = KCOM_ANKL_I*(IZMP_Actl[1]- IZMP_xmargin[i][1]-IZMP_MR_RTN);
			}
			else if  ( IZMP_Actl[1] > IZMP_xmargin[i][2] ) {		// ZMP control 
				now_val[i][6] = KCOM_ANKL_I*(IZMP_Actl[1]- IZMP_xmargin[i][2]+IZMP_MR_RTN);
			}

			/* Compliance control or return to planned pattern */
			else {

				if ( ZMP_Actl[i][1] > COP_Xmargin_W[i][2] ) {		//compliance control
					now_val[i][6] = -2.0*(IZMP_xmargin[i][2]-IZMP_Actl[1])/(IZMP_xmargin[i][2]-IZMP_xmargin[i][1])*
					                 2.0*(IZMP_xmargin[i][2]-IZMP_Actl[1])/(IZMP_xmargin[i][2]-IZMP_xmargin[i][1])*
					                      KCOM_ANKL*(ZMP_Actl[i][1]-COP_Xmargin_W[i][2]);
				} 
				else if ( ZMP_Actl[i][1] < COP_Xmargin_W[i][1] ) {	//compliance control
					now_val[i][6] = -2.0*(IZMP_Actl[1]-IZMP_xmargin[i][1])/(IZMP_xmargin[i][2]-IZMP_xmargin[i][1])*
					                 2.0*(IZMP_Actl[1]-IZMP_xmargin[i][1])/(IZMP_xmargin[i][2]-IZMP_xmargin[i][1])*
					                      KCOM_ANKL*(ZMP_Actl[i][1]-COP_Xmargin_W[i][1]);
				}
				else {					//return to planned pattern
					now_val[i][6] = K_RETURN*KS_RETURN*Mdf_Val[i][6];
				}
			}
		}
	}

   }
///////////////////////////////////////
//	Test_All[0]=90;
////////////////////////////////////////
  if ( (Force_Grnd[1][2]-FZ_Mins) >= 0 && (Force_Grnd[2][2]-FZ_Mins) >= 0 ){		//Actual double support
	now_val[1][5] =  KCOM_ANKL*(ZMP_Actl[1][2]-Actl_Ankle_W[1][1]);
	now_val[1][6] = -KCOM_ANKL*(ZMP_Actl[1][1]-Actl_Ankle_W[1][0]);
	now_val[2][5] =  KCOM_ANKL*(ZMP_Actl[2][2]-Actl_Ankle_W[2][1]);
	now_val[2][6] = -KCOM_ANKL*(ZMP_Actl[2][1]-Actl_Ankle_W[2][0]);

	if (Actl_Ankle_W[1][1] > (Actl_Ankle_W[2][1]+LFT_FB) ) {
		xmargin_lb_rb = IZMP_xmargin[2][2]+
		    (IZMP_xmargin[2][2]-IZMP_xmargin[1][2])*(IZMP_Actl[2] -IZMP_ymargin[2][1]) / (IZMP_ymargin[2][1]-IZMP_ymargin[1][1]);
		xmargin_lf_rf = IZMP_xmargin[2][1]+
		    (IZMP_xmargin[2][1]-IZMP_xmargin[1][1])*(IZMP_Actl[2] -IZMP_ymargin[2][2]) / (IZMP_ymargin[2][2]-IZMP_ymargin[1][2]);

		Test_All[4] = xmargin_lb_rb;
		Test_All[5] = xmargin_lf_rf;

		Db_Cmp_Margin[1] = IZMP_xmargin[1][2]+(IZMP_xmargin[1][1]-IZMP_xmargin[1][2])*
		                                      (IZMP_Actl[2] -IZMP_ymargin[1][1]) / (IZMP_ymargin[1][2]-IZMP_ymargin[1][1]);
		Db_Cmp_Margin[2] = xmargin_lf_rf;
		Db_Cmp_Margin[3] = IZMP_xmargin[2][2]+(IZMP_xmargin[2][1]-IZMP_xmargin[2][2])*
		                                      (IZMP_Actl[2] -IZMP_ymargin[2][1]) / (IZMP_ymargin[2][2]-IZMP_ymargin[2][1]);
		Db_Cmp_Margin[4] = xmargin_lb_rb;


		if ( IZMP_Actl[1] <= IZMP_xmargin[2][1] && IZMP_Actl[2] <= IZMP_ymargin[2][1] ) {

		  //			now_val[1][5] =  KCOM_ANKL_TT*(ZMP_Actl[1][2]-Ref_Ankle[1][1]);
		  //			now_val[1][6] = -KCOM_ANKL_TT*(ZMP_Actl[1][1]-Ref_Ankle[1][0]);
			now_val[2][5] = -KCOM_ANKL_I*(IZMP_Actl[2]-IZMP_ymargin[2][1]-IZMP_MR_RTN);
			now_val[2][6] =  KCOM_ANKL_I*(IZMP_Actl[1]-IZMP_xmargin[2][1]-IZMP_MR_RTN);
		}

		else if ( IZMP_Actl[1] > IZMP_xmargin[2][1] && IZMP_Actl[2] < IZMP_ymargin[2][1] ) {

//			now_val[1][5] =  KCOM_ANKL_TT*(ZMP_Actl[1][2]-Ref_Ankle[1][1]);
//			now_val[1][6] = -KCOM_ANKL_TT*(ZMP_Actl[1][1]-Ref_Ankle[1][0]);
			now_val[2][5] = -KCOM_ANKL_I*(IZMP_Actl[2]-IZMP_ymargin[2][1]-IZMP_MR_RTN);
//			now_val[2][6] = -KCOM_ANKL_TT*(ZMP_Actl[1][1]-Ref_Ankle[2][0]);
		}

		else if ( IZMP_Actl[1] > xmargin_lb_rb && (IZMP_Actl[2] >= IZMP_ymargin[2][1] && IZMP_Actl[2] <= IZMP_ymargin[1][1]) ) {

			now_val[1][5] =  KCOM_ANKL_TT*(IZMP_Actl[2]-Ref_Ankle[1][1]);
			now_val[1][6] =  KCOM_ANKL_I*(IZMP_Actl[1]-IZMP_xmargin[2][2]+IZMP_MR_RTN);
			now_val[2][5] = -KCOM_ANKL_I*(IZMP_Actl[2]-IZMP_ymargin[1][1]-IZMP_MR_RTN);
			now_val[2][6] = -KCOM_ANKL_TT*(ZMP_Actl[1][1]-Ref_Ankle[2][0]);
		}

		else if ( IZMP_Actl[1] > IZMP_xmargin[1][2] && (IZMP_Actl[2] > IZMP_ymargin[1][1] && IZMP_Actl[2] < IZMP_ymargin[1][2]) ) {
//			now_val[1][5] =  KCOM_ANKL_TT*(ZMP_Actl[1][2]-Ref_Ankle[1][1]);
			now_val[1][6] =  KCOM_ANKL_I*(IZMP_Actl[1]-IZMP_xmargin[1][2]+IZMP_MR_RTN);
//			now_val[2][5] =  KCOM_ANKL_TT*(ZMP_Actl[2][2]-Ref_Ankle[2][1]);
//                      now_val[2][6] = -KCOM_ANKL_TT*(ZMP_Actl[2][1]-Ref_Ankle[2][0]);
		}

		else if ( IZMP_Actl[1] >= IZMP_xmargin[1][2] && IZMP_Actl[2] >= IZMP_ymargin[1][2] ) {
			now_val[1][5] = -KCOM_ANKL_I*(IZMP_Actl[2]-IZMP_ymargin[1][2]+IZMP_MR_RTN);
			now_val[1][6] =  KCOM_ANKL_I*(IZMP_Actl[1]-IZMP_xmargin[1][2]+IZMP_MR_RTN);
			  //			now_val[2][5] =  KCOM_ANKL_TT*(ZMP_Actl[2][2]-Ref_Ankle[2][1]);
			  //now_val[2][6] = -KCOM_ANKL_TT*(ZMP_Actl[2][1]-Ref_Ankle[2][0]);
		}

		else if ( IZMP_Actl[1] < IZMP_xmargin[1][2] && IZMP_Actl[2] > IZMP_ymargin[1][2] ) {
			now_val[1][5] = -KCOM_ANKL_I*(IZMP_Actl[2]-IZMP_ymargin[1][2]+IZMP_MR_RTN);
			  //			now_val[1][6] = -KCOM_ANKL_TT*(ZMP_Actl[1][1]-Ref_Ankle[1][0]);
			  //now_val[2][5] =  KCOM_ANKL_TT*(ZMP_Actl[2][2]-Ref_Ankle[2][1]);
			  //now_val[2][6] = -KCOM_ANKL_TT*(ZMP_Actl[2][1]-Ref_Ankle[2][0]);
		}

		else if ( IZMP_Actl[1] < xmargin_lf_rf && (IZMP_Actl[2] >= IZMP_ymargin[2][2] && IZMP_Actl[2] <= IZMP_ymargin[1][2]) ) {
			now_val[1][5] = -KCOM_ANKL_I*(IZMP_Actl[2]-IZMP_ymargin[2][2]+IZMP_MR_RTN);
			now_val[1][6] = -KCOM_ANKL_TT*(ZMP_Actl[1][1]-Ref_Ankle[1][0]);
			now_val[2][5] =  KCOM_ANKL_TT*(ZMP_Actl[2][2]-Ref_Ankle[2][1]);
			now_val[2][6] =  KCOM_ANKL_I*(IZMP_Actl[1]-IZMP_xmargin[1][1]-IZMP_MR_RTN);
		}

		else if ( IZMP_Actl[1] < IZMP_xmargin[2][1] && (IZMP_Actl[2] > IZMP_ymargin[2][1] && IZMP_Actl[2] < IZMP_ymargin[2][2]) ) {
//			now_val[1][5] =  KCOM_ANKL_TT*(ZMP_Actl[1][2]-Ref_Ankle[1][1]);
//			now_val[1][6] = -KCOM_ANKL_TT*(ZMP_Actl[1][1]-Ref_Ankle[1][0]);
//			now_val[2][5] =  KCOM_ANKL_TT*(ZMP_Actl[2][2]-Ref_Ankle[2][1]);
			now_val[2][6] =  KCOM_ANKL_I*(IZMP_Actl[1]-IZMP_xmargin[2][1]-IZMP_MR_RTN);
		}

		else {
		  	 Ankle_Cmpl_Db();
		  //			Ankle_Cmpl_Sgtl();
		  //      Ankle_Cmpl_Frnt();
		}
	}

	else if (Actl_Ankle_W[1][1] < (Actl_Ankle_W[2][1]-LFT_FB) ) {
		xmargin_rf_lf = IZMP_xmargin[2][2]+
		    (IZMP_xmargin[2][2]-IZMP_xmargin[1][2])*(IZMP_Actl[2] -IZMP_ymargin[2][2]) / (IZMP_ymargin[2][2]-IZMP_ymargin[1][2]);
		xmargin_rb_lb = IZMP_xmargin[2][1]+
		    (IZMP_xmargin[2][1]-IZMP_xmargin[1][1])*(IZMP_Actl[2] -IZMP_ymargin[2][1]) / (IZMP_ymargin[2][1]-IZMP_ymargin[1][1]);

		Test_All[6] = xmargin_rf_lf;
		Test_All[7] = xmargin_rb_lb;

		Db_Cmp_Margin[1] = IZMP_xmargin[1][1]+(IZMP_xmargin[1][2]-IZMP_xmargin[1][1])*
		                                      (IZMP_Actl[2] -IZMP_ymargin[1][1]) / (IZMP_ymargin[1][2]-IZMP_ymargin[1][1]);
		Db_Cmp_Margin[2] = xmargin_rf_lf;
		Db_Cmp_Margin[3] = IZMP_xmargin[2][1]+(IZMP_xmargin[2][2]-IZMP_xmargin[2][1])*
		                                      (IZMP_Actl[2] -IZMP_ymargin[2][1]) / (IZMP_ymargin[2][2]-IZMP_ymargin[2][1]);
		Db_Cmp_Margin[4] = xmargin_rb_lb;

		if ( IZMP_Actl[1] >= IZMP_xmargin[1][2] && IZMP_Actl[2] <= IZMP_ymargin[1][1] ) {
			now_val[1][5] = -KCOM_ANKL_I*(IZMP_Actl[2]-IZMP_ymargin[1][1]-IZMP_MR_RTN);
			now_val[1][6] =  KCOM_ANKL_I*(IZMP_Actl[1]-IZMP_xmargin[1][2]+IZMP_MR_RTN);
			  //			now_val[2][5] =  KCOM_ANKL_TT*(ZMP_Actl[2][2]-Ref_Ankle[2][1]);
			  //now_val[2][6] = -KCOM_ANKL_TT*(ZMP_Actl[2][1]-Ref_Ankle[2][0]);
		}

		else if ( IZMP_Actl[1] > IZMP_xmargin[1][2] && (IZMP_Actl[2] > IZMP_ymargin[1][1] && IZMP_Actl[2] < IZMP_ymargin[1][2]) ) {
//			now_val[1][5] =  KCOM_ANKL_TT*(ZMP_Actl[1][2]-Ref_Ankle[1][1]);
			now_val[1][6] =  KCOM_ANKL_I*(IZMP_Actl[1]-IZMP_xmargin[1][2]+IZMP_MR_RTN);
			  //			now_val[2][5] =  KCOM_ANKL_TT*(ZMP_Actl[2][2]-Ref_Ankle[2][1]);
			  //now_val[2][6] = -KCOM_ANKL_TT*(ZMP_Actl[2][1]-Ref_Ankle[2][0]);
		}

		else if ( IZMP_Actl[1] > xmargin_rf_lf && (IZMP_Actl[2] >= IZMP_ymargin[1][2] && IZMP_Actl[2] <= IZMP_ymargin[2][2]) ) {
			now_val[1][5] =  KCOM_ANKL_TT*(ZMP_Actl[1][2]-Ref_Ankle[1][1]);
			now_val[1][6] =  KCOM_ANKL_I*(IZMP_Actl[1]-IZMP_xmargin[2][2]+IZMP_MR_RTN);
			now_val[2][5] = -KCOM_ANKL_I*(IZMP_Actl[2]-IZMP_ymargin[1][2]+IZMP_MR_RTN);
			now_val[2][6] = -KCOM_ANKL_TT*(ZMP_Actl[1][1]-Ref_Ankle[2][0]);
		}

		else if ( IZMP_Actl[1] > IZMP_xmargin[2][1] && IZMP_Actl[2] > IZMP_ymargin[2][2] ) {
		  //			now_val[1][5] =  KCOM_ANKL_TT*(ZMP_Actl[1][2]-Ref_Ankle[1][1]);
		  //	now_val[1][6] = -KCOM_ANKL_TT*(ZMP_Actl[1][1]-Ref_Ankle[1][0]);
			now_val[2][5] = -KCOM_ANKL_I*(IZMP_Actl[2]-IZMP_ymargin[2][2]+IZMP_MR_RTN);
			  //	now_val[2][6] = -KCOM_ANKL_TT*(ZMP_Actl[1][1]-Ref_Ankle[2][0]);
		}

		else if ( IZMP_Actl[1] <= IZMP_xmargin[2][1] && IZMP_Actl[2] >= IZMP_ymargin[2][2] ) {
//			now_val[1][5] =  KCOM_ANKL_TT*(ZMP_Actl[1][2]-Ref_Ankle[1][1]);
//			now_val[1][6] = -KCOM_ANKL_TT*(ZMP_Actl[1][1]-Ref_Ankle[1][0]);
			now_val[2][5] = -KCOM_ANKL_I*(IZMP_Actl[2]-IZMP_ymargin[2][2]+IZMP_MR_RTN);
			now_val[2][6] =  KCOM_ANKL_I*(IZMP_Actl[1]-IZMP_xmargin[2][1]-IZMP_MR_RTN);
		}

		else if ( IZMP_Actl[1] < IZMP_xmargin[2][1] && (IZMP_Actl[2] > IZMP_ymargin[2][1] && IZMP_Actl[2] < IZMP_ymargin[2][2]) ) {
//			now_val[1][5] =  KCOM_ANKL_TT*(ZMP_Actl[1][2]-Ref_Ankle[1][1]);
//			now_val[1][6] = -KCOM_ANKL_TT*(ZMP_Actl[1][1]-Ref_Ankle[1][0]);
//			now_val[2][5] =  KCOM_ANKL_TT*(ZMP_Actl[2][2]-Ref_Ankle[2][1]);
			now_val[2][6] =  KCOM_ANKL_I*(IZMP_Actl[1]-IZMP_xmargin[2][1]-IZMP_MR_RTN);
		}

		else if ( IZMP_Actl[1] < xmargin_rb_lb && (IZMP_Actl[2] >= IZMP_ymargin[1][1] && IZMP_Actl[2] <= IZMP_ymargin[2][1]) ) {
			now_val[1][5] = -KCOM_ANKL_I*(IZMP_Actl[2]-IZMP_ymargin[2][1]-IZMP_MR_RTN);
			now_val[1][6] = -KCOM_ANKL_TT*(ZMP_Actl[1][1]-Ref_Ankle[1][0]);
			now_val[2][5] =  KCOM_ANKL_TT*(ZMP_Actl[2][2]-Ref_Ankle[2][1]);
			now_val[2][6] =  KCOM_ANKL_I*(IZMP_Actl[1]-IZMP_xmargin[1][1]-IZMP_MR_RTN);
		}

		else if ( IZMP_Actl[1] < IZMP_xmargin[1][2] && IZMP_Actl[2] < IZMP_ymargin[1][1] ) {
			now_val[1][5] = -KCOM_ANKL_I*(IZMP_Actl[2]-IZMP_ymargin[1][1]-IZMP_MR_RTN);
			  //			now_val[1][6] = -KCOM_ANKL_TT*(ZMP_Actl[1][1]-Ref_Ankle[1][0]);
			  //now_val[2][5] =  KCOM_ANKL_TT*(ZMP_Actl[2][2]-Ref_Ankle[2][1]);
			  //now_val[2][6] = -KCOM_ANKL_TT*(ZMP_Actl[2][1]-Ref_Ankle[2][0]);
		}

		else {
		  	Ankle_Cmpl_Db();
		}
	}

	else {						//Actl_Ankle[1][1] = Actl_Ankle[2][1] 
		if ( IZMP_Actl[1] >= IZMP_xmargin[1][2] && IZMP_Actl[2] <= IZMP_ymargin[1][1] ) {
////////////////////////////////////
		Test_All[0]=91;
////////////////////////////////////
			now_val[1][5] = -KCOM_ANKL_I*(IZMP_Actl[2]-IZMP_ymargin[1][1]-IZMP_MR_RTN);
			now_val[1][6] =  KCOM_ANKL_I*(IZMP_Actl[1]-IZMP_xmargin[1][2]+IZMP_MR_RTN);
//			now_val[2][5] =  KCOM_ANKL*(ZMP_Actl[2][2]-Ref_Ankle[2][1]);
//			now_val[2][6] = -KCOM_ANKL*(ZMP_Actl[2][1]-Ref_Ankle[2][0]);
		}

		else if ( IZMP_Actl[1] > IZMP_xmargin[1][2] && (IZMP_Actl[2] > IZMP_ymargin[1][1] && IZMP_Actl[2] < IZMP_ymargin[1][2]) ) {
////////////////////////////////////
		Test_All[0]=92;
////////////////////////////////////
//			now_val[1][5] =  KCOM_ANKL*(ZMP_Actl[1][2]-Ref_Ankle[1][1]);
			now_val[1][6] =  KCOM_ANKL_I*(IZMP_Actl[1]-IZMP_xmargin[1][2]+IZMP_MR_RTN);
//			now_val[2][5] =  KCOM_ANKL*(ZMP_Actl[2][2]-Ref_Ankle[2][1]);
//			now_val[2][6] = -KCOM_ANKL*(ZMP_Actl[2][1]-Ref_Ankle[2][0]);
		}

		else if ( IZMP_Actl[1] >= IZMP_xmargin[1][2] && IZMP_Actl[2] >= IZMP_ymargin[1][2] ) {
////////////////////////////////////
		Test_All[0]=93;
////////////////////////////////////
			now_val[1][5] = -KCOM_ANKL_I*(IZMP_Actl[2]-IZMP_ymargin[1][2]+IZMP_MR_RTN);
			now_val[1][6] =  KCOM_ANKL_I*(IZMP_Actl[1]-IZMP_xmargin[1][2]+IZMP_MR_RTN);
//			now_val[2][5] =  KCOM_ANKL*(ZMP_Actl[2][2]-Ref_Ankle[2][1]);
//			now_val[2][6] = -KCOM_ANKL*(ZMP_Actl[2][1]-Ref_Ankle[2][0]);
		}

		else if (  IZMP_Actl[1] > IZMP_xmargin[2][1] && IZMP_Actl[1] < IZMP_xmargin[1][2] && 
			  (IZMP_Actl[2] > IZMP_ymargin[1][2] || IZMP_Actl[2] > IZMP_ymargin[2][2]) ) {
////////////////////////////////////
		Test_All[0]=94;
////////////////////////////////////
			now_val[1][5] = -KCOM_ANKL_I*(IZMP_Actl[2]-IZMP_ymargin[1][2]+IZMP_MR_RTN);
//			now_val[1][6] =  KCOM_ANKL*(IZMP_Actl[1]-IZMP_xmargin[1][2]);
			now_val[2][5] = -KCOM_ANKL_I*(IZMP_Actl[2]-IZMP_ymargin[2][2]+IZMP_MR_RTN);//IZMP_ymargin[1][2] = IZMP_ymargin[2][2]
//			now_val[2][6] = -KCOM_ANKL*(ZMP_Actl[2][1]-Ref_Ankle[2][0]);
		}

		else if ( IZMP_Actl[1] <= IZMP_xmargin[2][1] && IZMP_Actl[2] >= IZMP_ymargin[2][2] ) {
////////////////////////////////////
		Test_All[0]=95;
////////////////////////////////////
//			now_val[1][5] =  KCOM_ANKL*(ZMP_Actl[1][2]-Ref_Ankle[1][1]);
//			now_val[1][6] = -KCOM_ANKL*(ZMP_Actl[1][1]-Ref_Ankle[1][0]);
			now_val[2][5] = -KCOM_ANKL_I*(IZMP_Actl[2]-IZMP_ymargin[2][2]+IZMP_MR_RTN);
			now_val[2][6] =  KCOM_ANKL_I*(IZMP_Actl[1]-IZMP_xmargin[2][1]-IZMP_MR_RTN);
		}

		else if ( IZMP_Actl[1] < IZMP_xmargin[2][1] && (IZMP_Actl[2] > IZMP_ymargin[2][1] && IZMP_Actl[2] < IZMP_ymargin[2][2]) ) {
////////////////////////////////////
		Test_All[0]=96;
////////////////////////////////////
//			now_val[1][5] =  KCOM_ANKL*(ZMP_Actl[1][2]-Ref_Ankle[1][1]);
//			now_val[1][6] = -KCOM_ANKL*(ZMP_Actl[1][1]-Ref_Ankle[1][0]);
//			now_val[2][5] =  KCOM_ANKL*(ZMP_Actl[2][2]-Ref_Ankle[2][1]);
			now_val[2][6] =  KCOM_ANKL_I*(IZMP_Actl[1]-IZMP_xmargin[2][1]-IZMP_MR_RTN);
		}

		else if ( IZMP_Actl[1] <= IZMP_xmargin[2][1] && IZMP_Actl[2] <= IZMP_ymargin[2][1] ) {
////////////////////////////////////
		Test_All[0]=97;
////////////////////////////////////
//			now_val[1][5] =  KCOM_ANKL*(ZMP_Actl[1][2]-Ref_Ankle[1][1]);
//			now_val[1][6] = -KCOM_ANKL*(ZMP_Actl[1][1]-Ref_Ankle[1][0]);
			now_val[2][5] = -KCOM_ANKL_I*(IZMP_Actl[2]-IZMP_ymargin[2][1]-IZMP_MR_RTN);
			now_val[2][6] =  KCOM_ANKL_I*(IZMP_Actl[1]-IZMP_xmargin[2][1]-IZMP_MR_RTN);
		}

		else if (  IZMP_Actl[1] > IZMP_xmargin[2][1] && IZMP_Actl[1] < IZMP_xmargin[1][2] && 
			  (IZMP_Actl[2] < IZMP_ymargin[1][1] || IZMP_Actl[2] < IZMP_ymargin[2][1]) ) {
////////////////////////////////////
		Test_All[0]=98;
////////////////////////////////////
			now_val[1][5] = -KCOM_ANKL_I*(IZMP_Actl[2]-IZMP_ymargin[1][1]-IZMP_MR_RTN);
//			now_val[1][6] =  KCOM_ANKL_I*(IZMP_Actl[1]-IZMP_xmargin[1][2]);
			now_val[2][5] = -KCOM_ANKL_I*(IZMP_Actl[2]-IZMP_ymargin[2][1]-IZMP_MR_RTN);
//			now_val[2][6] = -KCOM_ANKL*(ZMP_Actl[2][1]-Ref_Ankle[2][0]);
		}

		else {
////////////////////////////////////
		Test_All[0]=99;
////////////////////////////////////
	  		Ankle_Cmpl_Db();

		  //			Ankle_Cmpl_Sgtl();
		  //	Ankle_Cmpl_Frnt();
		}
	}	
  }

  //printf("\nnow_val16=%f,now_val26=%f",now_val[1][6],now_val[2][6]);
  for(i = 1; i <= LEG_NUM ; i++){
	/* case B: no contact or //supported mainly by one foot: return to planned pattern*/
        if ( Force_Grnd[i][2] < FZ_Mins  ){
		  now_val[i][5] = 0.25*10.0*K_RETURN*TIME_D2S*Mdf_Val[i][5];//1.0
		  now_val[i][6] =0.25*10.0*K_RETURN*TIME_D2S*Mdf_Val[i][6];//1.0
		if(Ref_Timecntl[i]==1)
			{
		  now_val[i][5] = 0.5*10.0*K_RETURN*TIME_D2S*Mdf_Val[i][5];//1.0
		  now_val[i][6] = 0.5*10.0*K_RETURN*TIME_D2S*Mdf_Val[i][6];//1.0

			}
	}

	/* case C: unexpected contact, Compliance control; Note: Above-mentioned actual single & double support are expected contact*/
	if ( (Force_Grnd[i][2] >= FZ_Mins && Ref_Cnt_Sgnl[i] == 0 && Ref_Timecntl[i]==0) ){

	  now_val[i][5] =10.0*0.05*KCOM_ANKL*KCOM_ANKL_F*(ZMP_Actl[i][2]-Actl_Ankle_W[i][1]);
	  now_val[i][6] =-10.0*0.05*KCOM_ANKL*KCOM_ANKL_F*(ZMP_Actl[i][1]-Actl_Ankle_W[i][0]);

	}

    if ( Force_Grnd[i][2] >= FZ_Mins && Ref_Cnt_Sgnl[i] == 0 && Ref_Timecntl[i] ==1) { 

//compliance control
      now_val[i][5] = 10.0*0.5*0.05*KCOM_ANKL*KCOM_ANKL_F*(ZMP_Actl[i][2]-Actl_Ankle_W[i][1]);
//compliance control
      now_val[i][6] =-10.0*0.05*KCOM_ANKL*KCOM_ANKL_F*(ZMP_Actl[i][1]-Actl_Ankle_W[i][0]);
//}   
 }

/********20090816 added***********************
***********************************************/
//Actual double support
  for(i = 1; i <= LEG_NUM ; i++){
   if ( (Ref_Cnt_Sgnl[1]*Ref_Cnt_Sgnl[2]==1 )&&(LEG_NO==i)&&(Force_Grnd[i][2]>FZ_Mins))
{
	COP_R_Count = 0; 
/* sagittal plane */
    	       if (  ( ZMP_Actl[i][2] > (COP_Ymargin_W[i][2]-3.0*COP_R) ||
		       ZMP_Actl[i][2] < (COP_Ymargin_W[i][1]+3.0*COP_R) ) ) {	//compliance control

	  now_val[i][5] =0.1*KCOM_ANKL*KCOM_ANKL_F*(ZMP_Actl[i][2]-Actl_Ankle_W[i][1]);
		}
		else 

		   {    
	now_val[i][5] = 2.0*0.25*KTMP*K_RETURN*Mdf_Val[i][5];
		}

/* frontal plane */
		if( ( ZMP_Actl[i][1] > (COP_Xmargin_W[i][2]-3.0*COP_R) ||
		       ZMP_Actl[i][1] < (COP_Xmargin_W[i][1]+3.0*COP_R) ) ) {	//compliance control

	  now_val[i][6] =-2.0*0.1*KCOM_ANKL*KCOM_ANKL_F*(ZMP_Actl[i][1]-Actl_Ankle_W[i][0]);
		}
		else
		{

		       now_val[i][6] =2.0*0.25*K_RETURN*Mdf_Val[i][6];
		}
	}
}
//Actual single support

  for(i = 1; i <= LEG_NUM ; i++){
   if ( (Ref_Cnt_Sgnl[1]*Ref_Cnt_Sgnl[2]==0 )&&(Ref_Cnt_Sgnl[i]==1)&&(Force_Grnd[i][2]>FZ_Mins)){

/* sigattal plane */
/* Compliance control or return to planned pattern */
	if(Ref_Timecntl[3-i]==0)	//first half	  
		{
		if  ( IZMP_Actl[2] < IZMP_ymargin[i][1] ) {			// ZMP control 
			now_val[i][5] = -KCOM_ANKL_I*(IZMP_Actl[2]- IZMP_ymargin[i][1]-IZMP_MR_RTN);
		}
		else if  ( IZMP_Actl[2] > IZMP_ymargin[i][2] ) {		// ZMP control 
			now_val[i][5] = -KCOM_ANKL_I*(IZMP_Actl[2]- IZMP_ymargin[i][2]+IZMP_MR_RTN);
		}
		else{
	if(COP_R_Count++<=50) { 
    	       if (  ( ZMP_Actl[i][2] > (COP_Ymargin_W[i][2]-2.0*COP_R) ||
		       ZMP_Actl[i][2] < (COP_Ymargin_W[i][1]+2.0*COP_R) ) ) {	//compliance control

	  now_val[i][5] =0.1*KCOM_ANKL*KCOM_ANKL_F*(ZMP_Actl[i][2]-Actl_Ankle_W[i][1]);
		}

              else 
		{

		       	now_val[i][5] = 2.0*0.25*KTMP*K_RETURN*Mdf_Val[i][5];
		}
	}
			
	else //	if(COP_R_Count++<=20)  
	{

		COP_R_Count = 51; 
    	       if (  ( ZMP_Actl[i][2] > (COP_Ymargin_W[i][2]-2.0*COP_R) ||
		       ZMP_Actl[i][2] < (COP_Ymargin_W[i][1]+2.0*COP_R) ) ) {	//compliance control

	  now_val[i][5] =0.1*KCOM_ANKL*KCOM_ANKL_F*(ZMP_Actl[i][2]-Actl_Ankle_W[i][1]);
		}

              else 
		{
//return
		   	now_val[i][5] = 2.0*2.0*0.25*KTMP*K_RETURN*Mdf_Val[i][5];
		}
	
			}
		}
	}
	else  //second half 
		   {   

			if  ( IZMP_Actl[2] < IZMP_ymargin[i][1] ) {			// ZMP control 
				now_val[i][5] = -KCOM_ANKL_I*(IZMP_Actl[2]- IZMP_ymargin[i][1]-IZMP_MR_RTN);
			}
			else if  ( IZMP_Actl[2] > IZMP_ymargin[i][2] ) {		// ZMP control 
				now_val[i][5] = -KCOM_ANKL_I*(IZMP_Actl[2]- IZMP_ymargin[i][2]+IZMP_MR_RTN);
			}
			else {
     if (  ( ZMP_Actl[i][2] > (COP_Ymargin_W[i][2]+1.0*COP_R) ||
		       ZMP_Actl[i][2] < (COP_Ymargin_W[i][1]-1.0*COP_R) ) ) {	//compliance control

	  now_val[i][5] =0.5*0.5*0.1*KCOM_ANKL*KCOM_ANKL_F*(ZMP_Actl[i][2]-Actl_Ankle_W[i][1]);
			}		
			else {		//return to planned pattern
		       	now_val[i][5] = 2.0*2.0*KTMP*K_RETURN*Mdf_Val[i][5];
	       		}
			}
	}

	
  
  //printf("\n3333now_val16=%f,now_val26=%f",now_val[1][6],now_val[2][6]);
/* frontal plane */
			/* Compliance control or return to planned pattern */
if(Ref_Timecntl[3-i]==0)
	{
	if  ( IZMP_Actl[1] < IZMP_xmargin[i][1] ) {			// ZMP control 
		now_val[i][6] = KCOM_ANKL_I*(IZMP_Actl[1]- IZMP_xmargin[i][1]-IZMP_MR_RTN);
	}
	else if  ( IZMP_Actl[1] > IZMP_xmargin[i][2] ) {		// ZMP control 
		now_val[i][6] = KCOM_ANKL_I*(IZMP_Actl[1]- IZMP_xmargin[i][2]+IZMP_MR_RTN);
	}
	else{
	if(COP_R_Count<=50) { 
		if( ( ZMP_Actl[i][1] > (COP_Xmargin_W[i][2]-2.0*COP_R) ||
		       ZMP_Actl[i][1] < (COP_Xmargin_W[i][1]+2.0*COP_R) ) ) {	//compliance control

	  now_val[i][6] =-2.0*0.1*KCOM_ANKL*KCOM_ANKL_F*(ZMP_Actl[i][1]-Actl_Ankle_W[i][0]);
		}
		else
		{
				//return to planned pattern
		       now_val[i][6] =2.0*0.25*K_RETURN*Mdf_Val[i][6];
		}
			
			} //
	else //if (COP_R_)
	{
		if( ( ZMP_Actl[i][1] > (COP_Xmargin_W[i][2]-2.0*COP_R) ||
		       ZMP_Actl[i][1] < (COP_Xmargin_W[i][1]+2.0*COP_R) ) ) {	//compliance control

	  now_val[i][6] =-2.0*0.1*KCOM_ANKL*KCOM_ANKL_F*(ZMP_Actl[i][1]-Actl_Ankle_W[i][0]);
		}
		else
		{
				//return to planned pattern
		       now_val[i][6] =2.0*2.0*0.25*K_RETURN*Mdf_Val[i][6];
		}
		
	}//
	}
	}
	else //second half
		{	
		       
			if  ( IZMP_Actl[1] < IZMP_xmargin[i][1] ) {			// ZMP control 
				now_val[i][6] = KCOM_ANKL_I*(IZMP_Actl[1]- IZMP_xmargin[i][1]-IZMP_MR_RTN);
			}
			else if  ( IZMP_Actl[1] > IZMP_xmargin[i][2] ) {		// ZMP control 
				now_val[i][6] = KCOM_ANKL_I*(IZMP_Actl[1]- IZMP_xmargin[i][2]+IZMP_MR_RTN);
			}
			else {
		if( ( ZMP_Actl[i][1] > (COP_Xmargin_W[i][2]+0.0*COP_R) ||
		       ZMP_Actl[i][1] < (COP_Xmargin_W[i][1]-0.0*COP_R) ) ) {	//compliance control

	  now_val[i][6] =-0.5*2.0*0.5*0.1*KCOM_ANKL*KCOM_ANKL_F*(ZMP_Actl[i][1]-Actl_Ankle_W[i][0]);
					}
		else {			//return to planned pattern
			now_val[i][6] =2.0*2.0*K_RETURN*Mdf_Val[i][6];
		}
	       		}
	} 
		
}
}		///////for777
  //printf("\n4444now_val16=%f,now_val26=%f",now_val[1][6],now_val[2][6]);
//end of 090816 added
                
		if(now_val[i][5] >  Maxank_Step) now_val[i][5] =   Maxank_Step;
        if(now_val[i][5] < -Maxank_Step) now_val[i][5] =  -Maxank_Step;
        if(now_val[i][6] >  Maxank_Step) now_val[i][6] =   Maxank_Step;
        if(now_val[i][6] < -Maxank_Step) now_val[i][6] =  -Maxank_Step;

		if ( Force_Grnd[i][2] <= (FZ_Maxs+DFZ_ANK-DFZ_SWITCH) ){
			P_Ankfz_Switch = 0;
		}
	
//#endif
	Test_now[i][5]=now_val[i][5];
	Test_now[i][6]=now_val[i][6];


///comment out this following 2 lines, the ankle control is active, 
#ifndef USE_ZMP_CONTROL
  for(i = 1; i <= LEG_NUM ; i++){
			now_val[i][5] = 0.0;
            now_val[i][6] = 0.0;//cxc
            Mdf_Val[i][5] = 0.0;//cxc
            Mdf_Val[i][6] = 0.0;//cxc
	}
#endif
	if( (Ref_Qbody[1] > 0.01) || (Ref_Qbody[1] < -0.01)){
//		now_val[i][5] = 0.0;
//		now_val[i][6] = 0.0;
	}

}

   for( i=1; i<=LEG_NUM; i++){
            for(k=5;k<=6;k++){
              Mdf_Val[i][k] += now_val[i][k];
	     	
        }
   }
               




/*** Body posture control uing hip joints ***/ 

/*** Precise body-posture control should be computed by inverse kinemetics combining with foot Dz.
The following body-posture control:
Sigattal body-angle is always controlled by two hip joints, forntal body-angle is controlled by one hip joint
when the robot is supported by this foot.
***/

#define KBODY_KP 0.10 //0.20(OK)//0.30//-0.2 (0.2cxc ok) (-0.2 prof. ok)
#define KBODY_KV -0.1 //0.05// -0.1(-0.1cxc ok)(-0.1 prof. ok)   
#define OMEGA_LPF   20.0
#define Cof1 (2.0*OMEGA_LPF)/(2.0+DTIME*OMEGA_LPF) 
#define Cof2 (2.0-DTIME*OMEGA_LPF)/(2.0+DTIME*OMEGA_LPF)
#define Return_cxc 0.95

//else{//vibration reduction in double support 
	if ( (Force_Grnd[1][2]>0.25*FZ_Maxs)&&(Force_Grnd[1][2]<FZ_Maxs)&&(Force_Grnd[2][2]>0.25*FZ_Maxs)&&(Force_Grnd[2][2]<FZ_Maxs))
	{	//Actual double support//cxc  
		for(i = 1; i <= LEG_NUM ; i++){
			if ( Force_Grnd[i][2] > 0.25*FZ_Maxs ){    // contact 
					y_posture[i][1]=Cof1*Gyro_Filter[1]*DTIME+Cof2*y_posture[i][1];
					y_posture[i][2]=Cof1*Gyro_Filter[2]*DTIME+Cof2*y_posture[i][2];
					now_val_pos[i][5] = KBODY_KP*((0.0-Pitch_Angle[1])-KBODY_KV*y_posture[i][1]);
					now_val_pos[i][6] = KBODY_KP*((0.0-Roll_Angle[1])-KBODY_KV*y_posture[i][2]);
			} 
			else{  /* no contact */ //cxc
					if ((Ref_Cnt_Sgnl[i]==0)&&(Ref_Cnt_Sgnl[3-i]==1)){
						#if 0//cxc return
						now_val_pos[i][5] = Return_cxc*Mdf_Val_Pos[i][5];
						now_val_pos[i][6] = Return_cxc*Mdf_Val_Pos[i][6];
						#endif
					}
			} //end  of if
			if(now_val_pos[i][5] >  Maxpos_Step) now_val_pos[i][5] =  Maxpos_Step;
			if(now_val_pos[i][5] < -Maxpos_Step) now_val_pos[i][5] =  -Maxpos_Step;
			if(now_val_pos[i][6] >  Maxpos_Step) now_val_pos[i][6] =  Maxpos_Step;
			if(now_val_pos[i][6] < -Maxpos_Step) now_val_pos[i][6] =  -Maxpos_Step;
	///comment out the following two line the body-posture control is acctive/
	#ifndef USE_ANKLE_POSTURE_CONTROL
			now_val_pos[i][5] = 0.0; 
			now_val_pos[i][6] = 0.0; 
	#endif
			now_val_pos[i][6] = Return_cxc*Mdf_Val_Pos[i][6]; 
			Mdf_Val_Pos[i][5] = now_val_pos[i][5];  //xt-07-10-5
			Mdf_Val_Pos[i][6] = now_val_pos[i][6];//cxc
		} //end of for(
	}//end of Actual double support
//}//end of else

 
 
 
 
//old posture control, using hip joints 2,3 
   for(i = 1; i <= LEG_NUM ; i++){

	if ( Force_Grnd[i][2] > FZ_Mins ){    // contact 
            if ( Force_Grnd[i][2] > (FZ_Maxs+DFZ_BDY) || (Force_Grnd[i][2] > (FZ_Maxs+DFZ_BDY-DFZ_SWITCH) && 
		 		P_Posfz_Switch == 1) )  {
		  		//now_val[i][2] =0.04*K_HIPY[i-1] * (Force_Grnd[i][2]-FZ_Maxs-DFZ_BDY+DFZ_SWITCH)*(0.2*Q_body[2]-1.0*Ref_Qbody[2])/(WEIGHT-FZ_Maxs-DFZ_BDY+DFZ_SWITCH);  //roll: frontal plane
				// now_val[i][3] =0.04*K_HIPX[i-1] * (Force_Grnd[i][2]-FZ_Maxs-DFZ_BDY+DFZ_SWITCH)*(Q_body[1]-1.0*Ref_Qbody[1])/(WEIGHT-FZ_Maxs-DFZ_BDY+DFZ_SWITCH); // pitch: sigattal plane
		  		P_Posfz_Switch = 1;
		  		now_val[i][2] = K_HIPY[i-1] * (Force_Grnd[i][2]-FZ_Maxs-DFZ_BDY+DFZ_SWITCH)*(Q_body[2]-0.2*Ref_Qbody[2])/(WEIGHT-FZ_Maxs-DFZ_BDY+DFZ_SWITCH);  //roll: frontal plane

				 now_val[i][3] = K_HIPX[i-1] * (Force_Grnd[i][2]-FZ_Maxs-DFZ_BDY+DFZ_SWITCH)*(Q_body[1]-0.2*Ref_Qbody[1])/(WEIGHT-FZ_Maxs-DFZ_BDY+DFZ_SWITCH); // pitch: sigattal plane
	    	} else{ 
				now_val[i][2] = 0.006*K_RETURN_PS*Mdf_Val[i][2]*(FZ_Maxs+DFZ_BDY+FZ_Mins)/(Force_Grnd[i][2]+FZ_Mins);
             	 now_val[i][3] = 0.006*K_RETURN_PS*Mdf_Val[i][3]*(FZ_Maxs+DFZ_BDY+FZ_Mins)/(Force_Grnd[i][2]+FZ_Mins);
	   		}
	
	}  else {   
                             /* no contact */
        now_val[i][2] = K_RETURN_PS*Mdf_Val[i][2];
       	now_val[i][3] = K_RETURN_PS*Mdf_Val[i][3];
    }

	if(now_val[i][2] >  Maxpos_Step) now_val[i][2] =  Maxpos_Step;
	if(now_val[i][2] < -Maxpos_Step) now_val[i][2] =  -Maxpos_Step;
	if(now_val[i][3] >  Maxpos_Step) now_val[i][3] =  Maxpos_Step;
	if(now_val[i][3] < -Maxpos_Step) now_val[i][3] =  -Maxpos_Step;


     if (Force_Grnd[i][2] < (FZ_Maxs+DFZ_BDY-DFZ_SWITCH))
		 P_Posfz_Switch = 0; 


    } //end for


		if(now_val[i][2] >  Maxpos_Step) now_val[i][2] =  Maxpos_Step;
		if(now_val[i][2] < -Maxpos_Step) now_val[i][2] =  -Maxpos_Step;
		if(now_val[i][3] >  Maxpos_Step) now_val[i][3] =  Maxpos_Step;
		if(now_val[i][3] < -Maxpos_Step) now_val[i][3] =  -Maxpos_Step;
//add modified value to joint ref
    for( i=1; i<=LEG_NUM; i++){
     	for(k=2;k<=3;k++){
        	//Mdf_Val[i][k] += now_val[i][k]- BVL_T[k-2]*Gyro_Filter[4-k]; //Gyro_Filter[1]->pitch
        	Mdf_Val[i][k] += now_val[i][k];//- BVL_T[k-2]*Gyro_Filter[4-k]; //Gyro_Filter[1]->pitch
        //	Mdf_Val[i][k] = 1.5*M_PI/180.0;	//test
         }        
	}

#ifndef USE_HIP_POSTURE_CONTROL
	for( i=1; i<=LEG_NUM; i++){
   		Mdf_Val[i][2] = 0.0;//cxc
        Mdf_Val[i][3] = 0.0;
	}
#endif
       // Mdf_Val[1][3] = 0.0;
        //Mdf_Val[2][3] = 0.0;

/**********END of Posture Control********************/


 ///// new dz and ref_ankle control

 /***** Qhip, Qknee real-time computation *****/

/**************** Computation of foot Dz ***********************/
// Ref_Timecntl[i] is signal for land-time control, and is a necessary signal.

#if 1 
      for(i = 1; i <= LEG_NUM; i++){

	if ( Ref_Cnt_Sgnl[i] == 0 && Force_Grnd[i][2] >= FZ_MINS_DZ &&
		Ref_Timecntl[i] == 1 ) {
                	/* Land too fast */ 
                Land_Flag[i]=1;
		if (Force_Grnd[i][2] >FZ_MAX_DZ ){                       
		dz_foot[i] =0.1*TIME_D2S*TIME_D2S*0.075*KCOMP_DZ*(Force_Grnd[i][2]-FZ_MINS_DZ);
                Land_Flag[i]=2;
		}
		else {
		dz_foot[i] =0.1*TIME_D2S*0.015*KCOMP_DZ*(Force_Grnd[i][2]-FZ_MINS_DZ);
                Land_Flag[i]=3;
		}
		Park[i]=0;
		if(dz_foot[i] < 0) dz_foot[i] = 0.0;
		if( dz_foot[i] >= Max_Dz_Step ) dz_foot[i] = Max_Dz_Step;

		P_control=1;//xt-07-10-24

                }

	else if ( Ref_Cnt_Sgnl[i] == 1 && Force_Grnd[i][2] < 0.5*FZ_MINS_DZ &&
		Ref_Timecntl[i] == 1 ){
                	/* Land too late */
	  	dz_foot[i] = 0.1*Min_Dz_Step;//1.01
		P_control=0;        //xt-07-10-24
                Land_Flag[i]=4;
	}

	else if ( (Ref_Cnt_Sgnl[i] == 1 && Force_Grnd[i][2] > FZ_Maxs+FZ_DZ) ||
	 	  (Ref_Cnt_Sgnl[i] == 1 && P_Fotfz_Switch == 1 &&
		   Force_Grnd[i][2] > (FZ_Maxs+FZ_DZ-DFZ_SWITCH)) ) {
		Park[i]=Park[i]+1; //xt-07-5-28
		P_Fotfz_Switch = 1;
	       	dz_foot[i] = 3.0*K_RETURN_DZ*(Force_Grnd[i][2]-FZ_Maxs-FZ_DZ+DFZ_SWITCH)*Dz_Val[i]/(WEIGHT-FZ_Maxs-FZ_DZ+DFZ_SWITCH);
                Land_Flag[i]=5;

		}


        else if ( Ref_Cnt_Sgnl[i] == 0 && Force_Grnd[i][2] < FZ_MINS_DZ){
		dz_foot[i] =2.0*K_RETURN_DZ*Dz_Val[i];
		if (Ref_Timecntl[i] ==1 ){
			dz_foot[i] = 2.0*K_RETURN_DZ*KS_RETURN_DZ*Dz_Val[i];
                Land_Flag[i]=6;
		}
	}
	else {
		dz_foot[i] = 0.0;
	}
        
	if (Force_Grnd[i][2] < (FZ_Maxs-DFZ_SWITCH)) 	P_Fotfz_Switch=0;
	
//	Test_dz[i] = dz_foot[i];  //for test dz_foot xt-07-9-13

  }

        /** Sum of dz_foot **/
        for(i = 1; i <= LEG_NUM ; i++){

		if(P_control == 1 && Ref_Cnt_Sgnl[i] == 0) {
			dz_ref_ankle[i] = Ref_Ankle[i][2] - P_ZAnkle[i];
//	if((Plan_Count<=15200)||(Plan_Count>=15500))//cxc
			Dz_Val[i] =Dz_Val[i]+dz_foot[i]+dz_ref_ankle[i];
			Test_dz[i] = dz_ref_ankle[i];
		} 
                else
		{
//			if((Plan_Count<=15200)||(Plan_Count>=15500))//cxc
			Dz_Val[i] =Dz_Val[i]+dz_foot[i];
                }
		if ( Dz_Val[i] >= 0.015  ) Dz_Val[i] =  0.015;
                if ( Dz_Val[i] <= -0.015 ) Dz_Val[i] = -0.015;
 //comment the following line, active Dz_Val control
//	if((Plan_Count>15200)&&(Plan_Count<15500))//cxc
	//		Dz_Val[i]=0.99*Dz_Val[i];
#ifndef USE_DZ_CONTROL
			Dz_Val[i] = 0.0 ;//cxc
	//	P_ZAnkle[i] = Ref_Ankle[i][2];
#endif		
        }
        

/******************* computation of foot D_ANKLE **********************/

	Actl_Ankle_Position();  //xt-07-6-19
	for(i = 1; i <= LEG_NUM; i++)
	   for(k = 0; k < 3; k++){
		d_ankle[i][k] = 0.0;   //xt-07-6-19
      }

	for(i = 1; i <= LEG_NUM; i++){

	if(LEG_NO == i && Ref_Cnt_Sgnl[i] == 1 && Force_Grnd[i][2] >= FZ_Weight)	{
	Park[i]+=1;
	//if(Park[LEG_NO] > Nmax)   {
	if(Park[LEG_NO] > 1)   
	{  //08-11-14
	Park[LEG_NO] = Nmax+1;
	for(k = 0; k < 3; k++)  
	    d_ankle[i][k]=K_ANKLE*(Actl_Ankle[LEG_NO][k]-Ref_Ankle[LEG_NO][k]);
	}
	}
        
	else if(LEG_NO == i && Ref_Cnt_Sgnl[i] == 1 && Force_Grnd[i][2] > 3.0*FZ_MINS_DZ && Force_Grnd[i][2] < FZ_Weight){
	Park[i]+=1;
	if(Park[LEG_NO] > Nmax)   {
		Park[LEG_NO] = Nmax+1;
		for(k = 0; k < 3; k++){  
		    d_ankle[1][k]=K_ANKLE*(Actl_Ankle[LEG_NO][k]-Ref_Ankle[LEG_NO][k]);
	d_ankle[1][2] = K_ANKLE * (Actl_Ankle[1][2]-Ref_Ankle[1][2]);
		    d_ankle[2][k]=K_ANKLE*(Actl_Ankle[LEG_NO][k]-Ref_Ankle[LEG_NO][k]);  //xt-10-30
	d_ankle[2][2] = K_ANKLE * (Actl_Ankle[2][2]-Ref_Ankle[2][2]);
			}
		}
	}

	else if(LEG_NO == 0 && Force_Grnd[1][2] > FZ_MINS_DZ && Force_Grnd[2][2] > FZ_MINS_DZ){
//	Park[i] = 0;

	Park[LEG_NO]+=1;

        if(Parkstart==1){
	for(k = 0; k < 3; k++)  
	d_ankle[i][k] =  0.005 * K_ANKLE * (Actl_Ankle[1][k]-Ref_Ankle[1][k]+Actl_Ankle[2][k]-Ref_Ankle[2][k]);

	d_ankle[i][2] = 0.01 * K_ANKLE * (Actl_Ankle[i][2]-Ref_Ankle[i][2]);
	if(Park[LEG_NO] > 20*Nmax)    Parkstart=0;
	}
        
	if(Park[LEG_NO] > 20*Nmax)   {
		Park[LEG_NO] = Nmax+1;
/*	for(k = 0; k < 3; k++)  
		d_ankle[i][k] =0.1 * 0.5 * K_ANKLE * (Actl_Ankle[1][k]-Ref_Ankle[1][k]+Actl_Ankle[2][k]-Ref_Ankle[2][k]);
*/
	for(k = 0; k < 3; k++)  
		d_ankle[i][k] =0.025 * K_ANKLE * (Actl_Ankle[1][k]-Ref_Ankle[1][k]+Actl_Ankle[2][k]-Ref_Ankle[2][k]);

	d_ankle[i][2] = 0.05 * K_ANKLE * (Actl_Ankle[i][2]-Ref_Ankle[i][2]);
	}
	}

        else if ( Ref_Cnt_Sgnl[i] == 0 && Force_Grnd[i][2] < FZ_MINS_DZ){
	Park[i]=0; 
	for(k = 0; k < 3; k++)  
		d_ankle[i][k]=2.0*K_RETURN_ANKLE * D_ANKLE[i][k];
	}

	else {
	Park[i] = 0;
	}
	
	for(k = 0; k < 3; k++){
	if(d_ankle[i][k] >  Max_dankle) d_ankle[i][k] =  Max_dankle;
	if(d_ankle[i][k] < -Max_dankle) d_ankle[i][k] =  -Max_dankle;
	}
	for(k = 0; k < 3; k++)
	 	Test_dankle[i][k] = d_ankle[i][k];  //test d_ankle xt-07-9-16
  }
                /**sum of d_ankle**/     //xt-07-6-19

	for(i = 1; i <= LEG_NUM; i++){
		for(k = 0; k < 3; k++){       
		  D_ANKLE[i][k] = D_ANKLE[i][k] + d_ankle[i][k];
		  if( D_ANKLE[i][k] >= 0.06 ) D_ANKLE[i][k] = 0.06;//0.03
		  if( D_ANKLE[i][k] <= -0.06) D_ANKLE[i][k] = -0.06;
         //comment the following line, active D_ANKLE control, yuzg-09-04-08
#ifndef USE_DANKLE_CONTROL    
		 D_ANKLE[i][k] = 0.0;//cxc
#endif
			}
		}
	for(i = 1; i <= LEG_NUM ; i++)         //xt-07-6-19
		for(k = 0; k < 3; k++){
			Ref_Ankle_D2S[i][k] = Ref_Ankle[i][k] + D_ANKLE[i][k];
       		}


#endif
//Reflex Modification Limit, 2009-07-27

	for(i=1; i<=LEG_NUM; i++)
	     for(k=1; k<=6;k++)
     {       
	   if( Mdf_Val[i][k] >= MAX_MDF_LIMIT) 
		Mdf_Val[i][k]=MAX_MDF_LIMIT;
	   if( Mdf_Val[i][k] <=-MAX_MDF_LIMIT) 
		Mdf_Val[i][k]=-MAX_MDF_LIMIT;
	}
 /************add by xt---------new inv_leg uses*************/ 
 
		/** computation xknee, yknee, zknee **/

#if 1       //new inverse(peng)
for (i = 1; i <= LEG_NUM ; i++){
	      Wq_Mtrc[1] = 0.0;//Q_body[1];//-Ref_Qbody[1];
	      Wq_Mtrc[2] = 0.0;//Q_body[2];//-Ref_Qbody[2];
	      Matric_Wq();	
	      //Position at World Coordiante System

#if 1   //xt-07-6-6 for use final by xt

	      t_ankle[i][0] = Ref_Ankle_D2S[i][0]-Dz_Val[i]*Sin_RfQbody[2]*Wi_Matric[0][0]+
	                                      Dz_Val[i]*Cos_RfQbody[2]*Wi_Matric[2][0];   ///should be corrected
	      t_ankle[i][1] = Ref_Ankle_D2S[i][1]-Dz_Val[i]*Sin_RfQbody[2]*Wi_Matric[0][1]+
	                                      Dz_Val[i]*Cos_RfQbody[2]*Wi_Matric[2][1];
	      t_ankle[i][2] = Ref_Ankle_D2S[i][2]-Dz_Val[i]*Sin_RfQbody[2]*Wi_Matric[0][2]+
	                                      Dz_Val[i]*Cos_RfQbody[2]*Wi_Matric[2][2];
	  	      	     
#endif	  	      	     

          ldashi[i][0] = sqrt( (xwaist[i]-t_ankle[i][0])* (xwaist[i]-t_ankle[i][0])+(t_ankle[i][1])*(t_ankle[i][1])+t_ankle[i][2]*t_ankle[i][2]);

  if ( (LENG_LEG*LENG_LEG-0.25*ldashi[i][0]*ldashi[i][0]) > 0.0 ){
                  Rtf_Val[i][1]=Ref_Val[i][1]/Gear_Leg[i][0];	
  	          SIN_Q1=sin(Rtf_Val[i][1]);
              	  COS_Q1=cos(Rtf_Val[i][1]);
              	  
              	  
		  tad_ankle[i][0] = t_ankle[i][0];//*cos(Qhw)-t_ankle[i][1]*sin(Qhw);
		  tad_ankle[i][1] = t_ankle[i][1];//*sin(Qhw)+t_ankle[i][1]*cos(Qhw);
		  tad_ankle[i][2] = t_ankle[i][2];

		  a=(tad_ankle[i][0]-xwaist[i])*SIN_Q1-(tad_ankle[i][1])*COS_Q1;
		  b=(tad_ankle[i][0]-xwaist[i])*COS_Q1-(tad_ankle[i][1])*SIN_Q1;
              	  c=tad_ankle[i][2];
		  n2=b;
		  d2=c;
		  n4=pow((tad_ankle[i][0]-xwaist[i]),2.0)+pow((tad_ankle[i][1]),2.0)+pow(tad_ankle[i][2],2.0)-2*pow(L,2.0);
		 
		  Rtf_Val[i][4]=-acos(n4/(2*LENG_LEG*LENG_LEG));
		  Rtf_Val[i][2]=atan(n2/d2);
		  SIN_Q4=sin(Rtf_Val[i][4]);
          	  COS_Q4=cos(Rtf_Val[i][4]);
          	  SIN_Q2=sin(Rtf_Val[i][2]);
          	  COS_Q2=cos(Rtf_Val[i][2]);
          	  n3=a*COS_Q2*(L+L*COS_Q4)-L*c*SIN_Q4;
          	  d3=c*(L+L*COS_Q4)+L*a*COS_Q2*SIN_Q4;
          	  Rtf_Val[i][3]=atan(n3/d3);

#if 1  //old
			//Ref_Qbody[2]=0.0;//cxc			

		Rtf_Val[i][2] = Ref_Val[i][2]/Gear_Leg[i][1];
		Rtf_Val[i][5] = Ref_Val[i][5]/Gear_Leg[i][4];	  
		Rtf_Val[i][3] = Ref_Val[i][3]/Gear_Leg[i][2];
		Rtf_Val[i][6] = Ref_Val[i][6]/Gear_Leg[i][5];
			  
      //08-11-15
            if(((LEG_NO==i)&&(Force_Grnd[i][2]>0.2*FZ_CONT))||
		((LEG_NO!=i)&&(Force_Grnd[i][2]>0.4*FZ_CONT)))
      		      {	
                      P_Count[i]++;
                     if(P_Count[i]>=Nmax) P_Count[i]=Nmax+1;
                   }
      	   // if(P_Count[i]>=Nmax)
                 {
					 #if 0
				 #if 0	 
				  Rtf_Val[i][6] = Ref_Val[i][6]/Gear_Leg[i][5]+Ref_Val[i][2]/Gear_Leg[i][1]-Ref_Val[i][2]/Gear_Leg[i][1]- 1.0*Ref_Qbody[2];//buchang YTQ 1.2  //1.5-0.4km/h-20161125
				  Rtf_Val[i][2]=Ref_Val[i][2]/Gear_Leg[i][1]-0.5*Ref_Qbody[2];//0.5  //0.8-0.4km/h-20161125//
				 #endif			 

				 if (i==1) // Right 
				 {
					Rtf_Val[i][6] = Ref_Val[i][6]/Gear_Leg[i][5]+Ref_Val[i][2]/Gear_Leg[i][1]-Ref_Val[i][2]/Gear_Leg[i][1]- 0.0*0.3*Ref_Qbody[2]; //0.5//0.6  //0.3
					Rtf_Val[i][2]=Ref_Val[i][2]/Gear_Leg[i][1]- 0.0*1.2*Ref_Qbody[2]; //0.6//1.0  //0.5            
				 }
				 else
				 {
					Rtf_Val[i][6] = Ref_Val[i][6]/Gear_Leg[i][5]+Ref_Val[i][2]/Gear_Leg[i][1]-Ref_Val[i][2]/Gear_Leg[i][1]- 0.0*0.2*Ref_Qbody[2]; //0.3//0.6  //0.3
					Rtf_Val[i][2]=Ref_Val[i][2]/Gear_Leg[i][1]- 0.0*1.2*Ref_Qbody[2];//0.8//1.6 //0.8
				 }
					#endif
			 
  		}
           // else
       	if(Force_Grnd[i][2]<0.2*FZ_CONT)
            {
		if(P_Count2[i]++>=Nmax)
              {	 P_Count[i]=0;
		 P_Count2[i]=0;
    		}
           }
//2010-07-09

		
		Rtf_Val[i][1]=Ref_Val[i][1]/Gear_Leg[i][0];
		Rtf_Val[i][4]=Ref_Val[i][4]/Gear_Leg[i][3];
#endif


 }
        
#endif

/***********add by xt-------new inv_leg uses************/
}

}



