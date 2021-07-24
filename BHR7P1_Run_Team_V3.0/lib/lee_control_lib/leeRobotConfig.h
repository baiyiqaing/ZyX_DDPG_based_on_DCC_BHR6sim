/*--------------------------------------------------------------/
■ Lee, 2019-03-08, 创建, 与机器人型号相关参数定义
/*-------------------------------------------------------------*/
#ifndef _ROBOT_CONFIG_
#define _ROBOT_CONFIG_

	#ifndef VREP_SIM

		//#define _BHR_5_
		#define _BHR_6_
		//#define _BHR_6P_

		#ifdef _BHR_6_//BHR-6
			#define LEAD_CORRECTION_TIME 0.0//0.077//0.08
			#define LEAD_CORRECTION_RATE 0//0.040//0.02
			#define QBODY_LEFT_2 0.0
			#define QBODY_LEFT_6 1.2
			#define QBODY_RIGHT_2 0.0
			#define QBODY_RIGHT_6 1.2
			#define	ROBOT_WEIGHT ((535.0 + 70.0)/9.8 + 2.0 + 1.0)//( 56.371 + 50/9.8)//(570.0/9.8)//kg
		#endif

		#ifdef _BHR_6P_//BHR-6P
			#define LEAD_CORRECTION_TIME 0.08
			#define LEAD_CORRECTION_RATE 0.02
			#define QBODY_LEFT_2 0.0
			#define QBODY_LEFT_6 1.2
			#define QBODY_RIGHT_2 0.0
			#define QBODY_RIGHT_6 0.7
			#define	ROBOT_WEIGHT (520.0/9.8)//(552.0/9.8)//(570.0/9.8)//kg
		#endif

		#ifdef _BHR_5_//BHR-5
			#define LEAD_CORRECTION_TIME 0.037//0.05
			#define LEAD_CORRECTION_RATE 0.00
			#define QBODY_LEFT_2 1.8
			#define QBODY_LEFT_6 0.2
			#define QBODY_RIGHT_2 1.8
			#define QBODY_RIGHT_6 0.4
			#define	ROBOT_WEIGHT (369.0/9.8)//KG 
		#endif


	#else
		#define _BHR_6P_
		#define LEAD_CORRECTION_TIME 0//0.06
		#define LEAD_CORRECTION_RATE 0.00//0.0
		#define QBODY_LEFT_2 0
		#define QBODY_LEFT_6 0
		#define QBODY_RIGHT_2 0
		#define QBODY_RIGHT_6 0
		#define	ROBOT_WEIGHT (56.368)//(49.63)
	#endif

#endif