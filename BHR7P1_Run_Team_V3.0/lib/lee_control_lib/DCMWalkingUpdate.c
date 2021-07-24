#include "DCMWalkingUpdate.h"
void dcm_walk_init(BRobot *bhr6)
{
	double ss_rate;
#ifndef VREP_SIM
	int i, j;
	//粘弹性控制器参数填在这里
	//k1 = 1000, b1 = 100, k2 = 500 力矩控制粘弹性模型参数
	//-0.3732416040,-0.0005596609,190.3103471362,27.6102124236,	// 1.0000000000
	//-0.2300869041,-0.0003450062,148.0618023534,23.0339756306,	// 2.0000000000
	//-0.1175715432,-0.0001762939,104.4508637379,18.1207996875,	// 5.0000000000
	//-0.0690824179,-0.0001035863,79.1840812949,15.1156725769,	// 10
	//-0.0501508264,-0.0000751992,67.0024048658,13.5972849780,	// 15.0000000000
	//-0.0397983739,-0.0000596761,59.3846019032,12.6150690796,	// 20.0000000000
	//-0.0285756220,-0.0000428480,49.9486178892,11.3524999772	// 30.0000000000
	//-0.0186646128,-0.0000279868,39.9801921072,9.9442956312,	// 50
	//-0.0103245031,-0.0000154812,29.3370871056,8.3153604745,	// 100
	//-0.0056277465,-0.0000084386,21.3661147620,6.9599367375,	// 200
	//-0.0024744180,-0.0000037103,13.9180826994,5.5087911830,	// 500
	//-0.0013123238,-0.0000019678,10.0061125418,4.6199622884,	// 1000
	double vmck[6][4] = {-0.0186646128,-0.0000279868,39.9801921072,9.9442956312,	// 50
		-0.0186646128, -0.0000279868,39.9801921072,9.9442956312,	// 50
		-0.3732416040, -0.0005596609,190.3103471362,27.6102124236,	// 1.0000000000
		-0.0103245031,-0.0000154812,29.3370871056,8.3153604745,	// 100
		-0.0012527591, -0.0000037506, 12.5058054319, 5.6215246589,
		-0.0023988822, -0.0000071819, 17.3771893846,  6.8828123561};//500
		//-0.0055658138, -0.0000166632, 26.6059164466,  9.0826458118};//200
		//-0.0103477193, -0.0000309795, 36.4080545301, 11.2818183290};//100
		//-0.0189015586, -0.0000565883, 49.3721813611, 14.0824053096};//50

	//平衡控制器参数写在这里
	double vdt_k[5] = { 17.439141,45.056227,129.243453,89.501812,17.682256 };
	//double vdt_k[5] = { -3.6263120641,156.7290985693,2891.5406267562,532.1734583605,41.8092449744 };	//0.0000100000
	//double vdt_k[5] = { -1.9787455521,62.8229920322,953.8882379664,213.3153909487,22.8137722478 };	//0.0001000000
	//double vdt_k[5] = { -1.2291048911,26.8243179621,307.1935430265,91.0819381235,14.1708563920};		//0.0010000000
	//double vdt_k[5] = { -0.8108573293,11.9761387428,98.1160710420,40.6649641369,9.3487080318 };		//0.0100000000
	//double vdt_k[5] = { -0.5464744208,5.4670911767,31.2212963880,18.5635012593,6.3005286166};			//0.1000000000

	//TPC控制器参数
	double tpc_k[3] = { -15.828681,54.374267,32.946091 };


	//步态参数填在这里
	bhr6->dcmSetTab[DCM_SET_SYS_CIRCLE] = 0.004;	//控制周期[s]
	bhr6->dcmSetTab[DCM_SET_STEP_TIME] = 0.8;		//步行周期，单步时间[s]
	bhr6->dcmSetTab[DCM_SET_SS_RATE] = 0.8;			//单脚支撑期比例
	bhr6->dcmSetTab[DCM_SET_STEP_LEN] = 0.0;		//步长[m]
	bhr6->dcmSetTab[DCM_SET_STEP_WIDTH] = 0.0;		//横向移动步长
	bhr6->dcmSetTab[DCM_SET_STEP_WIDTH] = 0.0;		//横向步长[m]
	bhr6->dcmSetTab[DCM_SET_WALK_HEIGHT] = 0.68+0.127;	//行走时质心高度[m];
	bhr6->dcmSetTab[DCM_SET_STEP_NUM] = 10.0;		//步数
	bhr6->dcmSetTab[DCM_SET_FOOT_HEIGHT] = 0.05;	//抬脚高度[m]
	
	bhr6->dcmSetTab[DCM_SET_LEAD_COMP_TIME] = LEAD_CORRECTION_TIME;
	bhr6->dcmSetTab[DCM_SET_LEAD_COMP_ALPHA] = LEAD_CORRECTION_RATE;


	printf("walking speed is %f km/h\n", bhr6->dcmSetTab[DCM_SET_STEP_LEN] / bhr6->dcmSetTab[DCM_SET_STEP_TIME] * 3.6);
	printf("step number is %d\n", (int)bhr6->dcmSetTab[DCM_SET_STEP_NUM]);

	for (i = 0; i<6; i++)
	{
		for (j = 0; j<4; j++)
		{
			bhr6->dcmSetTab[DCM_SET_VMC_K_RX_1 + i * 4 + j] = vmck[i][j];
		}
	}

	for (i = 0; i < 5; i++)
	{
		bhr6->dcmSetTab[DCM_SET_VDT_K1 + i] = vdt_k[i];
	}

	for (i = 0; i < 3; i++)
	{
		bhr6->dcmSetTab[DCM_SET_TPC_K1 + i] = tpc_k[i];
	}
	

	DCM_JOINT_TRANS[0] = 1.0;
	DCM_JOINT_TRANS[1] = 1.0;
	DCM_JOINT_TRANS[2] = -1.0;
	DCM_JOINT_TRANS[3] = -1.0;
	DCM_JOINT_TRANS[4] = -1.0;
	DCM_JOINT_TRANS[5] = 1.0;
#endif

	bhr6->DCM.Step_Time = bhr6->dcmSetTab[DCM_SET_STEP_TIME];
	ss_rate = bhr6->dcmSetTab[DCM_SET_SS_RATE];
	bhr6->DCM.Step_Time_SS = bhr6->DCM.Step_Time*ss_rate;
	bhr6->DCM.Step_Time_DS = bhr6->DCM.Step_Time * (1.0 - ss_rate);
	bhr6->DCM.Step_Time_DS_Ini = bhr6->DCM.Step_Time_DS * 0.5;
	bhr6->DCM.Step_Time_DS_End = bhr6->DCM.Step_Time_DS * 0.5;
	bhr6->DCM.Step_Length = bhr6->dcmSetTab[DCM_SET_STEP_LEN];
	bhr6->DCM.Step_Num = (int)bhr6->dcmSetTab[DCM_SET_STEP_NUM];
	bhr6->DCM.COM_Height = bhr6->dcmSetTab[DCM_SET_WALK_HEIGHT];
	bhr6->DCM.B = sqrt(bhr6->DCM.COM_Height / 9.8);
	bhr6->DCM.Step_Width = (0.9*BASE_HIP_Y) / 1000.0;
	bhr6->DCM.Foot_Height = bhr6->dcmSetTab[DCM_SET_FOOT_HEIGHT];
	bhr6->DCM.Current_Step = 0;
	bhr6->DCM.Current_Time = 0;
}

void dcm_walking(BRobot *bhr6)
{
	double ini_base[6] = { 0 };
	double ini_joint[21] = { 0 };
	int i1;

#ifndef VREP_SIM
	ini_joint[LEFT_LEG_3] = RAD(-10);
	ini_joint[LEFT_LEG_4] = RAD(20);
	ini_joint[LEFT_LEG_5] = RAD(-10);
	ini_joint[RIGHT_LEG_3] = RAD(-10);
	ini_joint[RIGHT_LEG_4] = RAD(20);
	ini_joint[RIGHT_LEG_5] = RAD(-10);
	ini_base[5] = FOOT_HEIGHT + CRUS_LEN*_rcos(RAD(10)) + THIGH_LEN*_rcos(RAD(10)) + BASE_HIP_Z;
#else
	ini_base[5] = FOOT_HEIGHT + CRUS_LEN + THIGH_LEN + BASE_HIP_Z;
#endif
	ini_joint[LEFT_ARM_3] = RAD(-50.0);
	ini_joint[RIGHT_ARM_3] = RAD(-50.0);
	//bhr6 初始化设置
	if (bhr6->Init_Flag != HAD_SET)
	{
		dcm_walk_init(bhr6);
		two_link_model_init(bhr6);
		dcm_init(bhr6);
		vmc6_init(bhr6);
		zmp_tpc_init(bhr6);
		bhr6->VMC6.RealBodyPosture.p[5] = ini_base[5] / 1000.0;//设置初始Body/Base位置
		robot_init(bhr6);
		bhr6->Squat.inter_time = bhr6->dcmSetTab[DCM_SET_SYS_CIRCLE] * 1000.0;

		for (i1 = 0; i1 < 6; i1++)
		{
			bhr6->CurrentState.Posture[BASE].Posture.p[i1] = ini_base[i1];
		}
		for (i1 = 0; i1 < JOINT_NUMBER; i1++)
		{
			bhr6->CurrentState.Joint[i1] = ini_joint[i1];
			bhr6->CurrentState.O_Joint[i1] = ini_joint[i1];
		}
		bhr6->CurrentState.Posture[BASE].T = get_t(&(bhr6->CurrentState.Posture[BASE].Posture));
		update_current(bhr6);

		dcm_com_est_init(bhr6);
		init_filters(bhr6);
		init_estimators(bhr6);
	}

	//执行
	load_data(bhr6);

	//规划与控制更新
	robot_update(bhr6);
	
	//输出关节角
	for (i1 = 0; i1 < 6; i1++)
	{
		bhr6->dcmOutputTab[DCM_OUTPUT_JOINT_L1 + i1] = bhr6->CurrentState.Joint[LEFT_LEG_1 + i1];
		bhr6->dcmOutputTab[DCM_OUTPUT_JOINT_R1 + i1] = bhr6->CurrentState.Joint[RIGHT_LEG_1 + i1];
	}
	for (i1 = 0; i1 < 3; i1++)
	{
		bhr6->dcmOutputTab[DCM_OUTPUT_ARM_L1 + i1] = bhr6->CurrentState.Joint[LEFT_ARM_1 + i1];
		bhr6->dcmOutputTab[DCM_OUTPUT_ARM_R1 + i1] = bhr6->CurrentState.Joint[RIGHT_ARM_1 + i1];
		bhr6->dcmOutputTab[DCM_OUTPUT_WAIST_1 + i1] = bhr6->CurrentState.Joint[WAIST_1 + i1];
	}

	//腿2,6关节加上QBody补偿
	bhr6->dcmOutputTab[DCM_OUTPUT_JOINT_L2] -= QBODY_LEFT_2*bhr6->CurrentState.Posture[QBODY].Posture.p[0];
	bhr6->dcmOutputTab[DCM_OUTPUT_JOINT_L6] -= QBODY_LEFT_6*bhr6->CurrentState.Posture[QBODY].Posture.p[0];
	bhr6->dcmOutputTab[DCM_OUTPUT_JOINT_R2] -= QBODY_RIGHT_2*bhr6->CurrentState.Posture[QBODY].Posture.p[0];
	bhr6->dcmOutputTab[DCM_OUTPUT_JOINT_R6] -= QBODY_RIGHT_6*bhr6->CurrentState.Posture[QBODY].Posture.p[0];

	//输出结束判断标志（系统当前状态）
	bhr6->dcmOutputTab[DCM_OUTPUT_STATE_FLAG] = (double)(bhr6->CurrentState.StateFlag);

	if (bhr6->CurrentState.StateFlag == 0)
	{
		bhr6->Init_Flag = NOT_SET;
	}
}

void vmc6_init(BRobot *bhr6)
{
	int i, j;
	//粘弹性控制器调节量上下限
	double delta_limit_max[6] = { RAD(5.0),RAD(5.0),RAD(30.0),0.02,0.02,0.02};
	double delta_limit_min[6] = { RAD(-5.0),RAD(-5.0),RAD(-30.0),-0.02,-0.02,-0.02 };
	//姿态控制器系数
	double Posture_Control_Ankle_K[4] = {22077.8962128406,7052.3856508941,39.6683894053,8.8768592499};//R=0.000001
	//double Posture_Control_Ankle_K[4] = {992560.2364021223,317061.7370198448,1715.0586405719,239.8806470347};//R=0.000001
	//double Posture_Control_Ankle_K[4] = {136201.5772035130,43507.9978755992,238.0126123150,36.4533740365};//R=0.001000
	//double Posture_Control_Ankle_K[4] = {53604.4767513480,17123.3219821976,95.5277353877,16.8256244659};//R=0.010000
	//double Posture_Control_Ankle_K[4] = { 25731.6879169269,8219.6861928171,47.3960953062,10.1863553134 };//R=0.100000
	//double Posture_Control_Ankle_K[4] = {-16374.0863240345,-5230.5100126668,31.1471091251,7.9286431446};//R=1.000000
	//double Posture_Control_Ankle_K[4] = {14987.6950941389,4787.6435794016,28.7143230428,7.5860399349};//R=2.000000
	//double Posture_Control_Ankle_K[4] = {13667.4864742954,4365.9184053951,26.3793024396,7.2539172702};//R=5.000000
	//double Posture_Control_Ankle_K[4] = {12937.7176037241,4132.8022907792,25.0762324002,7.0663683315};//R=10.000000
	//double Posture_Control_Ankle_K[4] = {12370.8946769991,3951.7373485961,24.0551819384,6.9178823556};//R=20.000000
	//double Posture_Control_Ankle_K[4] = {11800.9813271681,3769.6852069596,23.0178384192,6.7650871686};//R=50.000000
	//double Posture_Control_Ankle_K[4] = {11469.4204856028,3663.7719811884,22.4080537914,6.6741658317};//R=100.000000
	//double Posture_Control_Ankle_K[4] = {10919.6814257568,3488.1642800902,21.3838393912,6.5183890331};//R=500.000000
	//double Posture_Control_Ankle_K[4] = {10629.1951658552,3395.3718481377,20.8358001648,6.4383105870};//R=2000.000000
	//double Posture_Control_Ankle_K[4] = {10417.1061330345,3327.6224917564,20.4317924648,6.3826197706};//R=10000.000000
	//double Posture_Control_Ankle_K[4] = {10358.6658721320,3308.9544351849,20.3238195634,6.4098381353};//R=100000.000000
	//double Posture_Control_Ankle_K[4] = {716.2443460638,228.7958637569,1.4055301887,0.4557992534};	//R=1000000000.000000
	ViscoelasticModelCompliance6 *vmc6;
	vmc6 = &bhr6->VMC6;
	
	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < 6; j++)
		{
			vmc6->LQR[i][j].track_num = 2;
			vmc6->LQR[i][j].T = bhr6->dcmSetTab[DCM_SET_SYS_CIRCLE];

			vmc6->LQR[i][j].ref_value = 0.0;
			vmc6->LQR[i][j].real_value = 0.0;
			vmc6->LQR[i][j].ref_value2 = 0.0;
			vmc6->LQR[i][j].real_value2 = 0.0;
			vmc6->LQR[i][j].delta = 0.0;
			vmc6->LQR[i][j].d_delta = 0.0;
			vmc6->LQR[i][j].dd_delta = 0.0;
			vmc6->LQR[i][j].last_delta = 0.0;
			vmc6->LQR[i][j].last_d_delta = 0.0;
			vmc6->LQR[i][j].limit[0] = delta_limit_min[j];
			vmc6->LQR[i][j].limit[1] = delta_limit_max[j];
			vmc6->LQR[i][j].k[0] = bhr6->dcmSetTab[DCM_SET_VMC_K_RX_1+j*4];
			vmc6->LQR[i][j].k[1] = bhr6->dcmSetTab[DCM_SET_VMC_K_RX_2+j*4];
			vmc6->LQR[i][j].k[2] = bhr6->dcmSetTab[DCM_SET_VMC_K_RX_3+j*4];
			vmc6->LQR[i][j].k[3] = bhr6->dcmSetTab[DCM_SET_VMC_K_RX_4+j*4];
			vmc6->LQR[i][j].Alpha = bhr6->dcmSetTab[DCM_SET_LEAD_COMP_ALPHA];//LEAD_CORRECTION_RATE;
			vmc6->LQR[i][j].Td = bhr6->dcmSetTab[DCM_SET_LEAD_COMP_TIME];//LEAD_CORRECTION_TIME;
			vmc6->LQR[i][j].c_delta = 0;
			vmc6->LQR[i][j].c_d_delta = 0;
			vmc6->LQR[i][j].ep = 0;

			vmc6->RealBodyPosture.p[j] = 0.0;
		}

		for (j = 0; j < 3; j++)
		{
			vmc6->RealBodyVel[j] = 0.0;
			vmc6->RealBodyAcc[j] = 0.0;
		}

	}
	for (i = 0; i < 2; i++)
	{
		vmc6->LQR_Posture_Control_Ankle[i].T = bhr6->dcmSetTab[DCM_SET_SYS_CIRCLE];
		vmc6->LQR_Posture_Control_Ankle[i].track_num = 2;
		vmc6->LQR_Posture_Control_Ankle[i].limit[0] = -500;
		vmc6->LQR_Posture_Control_Ankle[i].limit[1] = 500;
		vmc6->LQR_Posture_Control_Ankle[i].delta = 0.0;
		vmc6->LQR_Posture_Control_Ankle[i].d_delta = 0.0;
		vmc6->LQR_Posture_Control_Ankle[i].dd_delta = 0.0;
		vmc6->LQR_Posture_Control_Ankle[i].k[0] = Posture_Control_Ankle_K[0];
		vmc6->LQR_Posture_Control_Ankle[i].k[1] = Posture_Control_Ankle_K[1];
		vmc6->LQR_Posture_Control_Ankle[i].k[2] = Posture_Control_Ankle_K[2];
		vmc6->LQR_Posture_Control_Ankle[i].k[3] = Posture_Control_Ankle_K[3];
		vmc6->LQR_Posture_Control_Ankle[i].ep = 0.0;
	}
	printf("vmc6: %f,%f,%f,%f\n", vmc6->LQR[0][5].k[0], vmc6->LQR[0][5].k[1], vmc6->LQR[0][5].k[2], vmc6->LQR[0][5].k[3]);

	// LQR_Body_Compliance 初始化
	for (i = 0; i < 6; i++)
	{
		vmc6->LQR_Body_Compliance[i].track_num = 2;
		vmc6->LQR_Body_Compliance[i].T = bhr6->dcmSetTab[DCM_SET_SYS_CIRCLE];

		vmc6->LQR_Body_Compliance[i].ref_value = 0.0;
		vmc6->LQR_Body_Compliance[i].real_value = 0.0;
		vmc6->LQR_Body_Compliance[i].ref_value2 = 0.0;
		vmc6->LQR_Body_Compliance[i].real_value2 = 0.0;
		vmc6->LQR_Body_Compliance[i].delta = 0.0;
		vmc6->LQR_Body_Compliance[i].d_delta = 0.0;
		vmc6->LQR_Body_Compliance[i].dd_delta = 0.0;
		vmc6->LQR_Body_Compliance[i].last_delta = 0.0;
		vmc6->LQR_Body_Compliance[i].last_d_delta = 0.0;
		vmc6->LQR_Body_Compliance[i].limit[0] = 1.0*delta_limit_min[i];
		vmc6->LQR_Body_Compliance[i].limit[1] = 1.0*delta_limit_max[i];
		vmc6->LQR_Body_Compliance[i].k[0] = bhr6->dcmSetTab[DCM_SET_VMC_K_RX_1 + i * 4];
		vmc6->LQR_Body_Compliance[i].k[1] = bhr6->dcmSetTab[DCM_SET_VMC_K_RX_2 + i * 4];
		vmc6->LQR_Body_Compliance[i].k[2] = bhr6->dcmSetTab[DCM_SET_VMC_K_RX_3 + i * 4];
		vmc6->LQR_Body_Compliance[i].k[3] = bhr6->dcmSetTab[DCM_SET_VMC_K_RX_4 + i * 4];
		vmc6->LQR_Body_Compliance[i].Alpha = bhr6->dcmSetTab[DCM_SET_LEAD_COMP_ALPHA];//LEAD_CORRECTION_RATE;
		vmc6->LQR_Body_Compliance[i].Td = bhr6->dcmSetTab[DCM_SET_LEAD_COMP_TIME];//LEAD_CORRECTION_TIME;
		vmc6->LQR_Body_Compliance[i].c_delta = 0;
		vmc6->LQR_Body_Compliance[i].c_d_delta = 0;
		vmc6->LQR_Body_Compliance[i].ep = 0;
	}

	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 6; j++)
		{
			vmc6->PostureAdjust[i].p[j] = 0.0;
		}
	}
}


void load_data(BRobot *bhr6)
{
	int i, j;
	static PostureStruct last_real_body = INI_POS;
	TStruct tempT, tempT0;
	double d_omega_diff;
	double T = bhr6->dcmSetTab[DCM_SET_SYS_CIRCLE];
	//double M1[4] = {0.0003762544, 0.0036748945,0.0294597717, 1.0196101860};//小于45°
	double M1[4] = {0.0022017127, 0.0175341824,0.0017042150, 0.0582745447};//小于45°
	double M2[4] = {0.0001354253, 0.0020625530,0.0000325405, 1.0002396652};//大于45°
	ViscoelasticModelCompliance6 *vmc6;
	static double init_omega[3];
	vmc6 = &bhr6->VMC6;
	bhr6->LAST_VMC6 = bhr6->VMC6;

	//获取力传感器数据与关节实际角度
	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < 6; j++)
		{
		#ifdef USE_FT_FILTER
			vmc6->SensorFT[i][j] = filt_data(&bhr6->FilterFT[i][j],bhr6->dcmInputTab[DCM_INPUT_LEFT_TORQUE_X+i*6+j]);//力传感器
		#else
			vmc6->SensorFT[i][j] = bhr6->dcmInputTab[DCM_INPUT_LEFT_TORQUE_X + i * 6 + j];//力传感器
		#endif
			vmc6->RealJoint[i][j] = bhr6->dcmInputTab[DCM_INPUT_REAL_JOINT_L1+i*6+j];//关节实际角度
		}
		if (vmc6->SensorFT[i][5] < 0.0)vmc6->SensorFT[i][5] = 0.0;//竖直方向力小于0，则置为0，代表脚腾空
	}

	//获取姿态传感器数据
	if (bhr6->BW_IMU_Data.init_count < 10)
	{
		bhr6->BW_IMU_Data.init_count++;
		init_omega[0] = bhr6->dcmInputTab[DCM_INPUT_BODY_ROT_V_X];
		init_omega[1] = bhr6->dcmInputTab[DCM_INPUT_BODY_ROT_V_Y];
		init_omega[2] = bhr6->dcmInputTab[DCM_INPUT_BODY_ROT_V_Z];
		
		init_omega[0] = RAD(0);
		init_omega[1] = RAD(0);
		init_omega[2] = RAD(0);
	}
	for (i = 0; i < 3; i++)
	{
		//角度
		vmc6->IMU_Posture_Data[i] = bhr6->dcmInputTab[DCM_INPUT_BODY_RX+i];//获取Body/Base等基座标姿态角
		//处理 IMU 姿态角噪声,初始化时不能给IMU_Posture赋初值 //该情况待定
		vmc6->IMU_Posture_Data_d[i] = (vmc6->IMU_Posture_Data[i] - bhr6->LAST_VMC6.IMU_Posture_Data[i]) / T;
		//角速度
		bhr6->BW_IMU_Data.lastlast_Omega[i] = bhr6->BW_IMU_Data.last_Omega[i];
		bhr6->BW_IMU_Data.last_Omega[i] = bhr6->BW_IMU_Data.Omega[i];
		bhr6->BW_IMU_Data.Omega[i] = bhr6->dcmInputTab[DCM_INPUT_BODY_ROT_V_X + i] - init_omega[i];
		if (bhr6->BW_IMU_Data.init_count < 10)
		{
			//角度
			vmc6->IMU_Posture[i] = vmc6->IMU_Posture_Data[i];
			vmc6->IMU_Posture_d[i] = vmc6->IMU_Posture_Data_d[i];
			//角速度
			bhr6->BW_IMU_Data.new_Omega[i] = bhr6->BW_IMU_Data.last_Omega[i];
			bhr6->BW_IMU_Data.last_d_Omega[i] = bhr6->BW_IMU_Data.d_Omega[i];
			bhr6->BW_IMU_Data.d_Omega[i] = (bhr6->BW_IMU_Data.Omega[i] - bhr6->BW_IMU_Data.last_Omega[i]) / T;
		}
		else
		{
			//角度
			if (fabs(vmc6->IMU_Posture_Data_d[i]) > RAD(40000.0))
			{
				vmc6->IMU_Posture_d[i] = bhr6->LAST_VMC6.IMU_Posture_d[i];
			}
			else
			{
				vmc6->IMU_Posture_d[i] = vmc6->IMU_Posture_Data_d[i];
			}
			vmc6->IMU_Posture[i] = bhr6->LAST_VMC6.IMU_Posture[i] + vmc6->IMU_Posture_d[i] * T;
			//角速度
			bhr6->BW_IMU_Data.last_d_Omega[i] = bhr6->BW_IMU_Data.d_Omega[i];
			bhr6->BW_IMU_Data.d_Omega[i] = (bhr6->BW_IMU_Data.Omega[i] - bhr6->BW_IMU_Data.last_Omega[i]) / T;
			d_omega_diff = fabs(bhr6->BW_IMU_Data.d_Omega[i] - bhr6->BW_IMU_Data.last_d_Omega[i]);
			if (
				(d_omega_diff > fabs(bhr6->BW_IMU_Data.d_Omega[i]) + MIN_ERROR)
				&& (d_omega_diff > fabs(bhr6->BW_IMU_Data.last_d_Omega[i]) + MIN_ERROR)
				&& (d_omega_diff > RAD(1000.0))
				)//判断若上一时刻是否是跳点
			{
				bhr6->BW_IMU_Data.new_Omega[i] = 0.5*(bhr6->BW_IMU_Data.Omega[i] + bhr6->BW_IMU_Data.lastlast_Omega[i]);
			}
			else
			{
				bhr6->BW_IMU_Data.new_Omega[i] = bhr6->BW_IMU_Data.last_Omega[i];
			}
		}
	}

	//姿态角转换：旋转顺序定义不同，则同一个姿态矩阵算出的姿态角也不同
	//本程序的姿态角定义：Rz*Ry*Rx

	//get_imu_data(bhr6);
	for (i = 0; i < 2; i++)
	{
		vmc6->RawBodyPosture.p[i] = vmc6->IMU_Posture[i];
		vmc6->RawBodyVel[i] = bhr6->BW_IMU_Data.new_Omega[i];
	}
#ifdef VREP_SIM
	for (i = 0; i < 2; i++)
	{
		vmc6->RawBodyPosture.p[i] = vmc6->IMU_Posture_Data[i];
		vmc6->RawBodyVel[i] = bhr6->dcmInputTab[DCM_INPUT_BODY_ROT_V_X + i];
	}
#endif

	//角度与角速度状态估计
	
	if(fabs(bhr6->LAST_VMC6.RawBodyPosture.p[1])<RAD(45.0))
	{
		bhr6->EstimatorPosture[i].M.m[0][0] = M2[0];
		bhr6->EstimatorPosture[i].M.m[0][1] = M2[1];
		bhr6->EstimatorPosture[i].M.m[1][0] = M2[2];
		bhr6->EstimatorPosture[i].M.m[1][1] = M2[3];
	}
	else
	{
		bhr6->EstimatorPosture[i].M.m[0][0] = M1[0];
		bhr6->EstimatorPosture[i].M.m[0][1] = M1[1];
		bhr6->EstimatorPosture[i].M.m[1][0] = M1[2];
		bhr6->EstimatorPosture[i].M.m[1][1] = M1[3];
	}
	
	for (i = 0; i < 2; i++)
	{
		if(bhr6->BW_IMU_Data.init_count < 10)
		{
			printf("init,%f\n",DEG(init_omega[i]));
			bhr6->EstimatorPosture[i].X.m[0][0] = bhr6->LAST_VMC6.RawBodyPosture.p[i];
			bhr6->EstimatorPosture[i].X.m[1][0] = bhr6->LAST_VMC6.RawBodyVel[i];
			vmc6->RealBodyPosture.p[i] = vmc6->RawBodyPosture.p[i];
			vmc6->RealBodyVel[i] = vmc6->RawBodyVel[i];
		}
		else
		{
			bhr6->EstimatorPosture[i].Y.m[0][0] = bhr6->LAST_VMC6.RawBodyPosture.p[i];
			bhr6->EstimatorPosture[i].Y.m[1][0] = bhr6->LAST_VMC6.RawBodyVel[i];
			bhr6->EstimatorPosture[i].U.m[0][0] = 0.0;
			estimate_state(&(bhr6->EstimatorPosture[i]));
			vmc6->RealBodyPosture.p[i] = bhr6->EstimatorPosture[i].X.m[0][0];
			vmc6->RealBodyVel[i] = bhr6->EstimatorPosture[i].X.m[1][0];
		}
	}

	//vmc6->RealBodyPosture.p[1] = vmc6->IMU_Posture[1];

	tempT = get_t(&vmc6->RealBodyPosture);//重新计算旋转矩阵（机器人伴随坐标系下）
	tempT.t[_px] = 0;
	tempT.t[_py] = 0;
	tempT.t[_pz] = 0;
	tempT0 = tempT;

	//测量到的加速度是相对于传感器坐标系的，也就是固连于Body/Base的坐标系，所以将传感器坐标系下的加速度转换到机器人伴随坐标系下
	tempT = mt_mul(tempT, get_move(bhr6->dcmInputTab[DCM_INPUT_BODY_ACC_X], bhr6->dcmInputTab[DCM_INPUT_BODY_ACC_Y], bhr6->dcmInputTab[DCM_INPUT_BODY_ACC_Z]));
	
	for (i = 0; i < 3; i++)
	{
		vmc6->RealBodyAcc[i + 3] = tempT.t[i][3] + ((double)(i == 2))*G;//重力补偿
		//vmc6->RealBodyAcc[i + 3] = tempT.t[i][3];
	}

	//计算在世界坐标系下的双足受力
	get_ft_data(bhr6);

	//状态预估，预估世界坐标系下的Body/Base位置与DCM
	for (i = 0; i < 3; i++)
	{
		bhr6->DC_Est[i].input_vrp = bhr6->VMC6.InputVRP[i];
		bhr6->DC_Est[i].output_f = bhr6->VMC6.LQR[0][i + 3].real_value + bhr6->VMC6.LQR[1][i + 3].real_value - ((double)(i==2))*ROBOT_WEIGHT*G;
		bhr6->DC_Est[i].output_ddx = vmc6->RealBodyAcc[i + 3];

		dcm_com_est(&(bhr6->DC_Est[i]));

		vmc6->EstDCM[i] = bhr6->DC_Est[i].state_dcm;
		vmc6->RealBodyPosture.p[i + 3] = bhr6->DC_Est[i].state_com;
		vmc6->RealBodyVel[i + 3] = (vmc6->RealBodyPosture.p[i + 3] - bhr6->LAST_VMC6.RealBodyPosture.p[i + 3]) / bhr6->dcmSetTab[DCM_SET_SYS_CIRCLE];
		//vmc6->RealBodyVel[i + 3] += vmc6->RealBodyAcc[i + 3] * bhr6->dcmSetTab[DCM_SET_SYS_CIRCLE];
		//vmc6->RealBodyPosture.p[i + 3] += vmc6->RealBodyVel[i + 3] * bhr6->dcmSetTab[DCM_SET_SYS_CIRCLE];
	}
	vmc6->RealBodyT = get_t(&vmc6->RealBodyPosture);

	for (i = 0; i < 3; i++) bhr6->VMC6.RealBodyAcc[i] = (bhr6->VMC6.RealBodyVel[i] - bhr6->LAST_VMC6.RealBodyVel[i]) / (bhr6->Squat.inter_time / 1000.0);
}

void dcm_init(BRobot *bhr6)
{
	int i1, i2;
	WalkingSet *dcm;
	dcm = &bhr6->DCM;
	// Set VRP points
	dcm->VRP_Point[0][0] = 0.0;
	dcm->VRP_Point[0][1] = 0.0;
	dcm->VRP_Point[0][2] = dcm->COM_Height;

	dcm->VRP_Point[1][0] = 0.0;
	dcm->VRP_Point[1][1] = -dcm->Step_Width; //Swing Left Leg Firstly
	dcm->VRP_Point[1][2] = dcm->COM_Height;

	for (i1 = 2; i1 < dcm->Step_Num + 2; i1++)
	{
		dcm->VRP_Point[i1][0] = dcm->VRP_Point[i1 - 1][0] + dcm->Step_Length;
		dcm->VRP_Point[i1][1] = -dcm->VRP_Point[i1 - 1][1];
		dcm->VRP_Point[i1][2] = dcm->COM_Height;

		if (i1 >= 5) dcm->VRP_Point[i1][2] = dcm->COM_Height + STAGE_HEIGHT;
	}

	dcm->VRP_Point[dcm->Step_Num + 1][0] = dcm->Step_Length*dcm->Step_Num;
	dcm->VRP_Point[dcm->Step_Num + 2][0] = dcm->Step_Length*dcm->Step_Num;
	dcm->VRP_Point[dcm->Step_Num + 2][1] = 0.0;
	dcm->VRP_Point[dcm->Step_Num + 2][2] = dcm->COM_Height+STAGE_HEIGHT;


	// Set initial discrete dcm points
	dcm->DCM_End[dcm->Step_Num + 1][0] = dcm->VRP_Point[dcm->Step_Num + 2][0];
	dcm->DCM_End[dcm->Step_Num + 1][1] = dcm->VRP_Point[dcm->Step_Num + 2][1];
	dcm->DCM_End[dcm->Step_Num + 1][2] = dcm->VRP_Point[dcm->Step_Num + 2][2];

	for (i1 = dcm->Step_Num; i1 >= 0; i1--)
	{
		for (i2 = 0; i2 < 3; i2++)
		{
			dcm->DCM_End[i1][i2] = dcm->VRP_Point[i1 + 1][i2] + exp(-1.0 / dcm->B*dcm->Step_Time)*(dcm->DCM_End[i1 + 1][i2] - dcm->VRP_Point[i1 + 1][i2]);
		}
	}

	dcm->DCM_Ini[0][0] = dcm->VRP_Point[0][0];
	dcm->DCM_Ini[0][1] = dcm->VRP_Point[0][1];
	dcm->DCM_Ini[0][2] = dcm->VRP_Point[0][2];

	for (i1 = 1; i1 < dcm->Step_Num + 2; i1++)
	{
		for (i2 = 0; i2 < 3; i2++)
		{
			dcm->DCM_Ini[i1][i2] = dcm->DCM_End[i1 - 1][i2];
		}
	}

	// Set discrete dcm points in DS phase
	dcm->DCM_DS_Ini[0][0] = dcm->DCM_Ini[0][0];
	dcm->DCM_DS_Ini[0][1] = dcm->DCM_Ini[0][1];
	dcm->DCM_DS_Ini[0][2] = dcm->DCM_Ini[0][2];
	dcm->DDCM_DS_Ini[0][0] = 0;
	dcm->DDCM_DS_Ini[0][1] = 0;
	dcm->DDCM_DS_Ini[0][2] = 0;

	for (i1 = 1; i1 < dcm->Step_Num + 2; i1++)
	{
		for (i2 = 0; i2 < 3; i2++)
		{
			dcm->DCM_DS_Ini[i1][i2] = dcm->VRP_Point[i1][i2] + exp(-1.0 / dcm->B*dcm->Step_Time_DS_Ini)*(dcm->DCM_End[i1][i2] - dcm->VRP_Point[i1][i2]);
			dcm->DDCM_DS_Ini[i1][i2] = 1.0 / dcm->B*exp(-1.0 / dcm->B*dcm->Step_Time_DS_Ini)*(dcm->DCM_End[i1][i2] - dcm->VRP_Point[i1][i2]);
		}
	}

	for (i1 = 0; i1 < dcm->Step_Num + 1; i1++)
	{
		for (i2 = 0; i2 < 3; i2++)
		{
			dcm->DCM_DS_End[i1][i2] = dcm->VRP_Point[i1 + 1][i2] + exp(1.0 / dcm->B*dcm->Step_Time_DS_End)*(dcm->DCM_Ini[i1 + 1][i2] - dcm->VRP_Point[i1 + 1][i2]);
			dcm->DDCM_DS_End[i1][i2] = 1.0 / dcm->B*exp(1.0 / dcm->B*dcm->Step_Time_DS_Ini)*(dcm->DCM_Ini[i1 + 1][i2] - dcm->VRP_Point[i1 + 1][i2]);
		}
	}

	dcm->DCM_DS_End[dcm->Step_Num + 1][0] = dcm->DCM_End[dcm->Step_Num + 1][0];
	dcm->DCM_DS_End[dcm->Step_Num + 1][1] = dcm->DCM_End[dcm->Step_Num + 1][1];
	dcm->DCM_DS_End[dcm->Step_Num + 1][2] = dcm->DCM_End[dcm->Step_Num + 1][2];
	dcm->DDCM_DS_End[dcm->Step_Num + 1][0] = 0;
	dcm->DDCM_DS_End[dcm->Step_Num + 1][1] = 0;
	dcm->DDCM_DS_End[dcm->Step_Num + 1][2] = 0;
}

void dcm_com_est_init(BRobot *bhr6)
{
	DCM_COM_Estimator * dce;
	//double m[2][2] = { 0.0030644300,0.0000006006,0.0014967322,0.0000002934 };
	//double m[2][2] = { 0.0035263141,0.0000691158,0.0014682231,0.0000287772 };
	//double m[2][2] = { 0.0030644300,0,0.0014967322,0.000 };
	double m[2][2] = { 0,0.0250123309,0,0.0107889640 };
	double a[2][2] = { 1.0145641768,0.0000000000,0.0144596417,0.9856448935 };
	double b[2] = { -0.0145641768,-0.0001045352 };
	double c[2][2] = { 0.0000000000,666.6666666667,0.0000000000,13.0666666667 };
	double d[2] = { -666.6666666667,-13.0666666667 };
	int i, j, k;
	
	for (i = 0; i < 3; i++)
	{
		dce = &(bhr6->DC_Est[i]);
		dce->input_vrp = bhr6->CurrentState.Posture[BASE].Posture.p[i + 3] / 1000.0;
		dce->state_com = bhr6->CurrentState.Posture[BASE].Posture.p[i + 3] / 1000.0;
		dce->state_dcm = bhr6->CurrentState.Posture[BASE].Posture.p[i + 3] / 1000.0;
		dce->output_ddx = 0;
		dce->output_f = 0;
		dce->T = bhr6->Squat.inter_time / 1000.0;
		for (j = 0; j < 2; j++)
		{
			for (k = 0; k < 2; k++)
			{
				dce->M[j][k] = m[j][k];
				dce->A[j][k] = a[j][k];
				dce->C[j][k] = c[j][k];
			}
			dce->B[j] = b[j];
			dce->D[j] = d[j];
		}
	}
	
}


void zmp_tpc_init(BRobot *bhr6)
{
	TrunkPositionCompliance *tpc;
	int i;
	double limit_min[2] = { -0.03,-0.03 };
	double limit_max[2] = { 0.03,0.03 };

	tpc = &(bhr6->ZMP_TPC);
	tpc->TuneK[0] = TPC_TUNE_K_X;
	tpc->TuneK[1] = TPC_TUNE_K_Y;

	for (i = 0; i < 2; i++)
	{
		tpc->LQR[i].Alpha = 0.0;
		tpc->LQR[i].Td = 0.0;
		tpc->LQR[i].T = bhr6->dcmSetTab[DCM_SET_SYS_CIRCLE];
		tpc->LQR[i].delta = 0.0;
		tpc->LQR[i].d_delta = 0.0;
		tpc->LQR[i].dd_delta = 0.0;
		tpc->LQR[i].limit[0] = limit_min[i];
		tpc->LQR[i].limit[1] = limit_max[i];
		tpc->LQR[i].real_value = 0.0;
		tpc->LQR[i].ref_value = 0.0;
		tpc->LQR[i].track_num = 1;
		tpc->LQR[i].k[0] = bhr6->dcmSetTab[DCM_SET_TPC_K1];
		tpc->LQR[i].k[1] = bhr6->dcmSetTab[DCM_SET_TPC_K2];
		tpc->LQR[i].k[2] = bhr6->dcmSetTab[DCM_SET_TPC_K3];
	}
}


void init_filters(BRobot *bhr6)
{
	//3阶，20.000000Hz低通滤波器
	double b3_20[4] = { 0.0101825767,0.0305477302,0.0305477302,0.0101825767 };
	double a3_20[4] = { 1.0000000000,-2.0037974774,1.4470540195,-0.3617959282 };
	//3阶，40.000000Hz低通滤波器
	double b3_40[4] = { 0.0578903754,0.1736711262,0.1736711262,0.0578903754 };
	double a3_40[4] = { 1.0000000000,-1.0440641548,0.6252304302,-0.1180432721 };
	//3阶，60.000000Hz低通滤波器
	double b3_60[4] = { 0.1513923271,0.4541769812,0.4541769812,0.1513923271 };
	double a3_60[4] = { 1.0000000000,-0.1152020629,0.3368436806,-0.0105030013 };
	//3阶，80.000000Hz低通滤波器
	double b3_80[4] = { 0.3002723898,0.9008171694,0.9008171694,0.3002723898 };
	double a3_80[4] = { 1.0000000000,0.8098323577,0.5080731868,0.0842735738 };
	//3阶，100.000000Hz低通滤波器
	double b3_100[4] = { 0.5276243825,1.5828731475,1.5828731475,0.5276243825 };
	double a3_100[4] = { 1.0000000000,1.7600418803,1.1828932620,0.2780599176 };
	//3阶，120.000000Hz低通滤波器
	double b3_120[4] = { 0.8818381986,2.6455145957,2.6455145957,0.8818381986 };
	double a3_120[4] = { 1.0000000000,2.7488358092,2.5282312191,0.7776385602 };

	init_filter(&(bhr6->FilterIMU_Posture[0]), 3, 10,  a3_100, b3_100);
	init_filter(&(bhr6->FilterIMU_Posture[1]), 3, 10,  a3_100, b3_100);
	init_filter(&(bhr6->FilterIMU_Posture[2]), 3, 10,  a3_100, b3_100);

	init_filter(&(bhr6->FilterIMU_Acc[0]), 3, 10,  a3_100, b3_100);
	init_filter(&(bhr6->FilterIMU_Acc[1]), 3, 10,  a3_100, b3_100);
	init_filter(&(bhr6->FilterIMU_Acc[2]), 3, 10,  a3_100, b3_100);

	init_filter(&(bhr6->FilterIMU_Ang_Vel[0]), 3, 10,  a3_100, b3_100);
	init_filter(&(bhr6->FilterIMU_Ang_Vel[1]), 3, 10,  a3_100, b3_100);
	init_filter(&(bhr6->FilterIMU_Ang_Vel[2]), 3, 10,  a3_100, b3_100);

	init_filter(&(bhr6->FilterFT[0][0]), 3, 10,  a3_100, b3_100);
	init_filter(&(bhr6->FilterFT[0][1]), 3, 10,  a3_100, b3_100);
	init_filter(&(bhr6->FilterFT[0][2]), 3, 10,  a3_100, b3_100);
	init_filter(&(bhr6->FilterFT[0][3]), 3, 10,  a3_100, b3_100);
	init_filter(&(bhr6->FilterFT[0][4]), 3, 10,  a3_100, b3_100);
	init_filter(&(bhr6->FilterFT[0][5]), 3, 10,  a3_100, b3_100);

	init_filter(&(bhr6->FilterFT[1][0]), 3, 10,  a3_100, b3_100);
	init_filter(&(bhr6->FilterFT[1][1]), 3, 10,  a3_100, b3_100);
	init_filter(&(bhr6->FilterFT[1][2]), 3, 10,  a3_100, b3_100);
	init_filter(&(bhr6->FilterFT[1][3]), 3, 10,  a3_100, b3_100);
	init_filter(&(bhr6->FilterFT[1][4]), 3, 10,  a3_100, b3_100);
	init_filter(&(bhr6->FilterFT[1][5]), 3, 10,  a3_100, b3_100);

	init_filter(&(bhr6->FilterHandPos[0]), 3, 10, a3_60, b3_60);
	init_filter(&(bhr6->FilterHandPos[1]), 3, 10, a3_60, b3_60);
}


void init_estimators(BRobot *bhr6)
{
	// 身体姿态角度与角速度预估，基于倒立摆模型，假设输入力矩为0
	int posture_est_num[3] = { 2,1,2 };
	double posture_est_mat[8][2][2] = {
		//{ 0.0102603018,0.0066488525,0.0066488525,0.0973430661 }, // M
		//{ 1.0000784010,0.0040001045,0.0392010244,1.0000784010 }, // A
		//{ 0.0000001420,0.0000000000,0.0000710163,0.0000000000 }, // B
		//{ 1.0000000000,0.0000000000,0.0000000000,1.0000000000 }, // C
		//{ 0.0000000000,0.0000000000,0.0000000000,0.0000000000 }, // D
		//{ 0.0000000000,0.0000000000,0.0000000000,0.0000000000 }, // X
		//{ 0.0000000000,0.0000000000,0.0000000000,0.0000000000 }, // U
		//{ 0.0000000000,0.0000000000,0.0000000000,0.0000000000 }  // Y

		//{ 0.0093558960,0.0237805583,0.0002378056,0.6186069524 },//M
		//{ 0.0004223261,0.0000341405,0.0034140539,0.5232416957 },//M行走中摔倒用
		//{ 0.0000000080,0.0181337805,0.0000000021,0.1538725908 },//M空中摆动
		{ 0.0001354253,0.0020625530,0.0000325405,1.0002396652},//M 20190412空中摆动与行走摔倒组合优化结果，静态过程有漂移
		{ 1.0000784010,0.0040001045,0.0392010244,1.0000784010 },//A
		{ 0.0000001420,0.0000000000,0.0000710163,0.0000000000 },//B
		{ 1.0000000000,0.0000000000,0.0000000000,1.0000000000},//C
		{ 0.0000000000,0.0000000000,0.0000000000,0.0000000000},//D
		{ 0.0000000000,0.0000000000,0.0000000000,0.0000000000},//X
		{ 0.0000000000,0.0000000000,0.0000000000,0.0000000000},//U
		{ 0.0000000000,0.0000000000,0.0000000000,0.0000000000}//Y
	};
	double posture_est_mat2[8][2][2] = {
		//{ 0.0102603018,0.0066488525,0.0066488525,0.0973430661 }, // M
		//{ 1.0000784010,0.0040001045,0.0392010244,1.0000784010 }, // A
		//{ 0.0000001420,0.0000000000,0.0000710163,0.0000000000 }, // B
		//{ 1.0000000000,0.0000000000,0.0000000000,1.0000000000 }, // C
		//{ 0.0000000000,0.0000000000,0.0000000000,0.0000000000 }, // D
		//{ 0.0000000000,0.0000000000,0.0000000000,0.0000000000 }, // X
		//{ 0.0000000000,0.0000000000,0.0000000000,0.0000000000 }, // U
		//{ 0.0000000000,0.0000000000,0.0000000000,0.0000000000 }  // Y

		//{ 0.0093558960,0.0237805583,0.0002378056,0.6186069524 },//M
		{ 0.0022017127,  0.0175341824,0.0017042150, 0.0582745447 },//M
		{ 1.0000784010,0.0040001045,0.0392010244,1.0000784010 },//A
		{ 0.0000001420,0.0000000000,0.0000710163,0.0000000000 },//B
		{ 1.0000000000,0.0000000000,0.0000000000,1.0000000000 },//C
		{ 0.0000000000,0.0000000000,0.0000000000,0.0000000000 },//D
		{ 0.0000000000,0.0000000000,0.0000000000,0.0000000000 },//X
		{ 0.0000000000,0.0000000000,0.0000000000,0.0000000000 },//U
		{ 0.0000000000,0.0000000000,0.0000000000,0.0000000000 }//Y
	};
	//for (i = 0; i < 2; i++)
	{
		init_estimator(&(bhr6->EstimatorPosture[1]), posture_est_num, posture_est_mat);
		init_estimator(&(bhr6->EstimatorPosture[0]), posture_est_num, posture_est_mat2);
	}
}

void get_imu_data(BRobot *bhr6)
{
	TStruct tempT, R_T;
	PostureStruct tempP;
	MStruct R, dR, Omega;
	int i,j;
	double T = bhr6->dcmSetTab[DCM_SET_SYS_CIRCLE];
	double w_vel[3];

	//计算世界坐标系下旋转矩阵与转换姿态角
#ifdef VREP_SIM
	//VREP 姿态转换
	R_T = mt_mul(mt_mul(get_rot(AXIS_Z, bhr6->VMC6.IMU_Posture[2]), get_rot(AXIS_Y, bhr6->VMC6.IMU_Posture[1])), get_rot(AXIS_X, bhr6->VMC6.IMU_Posture[0]));
#else
	//机器人IMU 姿态转换
	#ifdef _BHR_6P_
		R_T = mt_mul(mt_mul(get_rot(AXIS_Z, bhr6->VMC6.IMU_Posture[2]), get_rot(AXIS_X, bhr6->VMC6.IMU_Posture[0])), get_rot(AXIS_Y, bhr6->VMC6.IMU_Posture[1]));
	#endif
	#ifdef _BHR_5_
		//R_T = mt_mul(mt_mul(get_rot(AXIS_Z, bhr6->VMC6.IMU_Posture[2]), get_rot(AXIS_Y, bhr6->VMC6.IMU_Posture[1])), get_rot(AXIS_X, bhr6->VMC6.IMU_Posture[0]));
		R_T = mt_mul(mt_mul(get_rot(AXIS_Z, bhr6->VMC6.IMU_Posture[2]), get_rot(AXIS_X, bhr6->VMC6.IMU_Posture[0])), get_rot(AXIS_Y, bhr6->VMC6.IMU_Posture[1]));
	#endif
#endif
//	test = get_rot_from_t(R_T);
	//show_matrix(&test);
	ik_posture(R_T.t, bhr6->VMC6.RawBodyPosture.p, bhr6->LAST_VMC6.RawBodyPosture.p);
	//printf("%f,%f,%f\n", DEG(bhr6->VMC6.RawBodyPosture.p[0]), DEG(bhr6->VMC6.RawBodyPosture.p[1]), DEG(bhr6->VMC6.RawBodyPosture.p[2]));
	//计算伴随坐标系下旋转矩阵与姿态角
	tempT = R_T;
	R_T = mt_mul(get_rot(AXIS_Y, bhr6->VMC6.RawBodyPosture.p[1]), get_rot(AXIS_X, bhr6->VMC6.RawBodyPosture.p[0]));
	ik_posture(R_T.t, bhr6->VMC6.RawBodyPosture.p, bhr6->LAST_VMC6.RawBodyPosture.p);

	//printf("%f,%f,%f\n", DEG(bhr6->VMC6.RawBodyPosture.p[0]), DEG(bhr6->VMC6.RawBodyPosture.p[1]), DEG(bhr6->VMC6.RawBodyPosture.p[2]));

	//计算伴随坐标系下角速度矢量
#ifdef VREP_SIM
	//VREP 速度相对于世界坐标系
	tempT = mt_mul(R_T, t_inv(&tempT));
	tempT = mt_mul(tempT, get_move(bhr6->dcmInputTab[DCM_INPUT_BODY_ROT_V_X], bhr6->dcmInputTab[DCM_INPUT_BODY_ROT_V_Y], bhr6->dcmInputTab[DCM_INPUT_BODY_ROT_V_Z]));
#else
	//机器人IMU绕自身轴旋转
	//tempT = mt_mul(R_T, get_move(bhr6->dcmInputTab[DCM_INPUT_BODY_ROT_V_X], bhr6->dcmInputTab[DCM_INPUT_BODY_ROT_V_Y], bhr6->dcmInputTab[DCM_INPUT_BODY_ROT_V_Z]));
	tempT = mt_mul(R_T, get_move(bhr6->BW_IMU_Data.new_Omega[0], bhr6->BW_IMU_Data.new_Omega[1], bhr6->BW_IMU_Data.new_Omega[2]));
#endif
	for (i = 0; i < 3; i++)
	{
		w_vel[i] = tempT.t[i][3];
	}


	//计算旋转矩阵微分
	R = get_rot_from_t(R_T);
	Omega = get_3lcross_m(w_vel);
	dR = m_mul(&Omega, &R);
	//计算新的旋转矩阵
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			R_T.t[i][j] = R.m[i][j] + T*dR.m[i][j];
		}
	}
	//计算新的姿态角
	ik_posture(R_T.t, tempP.p, bhr6->VMC6.RawBodyPosture.p);
	//计算角速度
	for (i = 0; i < 3; i++)
	{
		bhr6->VMC6.RawBodyVel[i] = (tempP.p[i] - bhr6->VMC6.RawBodyPosture.p[i]) / T;
	}

	//机器人与IMU之间可能存在一个角度偏置，在此补偿
	//BHR5，IMU pitch 方向相对于机器人存在一个2.4°的偏角
}

void two_link_model_init(BRobot *bhr6)
{
	int i,j;
	TwoLinkModelSet * set;
	double Ad[4][4] = { { 1.0000000000,0.0000000000,0.0040000000,0.0000000000 },{ 0.0000000000,1.0000000000,0.0000000000,0.0040000000 },{ 0.0000000000,0.0000000000,1.0000000000,0.0000000000 },{ 0.0000000000,0.0000000000,0.0000000000,1.0000000000 } };
	double Bd[4][2] = { { 0.0000080000,0.0000000000 },{ 0.0000000000,0.0000080000 },{ 0.0040000000,0.0000000000 },{ 0.0000000000,0.0040000000 } };

	for (i = 0; i < 2; i++) 
	{
		set = &(bhr6->TL_Control[i].Set);
		set->m[0] = 30-20;			set->Len[0] = 0.8;		set->s[0] = 0.5*set->Len[0];			set->I[0] = 1.0 / 12.0 * set->m[0] * pow(set->Len[0], 2.0);
		set->m[1] = 20+10;			set->Len[1] = 1;		set->s[1] = 0.5*set->Len[1];			set->I[1] = 1.0 / 12.0 * set->m[1] * pow(set->Len[1], 2.0);

		bhr6->TL_Control[i].A.dim[0] = 4;
		bhr6->TL_Control[i].A.dim[1] = 4;
		bhr6->TL_Control[i].X.dim[0] = 4;	
		bhr6->TL_Control[i].X.dim[1] = 1;
		bhr6->TL_Control[i].B.dim[0] = 4;	
		bhr6->TL_Control[i].B.dim[1] = 2;
		bhr6->TL_Control[i].U.dim[0] = 2;
		bhr6->TL_Control[i].U.dim[1] = 1;

		bhr6->TL_Control[i].adjust_dq[0] = 0.0;
		bhr6->TL_Control[i].adjust_dq[1] = 0.0;
		bhr6->TL_Control[i].adjust_q[0] = 0.0;
		bhr6->TL_Control[i].adjust_q[1] = 0.0;

		bhr6->TL_Control[i].model_state.q[0] = 0.0;
		bhr6->TL_Control[i].model_state.q[1] = 0.0;
		bhr6->TL_Control[i].model_state.dq[0] = 0.0;
		bhr6->TL_Control[i].model_state.dq[1] = 0.0;

		bhr6->TL_Control[i].real_state.q[0] = 0.0;
		bhr6->TL_Control[i].real_state.q[1] = 0.0;
		bhr6->TL_Control[i].real_state.dq[0] = 0.0;
		bhr6->TL_Control[i].real_state.dq[1] = 0.0;

		bhr6->TL_Control[i].ref_state.q[0] = 0.0;
		bhr6->TL_Control[i].ref_state.q[1] = 0.0;
		bhr6->TL_Control[i].ref_state.dq[0] = 0.0;
		bhr6->TL_Control[i].ref_state.dq[1] = 0.0;

	}
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			bhr6->TL_Control[0].A.m[i][j] = Ad[i][j];
			bhr6->TL_Control[1].A.m[i][j] = Ad[i][j];
		}
		for (j = 0; j < 2; j++)
		{
			bhr6->TL_Control[0].B.m[i][j] = Bd[i][j];
			bhr6->TL_Control[1].B.m[i][j] = Bd[i][j];
		}
	}
}

_rrm  void BW_IMU_init(BRobot *robot)
{
	IMU_DATA *bw = &(robot->BW_IMU_Data);
	int i;

	bw->init_flag = HAD_SET;
	bw->init_count = 0;
	for (i = 0; i < 3; i++)
	{
		bw->Omega[i] = 0.0;
		bw->last_Omega[i] = 0.0;
		bw->lastlast_Omega[i] = 0.0;
		bw->d_Omega[i] = 0.0;
		bw->last_d_Omega[i] = 0.0;
		bw->new_Omega[i] = 0.0;
	}
}