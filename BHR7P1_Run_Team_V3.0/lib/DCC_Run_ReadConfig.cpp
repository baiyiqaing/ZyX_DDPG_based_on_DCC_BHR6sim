#include "DCC_Run_ReadConfig.h"
DCCRunParms dccRunParms;
using namespace bhrrtx;

RtxCEvent<10> ceDCCRun(L"DCCRunEvent");

#ifndef UNDER_RTSS // 非实时读取
RtxCConfigJSON<DCCRunParms> cjDCCRun(L"DCCRunConfig");
BOOL NRT_ReadDCCRunParms(DCCRunParms * p, LPWSTR filename)
{
#ifndef SIMULATION
	if (!ceDCCRun.Open()) return FALSE;
	if (!cjDCCRun.Open()) return FALSE;
#endif
	if (!cjDCCRun.ReadJSONFile(filename))return FALSE;
	// 开始读取：
	std::cout << std::endl<< "Start read DCCRunConConfig.json" << std::endl;
	p->ZMP_Lag_T	= cjDCCRun.ReadFloatNum("ZMP_Lag_T");
	p->IMU_Lag_T	= cjDCCRun.ReadFloatNum("IMU_Lag_T");
	p->Fz_Lag_T		= cjDCCRun.ReadFloatNum("Fz_Lag_T");
	
	p->pitch_bias	= cjDCCRun.ReadFloatNum("pitch_bias");
	p->zmpbias_micro = cjDCCRun.ReadFloatNum("zmpbias_micro");
	p->zmpx_bias	= cjDCCRun.ReadFloatNum("zmpx_bias");
	p->zmpy_bias	= cjDCCRun.ReadFloatNum("zmpy_bias");
	
	p->m_robot_bias = cjDCCRun.ReadFloatNum("m_robot_bias");
	p->roll_ampli	= cjDCCRun.ReadFloatNum("roll_ampli");
	p->tau_pitch_bias = cjDCCRun.ReadFloatNum("tau_pitch_bias");
	p->tau_roll_bias = cjDCCRun.ReadFloatNum("tau_roll_bias");
	
	p->Zc = cjDCCRun.ReadFloatNum("Zc");
	cjDCCRun.ReadFloatArray("paras_LIPM", 6, p->paras_LIPM);
	cjDCCRun.ReadFloatArray("paras_TPC", 6, p->paras_TPC);
	cjDCCRun.ReadFloatArray("limit_TPC", 2, p->limit_TPC);
	cjDCCRun.ReadFloatArray("paras_GRFC_old", 6, p->paras_GRFC_old);
	cjDCCRun.ReadFloatArray("limit_GRFC_old", 3, p->limit_GRFC_old);
	cjDCCRun.ReadFloatArray("paras_GRFC", 6, p->paras_GRFC);
	cjDCCRun.ReadFloatArray("limit_GRFC", 2, p->limit_GRFC);
	cjDCCRun.ReadFloatArray("paras_Rot", 6, p->paras_Rot);
	cjDCCRun.ReadFloatArray("limit_Rot", 2, p->limit_Rot);
	cjDCCRun.ReadFloatArray("Balance_Pro", 2, p->Balance_Pro);
	cjDCCRun.ReadFloatArray("Micro_Pro", 2, p->Micro_Pro);
	cjDCCRun.ReadFloatArray("limit_Pro", 2, p->limit_Pro);

	cjDCCRun.ReadFloatArray("paras_FlyBody", 6, p->paras_FlyBody);
	cjDCCRun.ReadFloatArray("limit_FlyBody", 2, p->limit_FlyBody);
	cjDCCRun.ReadFloatArray("wake_FlyBody", 2, p->wake_FlyBody);

	cjDCCRun.ReadFloatArray("paras_FlyFoot", 4, p->paras_FlyFoot);
	cjDCCRun.ReadFloatArray("limit_FlyFoot", 2, p->limit_FlyFoot);
	cjDCCRun.ReadFloatArray("wake_FlyFoot", 2, p->wake_FlyFoot);
	cjDCCRun.ReadFloatArray("paras_Step", 12, p->paras_Step);
	cjDCCRun.ReadFloatArray("limit_Step", 3, p->limit_Step);
	cjDCCRun.ReadFloatArray("paras_Land", 11, p->paras_Land);
	cjDCCRun.ReadFloatArray("limit_Land", 2, p->limit_Land);
	cjDCCRun.ReadFloatArray("wake_Land", 3, p->wake_Land);
	cjDCCRun.ReadFloatArray("paras_Cont", 5, p->paras_Cont);
	cjDCCRun.ReadFloatArray("limit_Cont", 1, p->limit_Cont);
	cjDCCRun.ReadFloatArray("paras_Stepdown", 4, p->paras_Stepdown);

	p->k_down = (int)(p->paras_Stepdown[3]);
	p->k_down_fore = cjDCCRun.ReadIntNum("k_down_fore");
	p->H_ankle = p->paras_Stepdown[0];

	p->addi_pitch = cjDCCRun.ReadFloatNum("addi_pitch");
	p->addi_Rfoot_pitch = cjDCCRun.ReadFloatNum("addi_Rfoot_pitch");
	p->addi_Lfoot_pitch = cjDCCRun.ReadFloatNum("addi_Lfoot_pitch");
	p->roll_bias_foot = cjDCCRun.ReadFloatNum("roll_bias_foot");
	p->pitch_bias_foot = cjDCCRun.ReadFloatNum("pitch_bias_foot");


	std::cout << "Start send DCCRunConConfig.json" << std::endl;
#ifndef SIMULATION
	// 读取完毕，写入共享内存，并通知实时
	if (cjDCCRun.Lock())
	{
		(*cjDCCRun.GetData()) = *p;
	}
	else
	{
		return FALSE;
	}
	cjDCCRun.Unlock();
	ceDCCRun.Set();
#endif
	std::cout << "Reading DCCRunConConfig.json is finished" << std::endl;
	return TRUE;
}
#else// 实时接收
RtxCIPC<DCCRunParms> ciDCCRun(L"DCCRunConfig");
BOOL RT_InitDCCRunParms()
{
	if (!ciDCCRun.Create())return FALSE;
	if (!ceDCCRun.Create())return FALSE;
	return TRUE;
}

BOOL RT_LoadDCCRunParms(DCCRunParms * p)
{
	ceDCCRun.SetWaitTime(1000 * 30); //等待30s
	std::cout << "Waiting for NonRT process reading DCC run conroller parameters [30s]" << std::endl;
	ceDCCRun.Reset();
	if (!ceDCCRun.Wait())
	{
		return FALSE;
	};
	if (ciDCCRun.Lock())
	{
		(*p) = (*ciDCCRun.GetData());
	}
	else
	{
		return FALSE;
	}
	ciDCCRun.Unlock();
	std::cout << "Load DCC run conroller parameters from NonRT ok" << std::endl;
	return TRUE;
}
void RT_LoadDefalutDCCRunParms(DCCRunParms * p)
{
	std::cout<<"Load defalut DCC run conroller parameters" << std::endl;
	p->ZMP_Lag_T = 0.005;
	p->IMU_Lag_T = 0.04;
	p->Fz_Lag_T = 0.00;

	p->pitch_bias = +0.65 / 57.3;
	p->zmpbias_micro = 0.85;
	p->zmpx_bias = -0.0 * 0.02;  // for walk // -0.015; // for run
	p->zmpy_bias = -0.025; // * (k_pre < 3500? 1.0: (exp((-1.0 * k_pre + 3500.0) / 1000.0))); // for walk // -0.02;  // for run -0.02
							   // double zmpy_bias = -0.01;
	p->m_robot_bias = 4.5;
	p->roll_ampli = 2.5;
	p->tau_pitch_bias = -0.0 * 2.8;
	p->tau_roll_bias = 0.0;

	p->Zc = 0.7;
	double paras_LIPM[6] = {
		0.6 * 20.0, 10.4, 15.5,		// x -> kp, kv, kz
		1.5 * 12.0, 10.4, 22.5		// y -> kp, kv, kz
	};
	for (int i = 0; i < 6; i++) p->paras_LIPM[i] = paras_LIPM[i];

	double paras_TPC[6] = {
		22.0, 85.4, 20.0,		// x -> kzmp, kp, kd 
		18.0, 72.4, 28.0		// y -> kzmp, kp, kd 
	};
	for (int i = 0; i < 6; i++) p->paras_TPC[i] = paras_TPC[i];
	double limit_TPC[2] = { 0.04, 0.03 }; // x, y
	for (int i = 0; i < 2; i++) p->limit_TPC[i] = limit_TPC[i];

	double paras_GRFC_old[6] = {
			0.75 * 250.0, 0.85 * 70.0,			// pitch -> kp, kd 
			0.75 * 350.0, 0.85 * 75.0,			// roll  -> kp, kd 
			1e4, 8e3 //1e4,   1e4 	//5e4,   8e3				// zctrl -> kp, kd 
	};
	for (int i = 0; i < 6; i++) p->paras_GRFC_old[i] = paras_GRFC_old[i];

	double limit_GRFC_old[3] = { 10.0 / 57.3, 10.0 / 57.3, 0.015 }; // pitch, roll
	for (int i = 0; i < 3; i++) p->limit_GRFC[i] = limit_GRFC_old[i];
	double paras_GRFC[6] = {
		0.1, 10.0, 8.0,			// pitch -> kf, kp, kd 
		0.1, 10.0, 8.0				// roll  -> kf, kp, kd 
	};
	for (int i = 0; i < 6; i++) p->paras_GRFC[i] = paras_GRFC[i];

	double limit_GRFC[2] = { 30.0 / 57.3, 20.0 / 57.3 }; // pitch, roll
	for (int i = 0; i < 2; i++) p->limit_GRFC[i] = limit_GRFC[i];
	
	double paras_Rot[6] = {
		1.2 * 100.0, 50.0, 15.0,		// pitch -> kzmp, kp, kd 
		1.2 * 100.0, 50.0, 15.0		// roll  -> kzmp, kp, kd 
	};
	for (int i = 0; i < 6; i++)p->paras_Rot[i] = paras_Rot[i];

	double limit_Rot[2] = { 20.0 / 57.3, 15.0 / 57.3 };// pitch, roll
	for (int i = 0; i < 2; i++)p->limit_Rot[i] = limit_Rot[i];

	double Balance_Pro[2] = { 10.0, 10.0 }; // pitch, roll
	for (int i = 0; i < 2; i++)p->Balance_Pro[i] = Balance_Pro[i];
	
	double Micro_Pro[2] = { 0.2, 0.2 };
	for (int i = 0; i < 2; i++)p->Micro_Pro[i] = Micro_Pro[i];

	double limit_Pro[2] = { 50.0, 50.0 }; // pitch, roll
	for (int i = 0; i < 2; i++)p->limit_Pro[i]  = limit_Pro[i];

	
	double paras_FlyBody[6] = {
		145.9, 58.3, 16.7,		// pitch -> km, kp, kd
								// 120.9, 58.3, 16.7,		// pitch -> km, kp, kd
								145.9, 58.3, 16.7		// roll  -> km, kp, kd
														// 120.9, 58.3, 16.7		// roll  -> km, kp, kd
	};
	for (int i = 0; i < 6; i++) p->paras_FlyBody[i] = paras_FlyBody[i];

	double limit_FlyBody[2] = { 10.0 / 57.3, 10.0 / 57.3 }; // pitch, roll
	for (int i = 0; i < 2; i++) p->limit_FlyBody[i] = limit_FlyBody[i];
	
	double wake_FlyBody[2] = { 0.0 / 57.3,   0.0 / 57.3 }; // pitch, roll
	for (int i = 0; i < 2; i++) p->wake_FlyBody[i] = wake_FlyBody[i];

	double paras_FlyFoot[4] = {
		0.1 * 48.3, 3.0 * 19.7,		// pitch -> kp, kd
		0.1 * 48.3, 3.0 * 19.7		// roll  -> kp, kd
	};
	for (int i = 0; i < 4; i++) p->paras_FlyFoot[i] = paras_FlyFoot[i];
	
	double limit_FlyFoot[2] = { 5.0 / 57.3, 5.0 / 57.3 }; // pitch, roll
	for (int i = 0; i < 2; i++) p->limit_FlyFoot[i] = limit_FlyFoot[i];
	
	double wake_FlyFoot[2] = { 0.0 / 57.3, 0.0 / 57.3 }; // pitch, roll
														 // stepz
	for (int i = 0; i < 2; i++) p->wake_FlyFoot[i] = wake_FlyFoot[i];

	double paras_Step[12] = {
		60.0, 2.5 * 25.0, 1.25 * 9.0,		// z pitch -> kth, kp, kd
		60.0, 2.5 * 25.0, 1.25 * 9.0,		// z roll  -> kth, kp, kd
		20.0, 25.0, 9.0,					// r pitch -> rth, kp, kd
		20.0, 25.0, 9.0						// r roll  -> rth, kp, kd
	};
	for (int i = 0; i < 12; i++) p->paras_Step[i] = paras_Step[i];
	
	double limit_Step[3] = { 0.06, 10.0 * 57.3, 10 * 57.3 }; // z, pitch, roll
															 // LandRot
	for (int i = 0; i < 3; i++) p->limit_Step[i] = limit_Step[i];
	
	double paras_Land[11] = {
		1.2, 180.0, 275.0,		// pitch -> Kzmp, Kth, kom
		1.0, 150.0, 275.0,		// roll  -> Kzmp, Kth, kom
		0.3, 175.0, 28.0,		// I_robot, kp, kd
		0.04, 0.08				// kP2TY, kR2X 
	};
	for (int i = 0; i < 11; i++) p->paras_Land[i] = paras_Land[i];
	
	double limit_Land[2] = { 15.0 / 57.3, 10.0 / 57.3 }; // pitch, roll
	for (int i = 0; i < 2; i++) p->limit_Land[i]= limit_Land[i];
	
	double wake_Land[3] = { (1.0 + 5.3) / 57.3, -(1.0 + 7.0) / 57.3, (1.0 + 3.5) / 57.3 }; // pitch+ pitch-, roll
	for (int i = 0; i < 3; i++) p->wake_Land[i] = wake_Land[i];
																						   // contact
	
	double paras_Cont[5] = {
		100.0, 18000.0, 260, 0.00004, 0.36 // Kp, Kd, Km, Ki, Ke
	};
	for (int i = 0; i < 5; i++) p->paras_Cont[i] = paras_Cont[i];
	
	double limit_Cont[1] = { 0.1 };
	for (int i = 0; i < 1; i++) p->limit_Cont[i] = limit_Cont[i];
	
	double paras_Stepdown[4] = {
		0.112, 0.004, 250.0, 50     // H_ankle, StepDown, KeStepDown, k_down
	};
	for (int i = 0; i < 4; i++) p->paras_Stepdown[i] = paras_Stepdown[i];
	
	p->k_down = (int)(p->paras_Stepdown[3]);
	p->k_down_fore = 55; // no bigger than k_down
	p->H_ankle = p->paras_Stepdown[0];

	p->addi_pitch = 0.0 * 5.0 * 57.3;
	p->addi_Rfoot_pitch = 0;
	p->addi_Lfoot_pitch = 0;
	p->roll_bias_foot = 0.0;
	p->pitch_bias_foot = 28.0;
}
void RT_PrintfDCCRunParms(DCCRunParms * p)
{
	printf("\nDcc Running Control Parameters: \n");
	printf("ZMP_Lag_T: %.5f",p->ZMP_Lag_T); printf("\n");
	printf("IMU_Lag_T: %.5f", p->IMU_Lag_T); printf("\n");
	printf("Fz_Lag_T: %.5f", p->Fz_Lag_T); printf("\n");

	printf("pitch_bias: %.5f", p->pitch_bias); printf("\n");
	printf("zmpbias_micro: %.5f", p->zmpbias_micro); printf("\n");
	printf("zmpx_bias: %.5f", p->zmpx_bias); printf("\n");
	printf("zmpy_bias: %.5f", p->zmpy_bias); printf("\n");
	printf("m_robot_bias: %.5f", p->m_robot_bias); printf("\n");
	printf("roll_ampli: %.5f", p->roll_ampli); printf("\n");
	printf("tau_pitch_bias: %.5f", p->tau_pitch_bias); printf("\n");
	printf("tau_roll_bias: %.5f", p->tau_roll_bias); printf("\n");

	printf("Zc: %.5f", p->Zc); printf("\n");
	printf("paras_LIPM: ");	for (int i = 0; i < 6; i++)		printf("%.5f,", p->paras_LIPM[i]);		printf("\n");
	printf("paras_TPC: ");	for (int i = 0; i < 6; i++)		printf("%.5f,", p->paras_TPC[i]);		printf("\n");
	printf("limit_TPC: ");	for (int i = 0; i < 2; i++)		printf("%.5f,", p->limit_TPC[i]);		printf("\n");
	printf("paras_GRFC_old: ");	for (int i = 0; i < 6; i++)		printf("%.5f,", p->paras_GRFC_old[i]);		printf("\n");
	printf("limit_GRFC_old: ");	for (int i = 0; i < 3; i++)		printf("%.5f,", p->limit_GRFC_old[i]);		printf("\n");
	printf("paras_GRFC: ");	for (int i = 0; i < 6; i++)		printf("%.5f,", p->paras_GRFC[i]);		printf("\n");
	printf("limit_GRFC: ");	for (int i = 0; i < 3; i++)		printf("%.5f,", p->limit_GRFC[i]);		printf("\n");
	printf("paras_Rot: ");	for (int i = 0; i < 6; i++)		printf("%.5f,", p->paras_Rot[i]);		printf("\n");
	printf("limit_Rot: ");	for (int i = 0; i < 2; i++)		printf("%.5f,", p->limit_Rot[i]);		printf("\n");
	printf("Balance_Pro: "); for (int i = 0; i < 2; i++)		printf("%.5f,", p->Balance_Pro[i]);		printf("\n");
	printf("Micro_Pro: "); for (int i = 0; i < 2; i++)		printf("%.5f,", p->Micro_Pro[i]);		printf("\n");
	printf("limit_Pro: ");	for (int i = 0; i < 2; i++)		printf("%.5f,", p->limit_Pro[i]);		printf("\n");
	printf("paras_FlyBody: "); for (int i = 0; i < 6; i++)	printf("%.5f,", p->paras_FlyBody[i]);	printf("\n");
	printf("limit_FlyBody: "); for (int i = 0; i < 2; i++)	printf("%.5f,", p->limit_FlyBody[i]);	printf("\n");
	printf("wake_FlyBody[: "); for (int i = 0; i < 2; i++)	printf("%.5f,", p->wake_FlyBody[i]);	printf("\n");
	printf("paras_FlyFoot: "); for (int i = 0; i < 4; i++)	printf("%.5f,", p->paras_FlyFoot[i]);	printf("\n");
	printf("limit_FlyFoot: "); for (int i = 0; i < 2; i++)	printf("%.5f,", p->limit_FlyFoot[i]);	printf("\n");
	printf("wake_FlyFoot: ");  for (int i = 0; i < 2; i++)	printf("%.5f,", p->wake_FlyFoot[i]);	printf("\n");
	printf("paras_Step: ");	for (int i = 0; i <12; i++)		printf("%.5f,", p->paras_Step[i]);		printf("\n");
	printf("limit_Step: ");	for (int i = 0; i < 3; i++)		printf("%.5f,", p->limit_Step[i]);		printf("\n");
	printf("paras_Land: ");	for (int i = 0; i <11; i++)		printf("%.5f,", p->paras_Land[i]);		printf("\n");
	printf("limit_Land: ");	for (int i = 0; i < 2; i++)		printf("%.5f,", p->limit_Land[i]);		printf("\n");
	printf("wake_Land: ");	for (int i = 0; i < 3; i++)		printf("%.5f,", p->wake_Land[i]);		printf("\n");
	printf("paras_Cont: ");	for (int i = 0; i < 5; i++)		printf("%.5f,", p->paras_Cont[i]);		printf("\n");
	printf("limit_Cont: ");	for (int i = 0; i < 1; i++)		printf("%.5f,", p->limit_Cont[i]);		printf("\n");
	printf("paras_Stepdown: "); for (int i = 0; i < 4; i++)  printf("%.5f,", p->paras_Stepdown[i]);	printf("\n");
	
	printf("k_down: %d)",p->k_down);							printf("\n");
	printf("k_down_fore: %d)",p->k_down_fore);				printf("\n");
	printf("H_ankle: %.5f", p->H_ankle);						printf("\n");
	printf("addi_pitch: %.5f",p->addi_pitch);				printf("\n");
	printf("addi_Rfoot_pitch: %.5f",p->addi_Rfoot_pitch);	printf("\n");
	printf("addi_Lfoot_pitch: %.5f",p->addi_Lfoot_pitch);	printf("\n");
	printf("roll_bias_foot: %.5f",p->roll_bias_foot);		printf("\n");
	printf("pitch_bias_foot: %.5f",p->pitch_bias_foot);		printf("\n");

	printf("Dcc Running Control Parameters are Printed\n");
}
#endif