#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus
	double chz_LandingForce_getdelz(char lr, char yz);
	void chz_LandingForce_setmode(char m, char lr, char yz);
	void chz_LandingForce_update(double f, double z2next, char lr, char yz);

	void chz_ZMPMapping_getpq(double p[][2], double q[][2], char c);
	void chz_ZMPMapping_map(double* qx, double* qy, double px, double py, char c);

	void chz_LPFilters_init(int i, double xini);
	double chz_LPFilters_update(int i, double xin);
	
	void chz_LowLevelFoot_setmode(char xy, char lr, char mode1);
	void chz_LowLevelFoot_setxdes(char xy, double RXdes1, double LXdes1);
	void chz_LowLevelFoot_updatexcon(char xy, double* RXCon1, double* LXCon1);
	
	void chz_WaistComp_setmode(char lr, char mode);
	void chz_WaistComp_setparam(char lr, double ang1, int intenum);
	void chz_WaistComp_updatecon(double* lwaistcon, double* rwaistcon);
	
	void chz_FootDown_setmode(char lr, char mode);
	double chz_FootDown_updatecon(char lr);
	
	void chz_SwingFoot_setprop(double rprop1);
	void chz_SwingFoot_setmode(char lr, int mode);
	void chz_SwingFoot_setposlim(double pos_lim[6][2]);
	void chz_SwingFoot_update(double CoMd[6], double RFootd[6], double LFootd[6], double CoMr[6], double joints[12]);
	void chz_SwingFoot_outputcomr(double CoMr[3]);
	void chz_SwingFoot_outputcon(double Rcon[6], double Lcon[6]);

	double chz_Bias_getval(double prop);
#ifdef __cplusplus
}
#endif // __cplusplus
