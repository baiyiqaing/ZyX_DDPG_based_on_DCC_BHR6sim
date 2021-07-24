#include "ChzCpp2C.h"
#include <ChzControlLib/Chz_Include.h>
extern "C"
{
	extern double chz_log[40];
}
namespace Chz
{
	extern LandingForceController LandingFCon_LZ;
	extern LandingForceController LandingFCon_RZ;
	extern LandingForceController LandingFCon_LY;
	extern LandingForceController LandingFCon_RY;
	extern LandingForceController LandingFCon_LP;
	extern LandingForceController LandingFCon_RP;
	
	extern PerspectiveMapping2d ZMPMapping_L;
	extern PerspectiveMapping2d ZMPMapping_R;
	
	extern LowPassFilter5 LPFilters[10];
	
	extern LowLevelFootStepController FootConX, FootConY;
	
	extern WaistComp Qbodyr, Qbodyl;
	
	extern FootDownController FootDown_L, FootDown_R;
	
	extern SwingFootController SwingFootCon;
}

double chz_LandingForce_getdelz(char lr, char yz)
{
	if(lr == 'L' && yz == 'Y') return Chz::LandingFCon_LY.Getdelz();
	if(lr == 'L' && yz == 'Z') return Chz::LandingFCon_LZ.Getdelz();
	if(lr == 'L' && yz == 'P') return Chz::LandingFCon_LP.Getdelz();
	if(lr == 'R' && yz == 'Y') return Chz::LandingFCon_RY.Getdelz();
	if(lr == 'R' && yz == 'Z') return Chz::LandingFCon_RZ.Getdelz();
	if(lr == 'R' && yz == 'P') return Chz::LandingFCon_RP.Getdelz();
	return 0.0;
}
void chz_LandingForce_setmode(char m, char lr, char yz)
{
	Chz::LandingForceController* tempcon = NULL;
	if(lr == 'L' && yz == 'Y') tempcon = &Chz::LandingFCon_LY;
	if(lr == 'L' && yz == 'Z') tempcon = &Chz::LandingFCon_LZ;
	if(lr == 'L' && yz == 'P') tempcon = &Chz::LandingFCon_LP;
	if(lr == 'R' && yz == 'Y') tempcon = &Chz::LandingFCon_RY;
	if(lr == 'R' && yz == 'Z') tempcon = &Chz::LandingFCon_RZ;
	if(lr == 'R' && yz == 'P') tempcon = &Chz::LandingFCon_RP;
	
	if(m == 'W') tempcon->SetMode("WAIT");
	else if(m == 'O') tempcon->SetMode("PO");
	else if(m == 'C') tempcon->SetMode("PC");
	else if(m == 'I') tempcon->SetMode("INTE");
}
void chz_LandingForce_update(double f, double z2next, char lr, char yz)
{
	Chz::LandingForceController* tempcon = NULL;
	if(lr == 'L' && yz == 'Y') tempcon = &Chz::LandingFCon_LY;
	if(lr == 'L' && yz == 'Z') tempcon = &Chz::LandingFCon_LZ;
	if(lr == 'L' && yz == 'P') tempcon = &Chz::LandingFCon_LP;
	if(lr == 'R' && yz == 'Y') tempcon = &Chz::LandingFCon_RY;
	if(lr == 'R' && yz == 'Z') tempcon = &Chz::LandingFCon_RZ;
	if(lr == 'R' && yz == 'P') tempcon = &Chz::LandingFCon_RP;
	
	tempcon->Update(f, z2next);
}

void chz_ZMPMapping_getpq(double p[][2], double q[][2], char c)
{
	Eigen::Vector2d p1[4], q1[4];
	ChzF(i, 0, 3)
	{
		p1[i] << p[i][0], p[i][1];
		q1[i] << q[i][0], q[i][1];
	}
	if(c == 'R')
	{
		Chz::ZMPMapping_R.Setpq(p1, q1);
		Chz::ZMPMapping_R.CalH();
	}
	else if(c == 'L')
	{
		Chz::ZMPMapping_L.Setpq(p1, q1);
		Chz::ZMPMapping_L.CalH();
	}
}
void chz_ZMPMapping_map(double* qx, double* qy, double px, double py, char c)
{
	Eigen::Vector2d v1(px, py);
	if(c == 'R')
		v1 = Chz::ZMPMapping_R.Getq(v1);
	else if(c == 'L')
		v1 = Chz::ZMPMapping_L.Getq(v1);
	*qx = v1(0);
	*qy = v1(1);
}

void chz_LPFilters_init(int i, double xini)
{
	Chz::LPFilters[i].Init(xini);
}
double chz_LPFilters_update(int i, double xin)
{
	return Chz::LPFilters[i].Update(xin);
}

void chz_LowLevelFoot_setmode(char xy, char lr, char mode1)
{
	Chz::LowLevelFootStepController* tempcon = (xy == 'X'? &Chz::FootConX: &Chz::FootConY);
	tempcon->Setmode(lr, mode1);
}
void chz_LowLevelFoot_setxdes(char xy, double RXdes1, double LXdes1)
{
	Chz::LowLevelFootStepController* tempcon = (xy == 'X'? &Chz::FootConX: &Chz::FootConY);
	tempcon->SetXdes(RXdes1, LXdes1);
}
void chz_LowLevelFoot_updatexcon(char xy, double* RXCon1, double* LXCon1)
{
	Chz::LowLevelFootStepController* tempcon = (xy == 'X'? &Chz::FootConX: &Chz::FootConY);
	tempcon->UpdateXCon(RXCon1, LXCon1);
}

void chz_WaistComp_setmode(char lr, char mode)
{
	Chz::WaistComp* tempcon = (lr == 'R'? &Chz::Qbodyr: &Chz::Qbodyl);
	tempcon->SetMode(mode);
}
void chz_WaistComp_setparam(char lr, double ang1, int intenum)
{
	Chz::WaistComp* tempcon = (lr == 'R'? &Chz::Qbodyr: &Chz::Qbodyl);
	tempcon->SetParam(ang1, intenum);
}
void chz_WaistComp_updatecon(double* rwaistcon, double * lwaistcon)
{
	*rwaistcon = Chz::Qbodyr.UpdateCon();
	*lwaistcon = Chz::Qbodyl.UpdateCon();
}

void chz_FootDown_setmode(char lr, char mode)
{
	Chz::FootDownController* tempcon;
	if(lr == 'L') tempcon = &Chz::FootDown_L;
	else if(lr == 'R') tempcon = &Chz::FootDown_R;
	else return;
	tempcon->SetMode(mode);
}
double chz_FootDown_updatecon(char lr)
{
	Chz::FootDownController* tempcon;
	if(lr == 'L') tempcon = &Chz::FootDown_L;
	else if(lr == 'R') tempcon = &Chz::FootDown_R;
	else return 0.0;
	return tempcon->UpdateCon();
}

void chz_SwingFoot_setprop(double rprop1)
{
	Chz::SwingFootCon.SetProp(rprop1);
}
void chz_SwingFoot_setmode(char lr, int mode)
{
	Chz::SwingFootCon.SetMode(lr, mode);
}
void chz_SwingFoot_setposlim(double pos_lim[6][2])
{
	Chz::SwingFootCon.SetPoslim(pos_lim);
}
void chz_SwingFoot_update(double CoMd1[6], double RFootd1[6], double LFootd1[6], double CoMr1[6], double joints[12])
{
	using namespace Eigen;
	VectorXd CoMd = VectorXd::Zero(6), RFootd = VectorXd::Zero(6), LFootd = VectorXd::Zero(6), CoMr = VectorXd::Zero(6), Joints = VectorXd::Zero(12);
	ChzF(i, 0, 5)
	{
		CoMd(i) = CoMd1[i];
		RFootd(i) = RFootd1[i];
		LFootd(i) = LFootd1[i];
		CoMr(i) = CoMr1[i];
	}
	ChzF(i, 0, 11) Joints(i) = joints[i];
	Chz::SwingFootCon.Update(CoMd, RFootd, LFootd, CoMr, Joints);
}
void chz_SwingFoot_outputcomr(double CoMr[3])
{
	using namespace Eigen;
	Vector3d vtemp;
	Chz::SwingFootCon.OutputCoMr(vtemp);
	//ChzF(i, 0, 2) CoMr[i] = vtemp(i);
	CoMr[0] = -vtemp(1);
	CoMr[1] = vtemp(0);
	CoMr[2] = vtemp(2);
}
void chz_SwingFoot_outputcon(double Rcon1[6], double Lcon1[6])
{
	using namespace Eigen;
	VectorXd RCon = VectorXd::Zero(6), LCon = VectorXd::Zero(6);
	Chz::SwingFootCon.Outputcon(RCon, LCon);
	ChzF(i, 0, 5)
	{
		Rcon1[i] = RCon(i);
		Lcon1[i] = LCon(i);
	}
}

double chz_Bias_getval(double prop)
{
	static double a[7], x[3];
	static bool iffirst = 1;
	if(iffirst) Chz::GenerateSpline6(a, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0), iffirst = 0;
	Chz::GetSpline6(x, a, prop);
	if(prop <= 0) return 0.0;
	if(prop >= 1) return 1.0;
	else return x[0];
}