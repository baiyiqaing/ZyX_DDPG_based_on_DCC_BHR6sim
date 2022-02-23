//This function requires dcc_con_base.h 
#include "Dcc_lib\Base\dcc_con_base.h"
#include "TPCMPC.h" 

const double dVeMu_1x125[1][nNumPre] = {-13.3947858123839634,
-5.0528260672337266,
-1.8516603757795489,
-0.6238256495332778,
-0.1534600116377315,
0.0261570631281723,
0.0941808127406951,
0.1193817079026574,
0.1281579403129583,
0.1306426864010889,
0.1307256377572125,
0.1298998643504080,
0.1287384709024262,
0.1274614260120182,
0.1261531451256463,
0.1248460119312713,
0.1235524388950842,
0.1222771740413175,
0.1210220250579328,
0.1197876727182365,
0.1185743670078543,
0.1173821943672611,
0.1162111803029368,
0.1150613288032796,
0.1139326374965579,
0.1128251034973810,
0.1117387256790208,
0.1106735055739658,
0.1096294477480942,
0.1086065599730029,
0.1076048533210683,
0.1066243422310809,
0.1056650445628198,
0.1047269816476338,
0.1038101783377434,
0.1029146630553303,
0.1020404678418197,
0.1011876284075338,
0.1003561841817834,
0.0995461783634472,
0.0987576579720591,
0.0979906738994231,
0.0972452809617792,
0.0965215379525341,
0.0958195076955745,
0.0951392570991868,
0.0944808572105889,
0.0938443832711041,
0.0932299147719874,
0.0926375355109229,
0.0920673336492141,
0.0915194017696789,
0.0909938369352722,
0.0904907407484528,
0.0900102194113104,
0.0895523837864759,
0.0891173494588278,
0.0887052367980172,
0.0883161710218285,
0.0879502822603955,
0.0876077056212871,
0.0872885812554892,
0.0869930544242982,
0.0867212755671417,
0.0864734003703485,
0.0862495898368933,
0.0860500103571269,
0.0858748337805108,
0.0857242374883884,
0.0855984044677988,
0.0854975233863613,
0.0854217886682495,
0.0853714005712752,
0.0853465652650962,
0.0853474949105919,
0.0853744077403886,
0.0854275281405981,
0.0855070867337626,
0.0856133204630326,
0.0857464726776138,
0.0859067932194860,
0.0860945385114279,
0.0863099716463699,
0.0865533624780896,
0.0868249877132811,
0.0871251310050244,
0.0874540830476611,
0.0878121416731188,
0.0881996119487032,
0.0886168062763682,
0.0890640444935154,
0.0895416539753189,
0.0900499697386217,
0.0905893345474154,
0.0911600990199435,
0.0917626217374414,
0.0923972693545646,
0.0930644167115392,
0.0937644469481290,
0.0944977516195976,
0.0952647308151007,
0.0960657932795572,
0.0969013565417813,
0.0977718470559620,
0.0986777003749905,
0.0996193614037558,
0.1005972848577258,
0.1016119362532246,
0.1026637942797031,
0.1037533567691019,
0.1048811560328030,
0.1060477985989850,
0.1072540685125699,
0.1085011962197898,
0.1097915588167239,
0.1111305040481528,
0.1125311018044354,
0.1140265220910042,
0.1157022808511047,
0.1177802438760440,
0.1208374666086654,
0.1263762972814138,
0.1383095622931933,
0.1668296482912100,
0.2384879151551080};

const double dMaMx_125x3[nNumPre][nStateNum] = {{0.6703200460356393,0.3296799539643608,0.0007032004603564},
{0.4493289641172216,0.5506710358827785,0.0024932896411722},
{0.3011942119122021,0.6988057880877980,0.0050119421191220},
{0.2018965179946554,0.7981034820053448,0.0080189651799466},
{0.1353352832366127,0.8646647167633875,0.0113533528323661},
{0.0907179532894125,0.9092820467105878,0.0149071795328941},
{0.0608100626252180,0.9391899373747823,0.0186081006262522},
{0.0407622039783662,0.9592377960216341,0.0224076220397837},
{0.0273237224472926,0.9726762775527077,0.0262732372244729},
{0.0183156388887342,0.9816843611112661,0.0301831563888873},
{0.0122773399030684,0.9877226600969319,0.0341227733990307},
{0.0082297470490200,0.9917702529509803,0.0380822974704902},
{0.0055165644207608,0.9944834355792396,0.0420551656442076},
{0.0036978637164829,0.9963021362835174,0.0460369786371648},
{0.0024787521766664,0.9975212478233340,0.0500247875217667},
{0.0016615572731739,0.9983384427268264,0.0540166155727318},
{0.0011137751478448,0.9988862248521555,0.0580111377514785},
{0.0007465858083767,0.9992534141916236,0.0620074658580838},
{0.0005004514334406,0.9994995485665596,0.0660050045143344},
{0.0003354626279025,0.9996645373720977,0.0700033546262791},
{0.0002248673241788,0.9997751326758214,0.0740022486732418},
{0.0001507330750955,0.9998492669249048,0.0780015073307510},
{0.0001010394018371,0.9998989605981632,0.0820010103940184},
{0.0000677287364909,0.9999322712635095,0.0860006772873649},
{0.0000453999297625,0.9999546000702378,0.0900004539992977},
{0.0000304324830084,0.9999695675169920,0.0940003043248301},
{0.0000203995034112,0.9999796004965892,0.0980002039950342},
{0.0000136741960657,0.9999863258039347,0.1020001367419607},
{0.0000091660877362,0.9999908339122641,0.1060000916608774},
{0.0000061442123533,0.9999938557876470,0.1100000614421236},
{0.0000041185887075,0.9999958814112928,0.1140000411858871},
{0.0000027607725720,0.9999972392274283,0.1180000276077258},
{0.0000018506011976,0.9999981493988028,0.1220000185060120},
{0.0000012404950800,0.9999987595049205,0.1260000124049508},
{0.0000008315287191,0.9999991684712813,0.1300000083152872},
{0.0000005573903693,0.9999994426096311,0.1340000055739037},
{0.0000003736299380,0.9999996263700625,0.1380000037362994},
{0.0000002504516372,0.9999997495483632,0.1420000025045164},
{0.0000001678827530,0.9999998321172475,0.1460000016788276},
{0.0000001125351747,0.9999998874648258,0.1500000011253518},
{0.0000000754345835,0.9999999245654171,0.1540000007543459},
{0.0000000505653135,0.9999999494346871,0.1580000005056532},
{0.0000000338949433,0.9999999661050573,0.1620000003389495},
{0.0000000227204599,0.9999999772795406,0.1660000002272046},
{0.0000000152299797,0.9999999847700208,0.1700000001522998},
{0.0000000102089607,0.9999999897910399,0.1740000001020896},
{0.0000000068432710,0.9999999931567296,0.1780000000684327},
{0.0000000045871817,0.9999999954128189,0.1820000000458718},
{0.0000000030748799,0.9999999969251208,0.1860000000307488},
{0.0000000020611536,0.9999999979388470,0.1900000000206116},
{0.0000000013816326,0.9999999986183680,0.1940000000138164},
{0.0000000009261360,0.9999999990738645,0.1980000000092614},
{0.0000000006208075,0.9999999993791929,0.2020000000062081},
{0.0000000004161397,0.9999999995838608,0.2060000000041615},
{0.0000000002789468,0.9999999997210537,0.2100000000027895},
{0.0000000001869836,0.9999999998130169,0.2140000000018699},
{0.0000000001253389,0.9999999998746617,0.2180000000012535},
{0.0000000000840172,0.9999999999159834,0.2220000000008402},
{0.0000000000563184,0.9999999999436822,0.2260000000005633},
{0.0000000000377513,0.9999999999622492,0.2300000000003776},
{0.0000000000253055,0.9999999999746950,0.2340000000002531},
{0.0000000000169628,0.9999999999830377,0.2380000000001697},
{0.0000000000113705,0.9999999999886300,0.2420000000001138},
{0.0000000000076219,0.9999999999923787,0.2460000000000763},
{0.0000000000051091,0.9999999999948914,0.2500000000000511},
{0.0000000000034247,0.9999999999965757,0.2540000000000343},
{0.0000000000022957,0.9999999999977048,0.2580000000000230},
{0.0000000000015388,0.9999999999984617,0.2620000000000155},
{0.0000000000010315,0.9999999999989690,0.2660000000000105},
{0.0000000000006914,0.9999999999993091,0.2700000000000071},
{0.0000000000004635,0.9999999999995370,0.2740000000000048},
{0.0000000000003107,0.9999999999996898,0.2780000000000032},
{0.0000000000002083,0.9999999999997923,0.2820000000000022},
{0.0000000000001396,0.9999999999998609,0.2860000000000016},
{0.0000000000000936,0.9999999999999070,0.2900000000000011},
{0.0000000000000627,0.9999999999999378,0.2940000000000008},
{0.0000000000000420,0.9999999999999585,0.2980000000000006},
{0.0000000000000282,0.9999999999999724,0.3020000000000004},
{0.0000000000000189,0.9999999999999817,0.3060000000000003},
{0.0000000000000127,0.9999999999999879,0.3100000000000003},
{0.0000000000000085,0.9999999999999921,0.3140000000000002},
{0.0000000000000057,0.9999999999999949,0.3180000000000002},
{0.0000000000000038,0.9999999999999968,0.3220000000000002},
{0.0000000000000026,0.9999999999999980,0.3260000000000002},
{0.0000000000000017,0.9999999999999989,0.3300000000000002},
{0.0000000000000011,0.9999999999999994,0.3340000000000002},
{0.0000000000000008,0.9999999999999998,0.3380000000000002},
{0.0000000000000005,1.0000000000000000,0.3420000000000002},
{0.0000000000000003,1.0000000000000002,0.3460000000000002},
{0.0000000000000002,1.0000000000000004,0.3500000000000002},
{0.0000000000000002,1.0000000000000004,0.3540000000000002},
{0.0000000000000001,1.0000000000000004,0.3580000000000002},
{0.0000000000000001,1.0000000000000004,0.3620000000000002},
{0.0000000000000000,1.0000000000000004,0.3660000000000002},
{0.0000000000000000,1.0000000000000004,0.3700000000000002},
{0.0000000000000000,1.0000000000000004,0.3740000000000002},
{0.0000000000000000,1.0000000000000004,0.3780000000000002},
{0.0000000000000000,1.0000000000000004,0.3820000000000002},
{0.0000000000000000,1.0000000000000004,0.3860000000000002},
{0.0000000000000000,1.0000000000000004,0.3900000000000002},
{0.0000000000000000,1.0000000000000004,0.3940000000000002},
{0.0000000000000000,1.0000000000000004,0.3980000000000002},
{0.0000000000000000,1.0000000000000004,0.4020000000000002},
{0.0000000000000000,1.0000000000000004,0.4060000000000002},
{0.0000000000000000,1.0000000000000004,0.4100000000000003},
{0.0000000000000000,1.0000000000000004,0.4140000000000003},
{0.0000000000000000,1.0000000000000004,0.4180000000000003},
{0.0000000000000000,1.0000000000000004,0.4220000000000003},
{0.0000000000000000,1.0000000000000004,0.4260000000000003},
{0.0000000000000000,1.0000000000000004,0.4300000000000003},
{0.0000000000000000,1.0000000000000004,0.4340000000000003},
{0.0000000000000000,1.0000000000000004,0.4380000000000003},
{0.0000000000000000,1.0000000000000004,0.4420000000000003},
{0.0000000000000000,1.0000000000000004,0.4460000000000003},
{0.0000000000000000,1.0000000000000004,0.4500000000000003},
{0.0000000000000000,1.0000000000000004,0.4540000000000003},
{0.0000000000000000,1.0000000000000004,0.4580000000000003},
{0.0000000000000000,1.0000000000000004,0.4620000000000003},
{0.0000000000000000,1.0000000000000004,0.4660000000000003},
{0.0000000000000000,1.0000000000000004,0.4700000000000003},
{0.0000000000000000,1.0000000000000004,0.4740000000000003},
{0.0000000000000000,1.0000000000000004,0.4780000000000003},
{0.0000000000000000,1.0000000000000004,0.4820000000000003},
{0.0000000000000000,1.0000000000000004,0.4860000000000003},
{0.0000000000000000,1.0000000000000004,0.4860000000000003}};
const double dMPC_T = 0.004000;
double dTPCMPCConval[2];

/**
Ref trajectorys in x direction & y direction are required
To obtain a good control performance, States should be calcualted from an observer
To use as an tracking controller, States can calculate from Val^{sens} - Val^{ref}, and Ref trajectorys can be calculated from Tra^{ref} - Tra^{sens}, Tra^{sens} is expanded to the same dimension as Tra^{ref} but value is maintaining the same
*/
void fnvTPCMPCCalConval(double dVeZmpRefx_125x1[nNumPre][1], double dVeZmpRefy_125x1[nNumPre][1], double dVeStatex_3x1[nStateNum][1], double dVeStatey_3x1[nStateNum][1]) {
	double dVeZmpRelx_125x1[nNumPre][1], dVeZmpRely_125x1[nNumPre][1];
	dcc_fnvMatMet(&dMaMx_125x3[0][0], &dVeStatex_3x1[0][0], nNumPre, nStateNum, 1, '*', &dVeZmpRelx_125x1[0][0]);
	dcc_fnvMatMet(&dMaMx_125x3[0][0], &dVeStatey_3x1[0][0], nNumPre, nStateNum, 1, '*', &dVeZmpRely_125x1[0][0]);
	double dVeDeltaZmpx_125x1[nNumPre][1], dVeDeltaZmpy_125x1[nNumPre][1];
	dcc_fnvMatMet(&dVeZmpRefx_125x1[0][0], &dVeZmpRelx_125x1[0][0], nNumPre, 1, 1, '-', &dVeDeltaZmpx_125x1[0][0]);
	dcc_fnvMatMet(&dVeZmpRefy_125x1[0][0], &dVeZmpRely_125x1[0][0], nNumPre, 1, 1, '-', &dVeDeltaZmpy_125x1[0][0]);
	double dGaUx[1][1], dGaUy[1][1];
	dcc_fnvMatMet(&dVeMu_1x125[0][0], &dVeDeltaZmpx_125x1[0][0], 1, nNumPre, 1, '*', &dGaUx[0][0]);
	dcc_fnvMatMet(&dVeMu_1x125[0][0], &dVeDeltaZmpy_125x1[0][0], 1, nNumPre, 1, '*', &dGaUy[0][0]);

	dTPCMPCConval[0] = dGaUx[0][0];
	dTPCMPCConval[1] = dGaUy[0][0];

}
