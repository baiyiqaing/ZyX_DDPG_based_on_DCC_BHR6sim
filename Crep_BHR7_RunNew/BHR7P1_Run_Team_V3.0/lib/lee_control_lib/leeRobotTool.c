#include "leeRobotTool.h"
#include "leeRobotPlan.h"


/*-----------------基本计算函数-------------------*/
#ifdef ROBOT_BASIC_FUNCTION

_rf_t deg2rad(const _rv_t deg)
{
	return deg / 180.0*_pi;
}

_rf_t rad2deg(const _rv_t rad)
{
	return rad / _pi*180.0;
}

_rf_t aprox(const _rv_t value, const _rv_t standard)
{
	if (_rabs(value - standard) < MIN_ERROR)
	{
		if (standard < 0)
		{
			return (standard + MIN_ERROR);
		}
		else if (standard == 0)
		{
			return (standard);
		}
		else
		{
			return (standard - MIN_ERROR);
		}
	}
	else
	{
		return value;
	}
}

_rf_t aprox_all(const _rv_t value)
{
	_rv_t v = value;
	v = aprox(v, 1.0);
	v = aprox(v, 0.0);
	v = aprox(v, -1.0);
	return v;
}

_rf_t solve_sc(const _rv_t a, const _rv_t b, const _rv_t c, const int num)
{
	_rv_t sin_value;
	_rv_t cos_value;
	_rv_t sqrt_value;
	_rv_t res;
	sqrt_value = aprox_all(b*b - c*c + a*a);
	sqrt_value = _rsqrt(sqrt_value);
	if (num == 1)
	{
		sin_value = aprox_all((-a*c - b*sqrt_value) / (b*b + a*a));
		cos_value = aprox_all((-b*c + a*sqrt_value) / (b*b + a*a));
	}
	else
	{
		sin_value = aprox_all((-a*c + b*sqrt_value) / (b*b + a*a));
		cos_value = aprox_all((-b*c - a*sqrt_value) / (b*b + a*a));
	}
	res = _ratan2(sin_value, cos_value);
	return res;
}

_rf_t solve_sc_2(const _rv_t factor[6])
{
	_rv_t res[4];

	res[0] = solve_sc(factor[0], factor[1], factor[2], 1);
	res[1] = solve_sc(factor[0], factor[1], factor[2], 2);
	res[2] = solve_sc(factor[3], factor[4], factor[5], 1);
	res[3] = solve_sc(factor[3], factor[4], factor[5], 2);

	if (equal_check(res[0], res[2]) || equal_check(res[0], res[3]))
	{
		return res[0];
	}
	else
	{
		return res[1];
	}
}

_rf_flg	equal_check(const _rv_t v1, const _rv_t v2)
{
	if (_rabs(v1 - v2) <= MIN_ERROR)
		return 1;
	else
		return 0;
}

_rf_flg	res_choose(const _rv_t *ang, const _rv_t * ref, const int joint_num, const int res_num)
{
	int op_num = -1;
	int i1, i2;
	_rv_t dif = 0.0;
	_rv_t dif_min = 9999.9;

	for (i1 = 0; i1 < res_num; i1++)
	{
		dif = 0.0;
		//printf("%d:\t", i1);
		for (i2 = 0; i2 < joint_num; i2++)
		{
			dif += _rabs(ref[i2] - ang[i2*res_num + i1]);

			//test: 
			//printf("%.3f\t", rad2deg(ang[i2*res_num + i1]));
		}

		if (dif_min > dif)
		{
			dif_min = dif;
			op_num = i1;
		}
		//printf("\n");
		//printf("[dif]%.2f  %d\n",dif,op_num);
	}

	return op_num;
}

_rf_flg	get5_ipf_va0(const _rv_t t, const _rv_t x1, const _rv_t x2, _rv_t f[6])
{
	f[0] = x1;
	f[1] = 0;
	f[2] = 0;
	f[3] = -(10.0 * (x1 - x2)) / _rpow(t, 3.0);
	f[4] = (15.0 * (x1 - x2)) / _rpow(t, 4.0);
	f[5] = -(6.0 * (x1 - x2)) / _rpow(t, 5.0);
	return 0;
}
#endif
/*<----------------基本计算函数------------------>*/


/*-----------------逆运动学计算函数-------------------*/
#ifdef ROBOT_IK_FUNCTION
//inverted kinemics for arm with 3 joints:
//Shoulder 1-pitch, Shoulder 2-roll, Elbow -pitch
_rf_flg ik_arm(const _rv_t T[4][4], _rv_t ang[3], const _rv_t ref[3])
{
	_rv_t d12 = -UPPER_ARM_LEN;
	_rv_t d23 = -LOWER_ARM_LEN;
	_rv_t px = T[_px];
	_rv_t py = T[_py];
	_rv_t pz = T[_pz];
	_rv_t angle[3][4] = { 0.0 };
	_rv_t solve_trans[3] = { 0.0 };
	int i;
	_rv_t a, b, c;

	if (_rpow(px, 2.0) + _rpow(py, 2.0) + _rpow(pz, 2.0) >= _rpow((d12 + d23), 2.0))
	{
		ang[0] = ref[0]; ang[1] = ref[1]; ang[2] = ref[2];
		return IK_FAILED;
	}

	//Get angle of elbow, the 3rd joint
	//cosr3
	solve_trans[2] = aprox_all((px*px + py*py + pz*pz - (d12*d12 + d23*d23)) / (2 * d23*d12));
	angle[2][0] = -_racos(solve_trans[2]);
	angle[2][1] = angle[2][0];
	//printf("[%f,%f],", DEG(angle[2][0]), DEG(angle[2][1]));

	//Get angle of shoulder-Roll, the 2nd joint
	//sinr2
	solve_trans[1] = aprox_all(-py / (solve_trans[2] * d23 + d12));
	solve_trans[1] = fmax(fmin(solve_trans[1], 1.0 - MIN_ERROR), -1.0 + MIN_ERROR);
	angle[1][0] = _rasin(solve_trans[1]);
	if (angle[1][0] >= 0)
	{
		angle[1][1] = _pi - angle[1][0];
	}
	else
	{
		angle[1][1] = -_pi - angle[1][0];
	}

	//printf("[%.10f,%f,%f],", solve_trans[1],DEG(angle[1][0]), DEG(angle[1][1]));
	//Get angle of shoulder-Pitch,the 1st joint
	for (i = 0; i < 2; i++)
	{
		a = _rsin(angle[2][i])*d23;
		b = -_rcos(angle[1][i])*d12 - d23*cos(angle[1][i])*cos(angle[2][i]);
		c = pz;
		angle[0][0] = solve_sc(a, b, c, 1);
		angle[0][1] = solve_sc(a, b, c, 2);
		a = cos(angle[1][i])*d12 + d23*cos(angle[1][i])*cos(angle[2][i]);
		b = sin(angle[2][i])*d23;
		c = -px;
		angle[0][2] = solve_sc(a, b, c, 1);
		angle[0][3] = solve_sc(a, b, c, 2);
		if (_rabs(angle[0][0] - angle[0][2]) < MIN_ERROR || _rabs(angle[0][0] - angle[0][3]) < MIN_ERROR)
		{
			angle[2][2 + i] = angle[0][0];//loan memory of elbow not used
		}
		else
		{
			angle[2][2 + i] = angle[0][1];//loan memory of elbow not used
		}
	}
	angle[0][0] = angle[2][2];
	angle[0][1] = angle[2][3];
	//printf("[%f,%f]\n", DEG(angle[0][0]), DEG(angle[0][1]));
	//Select one fit result
	/**
	solve_trans[0] = _rabs(angle[0][0] - ref[0]) + _rabs(angle[1][0] - ref[1]) + _rabs(angle[2][0] - ref[2]);
	solve_trans[1] = _rabs(angle[0][1] - ref[0]) + _rabs(angle[1][1] - ref[1]) + _rabs(angle[2][1] - ref[2]);
	if (solve_trans[0] > solve_trans[1])
	{
	i = 1;
	}
	else
	{
	i = 0;
	}/**/

	i = res_choose(&angle[0][0], ref, 3, 2);

	/*!!!!!!!!!!!!!!!!!!!!!*/
	//need limitition check
	/*!!!!!!!!!!!!!!!!!!!!!*/
	ang[0] = angle[0][i]; ang[1] = angle[1][i]; ang[2] = angle[2][i];
	return IK_SUCCEEDED;
}

//inverted kinemics for waist with 3 joints:
_rf_flg ik_waist(const _rv_t T[4][4], _rv_t ang[3], const _rv_t ref[3])
{
	_rv_t ref_pos[6];
	_rv_t pos[6];
	ref_pos[0] = ref[2];
	ref_pos[1] = ref[1];
	ref_pos[2] = ref[0];
	ref_pos[3] = T[_px];
	ref_pos[4] = T[_py];
	ref_pos[5] = T[_pz];
	if (ik_posture(T, pos, ref_pos) == IK_FAILED)
	{
		ang[0] = ref[0];
		ang[1] = ref[1];
		ang[2] = ref[2];
		return IK_FAILED;
	}

	ang[0] = pos[2];
	ang[1] = pos[1];
	ang[2] = pos[0];

	return IK_SUCCEEDED;
}

//inverted kinemics for Euler angles
//Rotation x y z: T = Rz*Ry*Rx
_rf_flg ik_posture(const _rv_t T[4][4], _rv_t pos[6], const _rv_t ref[6])
{
	_rv_t angle[3][2] = { 0.0 };//0:x; 1:y; 2:z
	_rv_t cosry;
	//_rv_t diff[2];
	int i;
	//Get angle of rotation along axis y
	angle[1][0] = _rasin(aprox_all(-T[_nz]));
	if (angle[1][0] >= 0)
	{
		angle[1][1] = _pi - angle[1][0];
	}
	else
	{
		angle[1][1] = -_pi - angle[1][0];
	}

	cosry = _rcos(angle[1][0]);
	if (_rabs(cosry) < MIN_ERROR)
	{
		for (i = 0; i < 6; i++) { pos[i] = ref[i]; }
		return IK_FAILED;
	}

	for (i = 0; i < 2; i++)
	{
		cosry = _rcos(angle[1][i]);
		angle[0][i] = _ratan2(T[_oz] / cosry, T[_az] / cosry);
		angle[2][i] = _ratan2(T[_ny] / cosry, T[_nx] / cosry);
	}
	/**
	diff[0] = _rabs(angle[0][0] - ref[0]) + _rabs(angle[1][0] - ref[1]) + _rabs(angle[2][0] - ref[2]);
	diff[1] = _rabs(angle[0][1] - ref[0]) + _rabs(angle[1][1] - ref[1]) + _rabs(angle[2][1] - ref[2]);
	if (diff[0] < diff[1])
	{
	i = 0;
	}
	else
	{
	i = 1;
	}/**/
	i = res_choose(&angle[0][0], ref, 3, 2);
	pos[0] = angle[0][i]; pos[1] = angle[1][i]; pos[2] = angle[2][i];
	pos[3] = T[_px]; pos[4] = T[_py]; pos[5] = T[_pz];

	return IK_SUCCEEDED;
}

//腿部逆运动学求解，从髋到踝
_rrm _rf_flg ik_leg(const _rv_t T[4][4], _rv_t ang[6], const _rv_t ref[6])
{
	_rv_t d34, d45;
	_rv_t nx, ox, ax, px;
	_rv_t ny, oy, ay, py;
	_rv_t nz, oz, az, pz;

	_rv_t r1, r2, r3, r4, r5;
	_rv_t sinr1, cosr1;
	_rv_t sinr2, cosr2;
	_rv_t sinr3, cosr3;
	_rv_t sinr4, cosr4;
	_rv_t sinr5, cosr5;
	_rv_t sinr6, cosr6;

	_rv_t angle[6][8];

	_rv_t temp[6];

	int i1, i2;
	
	// printf("%.8f %.8f\n", THIGH_LEN, CRUS_LEN);
	d34 = -CRUS_LEN;
	d45 = -THIGH_LEN;
	nx = T[_nx]; ox = T[_ox]; ax = T[_ax]; px = T[_px];
	ny = T[_ny]; oy = T[_oy]; ay = T[_ay]; py = T[_py];
	nz = T[_nz]; oz = T[_oz]; az = T[_az]; pz = T[_pz];

	//[ Get joint 4 ]
	//cosr4
	temp[0] = (d34*d34 + d45*d45 - (px*px + py*py + pz*pz)) / (_rabs(2.0*d34*d45));
	angle[3][0] = _pi - _rabs(_racos(temp[0]));
	angle[3][1] = angle[3][0];
	angle[3][2] = -angle[3][0];
	angle[3][3] = -angle[3][0];
	angle[3][4] = angle[3][0];
	angle[3][5] = angle[3][0];
	angle[3][6] = -angle[3][0];
	angle[3][7] = -angle[3][0];


	//[ Get joint 1 ]
	//sinr1 = Aprox(1 / (ny ^ 2 * pz ^ 2 - 2 * ny*pz*nz*py + nz ^ 2 * py ^ 2 + nx ^ 2 * pz ^ 2 - 2 * nx*pz*px*nz + px ^ 2 * nz ^ 2) ^ (1 / 2)*(ny*pz - nz*py));
	//cosr1 = Aprox(1 / (ny ^ 2 * pz ^ 2 - 2 * ny*pz*nz*py + nz ^ 2 * py ^ 2 + nx ^ 2 * pz ^ 2 - 2 * nx*pz*px*nz + px ^ 2 * nz ^ 2) ^ (1 / 2)*(nx*pz - px*nz));
	//sinr1
	temp[0] = aprox_all(1.0 / _rsqrt(_rpow(ny, 2.0) * _rpow(pz, 2.0) - 2.0 * ny*pz*nz*py + nz*nz * py*py + nx *nx * pz *pz - 2.0 * nx*pz*px*nz + px*px * nz*nz)*(ny*pz - nz*py));
	//cosr1
	temp[1] = aprox_all(1.0 / _rsqrt(ny*ny * pz*pz - 2.0 * ny*pz*nz*py + nz*nz * py*py + nx *nx * pz *pz - 2.0 * nx*pz*px*nz + px*px * nz*nz)*(nx*pz - px*nz));
	//sinr2 = Aprox(-1 / (ny ^ 2 * pz ^ 2 - 2 * ny*pz*nz*py + nz ^ 2 * py ^ 2 + nx ^ 2 * pz ^ 2 - 2 * nx*pz*px*nz + px ^ 2 * nz ^ 2) ^ (1 / 2)*(ny*pz - nz*py));
	//cosr2 = Aprox(-1 / (ny ^ 2 * pz ^ 2 - 2 * ny*pz*nz*py + nz ^ 2 * py ^ 2 + nx ^ 2 * pz ^ 2 - 2 * nx*pz*px*nz + px ^ 2 * nz ^ 2) ^ (1 / 2)*(nx*pz - px*nz));
	//sinr2
	temp[2] = aprox_all(-1.0 / _rsqrt(ny*ny * pz *pz - 2.0 * ny*pz*nz*py + nz *nz * py*py + nx *nx * pz *pz - 2.0 * nx*pz*px*nz + px*px * nz*nz)*(ny*pz - nz*py));
	//cosr2
	temp[3] = aprox_all(-1.0 / _rsqrt(ny*ny * pz*pz - 2.0 * ny*pz*nz*py + nz*nz * py*py + nx *nx * pz*pz - 2.0 * nx*pz*px*nz + px *px * nz *nz)*(nx*pz - px*nz));

	angle[0][0] = _ratan2(temp[0], temp[1]);
	angle[0][4] = _ratan2(temp[2], temp[3]);
	for (i1 = 1; i1 < 4; i1++)
	{
		angle[0][i1] = angle[0][0];
	}
	for (i1 = 5; i1 < 8; i1++)
	{
		angle[0][i1] = angle[0][4];
	}

	//[ Get joint 2 ]
	//leg(1, 2) = -atan(1 / (ny ^ 2 * pz ^ 2 - 2 * ny*pz*nz*py + nz ^ 2 * py ^ 2 + nx ^ 2 * pz ^ 2 - 2 * nx*pz*px*nz + px ^ 2 * nz ^ 2) ^ (1 / 2)*(-px*ny + nx*py));
	angle[1][0] = -_ratan(1.0 / _rsqrt(ny*ny * pz*pz - 2.0 * ny*pz*nz*py + nz*nz * py*py + nx*nx * pz*pz - 2.0 * nx*pz*px*nz + px*px * nz*nz)*(-px*ny + nx*py));
	//leg(2, 2) = leg(1, 2) + _pi - 2 * _pi*(leg(1, 2)>0);			
	angle[1][1] = angle[1][0] + _pi - 2.0*_pi*(angle[1][0] > 0.0 ? 1.0 : 0.0);
	//if (equal_check(angle[1][0], 0.0))
	//angle[1][1] = angle[1][0];
	//leg(3, 2) = leg(1, 2);
	angle[1][2] = angle[1][0];
	//leg(4, 2) = leg(2, 2);
	angle[1][3] = angle[1][1];
	//leg(5, 2) = atan(1 / (ny ^ 2 * pz ^ 2 - 2 * ny*pz*nz*py + nz ^ 2 * py ^ 2 + nx ^ 2 * pz ^ 2 - 2 * nx*pz*px*nz + px ^ 2 * nz ^ 2) ^ (1 / 2)*(-px*ny + nx*py));
	angle[1][4] = -angle[1][0];
	//leg(6, 2) = leg(5, 2) + _pi - 2 * _pi*(leg(5, 2)>0);
	angle[1][5] = angle[1][4] + _pi - 2.0*_pi*(angle[1][4] > 0.0 ? 1.0 : 0.0);
	//if (equal_check(angle[1][4], 0.0))
	//angle[1][5] = angle[1][4];
	//leg(7, 2) = leg(5, 2);
	angle[1][6] = angle[1][4];
	//leg(8, 2) = leg(6, 2);
	angle[1][7] = angle[1][5];

	//[ Get joint 3 ]
	for (i1 = 0; i1 < 8; i1++)
	{
		r1 = angle[0][i1]; sinr1 = _rsin(r1); cosr1 = _rcos(r1);
		r2 = angle[1][i1]; sinr2 = _rsin(r2); cosr2 = _rcos(r2);
		r4 = angle[3][i1]; sinr4 = _rsin(r4); cosr4 = _rcos(r4);

		//sinr3 = Aprox((d34*cos(r1)*px + d45*cos(r4)*cos(r1)*px + d34*sin(r1)*py + d45*cos(r4)*sin(r1)*py + (d45 ^ 2 * (-1 + cos(r4) ^ 2)*(-2 * d45*cos(r4)*d34 + 2 * cos(r1)*px*sin(r1)*py - d45 ^ 2 - py ^ 2 * cos(r1) ^ 2 + px ^ 2 * cos(r1) ^ 2 + py ^ 2 - d34 ^ 2)) ^ (1 / 2)) / (2 * d45*cos(r4)*d34 + d34 ^ 2 + d45 ^ 2));
		//cosr3 = Aprox(-(d45 ^ 2 * cos(r4) ^ 2 * cos(r1)*px + d45 ^ 2 * cos(r4) ^ 2 * sin(r1)*py + d45*cos(r4)*(d45 ^ 2 * (-1 + cos(r4) ^ 2)*(-2 * d45*cos(r4)*d34 + 2 * cos(r1)*px*sin(r1)*py - d45 ^ 2 - py ^ 2 * cos(r1) ^ 2 + px ^ 2 * cos(r1) ^ 2 + py ^ 2 - d34 ^ 2)) ^ (1 / 2) + d34*(d45 ^ 2 * (-1 + cos(r4) ^ 2)*(-2 * d45*cos(r4)*d34 + 2 * cos(r1)*px*sin(r1)*py - d45 ^ 2 - py ^ 2 * cos(r1) ^ 2 + px ^ 2 * cos(r1) ^ 2 + py ^ 2 - d34 ^ 2)) ^ (1 / 2) - cos(r1)*px*d45 ^ 2 - sin(r1)*py*d45 ^ 2) / (2 * d45*cos(r4)*d34 + d34 ^ 2 + d45 ^ 2) / d45 / sin(r4));
		//r31(1) = atan2(sinr3, cosr3);
		sinr3 = aprox_all((d34*cosr1*px + d45*cosr4*cosr1*px + d34*sinr1*py + d45*cosr4*sinr1*py + _rsqrt(d45*d45*(-1 + cosr4*cosr4)*(-2 * d45*cosr4*d34 + 2 * cosr1*px*sinr1*py - d45*d45 - py*py*cosr1*cosr1 + px*px*cosr1*cosr1 + py*py - d34*d34))) / (2 * d45*cosr4*d34 + d34*d34 + d45*d45));
		cosr3 = aprox_all(-(d45*d45*cosr4*cosr4*cosr1*px + d45*d45*cosr4*cosr4*sinr1*py + d45*cosr4*_rsqrt(d45*d45*(-1 + cosr4*cosr4)*(-2 * d45*cosr4*d34 + 2 * cosr1*px*sinr1*py - d45*d45 - py*py*cosr1*cosr1 + px*px*cosr1*cosr1 + py*py - d34*d34)) + d34*_rsqrt(d45*d45*(-1 + cosr4*cosr4)*(-2 * d45*cosr4*d34 + 2 * cosr1*px*sinr1*py - d45*d45 - py*py*cosr1*cosr1 + px*px*cosr1*cosr1 + py*py - d34*d34)) - cosr1*px*d45*d45 - sinr1*py*d45*d45) / (2 * d45*cosr4*d34 + d34*d34 + d45*d45) / d45 / sinr4);
		temp[0] = _ratan2(sinr3, cosr3);
		//sinr3 = Aprox((d34*cos(r1)*px + d45*cos(r4)*cos(r1)*px + d34*sin(r1)*py + d45*cos(r4)*sin(r1)*py - (d45 ^ 2 * (-1 + cos(r4) ^ 2)*(-2 * d45*cos(r4)*d34 + 2 * cos(r1)*px*sin(r1)*py - d45 ^ 2 - py ^ 2 * cos(r1) ^ 2 + px ^ 2 * cos(r1) ^ 2 + py ^ 2 - d34 ^ 2)) ^ (1 / 2)) / (2 * d45*cos(r4)*d34 + d34 ^ 2 + d45 ^ 2));
		//cosr3 = Aprox(-(d45 ^ 2 * cos(r4) ^ 2 * cos(r1)*px + d45 ^ 2 * cos(r4) ^ 2 * sin(r1)*py - d45*cos(r4)*(d45 ^ 2 * (-1 + cos(r4) ^ 2)*(-2 * d45*cos(r4)*d34 + 2 * cos(r1)*px*sin(r1)*py - d45 ^ 2 - py ^ 2 * cos(r1) ^ 2 + px ^ 2 * cos(r1) ^ 2 + py ^ 2 - d34 ^ 2)) ^ (1 / 2) - d34*(d45 ^ 2 * (-1 + cos(r4) ^ 2)*(-2 * d45*cos(r4)*d34 + 2 * cos(r1)*px*sin(r1)*py - d45 ^ 2 - py ^ 2 * cos(r1) ^ 2 + px ^ 2 * cos(r1) ^ 2 + py ^ 2 - d34 ^ 2)) ^ (1 / 2) - cos(r1)*px*d45 ^ 2 - sin(r1)*py*d45 ^ 2) / (2 * d45*cos(r4)*d34 + d34 ^ 2 + d45 ^ 2) / d45 / sin(r4));
		//r31(2) = atan2(sinr3, cosr3);
		sinr3 = aprox_all((d34*cosr1*px + d45*cosr4*cosr1*px + d34*sinr1*py + d45*cosr4*sinr1*py - _rsqrt(d45*d45*(-1 + cosr4*cosr4)*(-2 * d45*cosr4*d34 + 2 * cosr1*px*sinr1*py - d45*d45 - py*py*cosr1*cosr1 + px*px*cosr1*cosr1 + py*py - d34*d34))) / (2 * d45*cosr4*d34 + d34*d34 + d45*d45));
		cosr3 = aprox_all(-(d45*d45*cosr4*cosr4*cosr1*px + d45*d45*cosr4*cosr4*sinr1*py - d45*cosr4*_rsqrt(d45*d45*(-1 + cosr4*cosr4)*(-2 * d45*cosr4*d34 + 2 * cosr1*px*sinr1*py - d45*d45 - py*py*cosr1*cosr1 + px*px*cosr1*cosr1 + py*py - d34*d34)) - d34*_rsqrt(d45*d45*(-1 + cosr4*cosr4)*(-2 * d45*cosr4*d34 + 2 * cosr1*px*sinr1*py - d45*d45 - py*py*cosr1*cosr1 + px*px*cosr1*cosr1 + py*py - d34*d34)) - cosr1*px*d45*d45 - sinr1*py*d45*d45) / (2 * d45*cosr4*d34 + d34*d34 + d45*d45) / d45 / sinr4);
		temp[1] = _ratan2(sinr3, cosr3);

		//sinr3 = Aprox((d45 ^ 2 * cos(r4) ^ 2 * sin(r2)*sin(r1)*px + d45 ^ 2 * cos(r4) ^ 2 * cos(r2)*pz - d45 ^ 2 * cos(r4) ^ 2 * cos(r1)*sin(r2)*py + d45*cos(r4)*(d45 ^ 2 * (-1 + cos(r4) ^ 2)*(-2 * d45*cos(r4)*d34 + 2 * sin(r2)*sin(r1)*px*cos(r2)*pz + py ^ 2 * cos(r1) ^ 2 + px ^ 2 * cos(r2) ^ 2 * cos(r1) ^ 2 - px ^ 2 * cos(r2) ^ 2 - 2 * cos(r1)*px*sin(r1)*py - px ^ 2 * cos(r1) ^ 2 - py ^ 2 * cos(r2) ^ 2 * cos(r1) ^ 2 - d45 ^ 2 + px ^ 2 + 2 * cos(r1)*px*sin(r1)*py*cos(r2) ^ 2 - 2 * cos(r1)*sin(r2)*py*cos(r2)*pz - d34 ^ 2 + pz ^ 2 * cos(r2) ^ 2)) ^ (1 / 2) + d34*(d45 ^ 2 * (-1 + cos(r4) ^ 2)*(-2 * d45*cos(r4)*d34 + 2 * sin(r2)*sin(r1)*px*cos(r2)*pz + py ^ 2 * cos(r1) ^ 2 + px ^ 2 * cos(r2) ^ 2 * cos(r1) ^ 2 - px ^ 2 * cos(r2) ^ 2 - 2 * cos(r1)*px*sin(r1)*py - px ^ 2 * cos(r1) ^ 2 - py ^ 2 * cos(r2) ^ 2 * cos(r1) ^ 2 - d45 ^ 2 + px ^ 2 + 2 * cos(r1)*px*sin(r1)*py*cos(r2) ^ 2 - 2 * cos(r1)*sin(r2)*py*cos(r2)*pz - d34 ^ 2 + pz ^ 2 * cos(r2) ^ 2)) ^ (1 / 2) - sin(r2)*sin(r1)*px*d45 ^ 2 + sin(r2)*cos(r1)*py*d45 ^ 2 - cos(r2)*pz*d45 ^ 2) / (2 * d45*cos(r4)*d34 + d34 ^ 2 + d45 ^ 2) / d45 / sin(r4));
		//cosr3 = Aprox((-cos(r1)*d34*sin(r2)*py + d45*cos(r4)*sin(r2)*sin(r1)*px + d34*cos(r2)*pz + d45*cos(r4)*cos(r2)*pz + d34*sin(r2)*sin(r1)*px - cos(r1)*d45*cos(r4)*sin(r2)*py + (d45 ^ 2 * (-1 + cos(r4) ^ 2)*(-2 * d45*cos(r4)*d34 + 2 * sin(r2)*sin(r1)*px*cos(r2)*pz + py ^ 2 * cos(r1) ^ 2 + px ^ 2 * cos(r2) ^ 2 * cos(r1) ^ 2 - px ^ 2 * cos(r2) ^ 2 - 2 * cos(r1)*px*sin(r1)*py - px ^ 2 * cos(r1) ^ 2 - py ^ 2 * cos(r2) ^ 2 * cos(r1) ^ 2 - d45 ^ 2 + px ^ 2 + 2 * cos(r1)*px*sin(r1)*py*cos(r2) ^ 2 - 2 * cos(r1)*sin(r2)*py*cos(r2)*pz - d34 ^ 2 + pz ^ 2 * cos(r2) ^ 2)) ^ (1 / 2)) / (2 * d45*cos(r4)*d34 + d34 ^ 2 + d45 ^ 2));
		//r32(1) = atan2(sinr3, cosr3);
		sinr3 = aprox_all((d45*d45*cosr4*cosr4*sinr2*sinr1*px + d45*d45*cosr4*cosr4*cosr2*pz - d45*d45*cosr4*cosr4*cosr1*sinr2*py + d45*cosr4*_rsqrt(d45*d45*(-1 + cosr4*cosr4)*(-2 * d45*cosr4*d34 + 2 * sinr2*sinr1*px*cosr2*pz + py*py*cosr1*cosr1 + px*px*cosr2*cosr2*cosr1*cosr1 - px*px*cosr2*cosr2 - 2 * cosr1*px*sinr1*py - px*px*cosr1*cosr1 - py*py*cosr2*cosr2*cosr1*cosr1 - d45*d45 + px*px + 2 * cosr1*px*sinr1*py*cosr2*cosr2 - 2 * cosr1*sinr2*py*cosr2*pz - d34*d34 + pz*pz*cosr2*cosr2)) + d34*_rsqrt(d45*d45*(-1 + cosr4*cosr4)*(-2 * d45*cosr4*d34 + 2 * sinr2*sinr1*px*cosr2*pz + py*py*cosr1*cosr1 + px*px*cosr2*cosr2*cosr1*cosr1 - px*px*cosr2*cosr2 - 2 * cosr1*px*sinr1*py - px*px*cosr1*cosr1 - py*py*cosr2*cosr2*cosr1*cosr1 - d45*d45 + px*px + 2 * cosr1*px*sinr1*py*cosr2*cosr2 - 2 * cosr1*sinr2*py*cosr2*pz - d34*d34 + pz*pz*cosr2*cosr2)) - sinr2*sinr1*px*d45*d45 + sinr2*cosr1*py*d45*d45 - cosr2*pz*d45*d45) / (2 * d45*cosr4*d34 + d34*d34 + d45*d45) / d45 / sinr4);
		cosr3 = aprox_all((-cosr1*d34*sinr2*py + d45*cosr4*sinr2*sinr1*px + d34*cosr2*pz + d45*cosr4*cosr2*pz + d34*sinr2*sinr1*px - cosr1*d45*cosr4*sinr2*py + _rsqrt(d45*d45*(-1 + cosr4*cosr4)*(-2 * d45*cosr4*d34 + 2 * sinr2*sinr1*px*cosr2*pz + py*py*cosr1*cosr1 + px*px*cosr2*cosr2*cosr1*cosr1 - px*px*cosr2*cosr2 - 2 * cosr1*px*sinr1*py - px*px*cosr1*cosr1 - py*py*cosr2*cosr2*cosr1*cosr1 - d45*d45 + px*px + 2 * cosr1*px*sinr1*py*cosr2*cosr2 - 2 * cosr1*sinr2*py*cosr2*pz - d34*d34 + pz*pz*cosr2*cosr2))) / (2 * d45*cosr4*d34 + d34*d34 + d45*d45));
		temp[2] = _ratan2(sinr3, cosr3);

		//sinr3 = Aprox((d45 ^ 2 * cos(r4) ^ 2 * sin(r2)*sin(r1)*px + d45 ^ 2 * cos(r4) ^ 2 * cos(r2)*pz - d45 ^ 2 * cos(r4) ^ 2 * cos(r1)*sin(r2)*py - d45*cos(r4)*(d45 ^ 2 * (-1 + cos(r4) ^ 2)*(-2 * d45*cos(r4)*d34 + 2 * sin(r2)*sin(r1)*px*cos(r2)*pz + py ^ 2 * cos(r1) ^ 2 + px ^ 2 * cos(r2) ^ 2 * cos(r1) ^ 2 - px ^ 2 * cos(r2) ^ 2 - 2 * cos(r1)*px*sin(r1)*py - px ^ 2 * cos(r1) ^ 2 - py ^ 2 * cos(r2) ^ 2 * cos(r1) ^ 2 - d45 ^ 2 + px ^ 2 + 2 * cos(r1)*px*sin(r1)*py*cos(r2) ^ 2 - 2 * cos(r1)*sin(r2)*py*cos(r2)*pz - d34 ^ 2 + pz ^ 2 * cos(r2) ^ 2)) ^ (1 / 2) - d34*(d45 ^ 2 * (-1 + cos(r4) ^ 2)*(-2 * d45*cos(r4)*d34 + 2 * sin(r2)*sin(r1)*px*cos(r2)*pz + py ^ 2 * cos(r1) ^ 2 + px ^ 2 * cos(r2) ^ 2 * cos(r1) ^ 2 - px ^ 2 * cos(r2) ^ 2 - 2 * cos(r1)*px*sin(r1)*py - px ^ 2 * cos(r1) ^ 2 - py ^ 2 * cos(r2) ^ 2 * cos(r1) ^ 2 - d45 ^ 2 + px ^ 2 + 2 * cos(r1)*px*sin(r1)*py*cos(r2) ^ 2 - 2 * cos(r1)*sin(r2)*py*cos(r2)*pz - d34 ^ 2 + pz ^ 2 * cos(r2) ^ 2)) ^ (1 / 2) - sin(r2)*sin(r1)*px*d45 ^ 2 + sin(r2)*cos(r1)*py*d45 ^ 2 - cos(r2)*pz*d45 ^ 2) / (2 * d45*cos(r4)*d34 + d34 ^ 2 + d45 ^ 2) / d45 / sin(r4));
		//cosr3 = Aprox((-cos(r1)*d34*sin(r2)*py + d45*cos(r4)*sin(r2)*sin(r1)*px + d34*cos(r2)*pz + d45*cos(r4)*cos(r2)*pz + d34*sin(r2)*sin(r1)*px - cos(r1)*d45*cos(r4)*sin(r2)*py - (d45 ^ 2 * (-1 + cos(r4) ^ 2)*(-2 * d45*cos(r4)*d34 + 2 * sin(r2)*sin(r1)*px*cos(r2)*pz + py ^ 2 * cos(r1) ^ 2 + px ^ 2 * cos(r2) ^ 2 * cos(r1) ^ 2 - px ^ 2 * cos(r2) ^ 2 - 2 * cos(r1)*px*sin(r1)*py - px ^ 2 * cos(r1) ^ 2 - py ^ 2 * cos(r2) ^ 2 * cos(r1) ^ 2 - d45 ^ 2 + px ^ 2 + 2 * cos(r1)*px*sin(r1)*py*cos(r2) ^ 2 - 2 * cos(r1)*sin(r2)*py*cos(r2)*pz - d34 ^ 2 + pz ^ 2 * cos(r2) ^ 2)) ^ (1 / 2)) / (2 * d45*cos(r4)*d34 + d34 ^ 2 + d45 ^ 2));
		//r32(2) = atan2(sinr3, cosr3);
		sinr3 = aprox_all((d45*d45*cosr4*cosr4*sinr2*sinr1*px + d45*d45*cosr4*cosr4*cosr2*pz - d45*d45*cosr4*cosr4*cosr1*sinr2*py - d45*cosr4*_rsqrt(d45*d45*(-1 + cosr4*cosr4)*(-2 * d45*cosr4*d34 + 2 * sinr2*sinr1*px*cosr2*pz + py*py*cosr1*cosr1 + px*px*cosr2*cosr2*cosr1*cosr1 - px*px*cosr2*cosr2 - 2 * cosr1*px*sinr1*py - px*px*cosr1*cosr1 - py*py*cosr2*cosr2*cosr1*cosr1 - d45*d45 + px*px + 2 * cosr1*px*sinr1*py*cosr2*cosr2 - 2 * cosr1*sinr2*py*cosr2*pz - d34*d34 + pz*pz*cosr2*cosr2)) - d34*_rsqrt(d45*d45*(-1 + cosr4*cosr4)*(-2 * d45*cosr4*d34 + 2 * sinr2*sinr1*px*cosr2*pz + py*py*cosr1*cosr1 + px*px*cosr2*cosr2*cosr1*cosr1 - px*px*cosr2*cosr2 - 2 * cosr1*px*sinr1*py - px*px*cosr1*cosr1 - py*py*cosr2*cosr2*cosr1*cosr1 - d45*d45 + px*px + 2 * cosr1*px*sinr1*py*cosr2*cosr2 - 2 * cosr1*sinr2*py*cosr2*pz - d34*d34 + pz*pz*cosr2*cosr2)) - sinr2*sinr1*px*d45*d45 + sinr2*cosr1*py*d45*d45 - cosr2*pz*d45*d45) / (2 * d45*cosr4*d34 + d34*d34 + d45*d45) / d45 / sinr4);
		cosr3 = aprox_all((-cosr1*d34*sinr2*py + d45*cosr4*sinr2*sinr1*px + d34*cosr2*pz + d45*cosr4*cosr2*pz + d34*sinr2*sinr1*px - cosr1*d45*cosr4*sinr2*py - _rsqrt(d45*d45*(-1 + cosr4*cosr4)*(-2 * d45*cosr4*d34 + 2 * sinr2*sinr1*px*cosr2*pz + py*py*cosr1*cosr1 + px*px*cosr2*cosr2*cosr1*cosr1 - px*px*cosr2*cosr2 - 2 * cosr1*px*sinr1*py - px*px*cosr1*cosr1 - py*py*cosr2*cosr2*cosr1*cosr1 - d45*d45 + px*px + 2 * cosr1*px*sinr1*py*cosr2*cosr2 - 2 * cosr1*sinr2*py*cosr2*pz - d34*d34 + pz*pz*cosr2*cosr2))) / (2 * d45*cosr4*d34 + d34*d34 + d45*d45));
		temp[3] = _ratan2(sinr3, cosr3);

		if (_rabs(temp[0] - temp[2]) < MIN_ERROR || _rabs(temp[0] - temp[3]) < MIN_ERROR)
		{
			angle[2][i1] = temp[0];
		}
		else
		{
			angle[2][i1] = temp[1];
		}
	}

	//[ Get joint 5 ]
	for (i1 = 0; i1 < 8; i1++)
	{
		r1 = angle[0][i1]; sinr1 = _rsin(r1); cosr1 = _rcos(r1);
		r2 = angle[1][i1]; sinr2 = _rsin(r2); cosr2 = _rcos(r2);
		r3 = angle[2][i1]; sinr3 = _rsin(r3); cosr3 = _rcos(r3);
		r4 = angle[3][i1]; sinr4 = _rsin(r4); cosr4 = _rcos(r4);
		//sinr345
		temp[0] = aprox_all(-sinr2*sinr1*nx + sinr2*cosr1*ny - cosr2*nz);
		//cosr345
		temp[1] = aprox_all(cosr1*nx + sinr1*ny);
		temp[3] = _ratan2(temp[0], temp[1]);
		angle[4][i1] = temp[3] - r3 - r4;
		if (angle[4][i1] > _pi)
			angle[4][i1] -= 2.0*_pi;
		else if (angle[4][i1] < -_pi)
			angle[4][i1] += 2.0*_pi;
	}

	//[ Get joint 6 ]
	for (i1 = 0; i1 < 8; i1++)
	{
		r1 = angle[0][i1]; sinr1 = _rsin(r1); cosr1 = _rcos(r1);
		r2 = angle[1][i1]; sinr2 = _rsin(r2); cosr2 = _rcos(r2);
		r3 = angle[2][i1]; sinr3 = _rsin(r3); cosr3 = _rcos(r3);
		r4 = angle[3][i1]; sinr4 = _rsin(r4); cosr4 = _rcos(r4);
		r5 = angle[4][i1]; sinr5 = _rsin(r5); cosr5 = _rcos(r5);
		sinr6 = 1.0 / 2.0*ox*_rsin(r5 + r4 + r1 + r3) - 1.0 / 2.0*ox*_rsin(-r5 - r4 + r1 - r3) + 1.0 / 4.0*ox*_rcos(-r5 - r4 - r3 + r1 - r2) - 1.0 / 4.0*ox*_rcos(r5 + r4 + r3 + r1 + r2) + 1.0 / 4.0*ox*_rcos(r5 + r4 + r3 + r1 - r2) - 1.0 / 4.0*ox*_rcos(-r5 - r4 - r3 + r1 + r2) + 1.0 / 2.0*oy*_rcos(-r5 - r4 + r1 - r3) - 1.0 / 2.0*oy*_rcos(r5 + r4 + r1 + r3) - 1.0 / 4.0*oy*_rsin(r5 + r4 + r3 + r1 + r2) + 1.0 / 4.0*oy*_rsin(-r5 - r4 - r3 + r1 - r2) - 1.0 / 4.0*oy*_rsin(-r5 - r4 - r3 + r1 + r2) + 1.0 / 4.0*oy*_rsin(r5 + r4 + r3 + r1 - r2) + 1.0 / 2.0*oz*_rcos(-r5 - r4 + r2 - r3) + 1.0 / 2.0*oz*_rcos(r5 + r4 + r2 + r3);
		cosr6 = 1.0 / 2.0*ax*_rsin(r5 + r4 + r1 + r3) - 1.0 / 2.0*ax*_rsin(-r5 - r4 + r1 - r3) + 1.0 / 4.0*ax*_rcos(r1 - r2 - r3 - r5 - r4) - 1.0 / 4.0*ax*_rcos(r1 + r2 + r3 + r5 + r4) + 1.0 / 4.0*ax*_rcos(r1 - r2 + r3 + r5 + r4) - 1.0 / 4.0*ax*_rcos(r1 + r2 - r3 - r5 - r4) + 1.0 / 2.0*ay*_rcos(-r5 - r4 + r1 - r3) - 1.0 / 2.0*ay*_rcos(r5 + r4 + r1 + r3) - 1.0 / 4.0*ay*_rsin(r1 + r2 + r3 + r5 + r4) + 1.0 / 4.0*ay*_rsin(r1 - r2 - r3 - r5 - r4) - 1.0 / 4.0*ay*_rsin(r1 + r2 - r3 - r5 - r4) + 1.0 / 4.0*ay*_rsin(r1 - r2 + r3 + r5 + r4) + 1.0 / 2.0*az*_rcos(r2 - r3 - r5 - r4) + 1.0 / 2.0*az*_rcos(r2 + r3 + r5 + r4); sinr6 = aprox_all(sinr6);
		sinr6 = aprox_all(sinr6);
		cosr6 = aprox_all(cosr6);
		angle[5][i1] = _ratan2(sinr6, cosr6);
	}
	//Select fit one 
	i1 = res_choose(&angle[0][0], ref, 6, 8);



	if (i1 == -1)
	{

		/**
		//test
		for (i1 = 0; i1 < 8; i1++)
		{
		for (i2 = 0; i2 < 6; i2++)
		{
		printf("%.3f\t", rad2deg(angle[i2][i1]));
		}
		printf("\n");
		}/**/
		return IK_FAILED;
	}
	else
	{
		for (i2 = 0; i2 < 6; i2++)
		{
			ang[i2] = angle[i2][i1];
		}
		/**
		//test
		for (i1 = 0; i1 < 8; i1++)
		{
		for (i2 = 0; i2 < 6; i2++)
		{
		printf("%.1f\t", rad2deg(angle[i2][i1]));
		}
		printf("\n");
		}
		/**
		for (i1 = 0; i1 < 8; i1++)
		{
		for (i2 = 0; i2 < 6; i2++)
		{
		test_angle[i2] = angle[i2][i1];
		}
		test_T = mt_mul(get_foot_t(test_angle),get_move(0,0,FOOT_HEIGHT));
		printf("\n");
		for (ti1 = 0; ti1 < 4; ti1++)
		{
		for (ti2 = 0; ti2 < 4; ti2++)
		{
		printf("%.3f\t",test_T.t[ti1][ti2]-T[ti1][ti2]);
		}
		printf("\n");
		}
		printf("\n");
		}

		/**/
		return IK_SUCCEEDED;
	}
}

//膝盖逆运动学求解
_rrm _rf_flg ik_knee(const _rv_flg xz_flg, const _rv_t hip_pos[6], const _rv_t patella_pos[6], const _rv_t foot_pos[6], _rv_t ang[6], const _rv_t ref[6])
{
	int i;
	_rv_t L_T, xh, zh, xp, zp, lp, rp, rk, rb, rf, xk, zk, L_TX, L_TZ;
	_rv_t Alpha, Beta;

	L_T = THIGH_LEN;
	xh = hip_pos[3];
	zh = hip_pos[5];
	xp = patella_pos[3];
	zp = patella_pos[5];
	lp = KNEE_HEIGHT;
	rp = patella_pos[1];
	rk = _rabs(deg2rad(KNEE_ROTATION));
	rb = hip_pos[1];
	rf = foot_pos[1];

	xk = xp + lp*_rsin(rp);
	zk = zp + lp*_rcos(rp);
	L_TX = xk - xh;
	L_TZ = zh - zk;

	for (i = 0; i < 6; i++)
		ang[i] = ref[i];
	if (xz_flg == KNEE_Z)
	{
		if (aprox_all(L_TZ / L_T) <= 1.0)
		{
			Alpha = _rasin(aprox_all(L_TZ / L_T));
			Beta = deg2rad(90.0) - rp - rk;

			ang[2] = Alpha - rb - deg2rad(90.0);
			ang[3] = _pi - (Alpha + Beta);
			ang[4] = rf + Beta - deg2rad(90.0);
			return IK_SUCCEEDED;
		}
		else
			return IK_FAILED;
	}
	else
	{
		if (aprox_all(L_TX / L_T) <= 1.0)
		{
			Alpha = _racos(aprox_all(L_TX / L_T));
			Beta = deg2rad(90.0) - rp - rk;

			ang[2] = Alpha - rb - deg2rad(90.0);
			ang[3] = _pi - (Alpha + Beta);
			ang[4] = rf + Beta - deg2rad(90.0);
			return IK_SUCCEEDED;
		}
		else
			return IK_FAILED;
	}
}
#endif
/*<----------------逆运动学计算函数------------------>*/


/*-----------------运动学计算函数-------------------*/
#ifdef ROBOT_K_FUNCTION

TStruct get_t(const PostureStruct *posture)
{
	TStruct res = UNIT_T;
	/**
	[cos(ry)*cos(rz), cos(rz)*sin(rx)*sin(ry) - cos(rx)*sin(rz), sin(rx)*sin(rz) + cos(rx)*cos(rz)*sin(ry), px]
	[cos(ry)*sin(rz), cos(rx)*cos(rz) + sin(rx)*sin(ry)*sin(rz), cos(rx)*sin(ry)*sin(rz) - cos(rz)*sin(rx), py]
	[-sin(ry), cos(ry)*sin(rx), cos(rx)*cos(ry), pz]
	[0, 0, 0, 1]
	/**/
	_rv_t sinrx, cosrx, sinry, cosry, sinrz, cosrz;
	sinrx = _rsin(posture->p[0]); cosrx = _rcos(posture->p[0]);
	sinry = _rsin(posture->p[1]); cosry = _rcos(posture->p[1]);
	sinrz = _rsin(posture->p[2]); cosrz = _rcos(posture->p[2]);

	res.t[_nx] = cosry*cosrz;
	res.t[_ox] = cosrz*sinrx*sinry - cosrx*sinrz;
	res.t[_ax] = sinrx*sinrz + cosrx*cosrz*sinry;
	res.t[_px] = posture->p[3];
	res.t[_ny] = cosry*sinrz;
	res.t[_oy] = cosrx*cosrz + sinrx*sinry*sinrz;
	res.t[_ay] = cosrx*sinry*sinrz - cosrz*sinrx;
	res.t[_py] = posture->p[4];
	res.t[_nz] = -sinry;
	res.t[_oz] = cosry*sinrx;
	res.t[_az] = cosrx*cosry;
	res.t[_pz] = posture->p[5];

	return res;
}

TStruct get_move(_rv_t dx, _rv_t dy, _rv_t dz)
{
	TStruct res = UNIT_T;

	res.t[_px] = dx;
	res.t[_py] = dy;
	res.t[_pz] = dz;
	return res;
}

TStruct get_rot(_rv_flg axis, _rv_t angle)
{
	TStruct res = UNIT_T;
	_rv_t sinv, cosv;
	sinv = _rsin(angle);
	cosv = _rcos(angle);
	switch (axis)
	{
	case AXIS_X:
		res.t[_oy] = cosv;	res.t[_ay] = -sinv;
		res.t[_oz] = sinv;	res.t[_az] = cosv;
		break;
	case AXIS_Y:
		res.t[_nx] = cosv;	res.t[_ax] = sinv;
		res.t[_nz] = -sinv;	res.t[_az] = cosv;
		break;
	case AXIS_Z:
		res.t[_nx] = cosv;	res.t[_ox] = -sinv;
		res.t[_ny] = sinv;	res.t[_oy] = cosv;
		break;
	}
	return res;
}

//腰部运动学
TStruct get_waist_t(_rv_t *waist_joint)
{
	//base: float base
	PostureStruct p = INI_POS;

	p.p[2] = *waist_joint;
	p.p[1] = *(waist_joint + 1);
	p.p[0] = *(waist_joint + 2);
	p.p[5] = WAIST_BASE_Z;
	return get_t(&p);
}

//手部运动学
TStruct get_arm_t(_rv_t *arm_joint)
{
	TStruct T1, T2, T3, T4, TEMP;
	T1 = get_rot(AXIS_Y, *arm_joint);
	T2 = get_rot(AXIS_X, *(arm_joint + 1));
	T3 = get_move(0, 0, -UPPER_ARM_LEN);
	TEMP = get_rot(AXIS_Y, *(arm_joint + 2));
	T3 = t_mul(&T3, &TEMP);
	T4 = get_move(0, 0, -LOWER_ARM_LEN);
	//TEMP = t_mul(&T1, &T2);
	//TEMP = t_mul(&TEMP, &T3);
	//TEMP = t_mul(&TEMP, &T4);
	return mt_mul(mt_mul(mt_mul(T1, T2), T3), T4);
}

//膝部运动学
TStruct get_knee_t(_rv_t *leg_joint)
{
	TStruct T1, T2, T3, T4, T5;
	T1 = get_rot(AXIS_Z, *leg_joint);
	T2 = get_rot(AXIS_X, *(leg_joint + 1));
	T3 = mt_mul(get_rot(AXIS_Y, *(leg_joint + 2)), get_move(0, 0, -CRUS_LEN));
	T4 = get_rot(AXIS_Y, *(leg_joint + 3));
	T5 = mt_mul(get_rot(AXIS_Y, deg2rad(KNEE_ROTATION)), get_move(0, 0, -KNEE_HEIGHT));
	return mt_mul(mt_mul(mt_mul(mt_mul(T1, T2), T3), T4), T5);
}

//脚部运动学
TStruct get_foot_t(_rv_t *leg_joint)
{
	TStruct T1, T2, T3, T4, T5, T6;
	T1 = get_rot(AXIS_Z, *leg_joint);
	T2 = get_rot(AXIS_X, *(leg_joint + 1));
	T3 = get_rot(AXIS_Y, *(leg_joint + 2));
	T4 = mt_mul(get_move(0, 0, -CRUS_LEN), get_rot(AXIS_Y, *(leg_joint + 3)));
	T5 = mt_mul(get_move(0, 0, -THIGH_LEN), get_rot(AXIS_Y, *(leg_joint + 4)));
	T6 = mt_mul(get_rot(AXIS_X, *(leg_joint + 5)), get_move(0, 0, -FOOT_HEIGHT));
	return mt_mul(mt_mul(mt_mul(mt_mul(mt_mul(T1, T2), T3), T4), T5), T6);
}
#endif
/*<----------------运动学计算函数------------------>*/


/*-------------------运动规划函数-------------------*/
#ifdef ROBOT_PLAN_FUNCTION
_rrm _rf_flg robot_update(BRobot * robot)
{
	//时间累加
	robot->CurrentState.Time += robot->Squat.inter_time;

	robot->PastState = robot->CurrentState;
	update_current(robot);
	get_next(robot);
	return 0;
}

_rf_flg update_current(BRobot *robot)
{
	BRobotState *p, *pg;
	int i;
	p = &(robot->CurrentState);
	pg = &(robot->GoalState);
	if (p->Posture[WAIST].IKC_FLAG != HAD_SET) { p->Posture[WAIST].T = mt_mul(p->Posture[BASE].T, get_waist_t(&(p->Joint[WAIST_1]))); }
	else { p->Posture[WAIST].T = get_t(&(p->Posture[WAIST].Posture)); }

	p->Posture[LEFT_SHOULDER].T = mt_mul(p->Posture[WAIST].T, get_move(-NECK_WAIST_X, SHOULDER_WIDTH / 2.0, NECK_WAIST_Z));
	if (pg->Posture[LEFT_HAND].IKC_FLAG != HAD_SET) { p->Posture[LEFT_HAND].T = mt_mul(p->Posture[LEFT_SHOULDER].T, get_arm_t(&(p->Joint[LEFT_ARM_1]))); }
	else { p->Posture[LEFT_HAND].T = get_t(&(p->Posture[LEFT_HAND].Posture)); }
	p->Posture[LEFT_HIP].T = mt_mul(p->Posture[BASE].T, get_move(-BASE_HIP_X, BASE_HIP_Y, -BASE_HIP_Z));
	if (pg->Posture[LEFT_KNEE].IKC_FLAG != HAD_SET) { p->Posture[LEFT_KNEE].T = mt_mul(p->Posture[LEFT_HIP].T, get_knee_t(&(p->Joint[LEFT_LEG_1]))); }
	else { p->Posture[LEFT_KNEE].T = get_t(&(p->Posture[LEFT_KNEE].Posture)); }
	if (pg->Posture[LEFT_FOOT].IKC_FLAG != HAD_SET) { p->Posture[LEFT_FOOT].T = mt_mul(p->Posture[LEFT_HIP].T, get_foot_t(&(p->Joint[LEFT_LEG_1]))); }
	else { p->Posture[LEFT_FOOT].T = get_t(&(p->Posture[LEFT_FOOT].Posture)); }
	//p->Posture[LEFT_ANKLE].T = mt_mul(p->Posture[LEFT_HIP].T, get_move(0.0,0.0,FOOT_HEIGHT));
	p->Posture[LEFT_SENSOR].T = mt_mul(p->Posture[LEFT_FOOT].T, get_move(0.0, 0.0, FOOT_SENSOR));

	p->Posture[RIGHT_SHOULDER].T = mt_mul(p->Posture[WAIST].T, get_move(-NECK_WAIST_X, -SHOULDER_WIDTH / 2.0, NECK_WAIST_Z));
	if (pg->Posture[RIGHT_HAND].IKC_FLAG != HAD_SET) { p->Posture[RIGHT_HAND].T = mt_mul(p->Posture[RIGHT_SHOULDER].T, get_arm_t(&(p->Joint[RIGHT_ARM_1]))); }
	else { p->Posture[RIGHT_HAND].T = get_t(&(p->Posture[RIGHT_HAND].Posture)); }
	p->Posture[RIGHT_HIP].T = mt_mul(p->Posture[BASE].T, get_move(-BASE_HIP_X, -BASE_HIP_Y, -BASE_HIP_Z));
	if (pg->Posture[RIGHT_KNEE].IKC_FLAG != HAD_SET) { p->Posture[RIGHT_KNEE].T = mt_mul(p->Posture[RIGHT_HIP].T, get_knee_t(&(p->Joint[RIGHT_LEG_1]))); }
	else { p->Posture[RIGHT_KNEE].T = get_t(&(p->Posture[RIGHT_KNEE].Posture)); }
	if (pg->Posture[RIGHT_FOOT].IKC_FLAG != HAD_SET) { p->Posture[RIGHT_FOOT].T = mt_mul(p->Posture[RIGHT_HIP].T, get_foot_t(&(p->Joint[RIGHT_LEG_1]))); }
	else { p->Posture[RIGHT_FOOT].T = get_t(&(p->Posture[RIGHT_FOOT].Posture)); }
	//p->Posture[RIGHT_ANKLE].T = mt_mul(p->Posture[RIGHT_HIP].T, get_move(0.0, 0.0, FOOT_HEIGHT));
	p->Posture[RIGHT_SENSOR].T = mt_mul(p->Posture[RIGHT_FOOT].T, get_move(0.0, 0.0, FOOT_SENSOR));

	p->Posture[COG].T = mt_mul(p->Posture[BASE].T, get_move(-BASE_HIP_X,0.0,0.0));

	for (i = 1; i < SPECIAL_MOTION; i++)
	{
		//if (i >= SPECIAL_MOTION)
		//	continue;
		ik_posture(p->Posture[i].T.t, p->Posture[i].Posture.p, p->Posture[i].Posture.p);
	}
	return 0;
}

_rf_flg get_next(BRobot *robot)
{
	//目标设置
	if (robot->GoalState.SetFlag == NOT_SET) 
	{
		set_goal(robot);
		get_int_pol_fac(robot);
		robot->GoalState.SetFlag = HAD_SET;
	}

	if (robot->MotionFlag == CARTESIAN_MOVE)
	{
		//获取下一时刻位姿
		get_next_postrure(robot);
		//姿态调整
		posture_adjust(robot);
	}
	//逆运动学计算求关节角度
	get_joint(robot);
	//状态判断与转换
	state_trans(robot);

	return 0;
}


//获取插值系数
_rf_flg get_int_pol_fac(BRobot * robot)
{
	int i, j;
	BRobotState * pc, *pg, *pmg;
	pc = &(robot->CurrentState);
	pg = &(robot->GoalState);
	pmg = &(robot->MidGoalState);
	if (robot->MotionFlag == CARTESIAN_MOVE)
	{
		for (i = 0; i < PART_NUM; i++)
		{
			for (j = 0; j < 6; j++)
			{
				if (pmg->Posture[i].MID_IKC_FLAG[j] != HAD_SET)
				{
					get5_ipf_va0(pg->Time - pc->Time, pc->Posture[i].Posture.p[j], pg->Posture[i].Posture.p[j], &(pc->Posture[i].IntPolFac[j][0]));
				}
				else
				{
					//mexPrintf("%d\n", j);
					get5_ipf_va0(pmg->Time - pc->Time, pc->Posture[i].Posture.p[j], pmg->Posture[i].Posture.p[j], &(pmg->Posture[i].IntPolFac[j][0]));
					get5_ipf_va0(pg->Time - pmg->Time, pmg->Posture[i].Posture.p[j], pg->Posture[i].Posture.p[j], &(pg->Posture[i].IntPolFac[j][0]));
				}
			}
		}
	}
	else if (robot->MotionFlag == JOINT_MOVE)
	{
		for (i = 0; i < JOINT_NUMBER; i++)
		{
			get5_ipf_va0(pg->Time - pc->Time, pc->Joint[i], pg->Joint[i], robot->JointIntPolFac[i]);
		}
	}
	return 0;
}

//插值计算下一时刻位姿
_rf_flg get_next_postrure(BRobot * robot)
{
	_rv_t t;
	int i, j, k;
	BRobotState *over_state;

	//t = robot->Squat.StepTime[robot->CurrentState.StateFlag - 1] - robot->GoalState.Time + robot->CurrentState.Time;
	for (i = 0; i < PART_NUM; i++)
	{
		for (j = 0; j < 6; j++)
		{
			if (robot->MidGoalState.Posture[i].MID_IKC_FLAG[j] != HAD_SET)
			{
				t = robot->Squat.StepTime[robot->CurrentState.StateFlag - 1] - robot->GoalState.Time + robot->CurrentState.Time;
				robot->CurrentState.Posture[i].Posture.p[j] = 0;
				for (k = 0; k < 6; k++)
				{
					robot->CurrentState.Posture[i].Posture.p[j] += robot->CurrentState.Posture[i].IntPolFac[j][k] * _rpow(t, (_rv_t)k);
				}

				//if (i == LEFT_FOOT)
				//{
				//mexPrintf("%d--%f ",j,t);
				//}

			}
			else
			{
				//mexPrintf("%d %d\n", j, robot->MidGoalState.Posture[i].MID_IKC_FLAG[j]);
				if (robot->CurrentState.Time <= robot->MidGoalState.Time)
				{
					over_state = &(robot->MidGoalState);
					//t = 0.5*robot->Squat.StepTime[robot->CurrentState.StateFlag - 1] - over_state->Time + robot->CurrentState.Time;
				}
				else
				{
					over_state = &(robot->GoalState);
					//t = robot->Squat.StepTime[robot->CurrentState.StateFlag - 1] - over_state->Time + robot->CurrentState.Time;
				}
				t = 0.5*robot->Squat.StepTime[robot->CurrentState.StateFlag - 1] - over_state->Time + robot->CurrentState.Time;
				//mexPrintf("%f %f %f\n", t, robot->CurrentState.Time, robot->MidGoalState.Time);
				robot->CurrentState.Posture[i].Posture.p[j] = 0;
				for (k = 0; k < 6; k++)
				{
					robot->CurrentState.Posture[i].Posture.p[j] += over_state->Posture[i].IntPolFac[j][k] * _rpow(t, (_rv_t)k);
				}
			}

			if (robot->CurrentState.Time >= robot->GoalState.Time)
			{
				robot->CurrentState.Posture[i].Posture.p[j] = robot->GoalState.Posture[i].Posture.p[j];
			}
		}
		robot->CurrentState.Posture[i].T = get_t(&(robot->CurrentState.Posture[i].Posture));
	}
	return 0;
}



//逆运动学求解关节角度
_rf_flg get_joint(BRobot * robot)
{
	//Init set
	BRobotState * pg, *pc;
	PostureStruct	temp_Posture = INI_POS;
	TStruct temp_T = UNIT_T;
	_rv_t t;
	int i, k;

	pg = &(robot->GoalState);
	pc = &(robot->CurrentState);

	if (robot->MotionFlag == CARTESIAN_MOVE)
	{//Left Knee Joint
		if (pg->Posture[LEFT_KNEE].IKC_FLAG == HAD_SET)
		{
			ik_knee(robot->Squat.KneeXZ, pc->Posture[LEFT_HIP].Posture.p, pc->Posture[LEFT_KNEE].Posture.p, pc->Posture[LEFT_FOOT].Posture.p, &(pc->Joint[LEFT_LEG_1]), &(pc->Joint[LEFT_LEG_1]));
		}
		//Right Knee Joint
		if (pg->Posture[RIGHT_KNEE].IKC_FLAG == HAD_SET)
		{
			ik_knee(robot->Squat.KneeXZ, pc->Posture[RIGHT_HIP].Posture.p, pc->Posture[RIGHT_KNEE].Posture.p, pc->Posture[RIGHT_FOOT].Posture.p, &pc->Joint[RIGHT_LEG_1], &pc->Joint[RIGHT_LEG_1]);
		}
		//Waist Joint
		if (pg->Posture[WAIST].IKC_FLAG == HAD_SET)
		{
			temp_T = mt_mul(t_inv(&(pc->Posture[BASE].T)), pc->Posture[WAIST].T);
			ik_waist(temp_T.t, &(pc->Joint[WAIST_1]), &(pc->Joint[WAIST_1]));
		}
		//Left Arm Joint
		if (pg->Posture[LEFT_HAND].IKC_FLAG == HAD_SET)
		{
			temp_T = mt_mul(t_inv(&(pc->Posture[LEFT_SHOULDER].T)), pc->Posture[LEFT_HAND].T);
			ik_arm(temp_T.t, &(pc->Joint[LEFT_ARM_1]), &(pc->Joint[LEFT_ARM_1]));
		}
		//Right Arm Joint
		if (pg->Posture[RIGHT_HAND].IKC_FLAG == HAD_SET)
		{
			temp_T = mt_mul(t_inv(&(pc->Posture[RIGHT_SHOULDER].T)), pc->Posture[RIGHT_HAND].T);
			ik_arm(temp_T.t, &(pc->Joint[RIGHT_ARM_1]), &(pc->Joint[RIGHT_ARM_1]));
		}
		//Left Leg Joint
		if (pg->Posture[LEFT_FOOT].IKC_FLAG == HAD_SET)
		{
			for (i = 0; i < 3; i++)
			{
				temp_Posture.p[i] = pc->Posture[LEFT_FOOT].Posture.p[i];
			}
			for (i = 0; i < 6; i++)
			{
				temp_Posture.p[i] = pc->Posture[LEFT_FOOT].Posture.p[i] + robot->VMC6.PostureAdjust[_L_LEFT].p[i];
			}

			temp_T = pc->Posture[LEFT_HIP].T;
			temp_T = t_inv(&temp_T);
			//temp_T = mt_mul(temp_T, mt_mul(pc->Posture[LEFT_FOOT].T, get_move(0, 0, FOOT_HEIGHT)));
			temp_T = mt_mul(temp_T, mt_mul(get_t(&temp_Posture), get_move(0, 0, FOOT_HEIGHT)));
			ik_leg(temp_T.t, &(pc->Joint[LEFT_LEG_1]), &(pc->Joint[LEFT_LEG_1]));
			//pc->Joint[LEFT_LEG_5] += robot->VMC6.PostureAdjust[_L_LEFT].p[1];
			//pc->Joint[LEFT_LEG_6] += robot->VMC6.PostureAdjust[_L_LEFT].p[0];
		}
		//Right Leg Joint
		if (pg->Posture[RIGHT_FOOT].IKC_FLAG == HAD_SET)
		{
			for (i = 0; i < 3; i++)
			{
				temp_Posture.p[i] = pc->Posture[RIGHT_FOOT].Posture.p[i];
			}
			for (i = 0; i < 6; i++)
			{
				temp_Posture.p[i] = pc->Posture[RIGHT_FOOT].Posture.p[i] + robot->VMC6.PostureAdjust[_L_RIGHT].p[i];
			}
			temp_T = pc->Posture[RIGHT_HIP].T;
			temp_T = t_inv(&temp_T);
			//temp_T = mt_mul(temp_T, mt_mul(pc->Posture[RIGHT_FOOT].T, get_move(0, 0, FOOT_HEIGHT)));
			temp_T = mt_mul(temp_T, mt_mul(get_t(&temp_Posture), get_move(0, 0, FOOT_HEIGHT)));

			ik_leg(temp_T.t, &(pc->Joint[RIGHT_LEG_1]), &(pc->Joint[RIGHT_LEG_1]));

			//pc->Joint[RIGHT_LEG_5] += robot->VMC6.PostureAdjust[_L_RIGHT].p[1];
			//pc->Joint[RIGHT_LEG_6] += robot->VMC6.PostureAdjust[_L_RIGHT].p[0];
		}
		
		for (i = 0; i < JOINT_NUMBER; i++)
		{
			pc->JointV[i] = (pc->Joint[i] - robot->PastState.Joint[i]) / (robot->Squat.inter_time / 1000.0);
			pc->JointA[i] = (pc->JointV[i] - robot->PastState.JointV[i]) / (robot->Squat.inter_time / 1000.0);
		}
	}
	else if (robot->MotionFlag == JOINT_MOVE)
	{
		t = robot->Squat.StepTime[robot->CurrentState.StateFlag - 1] - robot->GoalState.Time + robot->CurrentState.Time;

		for (i = 0; i < JOINT_NUMBER; i++)
		{
			robot->CurrentState.Joint[i] = 0.0;
			for (k = 0; k < 6; k++)
			{
				robot->CurrentState.Joint[i] += robot->JointIntPolFac[i][k] * _rpow(t, (_rv_t)k);
			}

			if (robot->CurrentState.Time >= robot->GoalState.Time)
			{
				robot->CurrentState.Joint[i] = robot->GoalState.Joint[i];
			}
		}
	}
	else if (robot->MotionFlag == THIRD_MOVE)
	{
		//Left Arm Joint
		if (pg->Posture[LEFT_HAND].IKC_FLAG == HAD_SET)
		{
			temp_T = mt_mul(t_inv(&(pc->Posture[LEFT_SHOULDER].T)), pc->Posture[LEFT_HAND].T);
			//MStruct m = get_ts_from_t(pc->Posture[LEFT_HAND].T);
			//show_matrix(&m);
			//printf("%f,%f,%f,", pc->Joint[LEFT_ARM_1], pc->Joint[LEFT_ARM_2], pc->Joint[LEFT_ARM_3]);
			ik_arm(temp_T.t, &(pc->Joint[LEFT_ARM_1]), &(pc->Joint[LEFT_ARM_1]));
			//printf("%f,%f,%f\n", pc->Joint[LEFT_ARM_1], pc->Joint[LEFT_ARM_2], pc->Joint[LEFT_ARM_3]);
		}
		//Right Arm Joint
		if (pg->Posture[RIGHT_HAND].IKC_FLAG == HAD_SET)
		{
			temp_T = mt_mul(t_inv(&(pc->Posture[RIGHT_SHOULDER].T)), pc->Posture[RIGHT_HAND].T);
			ik_arm(temp_T.t, &(pc->Joint[RIGHT_ARM_1]), &(pc->Joint[RIGHT_ARM_1]));
		}
	}
	return 0;
}
//状态判断及转换
_rf_flg state_trans(BRobot * robot)
{
	int i;
	if (robot->CurrentState.Time >= robot->GoalState.Time)
	{
		for (i = 0; i < PART_NUM; i++)
		{
			robot->CurrentState.Posture[i] = robot->GoalState.Posture[i];
		}
		//for (i = 0; i < JOINT_NUMBER; i++)
		//{
		//robot->CurrentState.Joint[i] = robot->GoalState.Joint[i];
		//}
		robot->CurrentState.StateFlag = get_next_state(robot);
		if (robot->CurrentState.StateFlag > robot->Squat.StepNum)
			robot->CurrentState.StateFlag = 0;
		robot->GoalState.SetFlag = NOT_SET;
		robot->Squat.SpecialMode = NORMAL_MOVE;
	}
	return 0;
}

_rf_flg get_body_com(BRobot * robot)
{
	static int flg = 0;
	BRobotState *pc;
	BRobotState *pp;
	double sum_mx, sum_m;
	double joint[2][6] = { 0.0 };
	static double last_joint[2][6] = { 0.0 };
	double trunk_mass = 30.08;
	double leg_mass[3] = { 1.438 + 1.055 + 2.639,2.966,0.7240 + 0.5204 + 0.4320 + 0.0864 };
	int i,j,k;
	double trunk_com[3];
	double leg_com[2][3][3];
	double hip_base_y[2] = { BASE_HIP_Y,-BASE_HIP_Y };
	TStruct temp_T = UNIT_T;

	pc = &(robot->CurrentState);
	pp = &(robot->PastState);
	
	if (flg == 0) 
	{
		for (i = 0; i < 6; i++)
		{
			last_joint[0][i] = *(&(pc->Joint[LEFT_LEG_1]) + i);
			last_joint[1][i] = *(&(pc->Joint[RIGHT_LEG_1]) + i);
		}
		flg = 1;
	}

	//左
	temp_T = mt_mul((pc->Posture[BASE].T), get_move(-BASE_HIP_X, BASE_HIP_Y, -BASE_HIP_Z));
	temp_T = mt_mul(t_inv(&temp_T), mt_mul(pc->Posture[LEFT_FOOT].T,get_move(0,0,FOOT_HEIGHT)));
	ik_leg(temp_T.t, joint[0],last_joint[0]);
	//右
	temp_T = mt_mul((pc->Posture[BASE].T), get_move(-BASE_HIP_X, -BASE_HIP_Y, -BASE_HIP_Z));
	temp_T = mt_mul(t_inv(&temp_T), mt_mul(pc->Posture[RIGHT_FOOT].T, get_move(0, 0, FOOT_HEIGHT)));
	ik_leg(temp_T.t, joint[1], last_joint[1]);

	
	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < 6; j++)
		{
			last_joint[i][j] = joint[i][j];
		}
	}
	
	for (i = 0; i < JOINT_NUMBER; i++)
	{
		pc->O_Joint[i] = pc->Joint[i];
		if (i >= LEFT_LEG_1&&i <= LEFT_LEG_6)
		{
			pc->O_Joint[i] = joint[0][i - LEFT_LEG_1];
		}
		if (i >= RIGHT_LEG_1&&i <= RIGHT_LEG_6)
		{
			pc->O_Joint[i] = joint[1][i - RIGHT_LEG_1];
		}
	}

	//trunk质心位置
	temp_T = mt_mul((pc->Posture[BASE].T), get_move(-BASE_HIP_X, 0, 1054.5-(BASE_HIP_Z+THIGH_LEN+CRUS_LEN+FOOT_HEIGHT)));
	for (i = 0; i < 3; i++)trunk_com[i] = temp_T.t[i][3];
	//腿质心位置
	for (j = 0; j < 2; j++)
	{
		temp_T = mt_mul((pc->Posture[BASE].T), get_move(-BASE_HIP_X, hip_base_y[j], -BASE_HIP_Z));
		temp_T = mt_mul(temp_T, get_rot(AXIS_Z, joint[j][0]));
		temp_T = mt_mul(temp_T, get_rot(AXIS_X, joint[j][1]));
		temp_T = mt_mul(temp_T, get_rot(AXIS_Y, joint[j][2]));
		temp_T = mt_mul(temp_T, get_move(0, 0, -THIGH_LEN / 2.0));
		for (i = 0; i < 3; i++)leg_com[j][0][i] = temp_T.t[i][3];//大腿
		temp_T = mt_mul(temp_T, get_move(0, 0, -THIGH_LEN / 2.0));
		temp_T = mt_mul(temp_T, get_rot(AXIS_Y, joint[j][3]));
		temp_T = mt_mul(temp_T, get_move(0, 0, -CRUS_LEN / 2.0));
		for (i = 0; i < 3; i++)leg_com[j][1][i] = temp_T.t[i][3];//小腿
		temp_T = mt_mul(temp_T, get_move(0, 0, -CRUS_LEN / 2.0));
		temp_T = mt_mul(temp_T, get_rot(AXIS_Y, joint[j][4]));
		temp_T = mt_mul(temp_T, get_rot(AXIS_X, joint[j][5]));
		temp_T = mt_mul(temp_T, get_move(0, 0, -FOOT_HEIGHT / 2.0));
		for (i = 0; i < 3; i++)leg_com[j][2][i] = temp_T.t[i][3];//脚
	}

	sum_m = 0;
	for (i = 0; i < 3; i++)
	{
		sum_m += leg_mass[i];
	}
	sum_m = sum_m*2.0 + trunk_mass;

	for (i = 0; i < 3; i++)//x,y,z
	{
		sum_mx = 0;
		for (j = 0; j < 2; j++)//左腿，右腿
		{
			for (k = 0; k < 3; k++)//大腿，小腿，脚
			{
				sum_mx += leg_mass[k] * leg_com[j][k][i];
			}
		}
		sum_mx += trunk_com[i] * trunk_mass;
		pc->Body_CoM[i] = sum_mx / sum_m / 1000.0;
		pc->Body_CoM_V[i] = (pc->Body_CoM[i] - pp->Body_CoM[i]) / (robot->Squat.inter_time / 1000.0);
		pc->Body_CoM_A[i] = (pc->Body_CoM_V[i] - pp->Body_CoM_V[i]) / (robot->Squat.inter_time / 1000.0);
	}
	return 0;
}
#endif
/*<------------------运动规划函数------------------>*/


/*------------------结构体处理函数------------------*/
#ifdef ROBOT_STRUCT_DISPOSE_FUNCTION

//机器人结构体初始化函数
_rf_flg robot_init(BRobot * robot)
{
	int i, j, k;
	PostureStruct ini_posture = { 0,0,0,0,0,0 };
	TStruct		  ini_T = { 1.0,0,0,0,0,1.0,0,0,0,0,1.0,0,0,0,0,1.0 };

	if (robot == NULL)
		return -1;

	robot->MotionFlag = CARTESIAN_MOVE;

	robot->Squat.SpecialMode = NORMAL_MOVE;
	robot->Command = CMD_NONE;
	for (i = 0; i < 10; i++)
	{
		robot->Squat.SpecialParameters[i] = 0;
	}

	robot->CurrentState.SetFlag = NOT_SET;

	robot->CurrentState.Time = 0.0;
	robot->CurrentState.Time2 = 0.0;

	for (i = 0; i < PART_NUM; i++)
	{
		robot->CurrentState.Posture[i].ID = i;
		robot->CurrentState.Posture[i].Posture = ini_posture;
		robot->CurrentState.Posture[i].T = ini_T;
		robot->CurrentState.Posture[i].IKC_FLAG = NOT_SET;
		for (k = 0; k < 6; k++)
		{
			robot->CurrentState.Posture[i].MID_IKC_FLAG[k] = NOT_SET;
			for (j = 0; j < 6; j++)
			{
				robot->CurrentState.Posture[i].IntPolFac[k][j] = 0.0;
			}
		}
	}

	for (i = 0; i < 3; i++)
	{
		robot->CurrentState.Body_CoM[i] = 0.0;
		robot->CurrentState.Body_CoM_V[i] = 0.0;
		robot->CurrentState.Body_CoM_A[i] = 0.0;
	}

	//Joint init
	for (i = 0; i < JOINT_NUMBER; i++)
	{
		robot->CurrentState.Joint[i] = 0.0; 
		robot->CurrentState.JointV[i] = 0.0; 
		robot->CurrentState.JointA[i] = 0.0;
		robot->CurrentState.O_Joint[i] = 0.0;
		for (j = 0; j < 6; j++)
		{
			robot->JointIntPolFac[i][j] = 0.0;
		}
	}
	update_current(robot);
	robot->GoalState = robot->CurrentState;
	robot->PastState = robot->CurrentState;
	robot->MidGoalState = robot->CurrentState;

	robot->Squat.StepNum = STEP_NUM;
	time_init(robot);

	robot->Init_Flag = HAD_SET;
	robot->dcmInputTab[DCM_INPUT_2LINK_FLAG] = 1.0;
	return 0;
}

//矩阵乘法
TStruct	t_mul(const TStruct * t1, const TStruct * t2)
{
	TStruct res;
	int i1, i2, i3;
	for (i1 = 0; i1 < 4; i1++)
	{
		for (i2 = 0; i2 < 4; i2++)
		{
			res.t[i1][i2] = 0;
			for (i3 = 0; i3 < 4; i3++)
			{
				res.t[i1][i2] += t1->t[i1][i3] * t2->t[i3][i2];
			}
		}
	}

	return res;
}

//多矩阵乘法
_rrm TStruct mt_mul(TStruct t1, TStruct t2)
{
	return t_mul(&t1, &t2);
}

//矩阵求逆
TStruct t_inv(const TStruct * t)
{
	TStruct res;
	int i1, i2;
	_rv_t a = 0;

	a = (t->t[0][0] * t->t[1][1] * t->t[2][2] - t->t[0][0] * t->t[1][2] * t->t[2][1] - t->t[0][1] * t->t[1][0] * t->t[2][2] + t->t[0][1] * t->t[1][2] * t->t[2][0] + t->t[0][2] * t->t[1][0] * t->t[2][1] - t->t[0][2] * t->t[1][1] * t->t[2][0]);
	res.t[0][0] = t->t[1][1] * t->t[2][2] - t->t[1][2] * t->t[2][1];
	res.t[0][1] = t->t[0][2] * t->t[2][1] - t->t[0][1] * t->t[2][2];
	res.t[0][2] = t->t[0][1] * t->t[1][2] - t->t[0][2] * t->t[1][1];
	res.t[0][3] = t->t[0][1] * t->t[1][3] * t->t[2][2] - t->t[0][1] * t->t[1][2] * t->t[2][3] + t->t[0][2] * t->t[1][1] * t->t[2][3] - t->t[0][2] * t->t[1][3] * t->t[2][1] - t->t[0][3] * t->t[1][1] * t->t[2][2] + t->t[0][3] * t->t[1][2] * t->t[2][1];
	res.t[1][0] = t->t[1][2] * t->t[2][0] - t->t[1][0] * t->t[2][2];
	res.t[1][1] = t->t[0][0] * t->t[2][2] - t->t[0][2] * t->t[2][0];
	res.t[1][2] = t->t[0][2] * t->t[1][0] - t->t[0][0] * t->t[1][2];
	res.t[1][3] = t->t[0][0] * t->t[1][2] * t->t[2][3] - t->t[0][0] * t->t[1][3] * t->t[2][2] - t->t[0][2] * t->t[1][0] * t->t[2][3] + t->t[0][2] * t->t[1][3] * t->t[2][0] + t->t[0][3] * t->t[1][0] * t->t[2][2] - t->t[0][3] * t->t[1][2] * t->t[2][0];
	res.t[2][0] = t->t[1][0] * t->t[2][1] - t->t[1][1] * t->t[2][0];
	res.t[2][1] = t->t[0][1] * t->t[2][0] - t->t[0][0] * t->t[2][1];
	res.t[2][2] = t->t[0][0] * t->t[1][1] - t->t[0][1] * t->t[1][0];
	res.t[2][3] = t->t[0][0] * t->t[1][3] * t->t[2][1] - t->t[0][0] * t->t[1][1] * t->t[2][3] + t->t[0][1] * t->t[1][0] * t->t[2][3] - t->t[0][1] * t->t[1][3] * t->t[2][0] - t->t[0][3] * t->t[1][0] * t->t[2][1] + t->t[0][3] * t->t[1][1] * t->t[2][0];
	for (i1 = 0; i1 < 3; i1++)
	{
		for (i2 = 0; i2 < 4; i2++)
		{
			res.t[i1][i2] = res.t[i1][i2] / a;
		}
		res.t[3][i1] = 0.0;
	}
	res.t[3][3] = 1.0;

	return res;
}

//滤波
double filt_data(Lee_Filter * f, double new_data)
{
	int N = f->order + 1;
	int M = f->data_num;
	int i;
	double sum_x = 0.0;
	double sum_y = 0.0;

	if (f->first_data_flag == NOT_SET)//第一次运行时，数组全部给第一个数据的值,2019-04-10
	{
		for (i = 0; i < M - 1; i++)
		{
			f->X[i] = new_data;
			f->Y[i] = new_data;
		}
		f->first_data_flag = HAD_SET;
	}
	else
	{
		for (i = 0; i < M - 1; i++)
		{
			f->X[i] = f->X[i + 1];
			f->Y[i] = f->Y[i + 1];
		}
	}
	f->X[M - 1] = new_data;
	//for (i = 0; i < N + 1; i++)//N+1好像不对,2019-04-08
	for (i = 0; i < N; i++)
	{
		sum_x += f->B[i] * f->X[M - 1 - i];
		if(i>0)
			sum_y += f->A[i] * f->Y[M - 1 - i];
	}
	f->Y[M - 1] = (sum_x - sum_y) / f->A[0];
	return f->Y[M - 1];
}


void init_filter(Lee_Filter *f, int order, int data_num, double A[], double B[])
{
	int i;
	f->first_data_flag = NOT_SET;//第一次运行时，数组全部给第一个数据的值,2019-04-10
	f->order = order;
	f->data_num = data_num;
	for (i = 0; i < order + 1; i++)
	{
		f->A[i] = A[i];
		f->B[i] = B[i];
	}
	if (!equal_check(f->A[0], 1.0))
	{
		for (i = 0; i < order + 1; i++)
		{
			f->A[i] = A[i]/A[0];
		}
	}
	for (i = 0; i < data_num; i++)
	{
		f->X[i] = 0.0;
		f->Y[i] = 0.0;
	}
}


void init_estimator(Lee_Estimator * e, int num[3], double sys_matrix[8][2][2])
{
	int i,j,k;
	MStruct * mat[8] = { &(e->M),&(e->A),&(e->B),&(e->C),&(e->D),&(e->X),&(e->U),&(e->Y) };
	e->x_num = num[0];
	e->u_num = num[1];
	e->y_num = num[2];
	e->M = init_matrix0(e->x_num, e->y_num);
	e->A = init_matrix0(e->x_num, e->x_num);
	e->B = init_matrix0(e->x_num, e->u_num);
	e->C = init_matrix0(e->y_num, e->x_num);
	e->D = init_matrix0(e->y_num, e->u_num);
	e->X = init_matrix0(e->x_num, 1);
	e->U = init_matrix0(e->u_num, 1);
	e->Y = init_matrix0(e->y_num, 1);

	for (i = 0; i < 8; i++)
	{
		for (j = 0; j < mat[i]->dim[0]; j++)
		{
			for (k = 0; k < mat[i]->dim[1]; k++)
			{
				mat[i]->m[j][k] = sys_matrix[i][j][k];
			}
		}
	}
}

void estimate_state(Lee_Estimator * e)
{
	MStruct temp1, temp2;
	// X = AX+BU
	temp1 = m_mul(&(e->A), & (e->X));
	temp2 = m_mul(& (e->B), & (e->U));
	e->X = m_sum(&temp1, &temp2);

	// X = X + M(Y-CX-DU)
	temp1 = m_mul(&(e->C), &(e->X));
	temp2 = m_mul(&(e->D), &(e->U));
	temp1 = m_sub(&(e->Y), &temp1);
	temp1 = m_sub(&temp1, &temp2);
	temp1 = m_mul(&(e->M), &temp1);
	e->X = m_sum(&(e->X), &temp1);
}


MStruct get_rot_from_t(TStruct T)
{
	MStruct M = init_matrix(3, 3);
	int i, j;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			M.m[i][j] = T.t[i][j];
		}
	}
	return M;
}

MStruct get_ts_from_t(TStruct T)
{
	MStruct M = init_matrix(4, 4);
	int i, j;
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			M.m[i][j] = T.t[i][j];
		}
	}
	return M;
}

void ikc_flag_reset(BRobot * robot)
{
	int i;
	for (i = 0; i < SPECIAL_MOTION; i++)
	{
		robot->GoalState.Posture[i].IKC_FLAG = NOT_SET;
	}
}
#endif
/*<-----------------结构体处理函数----------------->*/
