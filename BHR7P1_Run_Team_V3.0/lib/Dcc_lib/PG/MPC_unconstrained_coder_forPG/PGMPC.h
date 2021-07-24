#pragma once 

#define nNumPre 500
#define nStateNum 3

extern double dPGMPCConval[2];

void fnvPGMPCCalConval(double dVeZmpRefx_500x1[nNumPre][1], double dVeZmpRefy_500x1[nNumPre][1], double dVeStatex_3x1[nStateNum][1], double dVeStatey_3x1[nStateNum][1]);
