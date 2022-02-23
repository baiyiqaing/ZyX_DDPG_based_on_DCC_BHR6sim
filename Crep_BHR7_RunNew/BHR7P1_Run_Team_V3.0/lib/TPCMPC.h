#pragma once 

#define nNumPre 125
#define nStateNum 3

extern double dTPCMPCConval[2];

void fnvTPCMPCCalConval(double dVeZmpRefx_125x1[nNumPre][1], double dVeZmpRefy_125x1[nNumPre][1], double dVeStatex_3x1[nStateNum][1], double dVeStatey_3x1[nStateNum][1]);
