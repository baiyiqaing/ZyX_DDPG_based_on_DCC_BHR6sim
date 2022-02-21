#pragma once
#ifndef MJ_DATAEXCHANGE_H
#define  MJ_DATAEXCHANGE_H
#ifdef MJ_DATAEXCHENGE_C
#define Extern 
#else 
#define Extern extern
#endif

#ifdef __cplusplus
extern "C"
{
#endif
int fndMjDataExchange(double dptJointsPosition[50], double dptJointsVelocity[50], double dptGyroSensor[2][3], double dptFootFT[10][6], int k_pre, double dptCmdJointsPosition[50]);
#ifdef __cplusplus
}
#endif

#undef Extern

#endif