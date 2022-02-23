/***********************************************************************
 *  BHR BIPED CONTROL SOFTWARE UNDER Windows XP+Real_Time Extension    *  
 *	Filename:     can_ds402.h                                          *
 *  for Elmo and Kollmorgen drivers compliant with DS402 Protocol      *
 *	July.16, 2010, sniffered and sensed from Goldline controller       *
 *  to enable Interpolation mode of DS402                              *
 *  To begin with elmo drivers , Dec. 11, 2009,                        *
 *   sniffered and sensed from Maestro mutli-axis controller           *
 *   created by Z.G. YU and X.C. CHEN                                  *
 *   Modified for G_TWI motor drivers, Sept 1, 2016                    *
 ***********************************************************************/
#ifndef H__G_ELMO__CTRL
#define H__G_ELMO__CTRL

#define  NMT_ENTER_PREOPERATION    0  
#define  NMT_GO_OPERATION	 1
#define  SYNC_CMD   2
#define  MOEQ0 3 //mo=0
#define  MOEQ1 4 //mo=1
#define  BG_CMD  5
#define  RMEQ0    6  //RM=0
#define  MP1EQ1   7 //MP[1]=1,first index
#define  MP2EQ64  8 //MP[2]=64,Last index,
#define  MP3EQ1   9//MP[3]=1,the motion is to be cyclical
#define  MP4EQ10   10//MP[4]=10 ,T, should be modified
#define  MP5EQ0    11 // MP[5]=0, no emergency object is sent.
#define  MP6EQ1   12 //MP[6] = 1. MP[6] = Write pointer.
#define  DS402CONFIG_ONLY_ELMO  13 
#define  DS402BUF_CLR  14 
#define  DS402BUF_LEN  15 
#define  DS402MODE_IP_POSITION  16 
#define  DS402IP_CYCLE  17
#define  CLR_RPDO1   18
#define  MAP402_IP_DAT_RPDO1  19
#define  MAP402_CNTL_WORD_RPDO1  20 
#define  SYNC_RPDO1  21
#define  EN_RPDO1  22
#define  CLR_TPDO1  23 
#define  MAP_ENCODER_TPDO1  24  
#define  MAP_CNTL_STATUS_TPDO1  25 
#define  SYNC_TPDO1  26  
#define  EN_TPDO1  46
#define  SET_IP_REF_SHUTDOWN  27 
#define  SET_IP_REF_READY  28
#define  SET_IP_REF_MOTORON  29
#define  SET_IP_REF_PEROIDIC  30

#define  HOM_STEP1  31
#define  HOM_STEP2  32
#define  HOM_STEP3  33

#define  CLR_ENC_HM2EQ0  34
#define  CLR_ENC_HM3EQ0  35
#define  CLR_ENC_HM4EQ0  36
#define  CLR_ENC_HM5EQ0  37
#define  CLR_ENC_HM1EQ1  38

#define  HM2OFFSET  39

#define  TMEQ0  40
#define  PXEQ0  41
#define  QV01ASTIMEOUT  42
#define  QV02ASVMOVE  43
#define  QV03ASVSEARCH  44
#define  QV04ASMOVEBIAS  45
#define SET_IP_REF_MOTOROFF 47

//#define SWITCH_OFF_RPDO2 48
//#define SWITCH_OFF_RPDO3 49
//#define SWITCH_OFF_RPDO4 50

#define SWITCH_OFF_RPDO2_COB 48
#define SWITCH_OFF_RPDO3_COB 49
#define SWITCH_OFF_RPDO4_COB 50



//#define SWITCH_OFF_TPDO2 51
//#define SWITCH_OFF_TPDO3 52
//#define SWITCH_OFF_TPDO4 53
#define SWITCH_OFF_TPDO2_COB 51
#define SWITCH_OFF_TPDO3_COB 52
#define SWITCH_OFF_TPDO4_COB 53

#define DS402IP_TIME_INDEX 54
#define NMT_RESET 55

#define DIS_TPDO1_COBID 56
#define DIS_RPDO1_COBID 57
#define RPDO1_MAPPED_NUM 58
#define TPDO1_MAPPED_NUM 59

#define CLR_RPDO2_MAP 60
#define CLR_RPDO3_MAP 61
#define CLR_RPDO4_MAP 62

#define CLR_TPDO2_MAP 63
#define CLR_TPDO3_MAP 64
#define CLR_TPDO4_MAP 65

#define ELMO_CMD_LENGTH 66  //+1, i.e.,equals to the above max value plus 1

typedef struct {
  unsigned char cmd_name;  // sequence id
  unsigned short int  can_id;
  unsigned char dat[8];
  unsigned char len_cmd;
} elmo_cmd;

extern elmo_cmd elmo_cmds[ELMO_CMD_LENGTH];  	/* define actual body of variables cited in pid_process.c */

//elmo_cmd Elmo_DS301cmd[ELMO_CMD_LENGTH]={
extern elmo_cmd Elmo_DS301cmd[ELMO_CMD_LENGTH];
//extern void init_comm_DS402_para(int ip_cycle_time, int channel_num);
/*
extern void G_CANopen_Enter_Preoperation(int channel_num, int CAN_id );
extern void G_CAN_MapDS402_IPSet(int channel_num,int CAN_id, int ip_cycle_time);
extern void G_CANopen_NMT_Go_Opration(int channel_num, int CAN_id );
extern void G_CAN_MapDS402_IP_Shutdown(int channel_num, int CAN_id ); 
extern void G_CAN_DS402_Elmo_Specific(int channel_num, int CAN_id );
extern void G_CANopen_NMT_Reset(int channel_num, int CAN_id );
extern void G_CAN_DIS_R_T_PDO234(int channel_num,int CAN_id);
*/


void G_CANopen_Enter_Preoperation(int channel_num, int CAN_id );
void G_CANopen_NMT_Reset(int channel_num, int CAN_id ); 
void G_CAN_DIS_R_T_PDO234(int channel_num,int CAN_id);
void G_CAN_MapDS402_IPSet(int channel_num,int CAN_id, int ip_cycle_time); 
void G_CANopen_NMT_Go_Opration(int channel_num, int CAN_id );
void G_CAN_MapDS402_IP_Shutdown(int channel_num, int CAN_id );
void G_CAN_DS402_Elmo_Specific(int channel_num, int CAN_id );

#endif