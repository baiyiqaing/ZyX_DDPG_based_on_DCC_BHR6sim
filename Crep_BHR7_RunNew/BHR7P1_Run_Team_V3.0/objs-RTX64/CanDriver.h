#ifndef CAN_DRIVER_RTX_H
#define CAN_DRIVER_RTX_H

//------------------------------------------------------------------------------
#ifdef    CAN_DRIVER_RTX_C
#define   EXTERN    /*define*/
#else
#define   EXTERN    extern
#endif

#include "adv_can.h"
#include "sja1000.h"
EXTERN int GetCANCard_PCM3680I(void);
EXTERN int can_read_channel( int port,can_msg_t *msg);
EXTERN int can_write_channel (int port,can_msg_t *msg ); 
EXTERN BOOL can_wrready(int port);
EXTERN void can_Init_set_param( int port,CAN_STRUCT *canstructvar);


#undef EXTERN

//------------------------------------------------------------------------------
#endif

//END OF THE FILE