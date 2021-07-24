#ifndef AD_ADVCAN_INC
#define AD_ADVCAN_INC
// #############################################################################
// *****************************************************************************
//                  Copyright ( c ) 2003, Advantech Automation Corp.
//      THIS IS AN UNPUBLISHED WORK CONTAINING CONFIDENTIAL AND PROPRIETARY
//               INFORMATION WHICH IS THE PROPERTY OF ADVANTECH AUTOMATION CORP.
//
//    ANY DISCLOSURE, USE, OR REPRODUCTION, WITHOUT WRITTEN AUTHORIZATION FROM
//               ADVANTECH AUTOMATION CORP., IS STRICTLY PROHIBITED.
// *****************************************************************************
// #############################################################################
//
// File:     adv_can.h
// Author:   Xuefeng Tang
// Created:  02/15/2003
// Revision: 1.00 
// Description:  PCL841 CAN Bus Driver
// -----------------------------------------------------------------------------


#define DEVICE_NAME "pci_can"
#define DRIVER_VERSION "1.00"

#define CAN_MAJOR 60

/* Device and Driver Declarations **************************** */

#define CAN_RESET_OFF  0x00

#define CAN_PROTOCOL_20A 0
#define CAN_PROTOCOL_20B 1


// Data Frame Format
#define PELICAN_EFF                    0x80     // Extended Frame Format
#define PELICAN_SFF                    0x00     // Standard Frame format


#define CAN_INFO_EFF 0x80
#define CAN_INFO_RTR 0x40
#define CAN_INFO_LEN_MASK 0x0F

#define CAN_ID1_MASK 0xE0        /* ID20 ID19 ID18 0 0 0 0 0 */

/*--- Remote Request ---------------------------------*/
/*    Notes:	RTR is Bit 5 in TXDES1.
 */
#ifndef CAN_ID1_RTR
#define CAN_ID1_RTR 0x10
#endif

/* CAN operating modes */
enum {CAN_RESET=-1, CAN_10K, CAN_20K, CAN_40K, CAN_50K, CAN_100K, CAN_125K, CAN_500K, CAN_800K, CAN_1000K };

#define SUCCESS 0


/* Set the message of the device driver */
#define IOCTL_SET_CAN _IOR(CAN_MAJOR, 0, int)
#define IOCTL_RESET_CAN _IO(CAN_MAJOR,1)
//#define IOCTL_NOMALRUN_CAN _IO(CAN_MAJOR,2)
/* _IOR means that we're creating an ioctl command
 * number for passing information from a user process
 * to the kernel module.
 *
 * The first arguments, CAN_MAJOR, is the major device
 * number we're using.
 *
 * The second argument is the number of the command
 * (there could be several with different meanings).
 *
 * The third argument is the type we want to get from
 * the process to the kernel.
 */

/* Get the n'th byte of the message */
//#define IOCTL_GET_NTH_BYTE _IOWR(CAN_MAJOR, 2, int)
 /* The IOCTL is used for both input and output. It
  * receives from the user a number, n, and returns
  * Message[n]. */

/*
 * This is the CAN message
 */
typedef struct {
  unsigned char ff;     /*infomation of frame, 0 for sff, 128 for eff*/
  unsigned char rtr;     /*remore frame*/
  unsigned int id;       /* not present in standard frames */
  unsigned char dlen;
  unsigned char data[8];
//  unsigned char new_return_val;
} can_msg_t;

#define CAN_MSG_LEN (sizeof( can_msg_t ) )

typedef struct {
  unsigned char accode[4];
  unsigned char accmask[4];
  unsigned char speed;
  unsigned char interruptmask;
  unsigned char protocol;
  unsigned char filtertype;
}CAN_STRUCT;

extern struct file_operations Fops;


#endif
