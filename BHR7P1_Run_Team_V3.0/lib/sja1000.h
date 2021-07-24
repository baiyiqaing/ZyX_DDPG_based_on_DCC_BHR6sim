#ifndef _82C200_H
#define _82C200_H

/* 82c200 Definitions */

#define CAN_CR 0
/*--- 0 Control Register ---------------------------------*/
 
#define CAN_TEST_MODE				(1<<7)
#define CAN_SPEED_MODE				(1<<6)
#define CAN_OVERRUN_INT_ENABLE			(1<<4)
#define CAN_ERROR_INT_ENABLE			(1<<3)
#define CAN_TRANSMIT_INT_ENABLE			(1<<2)
#define CAN_RECEIVE_INT_ENABLE			(1<<1)
#define CAN_RESET_REQUEST			(1<<0)

#define CAN_CMR 1
/*--- 1 Command Register ------------------------------------*/
 
#define CAN_GOTO_SLEEP				(1<<4)
#define CAN_CLEAR_OVERRUN_STATUS		(1<<3)
#define CAN_RELEASE_RECEIVE_BUFFER		(1<<2)
#define CAN_ABORT_TRANSMISSION			(1<<1)
#define CAN_TRANSMISSION_REQUEST		(1<<0)


#define CAN_SR 2
/*--- 2 Status Register --------------------------------*/
 
#define CAN_BUS_STATUS 				(1<<7)
#define CAN_ERROR_STATUS			(1<<6)
#define CAN_TRANSMIT_STATUS			(1<<5)
#define CAN_RECEIVE_STATUS			(1<<4)
#define CAN_TRANSMISSION_COMPLETE_STATUS	(1<<3)
#define CAN_TRANSMIT_BUFFER_ACCESS		(1<<2)
#define CAN_DATA_OVERRUN			(1<<1)
#define CAN_RECEIVE_BUFFER_STATUS		(1<<0)

/*--- 2 Status Register --------------------------------*/
 
#define CAN_BUS_STATUS_BIT 			(1<<7)
#define CAN_ERROR_STATUS_BIT			(1<<6)
#define CAN_TRANSMIT_STATUS_BIT			(1<<5)
#define CAN_RECEIVE_STATUS_BIT			(1<<4)
#define CAN_TRANSMISSION_COMPLETE_STATUS_BIT	(1<<3)
#define CAN_TRANSMIT_BUFFER_ACCESS_BIT		(1<<2)
#define CAN_DATA_OVERRUN_BIT			(1<<1)
#define CAN_RECEIVE_BUFFER_STATUS_BIT		(1<<0)


#define CAN_IR 3
/*--- 3 Interrupt Register -----------------------------------*/
 
#define CAN_WAKEUP_INT				(1<<4)
#define CAN_OVERRUN_INT				(1<<3)
#define CAN_ERROR_INT				(1<<2)
#define CAN_TRANSMIT_INT			(1<<1)
#define CAN_RECEIVE_INT 			(1<<0)

#define CAN_ACR 4
#define CAN_AMR 5
#define CAN_BTR0 6
#define CAN_BTR1 7

#define CAN_OCR 8
/*--- 8 Output Control Register -----------------------------------------*/
/*
 *	7	6	5	4	3	2	1	0
 * 	OCTP1	OCTN1	OCPOL1	OCTP0	OCTN0	OCPOL0	OCMODE1	OCMODE0
 *	----------------------  ----------------------  ---------------
 *	    TX1 Output		    TX0 Output		  programmable
 *	  Driver Control	  Driver Control	  output functions
 *
 *	MODE
 *	OCMODE1	OCMODE0
 *	  1	  0	Normal Mode; TX0, TX1 bit sequenze TXData
 *	  1	  1	Normal Mode; TX0 bit sequenze, TX1 busclock TXCLK
 *	  0	  0	Biphase Mode
 *	  0	  1	Test Mode; TX0 bit sequenze, TX1 COMPOUT
 *
 *	In normal Mode Voltage Output Levels depend on 
 *	Driver Characteristic: OCTPx, OCTNx
 *	and programmed Output Polarity: OCPOLx
 *
 *	Driver Characteristic
 *	OCTPx	OCTNx
 *	  0	 0	always Floating Outputs,
 *	  0	 1	Pull Down
 *	  1	 0	Pull Up
 *	  1	 1	Push Pull
 */
 
/*--- 8 Output control register --------------------------------*/

#define CAN_OCTP1			(1<<7)
#define CAN_OCTN1			(1<<6)
#define CAN_OCPOL1			(1<<5)
#define CAN_OCTP0			(1<<4)
#define CAN_OCTN0			(1<<3)
#define CAN_OCPOL0			(1<<2)
#define CAN_OCMODE1			(1<<1)
#define CAN_OCMODE0			(1<<0)


#define CAN_TX 10

#define CAN_RX 20

#define CAN_CDR 31

#define CAN_HWRESET 0x100


/*--- Remote Request ---------------------------------*/
/*    Notes:	RTR is Bit 5 in TXDES1.
 */
#ifndef ID_RTR
#define ID_RTR					(1<<4)
#endif

/*---------- Timing values */
#undef XTAL_8
#undef  XTAL_12
#define  XTAL_16

#ifdef XTAL_8
/* the timings are valid for xtal 8 MHz -> clock 4Mhz */
#define CAN_TIM0_10K		  24    // 6.25 us
#define CAN_TIM1_10K		0x1c    // 1 + 2 + 13 = 16 -> 100 us

#define CAN_TIM0_20K		   9	// 2.5 us
#define CAN_TIM1_20K		0x2f	// 1 + 3 + 16 = 20 -> 50 us

#define CAN_TIM0_40K               4    // 1.25 us
#define CAN_TIM1_40K		0x2f    // 1 + 3 + 16 = 20 -> 25 us

#define CAN_TIM0_50K		   4    // 1.25 us
#define CAN_TIM1_50K		0x1c    // 1 + 2 + 13 = 16 -> 20 us

#define CAN_TIM0_100K              1    // 0.5 us
#define CAN_TIM1_100K		0x2f    // 1 + 3 + 16 = 20 -> 10 us

#define CAN_TIM0_125K              1    // 0.5 us
#define CAN_TIM1_125K		0x1c    // 1 + 2 + 13 = 16 -> 8 us

#endif


#ifdef XTAL_12
/* the timings are valid for xtal 12 MHz -> clock 6Mhz */
#define CAN_TIM0_10K		  29    // 5 us
#define CAN_TIM1_10K		0x2f    // 1 + 3 + 16 = 20 -> 100 us

#define CAN_TIM0_20K		  19	// 3.333... us
#define CAN_TIM1_20K		0x1b	// 1 + 2 + 12 = 15 -> 50 us

#define CAN_TIM0_40K               9    // 1.666... us
#define CAN_TIM1_40K		0x1b    // 1 + 2 + 12 = 15 -> 25 us

#define CAN_TIM0_50K		   7    // 1.333... us
#define CAN_TIM1_50K		0x1b    // 1 + 2 + 12 = 15 -> 20 us

#define CAN_TIM0_100K              3    // 0.666... us
#define CAN_TIM1_100K		0x1b    // 1 + 2 + 12 = 15 -> 10 us

#define CAN_TIM0_125K              2    // 0.5 us
#define CAN_TIM1_125K		0x1c    // 1 + 2 + 13 = 16 -> 8 us

#endif


#ifdef XTAL_16
/* the timings are valid for xtal 16 MHz -> clock 8Mhz */
#define CAN_TIM0_10K		  49
#define CAN_TIM1_10K		0x1c

#define CAN_TIM0_20K		  24	
#define CAN_TIM1_20K		0x1c

#define CAN_TIM0_40K		0x89	/* Old Bit Timing Standard of port */
#define CAN_TIM1_40K		0xEB	/* Old Bit Timing Standard of port */

#define CAN_TIM0_50K		   9
#define CAN_TIM1_50K		0x1c

#define CAN_TIM0_100K              4
#define CAN_TIM1_100K		0x1c

#define CAN_TIM0_125K		   3
#define CAN_TIM1_125K		0x1c

#define CAN_TIM0_250K		   1
#define CAN_TIM1_250K		0x1c

#define CAN_TIM0_500K		   0
#define CAN_TIM1_500K		0x1c

#define CAN_TIM0_800K		   0
#define CAN_TIM1_800K		0x07

#define CAN_TIM0_1000K		   0
#define CAN_TIM1_1000K		0x05
#endif

/*======================================================================*/

/************************************************************************/
/*regerster definition of protocol 2.0B*/
#define CAN_MOD 0
#define CAN_CMR 1
#define CAN_SR  2
#define CAN_IR  3
#define CAN_IER 4
#define CAN_BTR0 6
#define CAN_BTR1 7
#define CAN_OCR 8
#define CAN_ACR0 16
#define CAN_AMR0 20
#define CAN_TX_EX 16
#define CAN_RX_EX 16
#define CAN_HWRESET 0x100
/************************************************************************/

#endif
