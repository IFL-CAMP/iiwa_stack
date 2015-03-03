/*{{[PH]
****************************************************************************
 Project:  FRI

  This material is the exclusive property of KUKA Roboter GmbH 
  and must be returned to KUKA Roboter GmbH immediately upon 
  request.  This material and the information illustrated or 
  contained herein may not be used, reproduced, stored in a 
  retrieval system, or transmitted in whole or in part in any 
  way - electronic, mechanical, photocopying, recording, or 
  otherwise, without the prior written consent of KUKA Roboter GmbH.  
  
                 All Rights Reserved
                 Copyright (C)  2009
                  KUKA Roboter GmbH
                  Augsburg, Germany
  
[PH]}}
*/

/*
{{[FH]
****************************************************************************
  friComm.h
       
      NOTE: FRI (Fast Research inteface) is subject to radical change
	  
      
[FH]}}
*/ 
/** 
 \author (Andreas Stemmer,DLR)
 \author (Guenter Schreiber,KUKA)
  \file
  \brief   This header file describes the basic communcation structures for
   the FastRobotInterface
  
  
   - all data is transmitted in Intel byte order (little endian)
   - structs use 4 byte padding
   - all floats are IEEE 754 single precision floats with 4 bytes
   - for the sequence counters in the header, the number following
     #FRI_UINT16_MAX is assumed to be zero (normal integer overflow)
   - For Checking, that your compiler setup interprets the binary communication as expceted,
   use the macros #FRI_CHECK_SIZES_OK, #FRI_PREPARE_CHECK_BYTE_ORDER, #FRI_CHECK_BYTE_ORDER_OK in your code.
     
 */
/*****************************************************************************

 friComm.h

 created: 09/05/26  by  Andreas Stemmer

 $Revision: 1.17 $  $Author: stemmer $    $Date: 2010/02/12 15:15:18 $   

 (C) by Institute of Robotics and Mechatronics, DLR

 Description:

   This header file describes the basic communcation structures for
   the FastRobotInterface

 Note:

   

 *****************************************************************************/

#ifndef FRICOMM_H
#define FRICOMM_H


#define LBR_MNJ 7				/*!< Number of Managed Joints for Light Weight Robot */
#define FRI_USER_SIZE 16   		/*!< Number of Userdefined data, which are exchanged to KRL Interpreter  */
#define FRI_CART_FRM_DIM 12     /*!< Number of data within a 3x4 Cartesian frame - homogenous matrix */
#define FRI_CART_VEC  6			/*!< Number of data for a Cartesian vector */


typedef unsigned char  fri_uint8_t;  /*!< 1 byte unsigned */
typedef unsigned short fri_uint16_t; /*!< 2 byte unsigned */
typedef unsigned int   fri_uint32_t; /*!< 4 byte unsigned */
typedef int            fri_int32_t;  /*!< 4 byte signed */
typedef float          fri_float_t;  /*!< 4 byte IEEE 754 float */
#define FRI_UINT16_MAX ((1<<16) - 1) /*!< Biggest UINT16 w.r.t the interface - porting isno fun */


/** header data used in every packet in both directions
 *  
 *  */
typedef struct
{
    /** sequence counter (increasing with every packet) of the sender
      Used for detecting packet loss of UDP Transfer,
      put in your code s.th. like
     * fri_uint16_t seqCount;
     * seqCount++;
     * cmd.head.sendSeqCount=seqCount;
     */
    fri_uint16_t sendSeqCount;

    /** reflected sequence counter (mirrored from last received packet)
       Used for detecting packet loss of UDP transfer and latency statistics 
       \sa tFriIntfStatistics
     * remote side: put s.th. like
     * cmd.head.reflSeqCount=msr.head.sendSeqCount;
      */
    fri_uint16_t reflSeqCount;

    /** overall packet size 
       Remote side:  FRI_CMD_DATA_SIZE
      */
    fri_uint16_t packetSize;

    /** unique datagram id to detect version problems etc.
        Remote side: FRI_DATAGRAM_ID_CMD
      */
    fri_uint16_t datagramId;
} tFriHeader;
#define FRI_HEADER_SIZE 8




/** data for direct interaction with KRL */
typedef struct
{
    /** 16 float values (corresponds to $FRI_TO_REA[] and $FRI_FRM_REA[]) */
    fri_float_t realData[FRI_USER_SIZE];

    /** 16 int values (corresponds to $FRI_TO_INT[] and $FRI_FRM_INT[]) */
    fri_int32_t intData[FRI_USER_SIZE];

    /** 16 bool values, stored in one int */
    fri_uint16_t boolData;

    /** manual padding to a multiple of 4 bytes */
    fri_uint16_t fill;
} tFriKrlData;
#define FRI_KRL_DATA_SIZE 132




/** reported communication statistics (average rating over the last
    100 packets) */
typedef struct
{
    /** avg. answer rate (should be close to 1) */
    fri_float_t answerRate;

    /** avg. latency in seconds (should be as small as possible) */
    fri_float_t latency;

    /** avg. jitter in seconds (should be as small as possible) */
    fri_float_t jitter;

    /** avg. missing answer packets (should be as small as possible) */
    fri_float_t missRate;

    /** absolute missing answer packets */
    fri_uint32_t missCounter;
} tFriIntfStatistics;
#define FRI_INTF_STATISTICS_SIZE 20




/** feedback about the interface itself */
typedef struct
{
    /** current system time in seconds */
    fri_float_t timestamp;

    /** state of interface (monitor or command FRI_STATE) */
    fri_uint16_t state;

    /** quality of communication (FRI_QUALITY) */
    fri_uint16_t quality;

    /** configured sample time for sent measurment packets */
    fri_float_t desiredMsrSampleTime;
    
    /** configured sample time for received command packets */
    fri_float_t desiredCmdSampleTime;

    /** configured safety limits */
    fri_float_t safetyLimits;

    /** communication statistics */
    tFriIntfStatistics stat;
} tFriIntfState;
#define FRI_INTF_STATE_SIZE (20 + FRI_INTF_STATISTICS_SIZE)




/** these are the states that are reported */
typedef enum
{
	FRI_STATE_INVALID =-1, /**< State invalid e.g. not initialized */
    FRI_STATE_OFF = 0,  /**< internal state only (no active interface) */
    FRI_STATE_MON = 1,  /**< FRI is in monitor mode */
    FRI_STATE_CMD = 2   /**< FRI is in command mode */
} FRI_STATE;




/** quality of the connection is classified in four steps */
typedef enum
{
	FRI_QUALITY_INVALID =-1, /**<  not allowed, since improber initialized */
    FRI_QUALITY_UNACCEPTABLE = 0, /**< commanding is not allowed */
    FRI_QUALITY_BAD = 1,          /**< commanding is allowed */
    FRI_QUALITY_OK = 2,           /**< commanding is allowed */
    FRI_QUALITY_PERFECT = 3       /**< commanding is allowed */
} FRI_QUALITY;




/** currently active controller */
typedef enum
{
    /** only joint position can be commanded */
    FRI_CTRL_POSITION = 1,
    /** joint/cart positions, joint/cart stiffness, joint/cart damping
        and additional TCP F/T can be commanded */
    FRI_CTRL_CART_IMP = 2,
    /** joint positions, stiffness and damping and additional joint
        torques can be commanded */
    FRI_CTRL_JNT_IMP = 3,
    /** for all the other modes... */
    FRI_CTRL_OTHER = 0
} FRI_CTRL;




/** feedback about the robot */
typedef struct
{
    /** power state of drives (bitfield) */
    fri_uint16_t power;

    /** selected control strategy (FRI_CTRL) */
    fri_uint16_t control;

    /** drive errors (leads to power off) */
    fri_uint16_t error;

    /** drive warnings */
    fri_uint16_t warning;

    /** temperature of drives */
    fri_float_t temperature[LBR_MNJ];
} tFriRobotState;
#define FRI_ROBOT_STATE_SIZE 36




/** current robot data */
typedef struct
{
    /** measured joint angles (in rad) 
     * KRL: $AXIS_ACT_MSR
     * */
    fri_float_t msrJntPos[LBR_MNJ];

    /** measured Cartesian frame (in m)
     * KRL: $POS_ACT_MSR
     * Reference: Base and tool are specified by $stiffness.base, $stiffness.tool 
     * */
    fri_float_t msrCartPos[FRI_CART_FRM_DIM];

    /** commanded joint angle (in rad, before FRI)
     * KRL: $AXIS_ACT_CMD 
     * */
    fri_float_t cmdJntPos[LBR_MNJ];

    /** commanded joint offset (in rad, due to FRI) */
    fri_float_t cmdJntPosFriOffset[LBR_MNJ];

    /** commanded Cartesian frame (in m, before FRI) 
     * KRL: $POS_ACT_CMD
     * Reference: Base and tool are specified by $stiffness.base, $stiffness.tool 
     * */
    fri_float_t cmdCartPos[FRI_CART_FRM_DIM];

    /** commanded Cartesian frame (in m, due to FRI) */
    fri_float_t cmdCartPosFriOffset[FRI_CART_FRM_DIM];

    /** measured joint torque (in Nm)
     * KRL $TORQUE_AXIS_ACT
     *  */
    fri_float_t msrJntTrq[LBR_MNJ];

    /** estimated external torque (in Nm) 
     * KRL: $TORQUE_AXIS_EST
     * */
    fri_float_t estExtJntTrq[LBR_MNJ];

    /** estimated TCP force/torque (N, Nm) 
      KRL: $TORQUE_TCP_EST
     - reference frame: $STIFFNESS.TASKFRAME
     - Layout: Fx, Fy, Fz, Tz, Ty, Tx  
     */
    fri_float_t estExtTcpFT[FRI_CART_VEC];

    /** Jacobian matrix  
     * reference frame: $STIFFNESS.TASKFRAME
     * row major (#FRI_CART_VEC x #LBR_MNJ)
     * You should be able to cast it directly into a C-matrix with the layout of
       J[FRI_CART_VEC][LBR_MNJ]
     *  Or copy it like 
      for ( int i = 0; i < FRI_CART_VEC; i++) 
        for ( int j = 0; j < LBR_MNJ; j++) 
			J[i][j] = jacobian[i*LBR_MNJ+j]
     * Interpretation of the colums [ _X_ _Y_ _Z_ _RZ_ _RY_ _RX_]
     * The Jacobian is generated by "geometric" reasoning with a corresponding physical
      interpretation of instantaneous angular velocity
      */
    fri_float_t jacobian[FRI_CART_VEC*LBR_MNJ];

    /** mass matrix */
    fri_float_t massMatrix[LBR_MNJ*LBR_MNJ];

    /** gravity (in m/s^2) */
    fri_float_t gravity[LBR_MNJ];
} tFriRobotData;
#define FRI_ROBOT_DATA_SIZE (175*4) /**< Binary size of #tFriRobotData */



/** Identification constant for the structure #tFriMsrData 
    Note: This datagram id will be abused for versioning as well */
#define FRI_DATAGRAM_ID_MSR 0x2006
/** Data that is sent from the KRC to the external controller. 
    The structure is identified by #FRI_DATAGRAM_ID_MSR
  */
typedef struct
{
    /** the header */
    tFriHeader head;

    /** data from KRL */
    tFriKrlData krl;

    /** state of interface */
    tFriIntfState intf;

    /** robot state */
    tFriRobotState robot;
    
    /** robot data */
    tFriRobotData data;
} tFriMsrData;
#define FRI_MSR_DATA_SIZE (FRI_HEADER_SIZE + FRI_KRL_DATA_SIZE +    \
                           FRI_INTF_STATE_SIZE + FRI_ROBOT_STATE_SIZE + \
                           FRI_ROBOT_DATA_SIZE)




/** The bitfield is subject to change, any use of hardcoded constants
    is NOT RECOMMENDED! Use the provided defines instead! */
#define FRI_CMD_JNTPOS    0x0001
// Currently not supported #define FRI_CMD_JNTVEL    0x0002
#define FRI_CMD_JNTTRQ    0x0004
#define FRI_CMD_JNTSTIFF  0x0010
#define FRI_CMD_JNTDAMP   0x0020

#define FRI_CMD_CARTPOS   0x0100
//Currently not supported #define FRI_CMD_CARTVEL   0x0200
#define FRI_CMD_TCPFT     0x0400
#define FRI_CMD_CARTSTIFF 0x1000
#define FRI_CMD_CARTDAMP  0x2000




/** new robot commands */
typedef struct
{
    /** bitfield which selects which commanded values are relevant */
    fri_uint32_t cmdFlags;

    /** commanded joint angle (in rad) 
      #FRI_CMD_JNTPOS
     */
    fri_float_t jntPos[LBR_MNJ];

    /** commanded Cartesian frame (in m) 
      #FRI_CMD_CARTPOS
      */
    fri_float_t cartPos[FRI_CART_FRM_DIM];

    /** commanded additional joint torque (in Nm) 
      #FRI_CMD_JNTTRQ*/
    fri_float_t addJntTrq[LBR_MNJ];

    /** commanded additional TCP force/torque (N, Nm) 
      #FRI_CMD_TCPFT
     * reference frame: $STIFFNESS.TASKFRAM 
     * Layout [Fx, Fy, Fz, Tz, Ty, Tx]*/
    fri_float_t addTcpFT[FRI_CART_VEC];

    /** joint stiffness (Nm/rad) 
      #FRI_CMD_JNTSTIFF */
    fri_float_t jntStiffness[LBR_MNJ];

    /** joint damping (normalized) 
      #FRI_CMD_JNTDAMP*/
    fri_float_t jntDamping[LBR_MNJ];
    
    /** Cartesian stiffness (N/m, Nm/rad)
      * #FRI_CMD_CARTSTIFF 
      * Layout [Cx, Cy, Cz, Ca(Rz), Cb(Ry), Cc(Rx)]*/
    fri_float_t cartStiffness[FRI_CART_VEC];

    /** Cartesian damping (normalized) 
      * #FRI_CMD_CARTDAMP
      * Layout [Dx, Dy, Dz, Da(Rz), Db(Ry), Dc(Rx)]*/
    fri_float_t cartDamping[FRI_CART_VEC];
} tFriRobotCommand;
#define FRI_ROBOT_COMMAND_SIZE (59*4)



/** Symbolic define to identify #tFriCmdData 
  Note: This datagram id will be abused for versioning as well */
#define FRI_DATAGRAM_ID_CMD 0x1005
/** The commanding structure, which is sent to the KRC Side.
	Identified by #FRI_DATAGRAM_ID_CMD */
typedef struct
{
    /** the header */
    tFriHeader head;

    /** data to KRL */
    tFriKrlData krl;

    /** robot commands */
    tFriRobotCommand cmd;
} tFriCmdData;
#define FRI_CMD_DATA_SIZE (FRI_HEADER_SIZE + FRI_KRL_DATA_SIZE + FRI_ROBOT_COMMAND_SIZE)




/** a convenience macro for the user to check if padding on the used
    platform is as expected: if (!FRI_CHECK_SIZES_OK) ... */
#define FRI_CHECK_SIZES_OK \
    ((sizeof(tFriHeader)         == FRI_HEADER_SIZE) && \
     (sizeof(tFriKrlData)        == FRI_KRL_DATA_SIZE) && \
     (sizeof(tFriIntfStatistics) == FRI_INTF_STATISTICS_SIZE) && \
     (sizeof(tFriIntfState)      == FRI_INTF_STATE_SIZE) && \
     (sizeof(tFriRobotState)     == FRI_ROBOT_STATE_SIZE) && \
     (sizeof(tFriRobotData)      == FRI_ROBOT_DATA_SIZE) && \
     (sizeof(tFriRobotCommand)   == FRI_ROBOT_COMMAND_SIZE) && \
     (sizeof(tFriMsrData)        == FRI_MSR_DATA_SIZE) && \
     (sizeof(tFriCmdData)        == FRI_CMD_DATA_SIZE))




/** convenience macros for the user to check if byte order and float
    representation on the used platform is as expected:
    {
        FRI_PREPARE_CHECK_BYTE_ORDER
        if (!FRI_CHECK_BYTE_ORDER_OK) ...
    }*/
#define FRI_PREPARE_CHECK_BYTE_ORDER \
    union {fri_uint32_t a; fri_uint8_t b[4]; fri_float_t c;} _friTestByteOrder;   \
    _friTestByteOrder.a = 0x40490FDB;
	/** convenience macros for the user to check if byte order and float
		representation on the used platform is as expected:
		{
			FRI_PREPARE_CHECK_BYTE_ORDER
			if (!FRI_CHECK_BYTE_ORDER_OK) ...
		}*/	
#define FRI_CHECK_BYTE_ORDER_OK \
    ((_friTestByteOrder.a == 0x40490FDB) &&\
     (_friTestByteOrder.b[0] == 0xDB) &&\
     (_friTestByteOrder.b[1] == 0x0F) &&\
     (_friTestByteOrder.b[2] == 0x49) &&\
     (_friTestByteOrder.b[3] == 0x40) &&\
     (_friTestByteOrder.c > 3.141592) &&\
     (_friTestByteOrder.c < 3.141593))


/**! Versioning Major Version */   
#define FRI_MAJOR_VERSION 1
/**! Versioning Minor sub version for minor fixes */   
#define FRI_SUB_VERSION 0
#define EXPAND_CONCAT(X1,X2,X3,X4,X5,X6,X7) X1 ## X2 ## X3 ## X4 ## X5 ## X6 ## X7
#define CONCAT(name1,name2,name3,name4) EXPAND_CONCAT(#name1,".",#name2,".",#name3,".",#name4)

/**! The Versionstring with all significant information */   
#define FRI_VERSION_STRING CONCAT(FRI_MAJOR_VERSION, FRI_SUB_VERSION,FRI_DATAGRAM_ID_CMD,FRI_DATAGRAM_ID_MSR)
//"." #F "." #FRI_DATAGRAM_ID_MSR

#endif  /* FRICOMM_H */
