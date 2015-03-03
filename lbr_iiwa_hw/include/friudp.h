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
  friUdp.h
       
      NOTE: This sample, as the corresponding FRI (Fast Research inteface) is subject to radical change
	  
      
[FH]}}
*/ 
/**	\addtogroup friRemoteLib	\brief Library for FRI (FastResearchInterface) *//* @{ */
/** *************************************************************************
 \author (Guenter Schreiber)
\file
\brief header for udp Communications
 *                                                                         *
 ***************************************************************************/
#ifndef FRIFRIUDP_H
#define FRIFRIUDP_H

#ifdef VXWORKS
#else
#define HAVE_GETHOSTNAME
#endif

#include <stdio.h>
#include <stdlib.h>

#ifdef _MSC_VER
#include <winsock2.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <string.h>
#ifndef _WRS_KERNEL
#include <sys/time.h>
#endif
#include <time.h>
#endif
#include "friComm.h"

#include <iostream>

#ifdef VXWORKS // VxWorks Kernel

#include <sockLib.h>
#endif


#define FRI_DEFAULT_SERVER_PORT 49938

#ifdef QNX
#define HAVE_TIME_STAMP_RECEIVE
#endif


#ifdef HAVE_TIME_STAMP_RECEIVE
/** Receive Timestamp -- mechanism works under 
QNX, Linux?

Not under Windows - and under VxWorks
*/

typedef struct
{
    tFriMsrData load;
    /* add a received-timestamp */
    double timestamp;
} tFriMsrPacket;

#endif

/**
FRI Remote Sample Implementation

	@author Günter Schreiber <guenter@jacobus>
*/
class friUdp{
public:
    friUdp(int port=FRI_DEFAULT_SERVER_PORT, char *remoteHost = NULL);

    ~friUdp();

protected:
	/// Note: Remote host need not to be specified - if NULL, wait for the 
	/// incoming packages
  void Init(char * remoteHost=NULL);
  void Close(void);
#ifdef WIN32
	int StartWinsock(void);
#endif
public:
  int Send(tFriCmdData *data);
  int Recv(tFriMsrData *packet);
private:
int RecvPacket(int fd, tFriMsrData* p, struct timeval* ts, struct sockaddr_in* client);
 int udpSock ;
  int serverPort;
  struct sockaddr_in krcAddr;
  /// if timestamp on receive is available, last received value can be inquired here
  double m_timestamp;
public:
	/// This feature will be available only for systems, which support 
	/// SO_TIMESTAMP in the socket options, e.g. qnx
	double	getLastTimestamp() { return m_timestamp; }
};




inline std::ostream & operator<<(std::ostream &out , tFriHeader & head)
{
  out << "sendSeqCount " << head.sendSeqCount << "\n";
  out << "reflSeqCount " << head.reflSeqCount << "\n";
  out << "packetSize   " << head.packetSize   << "\n";
  out << "datagramId   " << std::hex  << head.datagramId << std::dec  ;
  switch (head.datagramId )
  {
  case FRI_DATAGRAM_ID_CMD:
	  out << " FRI_DATAGRAM_ID_CMD \n" ;
	  break;
  case FRI_DATAGRAM_ID_MSR:
	  out << " FRI_DATAGRAM_ID_MSR \n" ;
	  break;
  default:
	  out <<" Unkown \n";
  }
  return out;
}
inline std::ostream & operator<<(std::ostream &out , tFriKrlData& krl)
{
	out << "krl_real ";
  for ( int i = 0; i < FRI_USER_SIZE; i++)
  out << " " << krl.realData[i];
  out << "\n";
  	out << "krl_int ";
  for ( int i = 0; i < FRI_USER_SIZE; i++)
  out << " " << krl.intData[i];
  out << "\n";
  	out << "krl_bool ";
  out << std::hex << krl.boolData << std::dec << "\n";
  return out;
}

inline std::ostream & operator<<(std::ostream &out , tFriIntfStatistics & stat)
{
  out << "answerRate  " << stat.answerRate << "\n";
  out << "latency     " << stat.latency << "\n";
  out << "jitter      " << stat.jitter << "\n";
  out << "missRate    " << stat.missRate << "\n";
  out << "missCounter " << stat.missCounter << "\n";
  return out;
}

inline std::ostream & operator<<(std::ostream &out , tFriIntfState & state)
{
  out << "timestamp " << state.timestamp<< "\n";
  out << "state     " << state.state<< "\n";
  out << "quality   " << state.quality << "\n";
  out << "desiredMsrSampleTime " << state.desiredMsrSampleTime << "\n";
  out << "desiredCmdSampleTiintfme " << state.desiredCmdSampleTime  << "\n";
  out << "safetyLimits " << state.safetyLimits << "\n";
  out << "statistics " << state.stat << "\n";
return out;
}

inline std::ostream & operator<< (std::ostream & out, tFriRobotState & robot)
{
  out << "power   " << robot.power<< "\n";
  out << "control " << robot.control << "\n";
  out << "error   " << robot.error << "\n";
  out << "warning " << robot.warning << "\n";
  out << "temperature " ;
  for (int i = 0; i < LBR_MNJ; i++)
    out << robot.temperature[i] << " " ;
  out << "\n";
  return out;

}

#define WRITE_JNT_VEC(field) \
  out << ""#field; \
  for ( int i  = 0; i < LBR_MNJ; i++) \
  out << " " << robot.field[i]; \
  out << "\n";
inline std::ostream & operator<<(std::ostream &out, tFriRobotData & robot)
{
  WRITE_JNT_VEC(msrJntPos);
  WRITE_JNT_VEC(cmdJntPos);
  WRITE_JNT_VEC(msrJntTrq);
  WRITE_JNT_VEC(estExtJntTrq);
    return out;

}

inline std::ostream & operator<<(std::ostream &out, tFriRobotCommand & robot)
{
  out << std::hex << robot.cmdFlags << std::dec << "\n";
  WRITE_JNT_VEC(jntPos);
    return out;

}


inline std::ostream & operator<<(std::ostream &out, tFriMsrData & msr)
{
  out << "head " << msr.head;
  out << "krl " << msr.krl;
  out << "intf " << msr.intf;
  out << "robot " << msr.robot;
  out << "data " << msr.data;
  return out;
}




inline std::ostream & operator<<(std::ostream &out, tFriCmdData & cmd)
{
  out << "head " << cmd.head;
  out << "krl " << cmd.krl;
  out << "cmd " << cmd.cmd;
    return out;
}




#endif
/* @} */
