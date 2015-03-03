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
  friRemote.cpp
       
      NOTE: This sample, as the corresponding FRI (Fast Research inteface) is subject to radical change
	  
      
[FH]}}
*/ 
/**
	\addtogroup friRemoteLib
	\brief Library for FRI (FastResearchInterface) 
*/
/* @{ */

/** *************************************************************************
 \author (Guenter Schreiber)
\file
\brief  FRI Remote class encapsulating UDP package handshakes
 *                                                                         *
 ***************************************************************************/
#include "friremote.h"

#include <iostream>
#include <stdlib.h>

friRemote::friRemote(int port,char *hintToRemoteHost):remote(port,hintToRemoteHost)
{
	std::cout << __FUNCTION__ << " " <<port <<std::endl;
	std::cout << "FRI Version " << FRI_MAJOR_VERSION << "." << FRI_SUB_VERSION << "." <<FRI_DATAGRAM_ID_CMD << "." <<FRI_DATAGRAM_ID_MSR << std::endl;
	memset((void*)(& msr),0x0,sizeof(msr));
	memset((void*)(& cmd), 0x0,sizeof(cmd));
	{
		// do checks, whether the interface - and the host meets the requirements
		FRI_PREPARE_CHECK_BYTE_ORDER;
		if (!FRI_CHECK_BYTE_ORDER_OK) 
		{
			std::cerr << __FUNCTION__<<"Byte order on your system is not appropriate - expect deep trouble" <<std::endl;
		}
		if (!FRI_CHECK_SIZES_OK)
		{
			std::cout << __FUNCTION__<<"Sizes of datastructures not appropriate - expect even deeper trouble" << std::endl;

		}
	}
}

friRemote::~friRemote()
{
	std::cout << __FUNCTION__ << " bye for now "  <<std::endl;
}

int friRemote::doReceiveData()
{
  return remote.Recv(&msr);
}


/** Data Exchanger -- normally update within access routine implicitely ... */
int friRemote::doSendData()
{
  
  // received at least something 
   seqCount++;
   cmd.head.sendSeqCount = seqCount;
   cmd.head.reflSeqCount = msr.head.sendSeqCount;
   cmd.head.datagramId = FRI_DATAGRAM_ID_CMD;
   cmd.head.packetSize = sizeof(tFriCmdData);
    int rc=remote.Send(&cmd);
  return rc;
}

	/** send commands based on last msr datagram and 
		receives a new one
	*/

int friRemote::doDataExchange()
{
	doSendData();
	return doReceiveData();
}
  /** automatically do data exchange, if not otherwise specified */
  int friRemote::doPositionControl(float newJntPosition[LBR_MNJ], bool flagDataExchange)

{
	// Helper, if not properly initialized or the like...

	cmd.cmd.cmdFlags=FRI_CMD_JNTPOS;
	// Note:: If the interface is not in Command mode,
	// The commands have to be "mirrored" to get in sync
	// Note:: If the interface is not in Command mode,
	// The commands have to be "mirrored" to get in sync
	if ((getState() != FRI_STATE_CMD) || (!isPowerOn()))
	{
		for (int i = 0; i < LBR_MNJ; i++)
		{
			cmd.cmd.jntPos[i]=msr.data.cmdJntPos[i]+msr.data.cmdJntPosFriOffset[i];
		}
	}
	else
	{
		// compute new values here ...
		for (int i = 0; i < LBR_MNJ; i++)
			cmd.cmd.jntPos[i]=newJntPosition[i];
	}

	if (flagDataExchange)
	{
		return doDataExchange();
	}
	return 1;
}


/** automatically do data exchange, if not otherwise specified 
 if flagDataExchange is set to false, call doDataExchange() 
 or doReceiveData()/doSendData() on your own
 IN: newJntPosition   - joint positions
	 newJntStiff      - joint stiffness (Spring factor)
	 newJntDamp       - joint damping   (Damping factor)
	 newJntAddTorque  - additional torque 
       
 Note: If any of the pointers (newJntPosition, newJntStiff, newJntDamp, newJntAddTorque) is NULL, the 
	   value is ignored -> the respective  cmd.cmd.cmdFlags field is set properly
 Note: It is possible to change cmd.cmd.cmdFlags in monitor mode only !!
 */
int friRemote::doJntImpedanceControl(const float newJntPosition[LBR_MNJ], 
										const float newJntStiff[LBR_MNJ], 
										const float newJntDamp[LBR_MNJ], 
										const float newJntAddTorque[LBR_MNJ],
										bool flagDataExchange)

{
	// Helper, if not properly initialized or the like...
	cmd.cmd.cmdFlags=0;
	if (newJntPosition)
	{
		cmd.cmd.cmdFlags|=FRI_CMD_JNTPOS;
		// Note:: If the interface is not in Command mode,
		// The commands have to be "mirrored" to get in sync
		if ((getState() != FRI_STATE_CMD) || (!isPowerOn()))
		{
			for (int i = 0; i < LBR_MNJ; i++)
			{
				cmd.cmd.jntPos[i]=msr.data.cmdJntPos[i]+msr.data.cmdJntPosFriOffset[i];
			}
		}
		else
		{
			// compute new values here ...
			for (int i = 0; i < LBR_MNJ; i++)
				cmd.cmd.jntPos[i]=newJntPosition[i];
		}
	}
	if (newJntStiff)
	{
		cmd.cmd.cmdFlags|=FRI_CMD_JNTSTIFF;
		for (int i = 0; i < LBR_MNJ; i++)
			cmd.cmd.jntStiffness[i]=newJntStiff[i];
	}
	if (newJntDamp)
	{
		cmd.cmd.cmdFlags|=FRI_CMD_JNTDAMP;
		for (int i = 0; i < LBR_MNJ; i++)
			cmd.cmd.jntDamping[i]=newJntDamp[i];
	}
	if (newJntAddTorque)
	{
		cmd.cmd.cmdFlags|=FRI_CMD_JNTTRQ;
		for (int i = 0; i < LBR_MNJ; i++)
			cmd.cmd.addJntTrq[i]=newJntAddTorque[i];
	}

	if (flagDataExchange)
	{
		return doDataExchange();
	}
	return 1;
}

	/** automatically do data exchange, if not otherwise specified 
	if flagDataExchange is set to false, call doDataExchange() 
	or doReceiveData()/doSendData() on your own
	IN: newJntPosition   - joint positions
	newJntStiff      - joint stiffness (Spring factor)
	newJntDamp       - joint damping   (Damping factor)
	newJntAddTorque  - additional torque 

	Note: If any of the pointers (newJntPosition, newJntStiff, newJntDamp, newJntAddTorque) is NULL, the 
	value is ignored -> the respective  cmd.cmd.cmdFlags field is set properly
	Note: It is possible to change cmd.cmd.cmdFlags in monitor mode only !!
	*/
int friRemote::doCartesianImpedanceControl(const float newCartPosition[FRI_CART_FRM_DIM], 
										   const float newCartStiff[FRI_CART_VEC], 
										   const float newCartDamp[FRI_CART_VEC], 
										   const float newAddTcpFT[FRI_CART_VEC],
										   const float newJntNullspace[LBR_MNJ],  
										   bool flagDataExchange)
{

		// Helper, if not properly initialized or the like...
	cmd.cmd.cmdFlags=0;
	if ( newCartPosition )
	{
		cmd.cmd.cmdFlags|=FRI_CMD_CARTPOS;
		for ( int i = 0; i < FRI_CART_FRM_DIM; i++)
		{
			cmd.cmd.cartPos[i]=newCartPosition[i];

		}
	}
	if ( newCartStiff)
	{
		cmd.cmd.cmdFlags|=FRI_CMD_CARTSTIFF;
		for ( int i = 0; i < FRI_CART_VEC; i++)
		{
			cmd.cmd.cartStiffness[i]=newCartStiff[i];

		}

	}
	if ( newCartDamp)
	{
		cmd.cmd.cmdFlags|=FRI_CMD_CARTDAMP;
			;
		for ( int i = 0; i < FRI_CART_VEC; i++)
		{
			cmd.cmd.cartDamping[i]=newCartDamp[i];

		}
	}
	if ( newAddTcpFT)
	{
		cmd.cmd.cmdFlags|=FRI_CMD_TCPFT;
			;
		for ( int i = 0; i < FRI_CART_VEC; i++)
		{
			cmd.cmd.addTcpFT[i]=newAddTcpFT[i];

		}
	}

	if (newJntNullspace)
	{
		cmd.cmd.cmdFlags|=FRI_CMD_JNTPOS;
		// Note:: If the interface is not in Command mode,
		// The commands have to be "mirrored" to get in sync
		
			// compute new values here ...
			for (int i = 0; i < LBR_MNJ; i++)
				cmd.cmd.jntPos[i]=newJntNullspace[i];		
	}


if (flagDataExchange)
	{
		return doDataExchange();
	}
	return 1;
}






/* @} */

