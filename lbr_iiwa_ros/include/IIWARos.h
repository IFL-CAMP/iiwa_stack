/** (c) 2015 Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
 *
 * This class implements a communication with a KUKA LBR IIWA Robot via ROS.
 * The communication is performed on the topics :
 *	/iiwa/command 	- to control the robot position,
 *	/iiwa/state	- to monitor the robot position,
 * and is based on the ROS message created by the module lbr_iiwa_msg.
 * In order for the communication to be functional, the robot has to
 * communicate over the same topics with the same message type.
 * 
 * \author Salvatore Virga
 * \version 1.0.0
 * \date 13/03/2015
 * 
 * This class is based on an implementation by Mohammad Khansari 
 * from Stanford University. Although the code base has been refactored, 
 * the code code is subject to the following copyright statement:
 * 
/**************************************************************************
 ***   Copyright (c) 2014 S. Mohammad Khansari, Stanford Robotics Lab,   ***
 ***                      Stanford University, USA                       ***
 ***************************************************************************
 *
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Stanford University nor the name of the author may
#       be used to endorse or promote products derived from this software without
#       specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY MOHAMMAD KHANSARI ''AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL MOHAMMAD KHANSARI BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * To get latest upadate of the software please visit:
 *                          http://cs.stanford.edu/people/khansari
 *
 * Please send your feedbacks or questions to:
 *                           khansari_at_cs.stanford.edu
 ***************************************************************************/

#ifndef IIWAROS_H_
#define IIWAROS_H_

#include "boost/thread.hpp"
#include "time.h"

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include "IIWA/IIWAMsg.h"
#include <iostream>

#define IIWA_DOF_JOINTS 7

// Topics
#define IIWA_LISTEN "/iiwa/command"
#define IIWA_TALK   "/iiwa/state"

class IIWARos
{
public:
	/**
	 * Class constructor
	 */
	IIWARos();
	
	/**
	 * Class destructor
	 */
	virtual ~IIWARos();
	
	/**
	 * \brief Obtains the current state (Cartesian/Joint Position and Impedance)  of the connected IIWA robot.
	 * \param receiveMessage the IIWAMsg object that will containe the robot status.
	 */
	bool read(IIWA::IIWAMsg& receiveMessage);
	
	/**
	 * \brief Sends a control state to the connected IIWA robot
	 * \param sendMessage the IIWAMsg object that will be sent. 
	 */
	bool write(IIWA::IIWAMsg sendMessage);
	
	/**
	 * \brief Initialize the size of an IIWAMsg object
	 * \param msg the IIWAMsg object to initialize. 
	 */
	void resizeIIWAMessage(IIWA::IIWAMsg& msg);
	
	/**
	 * \brief Copy the values of an IIWAMSg object to another
	 * \param msg_copy the IIWAMsg object to copy from.
	 * \param msg_paste the IIWAMsg object to copy in.
	 */
	static void copyIIWAMessage(const IIWA::IIWAMsg& msg_copy, IIWA::IIWAMsg& msg_paste);
	
	/**
	 * Callback for the ROS Subscriber
	 */
	static void iiwaSubscriberCallback(const IIWA::IIWAMsg& msg);
	
	/**
	 * \brief Returns the current connection status of an IIWA robot.
	 */
	static bool getRobotIsConnected();

private:
	
	ros::Publisher iiwa_pub_;  /**< The ROS Publisher  */
	ros::Subscriber iiwa_sub_; /**< The ROS Subscriber */

	static IIWA::IIWAMsg from_robot_IIWA_message_; /**< Stores the IIWAMsg received directly from the robot. */
	static bool robot_is_connected_; /**< Stores the current connection state */
};

#endif //IIWAROS_H_
