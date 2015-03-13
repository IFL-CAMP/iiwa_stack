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

#include "IIWARos.h"

using namespace std;

bool IIWARos::robot_is_connected_ = false;
IIWA::IIWAMsg IIWARos::from_robot_IIWA_message_;

IIWARos::IIWARos(){
	ros::NodeHandle nh;

	// Initialize/Resize the IIWAMsg that will be used
	resizeIIWAMessage(from_robot_IIWA_message_);

	// Starts publisher and subscriber
	iiwa_pub_ = nh.advertise<IIWA::IIWAMsg>(IIWA_LISTEN, 1);
	iiwa_sub_ = nh.subscribe(IIWA_TALK, 1, IIWARos::iiwaSubscriberCallback);
}

IIWARos::~IIWARos()
{}

void IIWARos::resizeIIWAMessage(IIWA::IIWAMsg& msg){
	msg.cartPosition.resize(3);
	msg.cartForces.resize(3);
	msg.cartPositionStiffness.resize(3);

	msg.cartOrientation.resize(9);
	msg.cartMoments.resize(3);
	msg.cartOrientationStiffness.resize(3);

	msg.jointAngles.resize(IIWA_DOF_JOINTS);
	msg.jointTorques.resize(IIWA_DOF_JOINTS);
	msg.jointStiffness.resize(IIWA_DOF_JOINTS);
}

// Subscriber callback function that updates the current joint state
void IIWARos::iiwaSubscriberCallback(const IIWA::IIWAMsg& msg)
{
	copyIIWAMessage(msg, from_robot_IIWA_message_);

	if (!robot_is_connected_){
		cout << "IIWA robot is connected." << endl;
		robot_is_connected_ = true;
	}
}

void IIWARos::copyIIWAMessage(const IIWA::IIWAMsg& msg_copy, IIWA::IIWAMsg& msg_paste){
	for(int i = 0; i < 9; i++)
	{
		//reading orientation
		msg_paste.cartOrientation[i] = msg_copy.cartOrientation[i];

		if (i<3){
			//reading cartesian position
			msg_paste.cartPosition[i] = msg_copy.cartPosition[i];

			//reading force
			msg_paste.cartForces[i] = msg_copy.cartForces[i];

			//reading position stiffness
			msg_paste.cartPositionStiffness[i] = msg_copy.cartPositionStiffness[i];

			//reading moment
			msg_paste.cartMoments[i] = msg_copy.cartMoments[i];

			//reading orientation stiffness
			msg_paste.cartOrientationStiffness[i] = msg_copy.cartOrientationStiffness[i];

			//reading the tcp force
			msg_paste.cartForces[i] = msg_copy.cartForces[i];
		}

		//reading joint-related values
		if (i<IIWA_DOF_JOINTS){
			msg_paste.jointAngles[i] = msg_copy.jointAngles[i];
			msg_paste.jointTorques[i] = msg_copy.jointTorques[i];
			msg_paste.jointStiffness[i] = msg_copy.jointStiffness[i];
		}
	}
}

bool IIWARos::getRobotIsConnected() {
	return robot_is_connected_;
}

bool IIWARos::read(IIWA::IIWAMsg& incoming_message) {
	if (robot_is_connected_) {
		copyIIWAMessage(from_robot_IIWA_message_, incoming_message);
		return 1;
	}
	return 0;
}

bool IIWARos::write(IIWA::IIWAMsg send_message) {
	if (robot_is_connected_)
	{
		iiwa_pub_.publish(send_message);
		return 1;
	}
	return 0;
}
