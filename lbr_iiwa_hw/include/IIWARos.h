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
	IIWARos();
	virtual ~IIWARos();

	bool read(IIWA::IIWAMsg& receiveMessage);
	bool write(IIWA::IIWAMsg sendMessage);

	void resizeIIWAMessage(IIWA::IIWAMsg& msg);
	static void copyIIWAMessage(const IIWA::IIWAMsg& msg_copy, IIWA::IIWAMsg& msg_paste);
	static void iiwaSubscriberCallback(const IIWA::IIWAMsg& msg);
	static bool getRobotIsConnected();

private:
	ros::Publisher iiwa_pub;
	ros::Subscriber iiwa_sub;

	IIWA::IIWAMsg currentIIWAStateMessage;
	static bool robotIsConnected;
};

#endif //IIWAROS_H_
