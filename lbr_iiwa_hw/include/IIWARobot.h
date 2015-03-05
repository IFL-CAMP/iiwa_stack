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

#ifndef IIWAROBOT_H_
#define IIWAROBOT_H_

#include <QElapsedTimer>
#include "IIWARobot.h"

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

#ifndef RAD
#define RAD(A)	((A) * M_PI / 180.0 )
#endif

#ifndef DEG
#define DEG(A)	((A) * 180.0 / M_PI )
#endif

#define IIWACONTROLFREQUENCY 800 // Hz

class IIWARobot
{
public:
    IIWARobot();
   ~IIWARobot();
    virtual void    Controller(const IIWA::IIWAMsg &currentState, IIWA::IIWAMsg &desiredState, ros::Duration period) = 0;

    bool            RobotUpdate(ros::Duration period);

    void            ResizeIIWAMessage(IIWA::IIWAMsg &msg);

    static void     CopyIIWAMessage(const IIWA::IIWAMsg &msg_copy, IIWA::IIWAMsg &msg_paste);

    static void     IIWASubscriberCallback(const IIWA::IIWAMsg& msg);

protected:
    char static_txt[1025];

    double                      controlFrequency;

    QElapsedTimer               myClock;

    ros::Publisher              iiwa_pub;

    ros::Subscriber             iiwa_sub;

    IIWA::IIWAMsg               currentIIWAStateMessage;

    ros::Rate                  *loop_rate;
};



#endif //IIWAROBOT_H_
