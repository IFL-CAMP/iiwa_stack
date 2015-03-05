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

#include "IIWARobot.h"

using namespace std;

IIWA::IIWAMsg          receivedIIWAMessage;

bool b_robotIsConnected = false;

IIWARobot::IIWARobot(){
    ros::NodeHandle nh;

    // resize aspects of currentJointState
    ResizeIIWAMessage(receivedIIWAMessage);
    ResizeIIWAMessage(currentIIWAStateMessage);

    iiwa_pub = nh.advertise<IIWA::IIWAMsg>(IIWA_LISTEN, 1);
    iiwa_sub = nh.subscribe(IIWA_TALK, 1, IIWARobot::IIWASubscriberCallback);

    loop_rate = new ros::Rate(IIWACONTROLFREQUENCY);
}

IIWARobot::~IIWARobot()
{}

void IIWARobot::ResizeIIWAMessage(IIWA::IIWAMsg &msg){
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

long int cnt = 0;

bool IIWARobot::RobotUpdate(ros::Duration period){

    // Calculate loop time;
    controlFrequency = 1000000000.0/myClock.nsecsElapsed();
    myClock.restart();

    //reading the force/torque values
    if (b_robotIsConnected)
    {
        CopyIIWAMessage(receivedIIWAMessage, currentIIWAStateMessage);

        //This part is for sending command to the robot, we start by an empty message
        IIWA::IIWAMsg  sendMessage;

        //doing your stuff
        Controller(currentIIWAStateMessage, sendMessage, period);

        iiwa_pub.publish(sendMessage);
    }else{
        if (cnt%1000 == 0)
            cout << "waiting for the IIWA robot to connect ..." << endl;
    }

    cnt++;

    ros::spinOnce();
    loop_rate->sleep();
    return true;
}

/*
  Subscriber callback function that updates the current joint state
*/
void IIWARobot::IIWASubscriberCallback(const IIWA::IIWAMsg& msg)
{
    CopyIIWAMessage(msg, receivedIIWAMessage);

    if (!b_robotIsConnected){
        cout << "IIWA robot is connected." << endl;
        b_robotIsConnected = true;
    }
}

void IIWARobot::CopyIIWAMessage(const IIWA::IIWAMsg &msg_copy, IIWA::IIWAMsg &msg_paste){
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
