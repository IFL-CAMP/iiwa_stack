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

#include "signal.h"
#include <QThread>
#include "IIWARobot.h"

using namespace std;

bool b_firstRun = true;

IIWA::IIWAMsg goalState;

class IIWAController: public IIWARobot{
public:
    void Controller(const IIWA::IIWAMsg &currentState, IIWA::IIWAMsg &commandState);
};

void IIWAController::Controller(const IIWA::IIWAMsg &currentState, IIWA::IIWAMsg &commandState){
    //write your controller here
    cout << currentState.cartPosition[0] << " "
         << currentState.cartPosition[1] << " "
         << currentState.cartPosition[2] << endl;

    //write your desired command in the variable commandState.
    //You do not need to fill all the variables, just set the ones that you want to change now.
    //Important: Don't FORGET to resize them first.

    if (b_firstRun)
    {
        goalState.cartPosition.resize(3);
        goalState.cartPosition = currentState.cartPosition;

        goalState.cartOrientation.resize(9);
        goalState.cartOrientation = currentState.cartOrientation;

        goalState.cartPositionStiffness.resize(3);
        goalState.cartPositionStiffness.at(0) = 50.0;
        goalState.cartPositionStiffness.at(1) = 50.0;
        goalState.cartPositionStiffness.at(2) = 500.0;

        b_firstRun = false;
    }

    //You need to set this flag to define the control type
    commandState.isJointControl = false;

    //The following is a basic gravity compensation controller
    commandState.cartPosition.resize(3);
    commandState.cartPosition = goalState.cartPosition;

    commandState.cartOrientation.resize(9);
    commandState.cartOrientation = goalState.cartOrientation;

    commandState.cartPositionStiffness.resize(3);
    commandState.cartPositionStiffness = goalState.cartPositionStiffness;
}


void sighandle(int signal)
{
    exit(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iiwa_core");

    IIWAController myIIWAController;

    signal(SIGINT, sighandle);
    signal(SIGQUIT, sighandle);

    //control loop is here
    while(true)
    {
        myIIWAController.RobotUpdate();
    }

    return 0;
}


