/**  
 * Copyright (C) 2016 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
 * Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * \author Salvatore Virga
 * \version 2.0.0
 * \date 07/03/2016
 */

/** 
 * THIS IS JUST AN EXEMPLARY MAIN
 * */

#include "iiwaRos.h"

#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <stdexcept>

// Handles quit commands.
bool g_quit = false;

void quitRequested(int sig) {
    g_quit = true;
}

int main( int argc, char** argv ) {
    // Initialize ROS.
    ros::init(argc, argv, "iiwa_ros_example", ros::init_options::NoSigintHandler);
    
    // ROS spinner.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // Custom signal handlers.
    signal(SIGTERM, quitRequested);
    signal(SIGINT, quitRequested);
    signal(SIGHUP, quitRequested);
    
    // The iiwa_msgs to send and receive joint position of the robot.
    iiwa_msgs::JointPosition receiveMessage, sendMessage;
    
    // The IIWA - ROS interface.
    iiwaRos iiwa_ros;
    iiwa_ros.init(false);
    
    // Rate at which you want to send and receive messages.
    ros::Rate* loop_rate_ = new ros::Rate(1000);
    
    // BUILD YOUR MESSAGE TO SEND OR GET IT FROM SOMEWHERE ELSE
    // A GOOD IDEA IS TO HAVE ANOTHER FUNCTION THAT DOES THAT
    
    // reintroduce if necessary
    //   sendMessage.position[0] = 0.0;
    //   sendMessage.position[1] = 0.0;
    //   sendMessage.position[2] = 0.0;
    //   sendMessage.position[3] = 0.0;
    //   sendMessage.position[4] = 0.0;
    //   sendMessage.position[5] = 0.0;
    //   sendMessage.position[6] = 0.0;
    
    while( !g_quit ) {
        // Check if the iiwa Robot is connected and sending messages.
        if (iiwa_ros.getRobotIsConnected()) {
            
            // Check if a new JointPosition message is available
            if (iiwa_ros.getReceivedJointPosition(receiveMessage)) {
            // You received a new IIWA state message, do something with it.
            }
            
        }
        
        // Send command position to the robot.
        if (iiwa_ros.getRobotIsConnected()) {
            
            iiwa_ros.setCommandJointPosition(sendMessage);
            iiwa_ros.publish();
            // Your message was sent to the robot.
        }
        
        // Wait for 1 millisecond.
        loop_rate_->sleep();
        
    }
    
    std::cerr<<"Stopping spinner..."<<std::endl;
    spinner.stop();
    
    std::cerr<<"Bye!"<<std::endl;
    
    return 0;
}