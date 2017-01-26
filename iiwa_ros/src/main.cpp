/**
 * Copyright (C) 2016-2017 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
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
 */

#include "smart_servo_service.h"
#include "time_to_destination_service.h"
#include "path_parameters_service.h"
#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/DOF.h"
#include "ros/ros.h"
#include <iostream>
#include <chrono>
#include <thread>

int main( int argc, char** argv ) {
    // Initialize ROS.
    ros::init(argc, argv, "iiwa_ros_example", ros::init_options::NoSigintHandler);
    
    ros::NodeHandle nh;
    
    // ROS spinner.
    ros::AsyncSpinner spinner(1);
    spinner.start();
        
    // Rate at which you want to send and receive messages.
    ros::Rate* loop_rate_ = new ros::Rate(0.2);
    
    iiwa_ros::SmartServoService sss("/iiwa/configuration/configureSmartServo", false);
    iiwa_ros::PathParametersService pps("/iiwa/configuration/pathParameters", false);
    iiwa_ros::TimeToDestinationService tds("/iiwa/state/timeToDestination", false);
    
    ros::Publisher pub_joint_command = nh.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1);
    
    iiwa_msgs::JointPosition jp1;
    jp1.position.a1 = -4.19424424081e-07;
    jp1.position.a2 = 0.397833317518;    
    jp1.position.a3 = -3.94280796172e-05;
    jp1.position.a4 = -0.5131264925;
    jp1.position.a5 = 4.93750158057e-05;
    jp1.position.a6 = 0.688018083572;
    jp1.position.a7 = -2.42680398514e-05;

    iiwa_msgs::JointPosition jp2;
    jp2.position.a1 = -0.117264322937;
    jp2.position.a2 = 0.39688038826;    
    jp2.position.a3 = 0.0718542337418;
    jp2.position.a4 = -1.48047995567;
    jp2.position.a5 = -0.0143581563607;
    jp2.position.a6 = 1.32106602192;
    jp2.position.a7 = 0.00994055531919;

    while( ros::ok() ) {
      loop_rate_->sleep();
      sss.setSinePatternmode(3, 1.0, 2.0, 250.0);
      loop_rate_->sleep();
      loop_rate_->sleep();
      sss.setPositionControlMode();
      loop_rate_->sleep();
    }
    
    std::cerr<<"Stopping spinner..."<<std::endl;
    spinner.stop();
    
    std::cerr<<"Bye!"<<std::endl;
    
    return 0;
}