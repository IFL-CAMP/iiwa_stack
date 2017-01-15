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

#include "smart_servo_service.h"
#include "TimeToDestinationService.h"
#include "PathParametersService.h"
#include "ros/ros.h"

int main( int argc, char** argv ) {
    // Initialize ROS.
    ros::init(argc, argv, "iiwa_ros_example", ros::init_options::NoSigintHandler);
    
    // ROS spinner.
    ros::AsyncSpinner spinner(1);
    spinner.start();
        
    // Rate at which you want to send and receive messages.
    ros::Rate* loop_rate_ = new ros::Rate(60);
    
    iiwa_ros::SmartServoService sss;
    iiwa_ros::PathParametersService pps;
    iiwa_ros::TimeToDestinationService tds;
    
    while( !g_quit ) {
              
        // Wait for 1 millisecond.
        loop_rate_->sleep();
        
    }
    
    std::cerr<<"Stopping spinner..."<<std::endl;
    spinner.stop();
    
    std::cerr<<"Bye!"<<std::endl;
    
    return 0;
}