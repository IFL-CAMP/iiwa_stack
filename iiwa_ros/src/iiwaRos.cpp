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

#include "iiwaRos.h"

using namespace std;

ros::Time last_update_time;

iiwaRos::iiwaRos() { }

void iiwaRos::init(bool initRos)
{
    // hide ROS away for non-ROS applications
    if (initRos) {
        int argc = 1;
        char *argv[1];
        sprintf(argv[1],"iiwaRos");
        ros::init(argc, argv, "iiwaRos");
    }

    ros::NodeHandle node_handle;

    robot_is_connected_ = false;

    holder_state_pose.init("state/CartesianPose");
    holder_state_joint_position.init("state/JointPosition");
    holder_state_joint_torque.init("state/JointTorque");
//     holder_state_wrench.init("state/CartesianWrench");
//     holder_state_joint_stiffness.init("state/JointStiffness");

    holder_command_pose.init("command/CartesianPose");
    holder_command_joint_position.init("command/JointPosition");
//     holder_command_wrench.init("command/CartesianWrench");
//     holder_command_joint_stiffness.init("command/JointStiffness");
//     holder_command_joint_torque.init("command/JointTorque");
}



iiwaRos::~iiwaRos() { }

// bool iiwaRos::getRobotIsConnected() {
//     static int counter = 0;
//     if (!(counter++ % 1000)) {
//         bool is_connected = (ros::Time::now() - last_update_time) < ros::Duration(0.1);
//         if (is_connected != robot_is_connected_)
//             if (is_connected)
//                 ROS_INFO("IIWA robot is connected.");
//             else
//                 ROS_WARN("IIWA robot is NOT connected.");
//         robot_is_connected_ = is_connected;
//     }
//     return robot_is_connected_;
// }

bool iiwaRos::getRobotIsConnected() {
    return (ros::Time::now() - last_update_time) < ros::Duration(0.1);
}


bool iiwaRos::getReceivedCartesianPose(geometry_msgs::PoseStamped& value) {
    return holder_state_pose.get(value);
}

bool iiwaRos::getReceivedJointPosition(iiwa_msgs::JointPosition& value) {
    return holder_state_joint_position.get(value);
}

bool iiwaRos::getReceivedJointTorque(iiwa_msgs::JointTorque& value) {
    return holder_state_joint_torque.get(value);
}

geometry_msgs::PoseStamped iiwaRos::getCommandCartesianPose() {
    return holder_command_pose.get();
}

iiwa_msgs::JointPosition iiwaRos::getCommandJointPosition(){
    return holder_command_joint_position.get();
}
// geometry_msgs::WrenchStamped iiwaRos::getCommandCartesianWrench(){
//     return command_cartesian_wrench_;
// }
// iiwa_msgs::JointStiffness iiwaRos::getCommandJointStiffness(){
//     return command_joint_stiffness_;
// }
// iiwa_msgs::JointTorque iiwaRos::getCommandJointTorque(){
//     return command_joint_torque_;
// }

/*
 * Setters for command messages - set the message that you want to send
 */
void iiwaRos::setCommandCartesianPose(const geometry_msgs::PoseStamped& position) {
    holder_command_pose.set(position);
}
void iiwaRos::setCommandJointPosition(const iiwa_msgs::JointPosition& position)  {
    holder_command_joint_position.set(position);
}
// void iiwaRos::setCommandCartesianWrench(const geometry_msgs::WrenchStamped& wrench) {
//     command_cartesian_wrench_ = wrench;
// }
//
// void iiwaRos::setCommandJointStiffness(const iiwa_msgs::JointStiffness& stiffness)  {
//     command_joint_stiffness_ = stiffness;
// }
// void iiwaRos::setCommandJointTorque(const iiwa_msgs::JointTorque& torque)  {
//     command_joint_torque_ = torque;
// }
// void iiwaRos::setCommandJointVelocity(const iiwa_msgs::JointVelocity& velocity)  {
//     command_joint_velocity_ = velocity;
// }

bool iiwaRos::publish() {
    holder_command_pose.publishIfNew();
    holder_command_joint_position.publishIfNew();
}
