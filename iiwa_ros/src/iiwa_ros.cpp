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
 */

#include "iiwa_ros.h"

using namespace std;

ros::Time last_update_time;

iiwaRos::iiwaRos() { }

void iiwaRos::init()
{
	robot_is_connected_ = false;
	
	holder_state_pose_.init("state/CartesianPose");
	holder_state_joint_position_.init("state/JointPosition");
	holder_state_joint_torque_.init("state/JointTorque");
	holder_state_wrench_.init("state/CartesianWrench");
	holder_state_joint_stiffness_.init("state/JointStiffness");
	holder_state_joint_position_velocity_.init("state/JointPositionVelocity");
	holder_state_joint_damping_.init("state/JointDamping");
	holder_state_joint_velocity_.init("state/JointVelocity");
	holder_state_destination_reached_.init("state/DestinationReached");
	
	holder_command_pose_.init("command/CartesianPose");
	holder_command_joint_position_.init("command/JointPosition");
	holder_command_joint_position_velocity_.init("/command/JointPositionVelocity");
	holder_command_joint_velocity_.init("command/JointVelocity");
}

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

bool iiwaRos::getRobotIsConnected(bool verbose) {
	ros::Duration diff = (ros::Time::now() - last_update_time);
	if (verbose) {
		ROS_ERROR_STREAM("getRobotIsConnect - The time difference is: " << diff.toSec() << " It is checked against: " << ros::Duration(0.1).toSec() );
	}
	return (diff < ros::Duration(0.1));
}

bool iiwaRos::getReceivedCartesianPose(geometry_msgs::PoseStamped& value) {
	return holder_state_pose_.get(value);
}
bool iiwaRos::getReceivedJointPosition(iiwa_msgs::JointPosition& value) {
	return holder_state_joint_position_.get(value);
}
bool iiwaRos::getReceivedJointTorque(iiwa_msgs::JointTorque& value) {
	return holder_state_joint_torque_.get(value);
}
bool iiwaRos::getReceivedJointStiffness(iiwa_msgs::JointStiffness& value) {
	return holder_state_joint_stiffness_.get(value);
}
bool iiwaRos::getReceivedCartesianWrench(geometry_msgs::WrenchStamped& value) {
	return holder_state_wrench_.get(value);
}
bool iiwaRos::getReceivedJointVelocity(iiwa_msgs::JointVelocity& value) {
	return holder_state_joint_velocity_.get(value);
}
bool iiwaRos::getReceivedJointPositionVelocity(iiwa_msgs::JointPositionVelocity& value) {
	return holder_state_joint_position_velocity_.get(value);
}
bool iiwaRos::getReceivedJointDamping(iiwa_msgs::JointDamping& value) {
	return holder_state_joint_damping_.get(value);
}
bool iiwaRos::getReceivedDestinationReached(std_msgs::Time& value) {
	return holder_state_destination_reached_.get(value);
}

void iiwaRos::setCommandCartesianPose(const geometry_msgs::PoseStamped& position) {
	holder_command_pose_.set(position);
	holder_command_pose_.publishIfNew();
}
void iiwaRos::setCommandJointPosition(const iiwa_msgs::JointPosition& position)  {
	holder_command_joint_position_.set(position);
	holder_command_joint_position_.publishIfNew();
}
void iiwaRos::setCommandJointVelocity(const iiwa_msgs::JointVelocity& velocity) {
	holder_command_joint_velocity_.set(velocity);
	holder_command_joint_velocity_.publishIfNew();
	
}
void iiwaRos::setCommandJointPositionVelocity(const iiwa_msgs::JointPositionVelocity& value) {
	holder_command_joint_position_velocity_.set(value);
	holder_command_joint_position_velocity_.publishIfNew();
	
}