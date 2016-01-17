/** Copyright (C) 2015 Salvatore Virga - salvo.virga@tum.de
 * Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
 * 
 * LICENSE :
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * \author Salvatore Virga
 * \version 2.0.0
 * \date 26/10/2015
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
        argv[0] = "iiwaRos";
        ros::init(argc, argv, "iiwaRos");
    }
    
    ros::NodeHandle node_handle;
        
    robot_is_connected_ = false;
    
    holder_state_pose.init("command/CartesianPose");
    holder_state_joint_position.init("command/JointPosition");
    holder_state_joint_torque.init("command/JointTorque");
//     holder_state_wrench.init("command/CartesianWrench");
//     holder_state_joint_stiffness.init("command/JointStiffness");
    
    holder_command_pose.init("state/CartesianPose");
    holder_command_joint_position.init("state/JointPosition");
//     holder_command_wrench.init("state/CartesianWrench");
//     holder_command_joint_stiffness.init("state/JointStiffness");
//     holder_command_joint_torque.init("state/JointTorque");
}



iiwaRos::~iiwaRos() { }

bool iiwaRos::getRobotIsConnected() {
    static int counter = 0;
    if (!(counter++ % 1000)) {
        bool is_connected = (ros::Time::now() - last_update_time) < ros::Duration(0.1);
        if (is_connected != robot_is_connected_)
            if (is_connected)
                ROS_INFO("IIWA robot is connected.");
            else 
                ROS_WARN("IIWA robot is NOT connected.");
        robot_is_connected_ = is_connected;
    }
    return robot_is_connected_;
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
