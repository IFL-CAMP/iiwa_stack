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

#ifndef IIWAROS_H_
#define IIWAROS_H_

#include "boost/thread.hpp"
#include "time.h"

#include "ros/ros.h"

#include "iiwa_msgs/CartesianPosition.h"
#include "iiwa_msgs/CartesianRotation.h"
#include "iiwa_msgs/CartesianVelocity.h"
#include "iiwa_msgs/CartesianWrench.h"
#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointStiffness.h"
#include "iiwa_msgs/JointTorque.h"
#include "iiwa_msgs/JointVelocity.h"

#include <iostream>
#include <string>
#include <mutex>

#define IIWA_JOINTS 7

class iiwaRos {
public:
  /**
   * Class constructor
   */
  iiwaRos();
  
  /**
   * Class destructor
   */
  virtual ~iiwaRos();
  
  /** 
   * Init
   */
  void init(bool initRos = false, std::string iiwaName = "iiwa");
  
  /**
   * Getters
   */
  iiwa_msgs::CartesianPosition getReceivedCartesianPosition();
  iiwa_msgs::CartesianRotation getReceivedCartesianRotation();
  iiwa_msgs::CartesianVelocity getReceivedCartesianVelocity();
  iiwa_msgs::CartesianWrench getReceivedCartesianWrench();
  
  iiwa_msgs::JointPosition getReceivedJointPosition();
  iiwa_msgs::JointStiffness getReceivedJointStiffness();
  iiwa_msgs::JointTorque getReceivedJointTorque();
  iiwa_msgs::JointVelocity getReceivedJointVelocity();
  
  iiwa_msgs::CartesianPosition getCommandCartesianPosition();
  iiwa_msgs::CartesianRotation getCommandCartesianRotation();
  iiwa_msgs::CartesianVelocity getCommandCartesianVelocity();
  iiwa_msgs::CartesianWrench getCommandCartesianWrench();
  
  iiwa_msgs::JointPosition getCommandJointPosition();
  iiwa_msgs::JointStiffness getCommandJointStiffness();
  iiwa_msgs::JointTorque getCommandJointTorque();
  iiwa_msgs::JointVelocity getCommandJointVelocity();
  
  /*
   * 
   */
  void setCommandCartesianPosition(const iiwa_msgs::CartesianPosition& position);
  void setCommandCartesianRotation(const iiwa_msgs::CartesianRotation& rotation);
  void setCommandCartesianVelocity(const iiwa_msgs::CartesianVelocity& velocity);
  void setCommandCartesianWrench(const iiwa_msgs::CartesianWrench& wrench);
  
  void setCommandJointPosition(const iiwa_msgs::JointPosition& position);
  void setCommandJointStiffness(const iiwa_msgs::JointStiffness& stiffness);
  void setCommandJointTorque(const iiwa_msgs::JointTorque& torque);
  void setCommandJointVelocity(const iiwa_msgs::JointVelocity& velocity);
  
  /**
   * \brief Sends a control state to the connected IIWA robot 
   */
  bool publish();
  
  
  /**
   * TODO
   * \brief
   */
  bool isCartesianPositionAvailable();
  bool isCartesianRotationAvailable();
  bool isCartesianVelocityAvailable();
  bool isCartesianWrenchAvailable();
  bool isJointPositionAvailable();
  bool isJointStiffnessAvailable();
  bool isJointTorqueAvailable();
  bool isJointVelocityAvailable();
  
  
  /**
   * \brief Returns the current connection status of an IIWA robot.
   */
  bool getRobotIsConnected();
  
private:
  /**
   * Callback for the ROS Subscribers
   */	
  void cartesianPositionCallback(const iiwa_msgs::CartesianPosition& position);
  void cartesianRotationCallback(const iiwa_msgs::CartesianRotation& rotation);
  void cartesianVelocityCallback(const iiwa_msgs::CartesianVelocity& velocity);
  void cartesianWrenchCallback(const iiwa_msgs::CartesianWrench& wrench);
  
  void jointPositionCallback(const iiwa_msgs::JointPosition& position);
  void jointStiffnessCallback(const iiwa_msgs::JointStiffness& stiffness);
  void jointTorqueCallback(const iiwa_msgs::JointTorque& torque);
  void jointVelocityCallback(const iiwa_msgs::JointVelocity& velocity);
  
  template <typename T> 
  bool publishIfSubscriber(const ros::Publisher& p, const T& message);
  
  void robotConnected();
  
  /**< ROS Publishers  */
  ros::Publisher cartesian_position_pub_;  
  ros::Publisher cartesian_rotation_pub_;
  ros::Publisher cartesian_velocity_pub_;
  ros::Publisher cartesian_wrench_pub_;
  
  ros::Publisher joint_position_pub_;
  ros::Publisher joint_stiffness_pub_;
  ros::Publisher joint_torque_pub_;
  ros::Publisher joint_velocity_pub_;
  
  /**< ROS Subscribers */
  ros::Subscriber cartesian_position_sub_;
  ros::Subscriber cartesian_rotation_sub_;
  ros::Subscriber cartesian_velocity_sub_;
  ros::Subscriber cartesian_wrench_sub_;
  
  ros::Subscriber joint_position_sub_;
  ros::Subscriber joint_stiffness_sub_;
  ros::Subscriber joint_torque_sub_;
  ros::Subscriber joint_velocity_sub_;
  
  /*
   * Messages received from the robot
   */
  iiwa_msgs::CartesianPosition received_cartesian_position_;
  iiwa_msgs::CartesianRotation received_cartesian_rotation_;
  iiwa_msgs::CartesianVelocity received_cartesian_velocity_;
  iiwa_msgs::CartesianWrench received_cartesian_wrench_;
  iiwa_msgs::JointPosition received_joint_position_;
  iiwa_msgs::JointStiffness received_joint_stiffness_;
  iiwa_msgs::JointTorque received_joint_torque_;
  iiwa_msgs::JointVelocity received_joint_velocity_;
  
  /*
   * Messages that will be send to the robot
   */
  iiwa_msgs::CartesianPosition command_cartesian_position_;
  iiwa_msgs::CartesianRotation command_cartesian_rotation_;
  iiwa_msgs::CartesianVelocity command_cartesian_velocity_;
  iiwa_msgs::CartesianWrench command_cartesian_wrench_;
  iiwa_msgs::JointPosition command_joint_position_;
  iiwa_msgs::JointStiffness command_joint_stiffness_;
  iiwa_msgs::JointTorque command_joint_torque_;
  iiwa_msgs::JointVelocity command_joint_velocity_;
  
  bool new_cartesian_position_;
  bool new_cartesian_rotation_;
  bool new_cartesian_velocity_;
  bool new_cartesian_wrench_;
  bool new_joint_position_;
  bool new_joint_stiffness_;
  bool new_joint_torque_;
  bool new_joint_velocity_;
  
  bool robot_is_connected_; /**< Stores the current connection state */
  
  std::mutex cp_mutex_;
  std::mutex cr_mutex_;
  std::mutex cv_mutex_;
  std::mutex cw_mutex_;
  std::mutex jp_mutex_;
  std::mutex js_mutex_;
  std::mutex jt_mutex_;
  std::mutex jv_mutex_;
};

#endif //IIWAROCONNS_H_