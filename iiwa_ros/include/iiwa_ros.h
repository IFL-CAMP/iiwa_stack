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

#pragma once

#include <iiwa_msgs/CartesianVelocity.h>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointStiffness.h>
#include <iiwa_msgs/JointTorque.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/JointPositionVelocity.h>
#include <iiwa_msgs/JointDamping.h>
#include <std_msgs//Time.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <smart_servo_service.h>
#include <path_parameters_service.h>
#include <time_to_destination_service.h>
#include <ros/ros.h>

#include <string>
#include <mutex>

namespace iiwa_ros {
  
  extern ros::Time last_update_time;
  
  template <typename ROSMSG>
  class iiwaHolder {
  public:
    iiwaHolder() : is_new(false) {}
    
    void set_value(const ROSMSG& value) {
      mutex.lock();
      data = value;
      is_new = true;
      mutex.unlock();
    }
    
    bool get_value(ROSMSG& value) {
      bool was_new = false;
      
      mutex.lock();
      value = data;
      was_new = is_new;
      is_new = false;
      mutex.unlock();
      
      return was_new;
    }
    
    bool has_new_value() {
      return is_new;
    }
    
    ROSMSG get_value_unsynchronized() {
      return data;
    }
    
  private:
    ROSMSG data;
    bool is_new;
    std::mutex mutex;
  };
  
  template <typename ROSMSG>
  class iiwaStateHolder {
  public:
    void init(const std::string& topic) {
      ros::NodeHandle nh;
      subscriber = nh.subscribe<ROSMSG>(topic, 1, &iiwaStateHolder<ROSMSG>::set, this);
    }
    
    bool has_new_value() {
      return holder.has_new_value();
    }
    
    void set(ROSMSG value) {
      last_update_time = ros::Time::now();
      holder.set_value(value);
    }
    
    bool get(ROSMSG& value) {
      return holder.get_value(value);        
    }
  private:
    iiwaHolder<ROSMSG> holder;
    ros::Subscriber subscriber;
  };
  
  
  template <typename ROSMSG>
  class iiwaCommandHolder {
  public:
    void init(const std::string& topic) {
      ros::NodeHandle nh;
      publisher = nh.advertise<ROSMSG>(topic, 1);
    }
    
    void set(const ROSMSG& value) {
      holder.set_value(value);
    }
    
    ROSMSG get() {
      return holder.get_value_unsynchronized();
    }
    
    void publishIfNew() {
      static ROSMSG msg;
      if (publisher.getNumSubscribers() && holder.get_value(msg))
        publisher.publish(msg);
    }
  private:
    ros::Publisher publisher;
    iiwaHolder<ROSMSG> holder;
  };
  
  class iiwaRos {
  public:
    
    /**
     * @brief Constructor for class iiwaRos holding all the methods to command and get the state of the robot.
     */
    iiwaRos();
    
    /**
     * @brief Initializes the necessary topics for state and command methods.
     * 
     * @return void
     */
    void init();
    
    /**
     * @brief Returns true is a new Cartesian pose of the robot is available.
     * 
     * @param value the current Cartesian Pose of the robot.
     * @return bool
     */
    bool getCartesianPose(geometry_msgs::PoseStamped& value);
    
    /**
     * @brief Returns true is a new Joint position of the robot is available.
     * 
     * @param value the current joint position of the robot.
     * @return bool
     */
    bool getJointPosition(iiwa_msgs::JointPosition& value);
    
    /**
     * @brief Returns true is a new Joint torque of the robot is available.
     * 
     * @param value the current joint torque of the robot.
     * @return bool
     */
    bool getJointTorque(iiwa_msgs::JointTorque& value);
    
    /**
     * @brief Returns true is a new Joint stiffness of the robot is available.
     * 
     * @param value the current joint stiffness of the robot.
     * @return bool
     */
    bool getJointStiffness(iiwa_msgs::JointStiffness& value);
    
    /**
     * @brief Returns true is a new Cartesian wrench of the robot is available.
     * 
     * @param value the current cartesian wrench of the robot.
     * @return bool
     */
    bool getCartesianWrench(geometry_msgs::WrenchStamped& value);
    
    /**
     * @brief Returns true is a new Joint velocity of the robot is available.
     * 
     * @param value the current joint velocity of the robot.
     * @return bool
     */
    bool getJointVelocity(iiwa_msgs::JointVelocity& value);
    
    /**
     * @brief Returns true is a new Joint position velocity of the robot is available.
     * 
     * @param value the current joint position velocity of the robot.
     * @return bool
     */
    bool getJointPositionVelocity(iiwa_msgs::JointPositionVelocity& value);
    
    /**
     * @brief Returns true is a new Joint damping of the robot is available.
     * 
     * @param value the current joint damping of the robot.
     * @return bool
     */
    bool getJointDamping(iiwa_msgs::JointDamping& value);
    
    /**
     * @brief Returns the object that allows to call the configureSmartServo service.
     * 
     * @return iiwa_ros::SmartServoService
     */
    SmartServoService getSmartServoService() { return smart_servo_service_; }
    
    /**
     * @brief Returns the object that allows to call the timeToDestination service.
     * 
     * @return iiwa_ros::PathParametersService
     */
    PathParametersService getPathParametersService() { return path_parameters_service_; }
    
    /**
     * @brief Returns the object that allows to call the setPathParameters service.
     * 
     * @return iiwa_ros::TimeToDestinationService
     */
    TimeToDestinationService getTimeToDestinationService() { return time_to_destination_service_; };
    
    /**
     * @brief Set the cartesian pose of the robot.
     * 
     * @param position the cartesian pose to set the robot.
     * @return void
     */
    void setCartesianPose(const geometry_msgs::PoseStamped& position);
    
    /**
     * @brief Set the joint position of the robot.
     * 
     * @param position the joint position to set the robot.
     * @return void
     */
    void setJointPosition(const iiwa_msgs::JointPosition& position);
    
    /**
     * @brief Set the joint velocity of the robot.
     * 
     * @param velocity the joint velocity to set the robot.
     * @return void
     */
    void setJointVelocity(const iiwa_msgs::JointVelocity& velocity);
    
    /**
     * @brief Set the joint position velocity of the robot.
     * 
     * @param value the joint position velocity of the robot.
     * @return void
     */
    void setJointPositionVelocity(const iiwa_msgs::JointPositionVelocity& value);
    
    /**
     * \brief Returns the current connection status of an IIWA robot.
     */
    bool getRobotIsConnected();
    
  private:
    iiwaStateHolder<geometry_msgs::PoseStamped> holder_state_pose_;
    iiwaStateHolder<iiwa_msgs::JointPosition> holder_state_joint_position_;
    iiwaStateHolder<iiwa_msgs::JointTorque> holder_state_joint_torque_;
    iiwaStateHolder<geometry_msgs::WrenchStamped> holder_state_wrench_;
    iiwaStateHolder<iiwa_msgs::JointDamping> holder_state_joint_damping_;
    iiwaStateHolder<iiwa_msgs::JointStiffness> holder_state_joint_stiffness_;
    iiwaStateHolder<iiwa_msgs::JointVelocity> holder_state_joint_velocity_;
    iiwaStateHolder<iiwa_msgs::JointPositionVelocity> holder_state_joint_position_velocity_;
    iiwaStateHolder<std_msgs::Time> holder_state_destination_reached_;
    
    iiwaCommandHolder<geometry_msgs::PoseStamped> holder_command_pose_;
    iiwaCommandHolder<iiwa_msgs::JointPosition> holder_command_joint_position_;
    iiwaCommandHolder<iiwa_msgs::JointVelocity> holder_command_joint_velocity_;
    iiwaCommandHolder<iiwa_msgs::JointPositionVelocity> holder_command_joint_position_velocity_;
    
    SmartServoService smart_servo_service_;
    PathParametersService path_parameters_service_;
    TimeToDestinationService time_to_destination_service_;
  };
  
}
