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
#include <ros/ros.h>

#include <iostream>
#include <string>
#include <mutex>
#include <time.h>

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
	 * @brief Constructor for class iiwaRos holding all the methods to command and get states of the robot.
	 */
	iiwaRos();
        
	/**
	 * @brief Initializes the necessary topic strings for the states.
	 * 
	 * @return void
	 */
	void init();
    
	/**
	 * @brief Method to get the cartesian pose of the robot and additionally returns true if it is a new value.
	 * 
	 * @param value Reference to the variable where the pose is written to.
	 * @return bool
	 */
	bool getReceivedCartesianPose(geometry_msgs::PoseStamped& value);
	
	/**
	 * @brief Method to get the joint position of the robot and additionally returns true if it is a new value.
	 * 
	 * @param value Reference to the variable where the state is written to.
	 * @return bool
	 */
	bool getReceivedJointPosition(iiwa_msgs::JointPosition& value);
	
	/**
	 * @brief Method to get the joint torque of the robot and additionally returns true if it is a new value.
	 * 
	 * @param value Reference to the variable where the state is written to.
	 * @return bool
	 */
	bool getReceivedJointTorque(iiwa_msgs::JointTorque& value);
    
	/**
	 * @brief Method to get the joint stiffness of the robot and additionally returns true if it is a new value.
	 * 
	 * @param value Reference to the variable where the state is written to.
	 * @return bool
	 */
	bool getReceivedJointStiffness(iiwa_msgs::JointStiffness& value);
	
	/**
	 * @brief Method to get the cartesian wrench of the robot and additionally returns true if it is a new value.
	 * 
	 * @param value Reference to the variable where the state is written to.
	 * @return bool
	 */
	bool getReceivedCartesianWrench(geometry_msgs::WrenchStamped& value);
	
	/**
	 * @brief Method to get the joint velocity of the robot and additionally returns true if it is a new value.
	 * 
	 * @param value Reference to the variable where the state is written to.
	 * @return bool
	 */
	bool getReceivedJointVelocity(iiwa_msgs::JointVelocity& value);
    
	/**
	 * @brief Method to get the joint position velocity of the robot and additionally returns true if it is a new value.
	 * 
	 * @param value Reference to the variable where the state is written to.
	 * @return bool
	 */
	bool getReceivedJointPositionVelocity(iiwa_msgs::JointPositionVelocity& value);
	
	/**
	 * @brief Method to get the joint damping of the robot and additionally returns true if it is a new value.
	 * 
	 * @param value Reference to the variable where the state is written to.
	 * @return bool
	 */
	bool getReceivedJointDamping(iiwa_msgs::JointDamping& value);
	
	/**
	 * @brief Method to get the time of destination reaching of the robot and additionally returns true if it is a new value.
	 * 
	 * @param value Reference to the variable where the state is written to.
	 * @return bool
	 */
	bool getReceivedDestinationReached(std_msgs::Time& value);
	

	/**
	 * @brief Method to set the cartesian pose of the robot.
	 * 
	 * @param position Reference to the variable with the state.
	 * @return void
	 */
	void setCommandCartesianPose(const geometry_msgs::PoseStamped& position);
	
	/**
	 * @brief Method to set the joint position of the robot.
	 * 
	 * @param position Reference to the variable with the state.
	 * @return void
	 */
	void setCommandJointPosition(const iiwa_msgs::JointPosition& position);
    
	/**
	 * @brief Method to set the joint velocity of the robot.
	 * 
	 * @param velocity Reference to the variable with the state.
	 * @return void
	 */
	void setCommandJointVelocity(const iiwa_msgs::JointVelocity& velocity);
	
	/**
	 * @brief Method to set the joint position velocity of the robot.
	 * 
	 * @param value Reference to the variable with the state.
	 * @return void
	 */
	void setCommandJointPositionVelocity(const iiwa_msgs::JointPositionVelocity& value);
      
    /**
     * \brief Returns the current connection status of an IIWA robot.
     */
    bool getRobotIsConnected(bool verbose = false);
	
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
    
    bool robot_is_connected_; /**< Stores the current connection state */
};