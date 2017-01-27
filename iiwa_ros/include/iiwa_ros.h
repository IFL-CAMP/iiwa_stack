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
    void init();
    
    /**
     * Getters
     */
    bool getReceivedCartesianPose(geometry_msgs::PoseStamped& value);
    bool getReceivedJointPosition(iiwa_msgs::JointPosition& value);
    bool getReceivedJointTorque(iiwa_msgs::JointTorque& value);
    
    bool getReceivedJointStiffness(iiwa_msgs::JointStiffness& value);
    bool getReceivedCartesianWrench(geometry_msgs::WrenchStamped& value);
    bool getReceivedJointVelocity(iiwa_msgs::JointVelocity& value);
    
	bool getReceivedJointPositionVelocity(iiwa_msgs::JointPositionVelocity& value);
	bool getReceivedJointDamping(iiwa_msgs::JointDamping& value);
	bool getReceivedDestinationReached(std_msgs::Time& value);
	

	void setCommandCartesianPose(const geometry_msgs::PoseStamped& position);
    void setCommandJointPosition(const iiwa_msgs::JointPosition& position);
    
	void setCommandJointVelocity(const iiwa_msgs::JointVelocity& velocity);
	void setCommandJointPositionVelocity(const iiwa_msgs::JointPositionVelocity& value);
  
	 
	 
    /**
     * \brief Sends new commands to the connected IIWA robot, if any
     */
    bool publish();
    
    /**
     * \brief Returns the current connection status of an IIWA robot.
     */
    bool getRobotIsConnected();
	
	

	
private:
    iiwaStateHolder<geometry_msgs::PoseStamped> holder_state_pose;
    iiwaStateHolder<iiwa_msgs::JointPosition> holder_state_joint_position;
    iiwaStateHolder<iiwa_msgs::JointTorque> holder_state_joint_torque;
    iiwaStateHolder<geometry_msgs::WrenchStamped> holder_state_wrench;
    iiwaStateHolder<iiwa_msgs::JointDamping> holder_state_joint_damping;
    iiwaStateHolder<iiwa_msgs::JointStiffness> holder_state_joint_stiffness;
	iiwaStateHolder<iiwa_msgs::JointVelocity> holder_state_joint_velocity;
	iiwaStateHolder<iiwa_msgs::JointPositionVelocity> holder_state_joint_position_velocity;
	iiwaStateHolder<std_msgs::Time> holder_state_destination_reached;
    
    iiwaCommandHolder<geometry_msgs::PoseStamped> holder_command_pose;
    iiwaCommandHolder<iiwa_msgs::JointPosition> holder_command_joint_position;
	iiwaCommandHolder<iiwa_msgs::JointVelocity> holder_command_joint_velocity;
	iiwaCommandHolder<iiwa_msgs::JointPositionVelocity> holder_command_joint_position_velocity;
    

    bool robot_is_connected_; /**< Stores the current connection state */
};