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

#include "iiwa_msgs/CartesianRotation.h"
#include "iiwa_msgs/CartesianVelocity.h"
#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointStiffness.h"
#include "iiwa_msgs/JointTorque.h"
#include "iiwa_msgs/JointVelocity.h"

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <boost/thread.hpp>

#include <iostream>
#include <string>
#include <mutex>
#include <time.h>

template <typename ROSMSG>
class iiwaStateHolder;

template <typename ROSMSG>
class iiwaCommandHolder;

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
    void init(bool initRos = false);
    
    /**
     * Getters
     */
    geometry_msgs::PoseStamped getReceivedCartesianPosition();
    iiwa_msgs::CartesianRotation getReceivedCartesianRotation();
    iiwa_msgs::CartesianVelocity getReceivedCartesianVelocity();
    geometry_msgs::WrenchStamped getReceivedCartesianWrench();
    
    iiwa_msgs::JointPosition getReceivedJointPosition();
    iiwa_msgs::JointStiffness getReceivedJointStiffness();
    iiwa_msgs::JointTorque getReceivedJointTorque();
    iiwa_msgs::JointVelocity getReceivedJointVelocity();
    
    geometry_msgs::PoseStamped getCommandCartesianPosition();
    iiwa_msgs::CartesianRotation getCommandCartesianRotation();
    iiwa_msgs::CartesianVelocity getCommandCartesianVelocity();
    geometry_msgs::WrenchStamped getCommandCartesianWrench();
    
    iiwa_msgs::JointPosition getCommandJointPosition();
    iiwa_msgs::JointStiffness getCommandJointStiffness();
    iiwa_msgs::JointTorque getCommandJointTorque();
    iiwa_msgs::JointVelocity getCommandJointVelocity();
    
    /*
     * 
     */
    void setCommandCartesianPosition(const geometry_msgs::PoseStamped& position);
    void setCommandCartesianRotation(const iiwa_msgs::CartesianRotation& rotation);
    void setCommandCartesianVelocity(const iiwa_msgs::CartesianVelocity& velocity);
    void setCommandCartesianWrench(const geometry_msgs::WrenchStamped& wrench);
    
    void setCommandJointPosition(const iiwa_msgs::JointPosition& position);
    void setCommandJointStiffness(const iiwa_msgs::JointStiffness& stiffness);
    void setCommandJointTorque(const iiwa_msgs::JointTorque& torque);
    void setCommandJointVelocity(const iiwa_msgs::JointVelocity& velocity);
    
    /**
     * \brief Sends new commands to the connected IIWA robot, if any
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
    
    
    void robotHasConnected();
    bool robot_is_connected_; /**< Stores the current connection state */
};

template <typename ROSMSG>
class iiwaHolder {
public:
    void set_value(const ROSMSG& value) {
        mutex.lock();
        data = value;
        is_new = true;
        mutex.unlock();
    }
    
    bool get_value(ROSMSG& value) {
        bool will_be_new = false;
        
        mutex.lock();
        value = data;
        std::swap(will_be_new, is_new); // we put false into is_new since we just read it 
        mutex.unlock();
        
        return will_be_new; // return the old value of is_new
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
    iiwaStateHolder<ROSMSG>(const std::string& topic) {
        ros::NodeHandle nh;
        subscriber = nh.subscribe(topic, 1, iiwaStateHolder< ROSMSG >::set, this);
    }
    
    void set(const ROSMSG& value) {
        holder.set_value(value);
    }
    
    bool get(ROSMSG& value) {
        return holder.get_value(value);        
    }
private:
    iiwaHolder<ROSMSG> holder;
    ros::Subscriber<ROSMSG> subscriber;
};


template <typename ROSMSG>
class iiwaCommandHolder {
public:
    iiwaCommandHolder<ROSMSG>(const std::string& topic) {
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
    ros::Publisher<ROSMSG> publisher;
    iiwaHolder<ROSMSG> holder;
};

#endif //IIWAROCONNS_H_