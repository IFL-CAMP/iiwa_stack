/** Copyright (C) 2015 Salvatore Virga - salvo.virga@tum.de
 * Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
 *
 * This class implements a bridge between ROS hardware interfaces and a KUKA LBR IIWA Robot,
 * using an IIWARos communication described in the iiwa_ros package.
 *
 * It is a porting of the work from the Centro E. Piaggio in Pisa : https://github.com/CentroEPiaggio/kuka-lwr
 * for the LBR IIWA. We acknowledge the good work of their main contributors :
 * Carlos J. Rosales - cjrosales@gmail.com
 * Enrico Corvaglia
 * Marco Esposito - marco.esposito@tum.de
 * Manuel Bonilla - josemanuelbonilla@gmail.com
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

#include "iiwa_hw.h"

using namespace std;

IIWA_HW::IIWA_HW(ros::NodeHandle nh) {
    nh_ = nh;
    
    timer_ = ros::Time::now();
    control_frequency_ = DEFAULT_CONTROL_FREQUENCY;
    loop_rate_ = new ros::Rate(control_frequency_);
    
    interface_type_.push_back("PositionJointInterface");
    interface_type_.push_back("EffortJointInterface");
    interface_type_.push_back("VelocityJointInterface");
}

IIWA_HW::~IIWA_HW() {}

ros::Rate* IIWA_HW::getRate() {
    return loop_rate_;
}

double IIWA_HW::getFrequency() {
    return control_frequency_;
}

void IIWA_HW::setFrequency(double frequency) {
    control_frequency_ = frequency;
    loop_rate_ = new ros::Rate(control_frequency_);
}

bool IIWA_HW::start() {
    
    // construct a new IIWA device (interface and state storage)
    device_.reset( new IIWA_HW::IIWA_device() );
    
    // TODO : make use of this
    // get inteface param or give default values
    nh_.param("interface", interface_, std::string("PositionJointInterface"));
    nh_.param("robot_name", robot_name_, std::string("iiwa"));
    
    /* TODO
     * nh_.param("move_group", movegroup_name_, "arm");
     * group(movegroup_name_);
     */
    
    // TODO: use transmission configuration to get names directly from the URDF model
    if ( ros::param::get("joints", device_->joint_names) ) {
        if ( !(device_->joint_names.size() == IIWA_JOINTS) ) {
            ROS_ERROR("This robot has 7 joints, you must specify 7 names for each one");
        }
    } else {
        ROS_ERROR("No joints to be handled, ensure you load a yaml file naming the joint names this hardware interface refers to.");
        throw std::runtime_error("No joint name specification");
    }
    
    std::stringstream ss;
    ss << "/" << robot_name_ << "/robot_description";
    std::string robot_description = ss.str();
    
    if (!(urdf_model_.initParam(robot_description))) {
        ROS_ERROR("No URDF model in the robot_description parameter, this is required to define the joint limits.");
        throw std::runtime_error("No URDF model available");
    }
    
    iiwa_ros_conn_.init(false, robot_name_);
    
    // initialize and set to zero the state and command values
    device_->init();
    device_->reset();
    
    // general joint to store information
    boost::shared_ptr<const urdf::Joint> joint;
    
    // create joint handles given the list
    for(int i = 0; i < IIWA_JOINTS; ++i) {
        ROS_INFO_STREAM("Handling joint: " << device_->joint_names[i]);
        
        // get current joint configuration
        joint = urdf_model_.getJoint(device_->joint_names[i]);
        if(!joint.get()) {
            ROS_ERROR_STREAM("The specified joint "<< device_->joint_names[i] << " can't be found in the URDF model. "
            "Check that you loaded an URDF model in the robot description, or that you spelled correctly the joint name.");
            throw std::runtime_error("Wrong joint name specification");
        }
        
        // joint state handle
        hardware_interface::JointStateHandle state_handle(device_->joint_names[i],
                                                          &(device_->joint_position[i]),
                                                          &(device_->joint_velocity[i]),
                                                          &(device_->joint_effort[i]));
        
        state_interface_.registerHandle(state_handle);
        
        // position command handle
        hardware_interface::JointHandle position_joint_handle = hardware_interface::JointHandle(
            state_interface_.getHandle(device_->joint_names[i]),
                                                                                                &device_->joint_position_command[i]);
        
        position_interface_.registerHandle(position_joint_handle);
        
        // effort command handle
        hardware_interface::JointHandle joint_handle = hardware_interface::JointHandle(
            state_interface_.getHandle(device_->joint_names[i]),
                                                                                       &device_->joint_effort_command[i]);
        
        effort_interface_.registerHandle(joint_handle);
        
        registerJointLimits(device_->joint_names[i],
                            joint_handle,
                            &urdf_model_,
                            &device_->joint_lower_limits[i],
                            &device_->joint_upper_limits[i],
                            &device_->joint_effort_limits[i]);
    }
    
    ROS_INFO("Register state and effort interfaces");
    
    // TODO: CHECK
    // register ros-controls interfaces
    this->registerInterface(&state_interface_);
    this->registerInterface(&effort_interface_);
    this->registerInterface(&position_interface_);
    
    return true;
}

void IIWA_HW::registerJointLimits(const std::string& joint_name,
                                  const hardware_interface::JointHandle& joint_handle,
                                  const urdf::Model *const urdf_model,
                                  double *const lower_limit, double *const upper_limit,
                                  double *const effort_limit) {
    
    *lower_limit = -std::numeric_limits<double>::max();
    *upper_limit = std::numeric_limits<double>::max();
    *effort_limit = std::numeric_limits<double>::max();
    
    joint_limits_interface::JointLimits limits;
    bool has_limits = false;
    joint_limits_interface::SoftJointLimits soft_limits;
    bool has_soft_limits = false;
    
    if (urdf_model != NULL) {
        const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);
        
        if (urdf_joint != NULL) {
            // Get limits from the URDF file.
            if (joint_limits_interface::getJointLimits(urdf_joint, limits))
                has_limits = true;
            
            if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
                has_soft_limits = true;
        }
    }
    
    if (!has_limits)
        return;
    
    if (limits.has_position_limits) {
        *lower_limit = limits.min_position;
        *upper_limit = limits.max_position;
    }
    
    if (limits.has_effort_limits)
        *effort_limit = limits.max_effort;
    
    if (has_soft_limits) {
        const joint_limits_interface::EffortJointSoftLimitsHandle limits_handle(joint_handle, limits, soft_limits);
        ej_limits_interface_.registerHandle(limits_handle);
    } else {
        const joint_limits_interface::EffortJointSaturationHandle sat_handle(joint_handle, limits);
        ej_sat_interface_.registerHandle(sat_handle);
    }
}

bool IIWA_HW::read(ros::Duration period)
{
    ros::Duration delta = ros::Time::now() - timer_;
    
    if (iiwa_ros_conn_.getRobotIsConnected()) {
        
        joint_position_ = iiwa_ros_conn_.getReceivedJointPosition();
        joint_torque_ = iiwa_ros_conn_.getReceivedJointTorque();
        
        device_->joint_position_prev = device_->joint_position;
        device_->joint_position = joint_position_.position;
        device_->joint_effort = joint_torque_.torque;
        
        for (int j = 0; j < IIWA_JOINTS; j++)
            device_->joint_velocity[j] = filters::exponentialSmoothing((device_->joint_position[j]-device_->joint_position_prev[j])/period.toSec(), 
                                                                       device_->joint_velocity[j], 0.2);  
        
        return 1;
    } else if (delta.toSec() >= 10) {
        ROS_INFO("No LBR IIWA is connected. Waiting for the robot to connect ...");
        timer_ = ros::Time::now();
    }
    return 0;
}

bool IIWA_HW::write(ros::Duration period) {
    ej_sat_interface_.enforceLimits(period);
    ej_limits_interface_.enforceLimits(period);
    pj_sat_interface_.enforceLimits(period);
    pj_limits_interface_.enforceLimits(period);
    
    ros::Duration delta = ros::Time::now() - timer_;
    
    //reading the force/torque values
    if (iiwa_ros_conn_.getRobotIsConnected()) {
        // Joint Position Control
        if (interface_ == interface_type_.at(0)) {
            
            // Building the message
            command_joint_position_.position = device_->joint_position_command;
            command_joint_position_.header.stamp = ros::Time::now();
            
            iiwa_ros_conn_.setCommandJointPosition(command_joint_position_);
        }
        // Joint Impedance Control
        else if (interface_ == interface_type_.at(1)) {
            // TODO
        }
        // Joint Velocity Control
        else if (interface_ == interface_type_.at(2)) {
            // TODO
        }
        
        iiwa_ros_conn_.publish();
    } else if (delta.toSec() >= 10) {
        ROS_INFO("No LBR IIWA is connected. Waiting for the robot to connect ...");
        timer_ = ros::Time::now();
    }
    
    return 0;
}
