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

#include <smart_servo_service.h>
#include <iiwa_msgs/ControlMode.h>
#include <conversions.h>

namespace iiwa_ros {	
  
  SmartServoService::SmartServoService() : iiwaServices<iiwa_msgs::ConfigureSmartServo>() {}
  
  SmartServoService::SmartServoService(const std::string& service_name, const bool verbose) : iiwaServices<iiwa_msgs::ConfigureSmartServo>(service_name, verbose) {}
  
  bool SmartServoService::callService() {
    if (service_ready_) {
      if (client_.call(config_)) {
        if(!config_.response.success && verbose_) {
          service_error_ = config_.response.error;
          ROS_ERROR_STREAM(service_name_ << " failed, Java error: " << service_error_);
        }
        else if (verbose_) {
          ROS_INFO_STREAM(ros::this_node::getName() << ":" << service_name_ << " successfully called.");
        }
      }
      else if (verbose_) {
        ROS_ERROR_STREAM(service_name_ << " could not be called");
      }
      return config_.response.success;
    }
    ROS_ERROR_STREAM("The service client was not intialized yet.");
  }
  
  
  void SmartServoService::initCartesianLimits(const iiwa_msgs::CartesianQuantity& max_path_deviation, const iiwa_msgs::CartesianQuantity& max_cartesian_velocity,
                                              const iiwa_msgs::CartesianQuantity& max_control_force, const bool max_control_force_stop) {
    config_.request.limits.max_path_deviation = max_path_deviation;
    config_.request.limits.max_cartesian_velocity = max_cartesian_velocity;
    config_.request.limits.max_control_force = max_control_force;
    config_.request.limits.max_control_force_stop = max_control_force_stop;
  }
  
  void SmartServoService::initJointImpedanceMode(const iiwa_msgs::JointQuantity& joint_stiffnes, const iiwa_msgs::JointQuantity& joint_damping) {
    config_.request.control_mode = iiwa_msgs::ControlMode::JOINT_IMPEDANCE;
    config_.request.joint_impedance.joint_stiffness = joint_stiffnes;
    config_.request.joint_impedance.joint_damping = joint_damping;
    
  }
  
  void SmartServoService::initCartesianImpedanceMode(const iiwa_msgs::CartesianQuantity& cartesian_stiffness, const iiwa_msgs::CartesianQuantity& cartesian_damping, 
                                                     const double nullspace_stiffness, const double nullspace_damping, const iiwa_msgs::CartesianQuantity& max_path_deviation, 
                                                     const iiwa_msgs::CartesianQuantity& max_cartesian_velocity, const iiwa_msgs::CartesianQuantity& max_control_force, 
                                                     const bool max_control_force_stop) {
    config_.request.control_mode = iiwa_msgs::ControlMode::CARTESIAN_IMPEDANCE;
    config_.request.cartesian_impedance.cartesian_stiffness = cartesian_stiffness;
    config_.request.cartesian_impedance.cartesian_damping = cartesian_damping;
    config_.request.cartesian_impedance.nullspace_stiffness = nullspace_stiffness;
    config_.request.cartesian_impedance.nullspace_damping = nullspace_damping;
    initCartesianLimits(max_path_deviation, max_cartesian_velocity, max_control_force, max_control_force_stop);
  }
  
  void SmartServoService::initDesiredForceMode(const int cartesian_dof, const double desired_force, const double desired_stiffness, 
                                               const iiwa_msgs::CartesianQuantity& max_path_deviation, const iiwa_msgs::CartesianQuantity& max_cartesian_velocity, 
                                               const iiwa_msgs::CartesianQuantity& max_control_force, const bool max_control_force_stop) {
    config_.request.control_mode = iiwa_msgs::ControlMode::DESIRED_FORCE;
    config_.request.desired_force.cartesian_dof = cartesian_dof;
    config_.request.desired_force.desired_force = desired_force;
    config_.request.desired_force.desired_stiffness = desired_stiffness;
    initCartesianLimits(max_path_deviation, max_cartesian_velocity, max_control_force, max_control_force_stop);
  }
  
  void SmartServoService::initSinePatternMode(const int cartesian_dof, const double frequency, const double amplitude, const double stiffness, 
                                              const iiwa_msgs::CartesianQuantity& max_path_deviation, const iiwa_msgs::CartesianQuantity& max_cartesian_velocity, 
                                              const iiwa_msgs::CartesianQuantity& max_control_force, const bool max_control_force_stop) {
    config_.request.control_mode = iiwa_msgs::ControlMode::SINE_PATTERN;
    config_.request.sine_pattern.cartesian_dof = cartesian_dof;
    config_.request.sine_pattern.frequency = frequency;
    config_.request.sine_pattern.amplitude = amplitude;
    config_.request.sine_pattern.stiffness = stiffness;
    initCartesianLimits(max_path_deviation, max_cartesian_velocity, max_control_force, max_control_force_stop);
  }	
  
  bool SmartServoService::setPositionControlMode()
  {
    config_.request.control_mode = iiwa_msgs::ControlMode::POSITION_CONTROL;
    return callService();
  }
  
  bool SmartServoService::setJointImpedanceMode(const iiwa_msgs::JointQuantity& joint_stiffnes, const iiwa_msgs::JointQuantity& joint_damping) {
    initJointImpedanceMode(joint_stiffnes, joint_damping);
    return callService();
  }
  
  bool SmartServoService::setJointImpedanceMode(const double joint_stiffnes, const double joint_damping) {
    setJointImpedanceMode(jointQuantityFromDouble(joint_stiffnes), jointQuantityFromDouble(joint_damping));
  }	
  
  bool SmartServoService::setCartesianImpedanceMode(const iiwa_msgs::CartesianQuantity& cartesian_stiffness, const iiwa_msgs::CartesianQuantity& cartesian_damping, 
                                                    const double nullspace_stiffness, const double nullspace_damping, const iiwa_msgs::CartesianQuantity& max_path_deviation, 
                                                    const iiwa_msgs::CartesianQuantity& max_cartesian_velocity, const iiwa_msgs::CartesianQuantity& max_control_force, const bool max_control_force_stop) {
    initCartesianImpedanceMode(cartesian_stiffness, cartesian_damping, nullspace_stiffness, nullspace_damping, max_path_deviation, max_cartesian_velocity,max_control_force, max_control_force_stop);
    return callService();
  }
  
  bool SmartServoService::setCartesianImpedanceMode(const iiwa_msgs::CartesianQuantity& cartesian_stiffness, const iiwa_msgs::CartesianQuantity& cartesian_damping, 
                                                    const double nullspace_stiffness, const double nullspace_damping) {
    setCartesianImpedanceMode(cartesian_stiffness, cartesian_damping, nullspace_stiffness, nullspace_damping, CartesianQuantityFromDouble(-1), CartesianQuantityFromDouble(-1), CartesianQuantityFromDouble(-1), false);
  }
  
  bool SmartServoService::setCartesianImpedanceMode(const iiwa_msgs::CartesianQuantity& cartesian_stiffness, const iiwa_msgs::CartesianQuantity& cartesian_damping) {
    setCartesianImpedanceMode(cartesian_stiffness, cartesian_damping, -1, -1, CartesianQuantityFromDouble(-1), CartesianQuantityFromDouble(-1), CartesianQuantityFromDouble(-1), false);
  }
  
  bool SmartServoService::setCartesianImpedanceMode(const iiwa_msgs::CartesianQuantity& cartesian_stiffness, const iiwa_msgs::CartesianQuantity& cartesian_damping, 
                                                    const iiwa_msgs::CartesianQuantity& max_path_deviation, const iiwa_msgs::CartesianQuantity& max_cartesian_velocity, 
                                                    const iiwa_msgs::CartesianQuantity& max_control_force, const bool max_control_force_stop) {
    setCartesianImpedanceMode(cartesian_stiffness, cartesian_damping, -1, -1, max_path_deviation, max_cartesian_velocity, max_control_force, max_control_force_stop);
  }
  
  bool SmartServoService::setDesiredForceMode(const int cartesian_dof, const double desired_force, const double desired_stiffness, 
                                              const iiwa_msgs::CartesianQuantity& max_path_deviation, const iiwa_msgs::CartesianQuantity& max_cartesian_velocity, 
                                              const iiwa_msgs::CartesianQuantity& max_control_force, const bool max_control_force_stop) {
    initDesiredForceMode(cartesian_dof, desired_force, desired_stiffness, max_path_deviation, max_cartesian_velocity, max_control_force, max_control_force_stop);
    return callService();
  }
  
  bool SmartServoService::setDesiredForceMode(const int cartesian_dof, const double desired_force, const double desired_stiffness) {
    setDesiredForceMode(cartesian_dof, desired_force, desired_stiffness,CartesianQuantityFromDouble(-1), CartesianQuantityFromDouble(-1), CartesianQuantityFromDouble(-1), false);
  }
  
  bool SmartServoService::setSinePatternmode(const int cartesian_dof, const double frequency, const double amplitude, const double stiffness, 
                                             const iiwa_msgs::CartesianQuantity& max_path_deviation, const iiwa_msgs::CartesianQuantity& max_cartesian_velocity, 
                                             const iiwa_msgs::CartesianQuantity& max_control_force, const bool max_control_force_stop) {
    initSinePatternMode(cartesian_dof, frequency, amplitude, stiffness, max_path_deviation, max_cartesian_velocity, max_control_force, max_control_force_stop);
    return callService();
  }
  
  bool SmartServoService::setSinePatternmode(const int cartesian_dof, const double frequency, const double amplitude, const double stiffness) {
    setSinePatternmode(cartesian_dof, frequency, amplitude, stiffness, CartesianQuantityFromDouble(-1), CartesianQuantityFromDouble(-1), CartesianQuantityFromDouble(-1), false);
  }
}
