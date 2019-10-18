/**
 * Copyright (C) 2016 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
 * Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <pluginlib/class_list_macros.hpp>

#include "iiwa_hw.hpp"
#include <iiwa_ros/conversions.hpp>

namespace iiwa_hw {

HardwareInterface::HardwareInterface() :
  device_{std::make_shared<HardwareInterface::Device>()}
  , timer_{ros::Time::now()}
  , last_joint_position_command_(7, 0) {}

void HardwareInterface::setFrequency(double frequency) {
  control_frequency_ = frequency;
  loop_rate_ = ros::Rate{control_frequency_};
}

bool HardwareInterface::init(ros::NodeHandle& /*unused*/, ros::NodeHandle &robot_hw_nh) {

  robot_hw_nh.param("hardware_interface", interface_, std::string("PositionJointInterface"));
  robot_hw_nh.param("robot_name", robot_name_, std::string("iiwa"));

  // Initialize Publishers and Subscribers from iiwa_ros.
  joint_position_state_.init(robot_name_);
  joint_torque_state_.init(robot_name_);
  joint_position_command_.init(robot_name_);

  if (ros::param::get("joints", device_->joint_names)) {
    if (!(device_->joint_names.size() == IIWA_JOINTS)) {
      ROS_ERROR("This robot has 7 joints, you must specify 7 names for each one");
    }
  } else {
    ROS_ERROR("No joints to be handled, ensure you load a yaml file naming the joint names this hardware interface "
              "refers to.");
    throw std::runtime_error("No joint name specification");
  }

  if (!(urdf_model_.initParam("robot_description"))) {
    ROS_ERROR("No URDF model in the robot_description parameter, this is required to define the joint limits.");
    throw std::runtime_error("No URDF model available");
  }

  // Initialize and set to zero the state and command values.
  device_->init();
  device_->reset();

  // Create joint handles given the list.
  for (size_t i = 0; i < IIWA_JOINTS; ++i) {
    ROS_INFO_STREAM("Handling joint: " << device_->joint_names[i]);

    // Get current joint configuration.
    auto joint = urdf_model_.getJoint(device_->joint_names[i]);

    if (!joint.get()) {
      ROS_ERROR_STREAM("The specified joint " << device_->joint_names[i]
                                              << " can't be found in the URDF model. "
                                                 "Check that you loaded an URDF model in the robot description, or "
                                                 "that you spelled correctly the joint name.");
      throw std::runtime_error("Wrong joint name specification");
    }

    // Joint state handle.
    hardware_interface::JointStateHandle state_handle(device_->joint_names[i], &(device_->joint_position[i]),
                                                      &(device_->joint_velocity[i]), &(device_->joint_effort[i]));
    state_interface_.registerHandle(state_handle);

    // Position command handle.
    hardware_interface::JointHandle position_joint_handle = hardware_interface::JointHandle(
        state_interface_.getHandle(device_->joint_names[i]), &device_->joint_position_command[i]);
    position_interface_.registerHandle(position_joint_handle);

    // Effort command handle.
    hardware_interface::JointHandle joint_handle = hardware_interface::JointHandle(
        state_interface_.getHandle(device_->joint_names[i]), &device_->joint_effort_command[i]);
    effort_interface_.registerHandle(joint_handle);

    registerJointLimits(device_->joint_names[i], joint_handle, &urdf_model_, device_->joint_lower_limits[i],
                        device_->joint_upper_limits[i], device_->joint_effort_limits[i]);
  }

  ROS_INFO("Registering state and effort interfaces");

  // Register ros-controls interfaces.
  this->registerInterface(&state_interface_);
  this->registerInterface(&effort_interface_);
  this->registerInterface(&position_interface_);

  return true;
}

void HardwareInterface::registerJointLimits(const std::string& joint_name,
                                            const hardware_interface::JointHandle& joint_handle,
                                            const urdf::Model* const urdf_model, double lower_limit, double upper_limit,
                                            double effort_limit) {
  lower_limit = -std::numeric_limits<double>::max();
  upper_limit = std::numeric_limits<double>::max();
  effort_limit = std::numeric_limits<double>::max();

  joint_limits_interface::JointLimits limits;
  bool has_limits = false;
  joint_limits_interface::SoftJointLimits soft_limits;
  bool has_soft_limits = false;

  if (urdf_model != nullptr) {
    const auto urdf_joint = urdf_model->getJoint(joint_name);

    if (urdf_joint != nullptr) {
      // Get limits from the URDF file.
      if (joint_limits_interface::getJointLimits(urdf_joint, limits)) { has_limits = true; }
      if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits)) { has_soft_limits = true; }
    }
  }

  if (!has_limits) { return; }

  if (limits.has_position_limits) {
    lower_limit = limits.min_position;
    upper_limit = limits.max_position;
  }

  if (limits.has_effort_limits) { effort_limit = limits.max_effort; }

  if (has_soft_limits) {
    const joint_limits_interface::EffortJointSoftLimitsHandle limits_handle(joint_handle, limits, soft_limits);
    ej_limits_interface_.registerHandle(limits_handle);
  }
  else
  {
    const joint_limits_interface::EffortJointSaturationHandle sat_handle(joint_handle, limits);
    ej_sat_interface_.registerHandle(sat_handle);
  }
}

void HardwareInterface::read(const ros::Time& time, const ros::Duration& period) {
  ros::Duration delta = ros::Time::now() - timer_;

  static bool was_connected = false;

  if (joint_position_state_.isConnected()) {
    joint_position_ = joint_position_state_.getPosition();
    joint_torque_ = joint_torque_state_.getTorque();

    device_->joint_position_prev = device_->joint_position;

    auto current_joint_position = iiwa_ros::conversions::jointQuantityToVector<double>(joint_position_.position);
    device_->joint_position = current_joint_position;

    auto current_joint_torque = iiwa_ros::conversions::jointQuantityToVector<double>(joint_torque_.torque);
    device_->joint_effort = current_joint_torque;

    // if there is no controller active the robot goes to zero position
    if (!was_connected) {
      for (size_t j = 0; j < IIWA_JOINTS; j++) { device_->joint_position_command[j] = device_->joint_position[j]; }
      was_connected = true;
    }

    for (size_t j = 0; j < IIWA_JOINTS; j++) {
      device_->joint_velocity[j] =
          filters::exponentialSmoothing((device_->joint_position[j] - device_->joint_position_prev[j]) / period.toSec(),
                                        device_->joint_velocity[j], 0.2);
    }
  } else if (delta.toSec() >= 10) {
    ROS_INFO("No LBR IIWA is connected. Waiting for the robot to connect before reading ...");
    timer_ = ros::Time::now();
  }
}

void HardwareInterface::write(const ros::Time &time, const ros::Duration &period) {
  ej_sat_interface_.enforceLimits(period);
  ej_limits_interface_.enforceLimits(period);
  pj_sat_interface_.enforceLimits(period);
  pj_limits_interface_.enforceLimits(period);

  ros::Duration delta = ros::Time::now() - timer_;

  // Reading the joint values.
  if (joint_position_state_.isConnected()) {
    // Joint Position Control.
    if (interface_ == interface_type_.at(0)) {
      // Avoid sending the same joint command over and over.
      if (device_->joint_position_command == last_joint_position_command_) { return; }

      last_joint_position_command_ = device_->joint_position_command;

      // Building the message
      auto command = iiwa_ros::conversions::jointQuantityFromVector<double>(device_->joint_position_command);
      command_joint_position_.position = command;
      command_joint_position_.header.stamp = ros::Time::now();

      joint_position_command_.setPosition(command_joint_position_);
    }
    // Joint Impedance Control.
    else if (interface_ == interface_type_.at(1)) {
      // TODO
    }
    // Joint Velocity Control.
    else if (interface_ == interface_type_.at(2)) {
      // TODO
    }
  } else if (delta.toSec() >= 10) {
    ROS_INFO_STREAM("No LBR IIWA is connected. Waiting for the robot to connect before writing ...");
    timer_ = ros::Time::now();
  }
}

PLUGINLIB_EXPORT_CLASS(iiwa_hw::HardwareInterface, hardware_interface::RobotHW)


}  // namespace iiwa_hw
