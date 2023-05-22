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

#pragma once

// iiwa_msgs and ROS inteface includes
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointTorque.h>
#include <iiwa_ros/command/joint_position.hpp>
#include <iiwa_ros/state/joint_position.hpp>
#include <iiwa_ros/state/joint_torque.hpp>

// ROS headers
#include <control_toolbox/filters.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/ros.h>
#include <std_msgs/Duration.h>
#include <urdf/model.h>

#include <sstream>
#include <vector>

constexpr int DEFAULT_CONTROL_FREQUENCY = 1000;  // Hz
constexpr int IIWA_JOINTS = 7;

namespace iiwa_hw {
class HardwareInterface : public hardware_interface::RobotHW {
public:
  HardwareInterface();

  /**
   * \brief Initializes the Device struct and all the hardware and joint limits interfaces needed.
   *
   * A joint state handle is created and linked to the current joint state of the IIWA robot.
   * A joint position handle is created and linked  to the command joint position to send to the robot.
   */

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) override;

  /**
   * \brief Registers the limits of the joint specified by joint_name and joint_handle.
   *
   * The limits are retrieved from the urdf_model.
   * Returns the joint's type, lower position limit, upper position limit, and effort limit.
   */
  void registerJointLimits(const std::string& joint_name, const hardware_interface::JointHandle& joint_handle,
                           const urdf::Model* const urdf_model, double lower_limit, double upper_limit,
                           double effort_limit);

  /**
   * \brief Reads the current robot state via the interfae provided by iiwa_ros and sends the values to the Device struct.
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /**
   * \brief Sends the command joint position to the robot via the interface provided by iiwa_ros.
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

  /**
   * \brief Retuns the ros::Rate object to control the receiving/sending rate.
   */
  ros::Rate getRate() { return loop_rate_; }

  /**
   * \brief Retuns the current frequency used by a ros::Rate object to control the receiving/sending rate.
   */
  double getFrequency() { return control_frequency_; }

  /**
   * \brief Set the frequency to be used by a ros::Rate object to control the receiving/sending rate.
   */
  void setFrequency(double frequency);

  /** Structure for a lbr iiwa, joint handles, etc */
  struct Device {
    /** Vector containing the name of the joints - taken from yaml file */
    std::vector<std::string> joint_names{};

    std::vector<double> joint_lower_limits{};  /**< Lower joint limits */
    std::vector<double> joint_upper_limits{};  /**< Upper joint limits */
    std::vector<double> joint_effort_limits{}; /**< Effort joint limits */

    /**< Joint state and commands */
    std::vector<double> joint_position{};
    std::vector<double> joint_position_prev{};
    std::vector<double> joint_velocity{};
    std::vector<double> joint_effort{};
    std::vector<double> joint_position_command{};
    std::vector<double> joint_stiffness_command{};
    std::vector<double> joint_damping_command{};
    std::vector<double> joint_effort_command{};

    void init() {
      joint_position.resize(IIWA_JOINTS);
      joint_position_prev.resize(IIWA_JOINTS);
      joint_velocity.resize(IIWA_JOINTS);
      joint_effort.resize(IIWA_JOINTS);
      joint_position_command.resize(IIWA_JOINTS);
      joint_effort_command.resize(IIWA_JOINTS);
      joint_stiffness_command.resize(IIWA_JOINTS);
      joint_damping_command.resize(IIWA_JOINTS);

      joint_lower_limits.resize(IIWA_JOINTS);
      joint_upper_limits.resize(IIWA_JOINTS);
      joint_effort_limits.resize(IIWA_JOINTS);
    }

    void reset() {
      std::fill(joint_position.begin(), joint_position.end(), 0);
      std::fill(joint_position_prev.begin(), joint_position_prev.end(), 0);
      std::fill(joint_velocity.begin(), joint_velocity.end(), 0);
      std::fill(joint_effort.begin(), joint_effort.end(), 0);
      std::fill(joint_position_command.begin(), joint_position_command.end(), 0);
      std::fill(joint_effort_command.begin(), joint_effort_command.end(), 0);
      std::fill(joint_stiffness_command.begin(), joint_stiffness_command.end(), 0);
      std::fill(joint_damping_command.begin(), joint_damping_command.end(), 0);
    }
  };

private:
  /* Node handle */
  ros::NodeHandle nh_;

  /* Parameters */
  std::string robot_name_{""}, interface_{""}, movegroup_name_{""};
  urdf::Model urdf_model_;

  hardware_interface::JointStateInterface state_interface_;       /**< Interface for joint state */
  hardware_interface::EffortJointInterface effort_interface_;     /**< Interface for joint impedance control */
  hardware_interface::PositionJointInterface position_interface_; /**< Interface for joint position control */

  /** Interfaces for limits */
  joint_limits_interface::EffortJointSaturationInterface ej_sat_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface ej_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;

  std::shared_ptr<HardwareInterface::Device> device_{nullptr}; /**< IIWA device. */

  /** Objects to control send/receive rate. */
  ros::Time timer_{};
  ros::Rate loop_rate_{DEFAULT_CONTROL_FREQUENCY};
  double control_frequency_{DEFAULT_CONTROL_FREQUENCY};

  iiwa_ros::state::JointPosition joint_position_state_{};
  iiwa_ros::state::JointTorque joint_torque_state_{};
  iiwa_ros::command::JointPosition joint_position_command_{};

  iiwa_msgs::JointPosition joint_position_{};
  iiwa_msgs::JointTorque joint_torque_{};

  iiwa_msgs::JointPosition command_joint_position_{};
  iiwa_msgs::JointTorque command_joint_torque_{};

  std::vector<double> last_joint_position_command_{};

  std::vector<std::string> interface_type_{"PositionJointInterface", "EffortJointInterface", "VelocityJointInterface"};
};
}  // namespace iiwa_hw
