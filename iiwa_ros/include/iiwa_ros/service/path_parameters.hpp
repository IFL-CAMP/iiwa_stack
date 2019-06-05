/**
 * Copyright (C) 2019 Salvatore Virga - salvo.virga@tum.de
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

#include <iiwa_msgs/SetSmartServoJointSpeedLimits.h>
#include <iiwa_ros/service/iiwa_services.hpp>

namespace iiwa_ros {
namespace service {

/**
 * @brief This class provides a wrapper for the PathParametersService service.
 * Once an object of this class is initialized using the appropriate robot namespace name,
 * it is possible to call its functions to set the desired robot velocity and acceleration.
 */
class PathParametersService : public iiwaServices<iiwa_msgs::SetSmartServoJointSpeedLimits> {
public:
  PathParametersService() = default;
  virtual ~PathParametersService() override = default;

  virtual void init(const std::string& robot_namespace) override;

  /**
   * @brief Set the robot joint velocity.
   *
   * @param [in] joint_velocity - Values in (0,1]. e.g., 0.5 will make the robot joints move at 50% of their maximum speed.
   * @return bool - success status.
   */
  bool setJointVelocity(const double joint_relative_velocity);

  /**
   * @brief Set the robot joint acceleration.
   *
   * @param [in] joint_acceleration - Values in (0,1]. e.g., 0.5 will make the robot joints move at 50% of their maximum acceleration.
   * @return bool - success status.
   */
  bool setJointAcceleration(const double joint_relative_acceleration);

  /**
   * @brief Set the override joint acceleration.
   *
   * @param [in] override_acceleration - TODO
   * @return bool - success status.
   */
  bool setOverrideJointAcceleration(const double override_acceleration);

  /**
   * @brief Set both joint velocity and acceleration.
   *
   * @param [in] joint_velocity - Values in (0,1]. e.g., 0.5 will make the robot joints move at 50% of their maximum speed.
   * @param [in] joint_acceleration - alues in (0,1]. e.g., 0.5 will make the robot joints move at 50% of their maximum acceleration.
   * @return bool - success status.
   */
  bool setSmartServoJointSpeedLimits(const double joint_relative_velocity, const double joint_relative_acceleration);

  /**
   * @brief Set joint velocity, acceleration and override acceleration at once.
   *
   * @param [in] joint_velocity - Values in (0,1]. e.g., 0.5 will make the robot joints move at 50% of their maximum speed.
   * @param [in] joint_acceleration - alues in (0,1]. e.g., 0.5 will make the robot joints move at 50% of their maximum acceleration.
   * @param override_acceleration - TODO.
   * @return bool - success status.
   */
  bool setSmartServoJointSpeedLimits(const double joint_relative_velocity, const double joint_relative_acceleration,
                         const double override_joint_acceleration);

protected:
  virtual bool callService() override;
};

}  // namespace service
}  // namespace iiwa_ros
