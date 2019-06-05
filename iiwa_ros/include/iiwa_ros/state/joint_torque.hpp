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

#include "iiwa_msgs/JointTorque.h"
#include "iiwa_ros/state/generic_state.hpp"

namespace iiwa_ros {
namespace state {

class JointTorque : public GenericState {
public:
  JointTorque() = default;

  /**
   * @brief Initialize the object with a given robot namespace.
   * @param [in] robot_namespace - the namespace under which the command topics for the desired robot exist.
   */
  virtual void init(const std::string& robot_namespace) override;

  /**
   * @brief Initialize the object with the given robot namespace and callback function.
   * @param [in] robot_namespace - the namespace under which the state topics for the desired robot exist.
   * @param [in] callback - a callback function to call when a new state is received.
   */
  void init(const std::string& robot_namespace, const std::function<void(const iiwa_msgs::JointTorque&)> callback);

  /**
   * @brief Returns the current robot joint torque.
   */
  iiwa_msgs::JointTorque getTorque();

private:
  State<iiwa_msgs::JointTorque> state_{};
};

}  // namespace state
}  // namespace iiwa_ros
