/**
 * Copyright (C) 2016-2019 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
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

#include <iostream>
#include <csignal>

#include <ros/ros.h>
#include <iiwa_ros/state/joint_position.hpp>
#include <iiwa_ros/state/joint_torque.hpp>

static bool quit{false};

void signalHandler(int /*unused*/) { quit = true; }

int main() {
  iiwa_ros::state::JointPosition jp_state;
  iiwa_ros::state::JointTorque jt_state;

  jp_state.init("iiwa");
  jt_state.init("iiwa");

  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Signal handlers.
  signal(SIGTERM, signalHandler);
  signal(SIGINT, signalHandler);
  signal(SIGHUP, signalHandler);

  // Wait a bit, so that you can be sure the subscribers are connected.
  ros::Duration(0.5).sleep();

  while (!quit) {
    auto joint_position_ = jp_state.getPosition();

    auto joint_torque = jt_state.getTorque();
    ROS_INFO_STREAM(
        std::to_string(joint_position_.position.a1)
            << " " << std::to_string(joint_position_.position.a2) << " " << std::to_string(joint_position_.position.a3)
            << " " << std::to_string(joint_position_.position.a4) << " " << std::to_string(joint_position_.position.a5)
            << " " << std::to_string(joint_position_.position.a6) << " " << std::to_string(joint_position_.position.a7)
            << std::endl;);
    ros::Duration(0.1).sleep();
  }

  std::cerr << "Stopping spinner..." << std::endl;
  spinner.stop();

  std::cerr << "Bye!" << std::endl;

  return 0;
}
