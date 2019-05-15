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

#include <iiwa_ros/service/time_to_destination.hpp>

namespace iiwa_ros {
namespace service {

void TimeToDestinationService::init(const std::string& robot_namespace) {
  setup(robot_namespace);
  ros::NodeHandle node_handle{};
  service_name_ = ros_namespace_ + "state/timeToDestination";
  client_ = node_handle.serviceClient<iiwa_msgs::TimeToDestination>(service_name_);
  service_ready_ = true;
}

double TimeToDestinationService::getTimeToDestination() {
  if (service_ready_) {
    if (callService()) {
      return time_to_destination_;
    } else {
      return -999;  // It cannot return -1 since it might be a meaningfull result.
    }
  }
  ROS_ERROR_STREAM("The service client was not intialized yet. Call the init function of this object first.");
}

bool TimeToDestinationService::callService() {
  if (client_.call(config_)) {
    if (verbose_) { ROS_DEBUG_STREAM(ros::this_node::getName() << ": " << service_name_ << " successfully called."); }
    time_to_destination_ = config_.response.remaining_time;
    return true;
  } else if (verbose_) {
    ROS_ERROR_STREAM(service_name_ << " could not be called");
  }
  return false;
}

}  // namespace service
}  // namespace iiwa_ros
