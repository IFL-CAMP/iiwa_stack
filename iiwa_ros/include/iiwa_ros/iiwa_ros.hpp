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

#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <mutex>
#include <string>

namespace iiwa_ros {
extern ros::Time last_update_time;

template <typename ROSMSG>
class Holder {
public:
  Holder() = default;

  void set(const ROSMSG& value) {
    std::lock_guard<std::mutex> lock{mutex_};
    data_ = value;
  }

  ROSMSG get() {
    std::lock_guard<std::mutex> lock{mutex_};
    return data_;
  }

  ROSMSG getUnsynced() { return data_; }

private:
  ROSMSG data_;
  std::mutex mutex_;
};

template <typename ROSMSG>
class State {
public:
  State() = default;

  void init(const std::string& topic) {
    ros::NodeHandle nh;
    subscriber_ = nh.subscribe<ROSMSG>(topic, 1, &State<ROSMSG>::set, this);
  }

  void init(const std::string& topic, const std::function<void(const ROSMSG&)>& callback) {
    callback_ = std::move(callback);
    init(topic);
  }

  void set(ROSMSG value) {
    last_update_time = ros::Time::now();
    holder_.set(value);
    if (callback_ != nullptr) { callback_(value); }
  }

  ROSMSG get() { return holder_.get(); }

private:
  std::function<void(const ROSMSG&)> callback_{nullptr};
  Holder<ROSMSG> holder_;
  ros::Subscriber subscriber_;
};

template <typename ROSMSG>
class Command {
public:
  Command() = default;

  void init(const std::string& topic) {
    ros::NodeHandle nh;
    publisher_ = nh.advertise<ROSMSG>(topic, 1);
  }

  void set(const ROSMSG& value) { holder_.set(value); }

  ROSMSG get() { return holder_.getUnsynced(); }

  void publish() {
    if (publisher_.getNumSubscribers()) { publisher_.publish(get()); }
  }

private:
  ros::Publisher publisher_;
  Holder<ROSMSG> holder_;
};

class Robot {
public:
  virtual ~Robot() = default;
  virtual void init(const std::string& robot_namespace) = 0;

protected:
  Robot() = default;
  void initROS(const std::string& ros_node_name);
  void setup(const std::string& robot_namespace) {
    // Build the correct ROS namespace if one was given, else use the root namespace.
    if (!robot_namespace.empty() && robot_namespace != ros_namespace_) { ros_namespace_.append(robot_namespace + "/"); }
  }

protected:
  std::string ros_namespace_{"/"};
};

}  // namespace iiwa_ros
