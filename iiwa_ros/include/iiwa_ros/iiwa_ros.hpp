/**
 * Copyright (C) 2016-2017 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
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

// template <typename ROSMSG>
// class Holder {
// public:
//  Holder() = default;
//  ~Holder() = default;

//  Holder(Holder&& other) noexcept : data_{std::move(other.data_)} {}

//  Holder& operator=(Holder&& rhs) noexcept {
//    if (this != std::addressof(rhs)) {
//      // std::lock(mutex_, rhs.mutex_);
//      // std::lock_guard<std::mutex> lock{mutex_, std::adopt_lock};
//      // std::lock_guard<std::mutex> other_lock{rhs.mutex_, std::adopt_lock};
//      data_ = std::move(rhs.data_);
//    }
//    return *this;
//  }

//  void set(const ROSMSG& value) {
//    // std::lock_guard<std::mutex> lock(mutex_);
//    data_ = value;
//  }

//  ROSMSG get() {
//    // std::lock_guard<std::mutex> lock(mutex_);
//    return data_;
//  }

//  ROSMSG GetUnsynced() { return data_; }

// private:
//  ROSMSG data_;
//  // std::mutex mutex_{};
//};

// template <typename ROSMSG>
// class State {
// public:
//  void init(const std::string& topic) {
//    ros::NodeHandle nh;
//    subscriber_ = nh.subscribe<ROSMSG>(topic, 1, &State<ROSMSG>::set, this);
//  }

//  void set(ROSMSG value) {
//    last_update_time = ros::Time::now();
//    holder_.set(value);
//  }

//  ROSMSG get() { return holder_.get(); }

// private:
//  Holder<ROSMSG> holder_;
//  ros::Subscriber subscriber_;
//};

// template <typename ROSMSG>
// class Command {
// public:
//  void init(const std::string& topic) {
//    ros::NodeHandle nh;
//    publisher_ = nh.advertise<ROSMSG>(topic, 1);
//  }

//  void set(const ROSMSG& value) { holder_.set(value); }

//  ROSMSG get() { return holder_.GetUnsynced(); }

//  void publish() {
//    if (publisher_.getNumSubscribers()) { publisher_.publish(holder_.get()); }
//  }

// private:
//  ros::Publisher publisher_;
//  Holder<ROSMSG> holder_;
//};

template <typename ROSMSG>
class iiwaHolder {
public:
  iiwaHolder() : is_new(false) {}

  void set_value(const ROSMSG& value) {
    mutex_.lock();
    data_ = value;
    is_new = true;
    mutex_.unlock();
  }

  bool get_value(ROSMSG& value) {
    bool was_new = false;

    mutex_.lock();
    value = data_;
    was_new = is_new;
    is_new = false;
    mutex_.unlock();

    return was_new;
  }

  bool has_new_value() { return is_new; }

  ROSMSG get_value_unsynchronized() { return data_; }

private:
  ROSMSG data_;
  bool is_new;
  std::mutex mutex_;
};

template <typename ROSMSG>
class iiwaStateHolder {
public:
  void init(const std::string& topic) {
    ros::NodeHandle nh;
    subscriber = nh.subscribe<ROSMSG>(topic, 1, &iiwaStateHolder<ROSMSG>::set, this);
  }

  bool has_new_value() { return holder.has_new_value(); }

  void set(ROSMSG value) {
    last_update_time = ros::Time::now();
    holder.set_value(value);
  }

  bool get(ROSMSG& value) { return holder.get_value(value); }

private:
  iiwaHolder<ROSMSG> holder;
  ros::Subscriber subscriber;
};

template <typename ROSMSG>
class iiwaCommandHolder {
public:
  void init(const std::string& topic) {
    ros::NodeHandle nh;
    publisher = nh.advertise<ROSMSG>(topic, 1);
  }

  void set(const ROSMSG& value) { holder.set_value(value); }

  ROSMSG get() { return holder.get_value_unsynchronized(); }

  void publishIfNew() {
    static ROSMSG msg;
    if (publisher.getNumSubscribers() && holder.get_value(msg)) publisher.publish(msg);
  }

private:
  ros::Publisher publisher;
  iiwaHolder<ROSMSG> holder;
};

class Robot {
public:
  /**
   * @brief Constructor for class iiwaRos holding all the methods to command and get the state of the robot.
   */
  Robot();
  //  Robot(const std::string& robot_namespace);
  virtual ~Robot() = default;

  /**
   * \brief Returns the current connection status of an IIWA robot.
   */
  bool isConnected();
  virtual void init(const std::string& robot_namespace) = 0;

protected:
  void initROS(const std::string& ros_node_name);
  void setup(const std::string& robot_namespace) {
    // Build the correct ROS namespace if one was given, else use the root namespace.
    if (!robot_namespace.empty() && robot_namespace != ros_namespace_) { ros_namespace_.append(robot_namespace + "/"); }
  }

protected:
  std::string ros_namespace_{"/"};
};
}  // namespace iiwa_ros
