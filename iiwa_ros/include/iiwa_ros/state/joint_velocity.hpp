#pragma once

#include "iiwa_msgs/JointVelocity.h"
#include "iiwa_ros/iiwa_ros.hpp"

namespace iiwa_ros {
namespace state {

class JointVelocity : public Robot {
public:
  JointVelocity() = default;

  iiwa_msgs::JointVelocity getVelocity();
  virtual void init(const std::string& robot_namespace) override;

private:
  iiwaStateHolder<iiwa_msgs::JointVelocity> state_{};
};

}  // namespace state
}  // namespace iiwa_ros
