#pragma once

#include "iiwa_msgs/JointPosition.h"
#include "iiwa_ros/iiwa_ros.hpp"

namespace iiwa_ros {
namespace state {

class JointPosition : public Robot {
public:
  JointPosition() = default;

  iiwa_msgs::JointPosition getPosition();
  virtual void init(const std::string& robot_namespace) override;

private:
  iiwaStateHolder<iiwa_msgs::JointPosition> state_{};
};

}  // namespace state
}  // namespace iiwa_ros
