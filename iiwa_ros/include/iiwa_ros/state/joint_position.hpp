#pragma once

#include "iiwa_msgs/JointPosition.h"
#include "iiwa_ros/iiwa_ros.hpp"

namespace iiwa_ros {
namespace state {

class JointPosition : public Robot {
public:
  JointPosition() = default;
  virtual void init(const std::string& robot_namespace) override;

  iiwa_msgs::JointPosition getPosition();

private:
  State<iiwa_msgs::JointPosition> state_{};
};

}  // namespace state
}  // namespace iiwa_ros
