#pragma once

#include "iiwa_msgs/JointTorque.h"
#include "iiwa_ros/iiwa_ros.hpp"

namespace iiwa_ros {
namespace state {

class JointTorque : public Robot {
public:
  JointTorque() = default;

  iiwa_msgs::JointTorque getTorque();
  virtual void init(const std::string& robot_namespace) override;

private:
  iiwaStateHolder<iiwa_msgs::JointTorque> state_{};
};

}  // namespace state
}  // namespace iiwa_ros
