#include "iiwa_ros/state/joint_velocity.hpp"

namespace iiwa_ros {
namespace state {

void JointVelocity::init(const std::string& robot_namespace) {
  setup(robot_namespace);
  initROS("JointTorqueState");
  state_.init(ros_namespace_ + "state/JointVelocity");
}

iiwa_msgs::JointVelocity JointVelocity::getVelocity() {
  iiwa_msgs::JointVelocity jv;
  state_.get(jv);
  return jv;
}

}  // namespace state
}  // namespace iiwa_ros
