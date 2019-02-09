#include "iiwa_ros/state/joint_velocity.hpp"

namespace iiwa_ros {
namespace state {

void JointVelocity::init(const std::string& robot_namespace) {
  setup(robot_namespace);
  initROS("JointTorqueState");
  state_.init(ros_namespace_ + "state/JointVelocity");
}

iiwa_msgs::JointVelocity JointVelocity::getVelocity() { return state_.get(); }

}  // namespace state
}  // namespace iiwa_ros
