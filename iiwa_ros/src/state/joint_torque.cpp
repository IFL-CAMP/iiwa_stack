#include "iiwa_ros/state/joint_torque.hpp"

namespace iiwa_ros {
namespace state {

void JointTorque::init(const std::string& robot_namespace) {
  setup(robot_namespace);
  initROS("JointTorqueState");
  state_.init(ros_namespace_ + "state/JointTorque");
}

iiwa_msgs::JointTorque JointTorque::getTorque() {
  iiwa_msgs::JointTorque jt;
  state_.get(jt);
  return jt;
}

}  // namespace state
}  // namespace iiwa_ros
