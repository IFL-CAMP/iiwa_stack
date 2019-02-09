#include "iiwa_ros/state/joint_torque.hpp"

namespace iiwa_ros {
namespace state {

void JointTorque::init(const std::string& robot_namespace) {
  setup(robot_namespace);
  initROS("JointTorqueState");
  state_.init(ros_namespace_ + "state/JointTorque");
}

iiwa_msgs::JointTorque JointTorque::getTorque() { return state_.get(); }

}  // namespace state
}  // namespace iiwa_ros
