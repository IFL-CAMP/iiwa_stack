#include "iiwa_ros/state/joint_position.hpp"

namespace iiwa_ros {
namespace state {

void JointPosition::init(const std::string& robot_namespace) {
  setup(robot_namespace);
  initROS("JointPositionState");
  state_.init(ros_namespace_ + "state/JointPosition");
}

iiwa_msgs::JointPosition JointPosition::getPosition() { return state_.get(); }

}  // namespace state
}  // namespace iiwa_ros
