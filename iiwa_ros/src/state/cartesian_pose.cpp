#include "iiwa_ros/state/cartesian_pose.hpp"

namespace iiwa_ros {
namespace state {

void CartesianPose::init(const std::string& robot_namespace) {
  setup(robot_namespace);
  initROS("CartesianPoseState");
  state_.init(ros_namespace_ + "state/CartesianPose");
}

iiwa_msgs::CartesianPose CartesianPose::getPose() {
  iiwa_msgs::CartesianPose cp;
  state_.get(cp);
  return cp;
}

}  // namespace state
}  // namespace iiwa_ros
