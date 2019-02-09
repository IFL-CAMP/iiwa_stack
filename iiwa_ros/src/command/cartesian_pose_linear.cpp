#include "iiwa_ros/command/cartesian_pose_linear.hpp"
#include <thread>

namespace iiwa_ros {
namespace command {

void CartesianPoseLinear::init(const std::string& robot_namespace) {
  setup(robot_namespace);
  initROS("CartesianPoseLinearCommand");
  command_.init(ros_namespace_ + "command/CartesianPoseLin");
}

void CartesianPoseLinear::setPose(const geometry_msgs::PoseStamped& pose) {
  command_.set(pose);
  command_.publish();
}

void CartesianPoseLinear::setPose(const geometry_msgs::PoseStamped& pose, const std::function<void()> callback) {
  setPose(pose);
  callback_ = callback;
  std::thread t(&CartesianPoseLinear::completedMotionWatcher, this);
  t.detach();
}

}  // namespace command
}  // namespace iiwa_ros
