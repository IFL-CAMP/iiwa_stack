#include "iiwa_ros/command/cartesian_pose.hpp"
#include <thread>

namespace iiwa_ros {
namespace command {

void CartesianPose::init(const std::string& robot_namespace) {
  setup(robot_namespace);
  initROS("CartesianPoseCommand");
  command_.init(ros_namespace_ + "command/CartesianPose");
}

void CartesianPose::setPose(const geometry_msgs::PoseStamped& pose) {
  command_.set(pose);
  command_.publish();
}

void CartesianPose::setPose(const geometry_msgs::PoseStamped& pose, const std::function<void()> callback) {
  setPose(pose);
  callback_ = callback;
  std::thread t(&CartesianPose::completedMotionWatcher, this);
  t.detach();
}

}  // namespace command
}  // namespace iiwa_ros
