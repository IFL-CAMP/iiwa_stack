#include "iiwa_ros/command/joint_velocity.hpp"
#include <thread>

namespace iiwa_ros {
namespace command {

void JointVelocity::init(const std::string& robot_namespace) {
  setup(robot_namespace);
  initROS("JointVelocityCommand");
  command_.init(ros_namespace_ + "command/JointVelocity");
}

void JointVelocity::setVelocity(const iiwa_msgs::JointVelocity& velocity) {
  command_.set(velocity);
  command_.publish();
}

void JointVelocity::setVelocity(const iiwa_msgs::JointVelocity& velocity, const std::function<void()> callback) {
  setVelocity(velocity);
  callback_ = callback;
  std::thread t(&JointVelocity::completedMotionWatcher, this);
  t.detach();
}

}  // namespace command
}  // namespace iiwa_ros
