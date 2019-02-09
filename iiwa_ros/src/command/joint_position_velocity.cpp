#include "iiwa_ros/command/joint_position_velocity.hpp"
#include <thread>

namespace iiwa_ros {
namespace command {

void JointPositionVelocity::init(const std::string& robot_namespace) {
  setup(robot_namespace);
  initROS("JointPositionVelocityCommand");
  command_.init(ros_namespace_ + "command/JointPositionVelocity");
}

void JointPositionVelocity::setPosition(const iiwa_msgs::JointPositionVelocity& position) {
  command_.set(position);
  command_.publish();
}

void JointPositionVelocity::setPosition(const iiwa_msgs::JointPositionVelocity& position,
                                        const std::function<void()> callback) {
  setPosition(position);
  callback_ = callback;
  std::thread t(&JointPositionVelocity::completedMotionWatcher, this);
  t.detach();
}

}  // namespace command
}  // namespace iiwa_ros
