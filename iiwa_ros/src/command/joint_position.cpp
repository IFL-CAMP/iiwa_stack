#include "iiwa_ros/command/joint_position.hpp"
#include <thread>

namespace iiwa_ros {
namespace command {

void JointPosition::init(const std::string& robot_namespace) {
  setup(robot_namespace);
  initROS("JointPositionCommand");
  command_.init(ros_namespace_ + "command/JointPosition");
}

void JointPosition::setPosition(const iiwa_msgs::JointPosition& position) {
  command_.set(position);
  command_.publishIfNew();
}

void JointPosition::setPosition(const iiwa_msgs::JointPosition& position, const std::function<void()> callback) {
  setPosition(position);
  callback_ = callback;
  std::thread t(&JointPosition::completedMotionWatcher, this);
  t.detach();
}

}  // namespace command
}  // namespace iiwa_ros
