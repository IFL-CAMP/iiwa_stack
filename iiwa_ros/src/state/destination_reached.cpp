#include "iiwa_ros/state/destination_reached.hpp"

namespace iiwa_ros {
namespace state {

void DestinationReached::init(const std::string& robot_namespace) {
  setup(robot_namespace);
  initROS("DestinationReachedState");
  state_.init(ros_namespace_ + "state/DestinationReached");
}

std_msgs::Time DestinationReached::getTime() {
  std_msgs::Time t;
  state_.get(t);
  return t;
}

}  // namespace state
}  // namespace iiwa_ros
