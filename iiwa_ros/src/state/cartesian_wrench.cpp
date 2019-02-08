#include "iiwa_ros/state/cartesian_wrench.hpp"

namespace iiwa_ros {
namespace state {

void CartesianWrench::init(const std::string& robot_namespace) {
  setup(robot_namespace);
  initROS("CartesianWrenchState");
  state_.init(ros_namespace_ + "state/CartesianWrench");
}

iiwa_msgs::CartesianWrench CartesianWrench::getWrench() {
  iiwa_msgs::CartesianWrench cw;
  state_.get(cw);
  return cw;
}

}  // namespace state
}  // namespace iiwa_ros
