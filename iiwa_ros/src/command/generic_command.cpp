#include "iiwa_ros/command/generic_command.hpp"

namespace iiwa_ros {

namespace command {

void GenericCommand::completedMotionWatcher() {
  bool flag{false};
  ros::Duration(0.25).sleep();
  while (true) {
    auto missing_time = time_to_destination_.getTimeToDestination();
    if (missing_time < -998) {
      ROS_ERROR_STREAM("IIWA_ROS - Error while asking for Time to Destination.");
      continue;
    }
    if (missing_time > 0) {
      if (flag == false) { flag = true; }
      ros::Duration(missing_time / 2).sleep();
    } else {
      if (flag == true) {
        callback_();
        return;
      }
    }
  }
}

}  // namespace command
}  // namespace iiwa_ros
