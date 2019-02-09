#pragma once

#include <std_msgs/Time.h>
#include "iiwa_ros/iiwa_ros.hpp"

namespace iiwa_ros {
namespace state {

class DestinationReached : public Robot {
public:
  DestinationReached() = default;
  virtual void init(const std::string& robot_namespace) override;

  std_msgs::Time getTime();

private:
  State<std_msgs::Time> state_{};
};

}  // namespace state
}  // namespace iiwa_ros
