#pragma once

#include <std_msgs/Time.h>
#include "iiwa_ros/iiwa_ros.hpp"

namespace iiwa_ros {
namespace state {

class DestinationReached : public Robot {
public:
  DestinationReached() = default;

  std_msgs::Time getTime();
  virtual void init(const std::string& robot_namespace) override;

private:
  iiwaStateHolder<std_msgs::Time> state_{};
};

}  // namespace state
}  // namespace iiwa_ros
