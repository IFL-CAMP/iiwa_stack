#pragma once

#include "iiwa_msgs/CartesianWrench.h"
#include "iiwa_ros/iiwa_ros.hpp"

namespace iiwa_ros {
namespace state {

class CartesianWrench : public Robot {
public:
  CartesianWrench() = default;
  virtual void init(const std::string& robot_namespace) override;

  iiwa_msgs::CartesianWrench getWrench();

private:
  State<iiwa_msgs::CartesianWrench> state_{};
};

}  // namespace state
}  // namespace iiwa_ros
