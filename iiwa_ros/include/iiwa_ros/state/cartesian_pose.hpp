#pragma once

#include <iiwa_msgs/CartesianPose.h>
#include "iiwa_ros/iiwa_ros.hpp"

namespace iiwa_ros {
namespace state {

class CartesianPose : public Robot {
public:
  CartesianPose() = default;
  //  CartesianPose(const std::string& ros_namespace) : Robot{ros_namespace} {}

  iiwa_msgs::CartesianPose getPose();
  virtual void init(const std::string& robot_namespace) override;

private:
  iiwaStateHolder<iiwa_msgs::CartesianPose> state_{};
};

}  // namespace state
}  // namespace iiwa_ros
