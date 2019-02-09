#pragma once

#include <iiwa_msgs/CartesianPose.h>
#include "iiwa_ros/iiwa_ros.hpp"

namespace iiwa_ros {
namespace state {

class CartesianPose : public Robot {
public:
  CartesianPose() = default;
  virtual void init(const std::string& robot_namespace) override;

  iiwa_msgs::CartesianPose getPose();

private:
  State<iiwa_msgs::CartesianPose> state_{};
};

}  // namespace state
}  // namespace iiwa_ros
