#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <iiwa_ros/command/generic_command.hpp>
#include <iiwa_ros/iiwa_ros.hpp>

namespace iiwa_ros {
namespace command {

class CartesianPose : public GenericCommand {
public:
  CartesianPose() = default;
  void init(const std::string& robot_namespace) override;

  void setPose(const geometry_msgs::PoseStamped& pose);
  void setPose(const geometry_msgs::PoseStamped& pose, const std::function<void()> callback);

private:
  Command<geometry_msgs::PoseStamped> command_{};
};

}  // namespace command
}  // namespace iiwa_ros
