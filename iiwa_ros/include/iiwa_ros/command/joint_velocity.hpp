#pragma once

#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_ros/command/generic_command.hpp>
#include <iiwa_ros/iiwa_ros.hpp>

namespace iiwa_ros {
namespace command {

class JointVelocity : public GenericCommand {
public:
  JointVelocity() = default;
  void init(const std::string& robot_namespace) override;

  void setVelocity(const iiwa_msgs::JointVelocity& velocity);
  void setVelocity(const iiwa_msgs::JointVelocity& velocity, const std::function<void()> callback);

private:
  Command<iiwa_msgs::JointVelocity> command_{};
};

}  // namespace command
}  // namespace iiwa_ros
