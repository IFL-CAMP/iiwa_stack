#pragma once

#include <iiwa_msgs/JointPositionVelocity.h>
#include <iiwa_ros/command/generic_command.hpp>
#include <iiwa_ros/iiwa_ros.hpp>

namespace iiwa_ros {
namespace command {

class JointPositionVelocity : public GenericCommand {
public:
  JointPositionVelocity() = default;
  void init(const std::string& robot_namespace) override;

  void setPosition(const iiwa_msgs::JointPositionVelocity& position);
  void setPosition(const iiwa_msgs::JointPositionVelocity& position, const std::function<void()> callback);

private:
  Command<iiwa_msgs::JointPositionVelocity> command_{};
};

}  // namespace command
}  // namespace iiwa_ros
