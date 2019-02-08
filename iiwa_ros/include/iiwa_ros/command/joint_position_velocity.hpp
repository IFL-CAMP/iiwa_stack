#pragma once

#include <iiwa_msgs/JointPositionVelocity.h>
#include <iiwa_ros/command/generic_command.hpp>
#include <iiwa_ros/iiwa_ros.hpp>

namespace iiwa_ros {
namespace command {

class JointPositionVelocity : public GenericCommand {
public:
  JointPositionVelocity() = default;
  //  JointPositionVelocity(const std::string& ros_namespace) : GenericCommand{ros_namespace} {}

  void setPosition(const iiwa_msgs::JointPositionVelocity& position);
  void setPosition(const iiwa_msgs::JointPositionVelocity& position, const std::function<void()> callback);
  void init(const std::string& robot_namespace) override;

private:
  iiwaCommandHolder<iiwa_msgs::JointPositionVelocity> command_{};
};

}  // namespace command
}  // namespace iiwa_ros
