#pragma once

#include <iiwa_msgs/JointPosition.h>
#include <iiwa_ros/command/generic_command.hpp>
#include <iiwa_ros/iiwa_ros.hpp>

namespace iiwa_ros {
namespace command {

class JointPosition : public GenericCommand {
public:
  JointPosition() = default;
  //  JointPosition(const std::string& ros_namespace) : GenericCommand{ros_namespace} { init(); }

  void setPosition(const iiwa_msgs::JointPosition& position);
  void setPosition(const iiwa_msgs::JointPosition& position, const std::function<void()> callback);
  void init(const std::string& robot_namespace) override;

private:
  iiwaCommandHolder<iiwa_msgs::JointPosition> command_{};
};

}  // namespace command
}  // namespace iiwa_ros
