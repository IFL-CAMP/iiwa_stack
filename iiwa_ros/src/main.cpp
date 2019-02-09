#include <ros/ros.h>
#include <iiwa_ros/state/joint_position.hpp>
#include <iiwa_ros/state/joint_torque.hpp>
#include <iostream>

#include <signal.h>

bool g_quit = false;

void quitRequested(int sig) { g_quit = true; }

int main() {
  iiwa_ros::state::JointPosition jp_state;
  iiwa_ros::state::JointTorque jt_state;

  jp_state.init("iiwa");
  jt_state.init("iiwa");
  // ros spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // custom signal handlers
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  while (!g_quit) {
    auto joint_position_ = jp_state.getPosition();

    auto joint_torque = jt_state.getTorque();
    ROS_INFO_STREAM(
        std::to_string(joint_position_.position.a1)
            << " " << std::to_string(joint_position_.position.a2) << " " << std::to_string(joint_position_.position.a3)
            << " " << std::to_string(joint_position_.position.a4) << " " << std::to_string(joint_position_.position.a5)
            << " " << std::to_string(joint_position_.position.a6) << " " << std::to_string(joint_position_.position.a7)
            << std::endl;);
    ros::Duration(0.1).sleep();
  }

  std::cerr << "Stopping spinner..." << std::endl;
  spinner.stop();

  std::cerr << "Bye!" << std::endl;

  return 0;
}
