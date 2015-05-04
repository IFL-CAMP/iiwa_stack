/** (c) 2015 Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
 * 
 * \author Salvatore Virga
 * \version 1.0.0
 * \date 27/04/2015
 */

#include "iiwaRos.h"

#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <stdexcept>

// Handles quit commands.
bool g_quit = false;

void quitRequested(int sig) 
{
  g_quit = true;
}

int main( int argc, char** argv )
{
  // Initialize ROS.
  ros::init(argc, argv, "iiwa_ros_example", ros::init_options::NoSigintHandler);
  
  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  // Custom signal handlers.
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);
  
  // The iiwa_msgs to send and receive joint position of the robot.
  iiwa_msgs::JointPosition receiveMessage, sendMessage;
  
  // The IIWA - ROS interface.
  iiwaRos iiwa_ros("my_iiwa");
  
  // Rate at which you want to send and receive messages.
  ros::Rate* loop_rate_ = new ros::Rate(1000);
  
  // BUILD YOUR MESSAGE TO SEND OR GET IT FROM SOMEWHERE ELSE
  // A GOOD IDEA IS TO HAVE ANOTHER FUNCTION THAT DOES THAT
  sendMessage.position.resize(IIWA_JOINTS);
  receiveMessage.position.resize(IIWA_JOINTS);
  
  sendMessage.position[0] = 0.0;
  sendMessage.position[1] = 0.0;
  sendMessage.position[2] = 0.0;
  sendMessage.position[3] = 0.0;
  sendMessage.position[4] = 0.0;
  sendMessage.position[5] = 0.0;
  sendMessage.position[6] = 0.0;
  
  while( !g_quit ) 
  {
    // Check if the iiwa Robot is connected and sending messages.
    if (iiwa_ros.getRobotIsConnected()) {
      
      // Check if a new JointPosition message is available
      if (iiwa_ros.isJointPositionAvailable())
      receiveMessage = iiwa_ros.getReceivedJointPosition();
      // You received a new IIWA state message, do something with it.
      
    }
    
    // Send command position to the robot.
    if (iiwa_ros.getRobotIsConnected()) {
      
      iiwa_ros.setCommandJointPosition(sendMessage);
      iiwa_ros.publish();
      // Your messange was sent to the robot.
    }
    
    // Wait for some milliseconds.
    loop_rate_->sleep();
    
  }
  
  std::cerr<<"Stopping spinner..."<<std::endl;
  spinner.stop();
  
  std::cerr<<"Bye!"<<std::endl;
  
  return 0;
}