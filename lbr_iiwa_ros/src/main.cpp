#include "IIWARos.h"

#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <stdexcept>

bool g_quit = false;

void quitRequested(int sig) 
{
  g_quit = true;
}

int main( int argc, char** argv )
{
  // initialize ROS
  ros::init(argc, argv, "lbr_iiwa_ros_example", ros::init_options::NoSigintHandler);
  
  // ros spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // custom signal handlers
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  // The IIWAMsg to send and receive states of the robot
  IIWA::IIWAMsg receiveMessage, sendMessage;
  
  // The IIWA - ROS interface
  IIWARos iiwa_ros;
  
  
  ros::Rate* loop_rate_ = new ros::Rate(1000);
  
  // BUILD YOUR MESSAGE TO SEND OR GET IT FROM SOMEWHERE ELSE
  // A GOOD IDEA IS TO HAVE ANOTHER FUNCTION THAT DOES THAT
  sendMessage.isJointControl = true;
  sendMessage.jointAngles.resize(7);
  
  sendMessage.jointAngles[0] = 0.0;
  sendMessage.jointAngles[1] = 0.0;
  sendMessage.jointAngles[2] = 0.0;
  sendMessage.jointAngles[3] = 0.0;
  sendMessage.jointAngles[4] = 0.0;
  sendMessage.jointAngles[5] = 0.0;
  sendMessage.jointAngles[6] = 0.0;

  while( !g_quit ) 
  {
    
    if (iiwa_ros.read(receiveMessage)) {
      
      // You received a new IIWA state message, do something with it.
      
    }
    
    // send command position to the robot
    if (iiwa_ros.write(sendMessage)) {
      
      // Your messange was sent to the robot.
      
    }
    
    // Wait for some milliseconds
    loop_rate_->sleep();
    
  }

  std::cerr<<"Stopping spinner..."<<std::endl;
  spinner.stop();

  std::cerr<<"Bye!"<<std::endl;

  return 0;
}