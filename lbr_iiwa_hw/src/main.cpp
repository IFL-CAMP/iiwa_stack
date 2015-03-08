#include "lbr_iiwa_hw.h"

#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <stdexcept>

bool g_quit = false;

void quitRequested(int sig) {
  g_quit = true;
}

int main( int argc, char** argv )
{
  // initialize ROS
  ros::init(argc, argv, "iiwa_core", ros::init_options::NoSigintHandler);
  
  // ros spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // custom signal handlers
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  // construct the lbr iiwa
  ros::NodeHandle lwr_nh("");
  IIWA_HW iiwa_robot(lwr_nh);

  // configuration routines
  iiwa_robot.start();

  // timer variables
  struct timespec ts = {0, 0};
  ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);
  ros::Duration period(1.0);

  //the controller manager
  controller_manager::ControllerManager manager(&iiwa_robot, lwr_nh);


  // run as fast as possible
  while( !g_quit ) 
  {
    // get the time / period
    if (!clock_gettime(CLOCK_REALTIME, &ts)) 
    {
      now.sec = ts.tv_sec;
      now.nsec = ts.tv_nsec;
      period = now - last;
      last = now;
    } 
    else 
    {
      ROS_FATAL("Failed to poll realtime clock!");
      break;
    } 
    
    // read current robot position
    iiwa_robot.read(period);
    
    // update the controllers
    manager.update(now, period);
    
    // send command position to the robot
    iiwa_robot.write(period);

    // wait for some milliseconds defined in controlFrequency
    iiwa_robot.getRate()->sleep();

  }

  std::cerr<<"Stopping spinner..."<<std::endl;
  spinner.stop();

  std::cerr<<"Bye!"<<std::endl;

  return 0;
}
