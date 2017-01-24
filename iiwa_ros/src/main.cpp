#include "smart_servo_service.h"
#include "time_to_destination_service.h"
#include "path_parameters_service.h"
#include "ros/ros.h"

int main( int argc, char** argv ) {
    // Initialize ROS.
    ros::init(argc, argv, "iiwa_ros_example", ros::init_options::NoSigintHandler);
    
    // ROS spinner.
    ros::AsyncSpinner spinner(1);
    spinner.start();
        
    // Rate at which you want to send and receive messages.
    ros::Rate* loop_rate_ = new ros::Rate(60);
    
    iiwa_ros::SmartServoService sss("", true);
    iiwa_ros::PathParametersService pps("", true);
    iiwa_ros::TimeToDestinationService tds("", true);
    
    while( ros::ok() ) {
        loop_rate_->sleep();
    }
    
    std::cerr<<"Stopping spinner..."<<std::endl;
    spinner.stop();
    
    std::cerr<<"Bye!"<<std::endl;
    
    return 0;
}