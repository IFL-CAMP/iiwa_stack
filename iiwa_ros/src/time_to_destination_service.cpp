#include <time_to_destination_service.h>
#include <iiwa_msgs/ControlMode.h>

namespace iiwa_ros {
	
	TimeToDestinationService::TimeToDestinationService() {
		initService();
	}
	
	TimeToDestinationService::TimeToDestinationService(const std::string& service_name, const bool verbose) : service_name(service_name), verbose(verbose){
		initService();
	}
	
	void TimeToDestinationService::setServiceName(const std::string& service_name)
	{
		service_name = service_name;
		initService();
	}
	
	bool TimeToDestinationService::getTimeToDestination(const iiwa_msgs::JointQuantity& joint_relative_velocity)
	{
		return callService();	  
	}
	
	bool TimeToDestinationService::callService()
	{
		if (client_.call(config_)) {
			if(verbose_) {

				ROS_INFO_STREAM(ros::this_node::getName() << ": SmartServoService successfully called.");
			}
		}
		else if (verbose_) {
			ROS_ERROR_STREAM("SmartServoService could not be called");
		}
		return 0;		
	}
}