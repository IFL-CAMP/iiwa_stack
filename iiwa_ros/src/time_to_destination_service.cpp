#include <time_to_destination_service.h>
#include <iiwa_msgs/ControlMode.h>

namespace iiwa_ros {
	TimeToDestinationService::TimeToDestinationService(const std::string& service_name, const bool verbose) : iiwaServices< iiwa_msgs::TimeToDestination >(service_name, verbose) {}
		
	double TimeToDestinationService::getTimeToDestination()
	{
		if (callService()) {
			return time_to_destination_;
		}
		else {
			return -999; // It cannot return -1 since it might be a meaningfull result.
		}
	}
	
	bool TimeToDestinationService::callService()
	{
		if (client_.call(config_)) {
			if(verbose_) {
				ROS_INFO_STREAM(ros::this_node::getName() << ": " << service_name_ << " successfully called.");
			}
			time_to_destination_ = config_.response.remaining_time;
			return true;
		}
		else if (verbose_) {
			ROS_ERROR_STREAM(service_name_ << " could not be called");
		}
		return false;		
	}
}