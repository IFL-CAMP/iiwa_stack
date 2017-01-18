#pragma once

#include <ros/ros.h>
#include <iiwa_msgs/TimeToDestination.h>
#include <iiwa_services.h>


namespace iiwa_ros {
	
	class TimeToDestinationService : public iiwaServices {
	public:
		TimeToDestinationService();
		TimeToDestinationService(const std::string& service_name, const bool verbose = true);

		bool getTimeToDestination();
		
		iiwa_msgs::TimeToDestination config_;			
	private:
	
		
	};
	
}