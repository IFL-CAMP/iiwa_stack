#pragma once

#include <ros/ros.h>
#include <iiwa_msgs/ConfigureSmartServo.h>
#include <iiwa_msgs/DOF.h>
#include <iiwa_msgs/CartesianPlane.h>
#include <smart_servo_service.h>


namespace iiwa_ros {
	
	class TimeToDestinationService : public iiwaServices {
	public:
		TimeToDestinationService();
		TimeToDestinationService(const std::string& service_name, const bool verbose = true);

		bool getTimeToDestination();
		
	};
	
}