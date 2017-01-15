#pragma once

#include <ros/ros.h>
#include <iiwa_msgs/ConfigureSmartServo.h>
#include <iiwa_msgs/DOF.h>
#include <iiwa_msgs/CartesianPlane.h>

namespace iiwa_ros {
	
	class iiwaServices {
	public:
		iiwaServices();
		iiwaServices(const std::string& service_name, const bool verbose = true);
		virtual setServiceName(const std::string& service_name);
		virtual void setVerbosity(const bool verbose) { verbose_ = verbose; }
		virtual std::string getLastError() { return service_error_; }
		

	private:
		std::string service_name_ = "";
		ros::NodeHandle nh_;
		ros::ServiceClient client_;
		iiwa_msgs::ConfigureSmartServo config_;
		bool verbose_ = true;
		std::string service_error_;

		void initService() { client_ = nh_.serviceClient<iiwa_msgs::ConfigureSmartServo>(service_name_); };
		virtual bool callService = 0;
		
		virtual void ciccio();
	};
	
}