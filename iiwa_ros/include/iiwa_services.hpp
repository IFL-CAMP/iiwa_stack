#pragma once

#include <ros/ros.h>
#include <iiwa_msgs/ConfigureSmartServo.h>

namespace iiwa_ros {
	
	template<typename T>
	class iiwaServices {
	public:
		/**
		 * @brief ...
		 * 
		 * @param service_name ...
		 * @param verbose ...
		 */
		iiwaServices(const std::string& service_name, const bool verbose = true) : service_name_(service_name), verbose_(verbose) {
			initService();
		}

		/**
		 * @brief ...
		 * 
		 * @return void
		 */
		virtual void initService() { client_ = nh_.serviceClient<T>(service_name_); };
		
		/**
		 * @brief ...
		 * 
		 * @return void
		 */
		virtual void setServiceName(const std::string& service_name) { service_name_ = service_name; initService();};
		
		/**
		 * @brief ...
		 * 
		 * @return void
		 */
		virtual void setVerbosity(const bool verbose) { verbose_ = verbose; }
		
		/**
		 * @brief ...
		 * 
		 * @return std::__cxx11::string
		 */
		virtual std::string getLastError() { return service_error_; }
		
	protected:
		/**
		 * @brief ...
		 * 
		 * @return bool
		 */
		virtual bool callService() = 0;
		
		std::string service_name_ = "";
		ros::NodeHandle nh_;
		ros::ServiceClient client_;
		T config_;
		bool verbose_ = true;
		std::string service_error_;
	};
	
}