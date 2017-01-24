#pragma once

#include <ros/ros.h>
#include <iiwa_msgs/ConfigureSmartServo.h>

namespace iiwa_ros {
	
	template<typename T>
	/**
	 * @brief This class provides a templated base class for service wrappers.
	 */
	class iiwaServices {
	public:
		/**
		 * @brief Creates a ROS Service client for the Service server with the given name
		 * 
		 * @param service_name Name of the ROS Service server to connect to.
		 * @param verbose If true some ROS_INFO messages will be printed out during service calls.
		 */
		iiwaServices(const std::string& service_name, const bool verbose = true) : service_name_(service_name), verbose_(verbose) {
			initService();
		}

		/**
		 * @brief Initializes a ROS Service client for the Service server with the name set using setServiceName.
		 * A client is created by the class constructor, this function might be useful to switch to another Service server with another name on the fly.
		 * To do that, first set the new server name with setServiceName.
		 */
		virtual void initService() { client_ = nh_.serviceClient<T>(service_name_); };
		
		/**
		 * @brief Sets the name of the ROS Service serve to connect to. Use initService to create a Service client to a service with this name.
		 */
		virtual void setServiceName(const std::string& service_name) { service_name_ = service_name; initService();};
		
		/**
		 * @brief Sets the verbosity level.
		 * If true some ROS_INFO messages will be printed out during service calls.
		 */
		virtual void setVerbosity(const bool verbose) { verbose_ = verbose; }
		
		/**
		 * @brief Returns the error string obtained from the last service call that produced an error.
		 * Available only if the implementation of the service call produces a string error in case of failure.
		 * 
		 * @return std::string 
		 */
		virtual std::string getLastError() { return service_error_; }
		
	protected:
		/**
		 * @brief Implements the actuall call to the service
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