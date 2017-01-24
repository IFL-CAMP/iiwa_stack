#include <path_parameters_service.h>
#include <iiwa_msgs/ControlMode.h>

namespace iiwa_ros {
	
	PathParametersService::PathParametersService(const std::string& service_name, const bool verbose) : iiwaServices< iiwa_msgs::SetPathParameters >(service_name, verbose) {}
	
	bool PathParametersService::callService() {
		if (client_.call(config_)) {
			if(!config_.response.success && verbose_) {
				service_error_ = config_.response.error;
				ROS_ERROR_STREAM(service_name_ << " failed, Java error: " << service_error_);
			}
			else if (verbose_) {
				ROS_INFO_STREAM(ros::this_node::getName() << ":" << service_name_ << " successfully called.");
			}
		}
		else if (verbose_) {
			ROS_ERROR_STREAM(service_name_ << " could not be called");
		}
		return config_.response.success;
	}
	
	bool PathParametersService::setJointRelativeVelocity(const double joint_relative_velocity)
	{
		config_.request.joint_relative_velocity = joint_relative_velocity;
		return callService();	  
	}
	
	bool PathParametersService::setJointRelativeAcceleration(const double joint_relative_acceleration)
	{
		config_.request.joint_relative_acceleration = joint_relative_acceleration;
		return callService();	  
	}
		
	bool PathParametersService::setOverrideJointAcceleration(const double override_joint_acceleration)
	{
		config_.request.override_joint_acceleration = override_joint_acceleration;
		return callService();	  
	}
		
	bool PathParametersService::setPathParameters(const double joint_relative_velocity, const double joint_relative_acceleration)
	{
		config_.request.joint_relative_velocity = joint_relative_velocity;
		config_.request.joint_relative_acceleration = joint_relative_acceleration;
		return callService();	  
	}
				
	bool PathParametersService::setPathParameters(const double joint_relative_velocity, const double joint_relative_acceleration, const double override_joint_acceleration)
	{
		config_.request.joint_relative_velocity = joint_relative_velocity;
		config_.request.joint_relative_acceleration = joint_relative_acceleration;
		config_.request.override_joint_acceleration = override_joint_acceleration;
		return callService();		  
	}
		
}