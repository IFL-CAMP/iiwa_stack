#include <path_parameters_service.h>
#include <iiwa_msgs/ControlMode.h>

namespace iiwa_ros {
	
	PathParametersService::PathParametersService() {
		initService();
	}
	
	PathParametersService::PathParametersService(const std::string& service_name, const bool verbose) : service_name_(service_name), verbose_(verbose){
		initService();
	}
	
	void PathParametersService::setServiceName(const std::string& service_name)
	{
		service_name_ = service_name;
		initService();
	}
	
	bool PathParametersService::setJointRelativeVelocity(const iiwa_msgs::JointQuantity& joint_relative_velocity)
	{
		return callService();	  
	}

	bool PathParametersService::setJointRelativeVelocity(const double& joint_relative_velocity)
	{
		return callService();	  
	}	
	
	bool PathParametersService::setJointRelativeAcceleration(const iiwa_msgs::JointQuantity& joint_relative_acceleration)
	{
		return callService();	  
	}
	
	bool PathParametersService::setJointRelativeAcceleration(const double& joint_relative_acceleration)
	{
		return callService();	  
	}	
	
	bool PathParametersService::setOverrideJointAcceleration(float64 override_joint_acceleration)
	{
		return callService();	  
	}
	
	bool setPathParameters(const iiwa_msgs::JointQuantity& joint_relative_velocity, const iiwa_msgs::JointQuantity& joint_relative_acceleration)
	{
		return callService();	  
	}
	
	bool setPathParameters(const double& joint_relative_velocity, const double& joint_relative_acceleration)
	{
		return callService();	  
	}
		
	bool setPathParameters(const iiwa_msgs::JointQuantity& joint_relative_velocity, const iiwa_msgs::JointQuantity& joint_relative_acceleration, float64 override_joint_acceleration)
	{
		return callService();	  
	}
		
	bool setPathParameters(const double& joint_relative_velocity, const double& joint_relative_acceleration, float64 override_joint_acceleration)
	{
		return callService();	  
	}
		
	
	bool PathParametersService::callService()
	{
		if (client_.call(config_)) {
			if(!config_.response.success && verbose_) {
				service_error_ = config_.response.error;
				ROS_ERROR_STREAM("SmartServoService failed, Java error: " << service_error_);
			}
			else if (verbose_) {
				ROS_INFO_STREAM(ros::this_node::getName() << ": SmartServoService successfully called.");
			}
		}
		else if (verbose_) {
			ROS_ERROR_STREAM("SmartServoService could not be called");
		}
		return config_.response.success;		
	}
}