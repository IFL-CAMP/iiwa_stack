#pragma once

#include <ros/ros.h>
#include <iiwa_msgs/SetPathParameters.h>
#include <iiwa_services.h>


namespace iiwa_ros {
	
	class PathParametersService : public iiwaServices {
	public:
		PathParametersService();
		PathParametersService(const std::string& service_name, const bool verbose = true);

		bool setJointRelativeVelocity(const iiwa_msgs::JointQuantity& joint_relative_velocity);
		bool setJointRelativeVelocity(const double& joint_relative_velocity);
		bool setJointRelativeAcceleration(const iiwa_msgs::JointQuantity& joint_relative_acceleration);
		bool setJointRelativeAcceleration(const double& joint_relative_acceleration);
		bool setOverrideJointAcceleration(float64 override_joint_acceleration);
		bool setPathParameters(const iiwa_msgs::JointQuantity& joint_relative_velocity, const iiwa_msgs::JointQuantity& joint_relative_acceleration);
		bool setPathParameters(const double& joint_relative_velocity, const double& joint_relative_acceleration);
		bool setPathParameters(const iiwa_msgs::JointQuantity& joint_relative_velocity, const iiwa_msgs::JointQuantity& joint_relative_acceleration, float64 override_joint_acceleration);
		bool setPathParameters(const double& joint_relative_velocity, const double& joint_relative_acceleration, float64 override_joint_acceleration);
		
			
		
	private:
		iiwa_msgs::SetPathParameters config_;	
		
	};
	
}