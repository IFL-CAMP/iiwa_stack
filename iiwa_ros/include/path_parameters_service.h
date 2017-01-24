#pragma once

#include <iiwa_msgs/SetPathParameters.h>
#include <iiwa_services.hpp>

namespace iiwa_ros {
	
	class PathParametersService : public iiwaServices<iiwa_msgs::SetPathParameters> {
	public:
		PathParametersService(const std::string& service_name, const bool verbose = true);
		bool setJointRelativeVelocity(const double joint_relative_velocity);
		bool setJointRelativeAcceleration(const double joint_relative_acceleration);
		bool setOverrideJointAcceleration(const double override_joint_acceleration);
		bool setPathParameters(const double joint_relative_velocity, const double joint_relative_acceleration);
		bool setPathParameters(const double joint_relative_velocity, const double joint_relative_acceleration, const double override_joint_acceleration);
		
	protected:
		virtual bool callService();
	};
	
}