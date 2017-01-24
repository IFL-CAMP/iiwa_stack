#pragma once

#include <iiwa_msgs/ConfigureSmartServo.h>
#include <iiwa_services.hpp>

namespace iiwa_ros {
	
	class SmartServoService : public iiwaServices<iiwa_msgs::ConfigureSmartServo> {
	public:
		SmartServoService(const std::string& service_name, const bool verbose = true);
		
		bool setJointImpedanceMode(const iiwa_msgs::JointQuantity& joint_stiffnes, const iiwa_msgs::JointQuantity& joint_damping);
		bool setJointImpedanceMode(const double joint_stiffnes, const double joint_damping);
		
		bool setCartesianImpedanceMode(const iiwa_msgs::CartesianQuantity& cartesian_stiffness, const iiwa_msgs::CartesianQuantity& cartesian_damping);
		bool setCartesianImpedanceMode(const double cartesian_stiffness, const double cartesian_damping);
		bool setCartesianImpedanceMode(const iiwa_msgs::CartesianQuantity& cartesian_stiffness, const iiwa_msgs::CartesianQuantity& cartesian_damping, 
									   const double nullspace_stiffness, const double nullspace_damping);
		bool setCartesianImpedanceMode(const double cartesian_stiffness, const double cartesian_damping, 
									   const double nullspace_stiffness, const double nullspace_damping);
				
		bool setDesiredForceMode(const int cartesian_dof, const  double desired_force, const double desired_stiffness);
		bool setSinePatternmode(const int cartesian_dof, const double frequency, const double amplitude, const double stiffness);
	protected:
		virtual bool callService();

	};
	
}