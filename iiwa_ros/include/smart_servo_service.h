#pragma once

#include <ros/ros.h>
#include <iiwa_msgs/ConfigureSmartServo.h>
#include <iiwa_msgs/DOF.h>
#include <iiwa_msgs/CartesianPlane.h>
#include <iiwa_services.h>

namespace iiwa_ros {
	
	class SmartServoService : public iiwaServices {
	public:
		SmartServoService();
		SmartServoService(const std::string& service_name, const bool verbose = true);

		bool setJointImpedanceMode(const iiwa_msgs::JointQuantity& joint_stiffnes, const iiwa_msgs::JointQuantity& joint_damping);
		bool setJointImpedanceMode(const double& joint_stiffnes, const double& joint_damping);
		
		//TODO : find a proper way to call these two
		bool setCartesianImpedanceMode(const iiwa_msgs::CartesianQuantity& cartesian_stiffness, const iiwa_msgs::CartesianQuantity& cartesian_damping);
		bool setCartesianImpedanceMode(const double& cartesian_stiffness, const double& cartesian_damping);

		bool setCartesianSineImpedanceMode();
		
		bool setDesiredForceMode(const int cartesian_dof, const  double desired_force, const double desired_stiffness);
		bool setSinePatternmode(const int cartesian_dof, const double frequency, const double amplitude, const double stiffness);
		bool setLissajousPatterMode(const int cartesian_plane, const double frequency, const double amplitude, const double stiffness);
		bool setSpiralPatternMode(const int cartesian_plane, const double frequency, const double amplitude, const double stiffness, const double total_time);
		
	};
	
}