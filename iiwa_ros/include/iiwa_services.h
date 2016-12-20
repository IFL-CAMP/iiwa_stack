#pragma once

#include <ros/ros.h>
#include <iiwa_msgs/ConfigureSmartServo.h>
#include <iiwa_msgs/DOF.h>
#include <iiwa_msgs/CartesianPlane.h>

namespace iiwa_ros {
	
	class SmartServoService {
	public:
		SmartServoService();
		SmartServoService(const std::string& service_name, const bool verbose = true);
		void setServiceName(const std::string& service_name);
		void setVerbosity(const bool verbose) { verbose_ = verbose; }
		std::string getLastError() { return service_error_; }
		
		bool setJointImpedanceMode(const iiwa_msgs::JointQuantity& joint_stiffnes, const iiwa_msgs::JointQuantity& joint_damping);
		
		//TODO : find a proper way to call these two
		bool setCartesianImpedanceMode(const iiwa_msgs::CartesianQuantity& cartesian_stiffness, const iiwa_msgs::CartesianQuantity& cartesian_damping);
		bool setCartesianSineImpedanceMode();
		
		bool setDesiredForceMode(const int cartesian_dof, const  double desired_force, const double desired_stiffness);
		bool setSinePatternmode(const int cartesian_dof, const double frequency, const double amplitude, const double stiffness);
		bool setLissajousPatterMode(const int cartesian_plane, const double frequency, const double amplitude, const double stiffness);
		bool setSpiralPatternMode(const int cartesian_plane, const double frequency, const double amplitude, const double stiffness, const double total_time);
		
	private:
		std::string service_name_ = "/iiwa/configuration/configureSmartServo";
		ros::NodeHandle nh_;
		ros::ServiceClient client_;
		iiwa_msgs::ConfigureSmartServo config_;
		bool verbose_ = true;
		std::string service_error_;

		void initService() { client_ = nh_.serviceClient<iiwa_msgs::ConfigureSmartServo>(service_name_); };
		bool callService();
		
		void ciccio();
	};
	
}