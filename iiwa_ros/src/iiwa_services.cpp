#include <iiwa_services.h>
#include <iiwa_msgs/ControlMode.h>

namespace iiwa_ros {
	
	SmartServoService::SmartServoService() {
		initService();
	}
	
	SmartServoService::SmartServoService(const std::__cxx11::string& service_name, const bool verbose) : service_name_(service_name), verbose_(verbose){
		initService();
	}
	
	void SmartServoService::setServiceName(const std::__cxx11::string& service_name)
	{
		service_name_ = service_name;
		initService();
	}
	
	bool SmartServoService::setJointImpedanceMode(const iiwa_msgs::JointQuantity& joint_stiffnes, const iiwa_msgs::JointQuantity& joint_damping)
	{
		config_.request.control_mode = iiwa_msgs::ControlMode::JOINT_IMPEDANCE;
		config_.request.joint_impedance.joint_stiffness = joint_stiffnes;
		config_.request.joint_impedance.joint_damping = joint_damping;
		
		return callService();
	}
	
	bool SmartServoService::setCartesianImpedanceMode(const iiwa_msgs::CartesianQuantity& cartesian_stiffness, const iiwa_msgs::CartesianQuantity& cartesian_damping)
	{
		config_.request.control_mode = iiwa_msgs::ControlMode::CARTESIAN_IMPEDANCE;
		config_.request.cartesian_impedance.cartesian_stiffness = cartesian_stiffness;
		config_.request.cartesian_impedance.cartesian_damping = cartesian_damping;
		
		return callService();
		
		//TODO
	}
	
	bool SmartServoService::setCartesianSineImpedanceMode()
	{
		// TODO
	}
	
	bool SmartServoService::setDesiredForceMode(const int cartesian_dof, const double desired_force, const double desired_stiffness) {
		config_.request.control_mode = iiwa_msgs::ControlMode::DESIRED_FORCE;
		config_.request.desired_force.cartesian_dof = cartesian_dof;
		config_.request.desired_force.desired_force = desired_force;
		config_.request.desired_force.desired_stiffness = desired_stiffness;
		
		return callService();
	}
	
	bool SmartServoService::setLissajousPatterMode(const int cartesian_plane, const double frequency, const double amplitude, const double stiffness)
	{
		config_.request.control_mode = iiwa_msgs::ControlMode::LISSAJOUS_PATTERN;
		config_.request.lissajous_pattern.cartesian_plane = cartesian_plane;
		config_.request.lissajous_pattern.frequency = frequency;
		config_.request.lissajous_pattern.amplitude = amplitude;
		config_.request.lissajous_pattern.stiffness = stiffness;
		
		return callService();
	}
	
	bool SmartServoService::setSinePatternmode(const int cartesian_dof, const double frequency, const double amplitude, const double stiffness)
	{
		config_.request.control_mode = iiwa_msgs::ControlMode::SINE_PATTERN;
		config_.request.sine_pattern.cartesian_dof = cartesian_dof;
		config_.request.sine_pattern.frequency = frequency;
		config_.request.sine_pattern.amplitude = amplitude;
		config_.request.sine_pattern.stiffness = stiffness;
		
		return callService();
	}
	
	bool SmartServoService::setSpiralPatternMode(const int cartesian_plane, const double frequency, const double amplitude, const double stiffness, const double total_time)
	{
		config_.request.control_mode = iiwa_msgs::ControlMode::SPIRAL_PATTERN;
		config_.request.spiral_pattern.cartesian_plane = cartesian_plane;
		config_.request.spiral_pattern.frequency = frequency;
		config_.request.spiral_pattern.amplitude = amplitude;
		config_.request.spiral_pattern.stiffness = stiffness;
		
		return callService();
	}
	
	bool SmartServoService::callService()
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