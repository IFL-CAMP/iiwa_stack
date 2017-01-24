#include <smart_servo_service.h>
#include <iiwa_msgs/ControlMode.h>
#include <conversions.h>

namespace iiwa_ros {	
	SmartServoService::SmartServoService(const std::string& service_name, const bool verbose) : iiwaServices<iiwa_msgs::ConfigureSmartServo>(service_name, verbose) {}
	
	bool SmartServoService::callService() {
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

		
	bool SmartServoService::setJointImpedanceMode(const iiwa_msgs::JointQuantity& joint_stiffnes, const iiwa_msgs::JointQuantity& joint_damping) {
		config_.request.control_mode = iiwa_msgs::ControlMode::JOINT_IMPEDANCE;
		config_.request.joint_impedance.joint_stiffness = joint_stiffnes;
		config_.request.joint_impedance.joint_damping = joint_damping;
		
		return callService();
	}
	
	bool SmartServoService::setJointImpedanceMode(const double joint_stiffnes, const double joint_damping) {
		config_.request.control_mode = iiwa_msgs::ControlMode::JOINT_IMPEDANCE;
		config_.request.joint_impedance.joint_stiffness = jointQuantityFromDouble(joint_stiffnes);;
		config_.request.joint_impedance.joint_damping = jointQuantityFromDouble(joint_damping);;
		
		return callService();
	}	
	
	bool SmartServoService::setCartesianImpedanceMode(const iiwa_msgs::CartesianQuantity& cartesian_stiffness, const iiwa_msgs::CartesianQuantity& cartesian_damping) {
		config_.request.control_mode = iiwa_msgs::ControlMode::CARTESIAN_IMPEDANCE;
		config_.request.cartesian_impedance.cartesian_stiffness = cartesian_stiffness;
		config_.request.cartesian_impedance.cartesian_damping = cartesian_damping;
		config_.request.cartesian_impedance.nullspace_stiffness = -1;
		config_.request.cartesian_impedance.nullspace_damping = -1;
		
		return callService();
	}
	
	bool SmartServoService::setCartesianImpedanceMode(const double cartesian_stiffness, const double cartesian_damping) {
		config_.request.control_mode = iiwa_msgs::ControlMode::CARTESIAN_IMPEDANCE;
		config_.request.cartesian_impedance.cartesian_stiffness = CartesianQuantityFromDouble(cartesian_stiffness);
		config_.request.cartesian_impedance.cartesian_damping = CartesianQuantityFromDouble(cartesian_damping);
		config_.request.cartesian_impedance.nullspace_stiffness = -1;
		config_.request.cartesian_impedance.nullspace_damping = -1;
		
		return callService();
	}	
	
	bool SmartServoService::setCartesianImpedanceMode(const iiwa_msgs::CartesianQuantity& cartesian_stiffness, const iiwa_msgs::CartesianQuantity& cartesian_damping, 
								const double nullspace_stiffness, const double nullspace_damping) {
	// TODO: fill all the values
		config_.request.control_mode = iiwa_msgs::ControlMode::CARTESIAN_IMPEDANCE;
		config_.request.cartesian_impedance.cartesian_stiffness = cartesian_stiffness;
		config_.request.cartesian_impedance.cartesian_damping = cartesian_damping;
		config_.request.cartesian_impedance.nullspace_stiffness = nullspace_stiffness;
		config_.request.cartesian_impedance.nullspace_damping = nullspace_damping;
		
		return callService();
	}
	
	bool SmartServoService::setCartesianImpedanceMode(const double cartesian_stiffness, const double cartesian_damping, 
								const double nullspace_stiffness, const double nullspace_damping) {
	// TODO: fill all the values
		config_.request.control_mode = iiwa_msgs::ControlMode::CARTESIAN_IMPEDANCE;
		config_.request.cartesian_impedance.cartesian_stiffness = CartesianQuantityFromDouble(cartesian_stiffness);
		config_.request.cartesian_impedance.cartesian_damping = CartesianQuantityFromDouble(cartesian_damping);
		config_.request.cartesian_impedance.nullspace_stiffness = nullspace_stiffness;
		config_.request.cartesian_impedance.nullspace_damping = nullspace_damping;
		
		return callService();
	}
		
	bool SmartServoService::setDesiredForceMode(const int cartesian_dof, const double desired_force, const double desired_stiffness) {
		config_.request.control_mode = iiwa_msgs::ControlMode::DESIRED_FORCE;
		config_.request.desired_force.cartesian_dof = cartesian_dof;
		config_.request.desired_force.desired_force = desired_force;
		config_.request.desired_force.desired_stiffness = desired_stiffness;
		
		return callService();
	}
	
	bool SmartServoService::setSinePatternmode(const int cartesian_dof, const double frequency, const double amplitude, const double stiffness) {
		config_.request.control_mode = iiwa_msgs::ControlMode::SINE_PATTERN;
		config_.request.sine_pattern.cartesian_dof = cartesian_dof;
		config_.request.sine_pattern.frequency = frequency;
		config_.request.sine_pattern.amplitude = amplitude;
		config_.request.sine_pattern.stiffness = stiffness;
		
		return callService();
	}

}