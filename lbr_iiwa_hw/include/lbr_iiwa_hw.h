#ifndef LBR_IIWA_HW_H_
#define LBR_IIWA_HW_H_

#include <vector>

// ROS headers
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/Duration.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <control_toolbox/filters.h>
#include <urdf/model.h>

// IIWAMsg include
#include "IIWA/IIWAMsg.h"
#include "IIWARos.h"

#define DEFAULTCONTROLFREQUENCY 800 // Hz

class IIWA_HW : public hardware_interface::RobotHW
{
public:
	// Constructor
	IIWA_HW(ros::NodeHandle nh);
	// Deconstructor
	virtual ~IIWA_HW();


	bool start();
	void registerJointLimits(const std::string& joint_name,
			const hardware_interface::JointHandle& joint_handle,
			const urdf::Model *const urdf_model,
			double *const lower_limit, double *const upper_limit,
			double *const effort_limit);

	bool read(ros::Duration period);
	bool write(ros::Duration period);

	ros::Rate* getRate();
	double getFrequency();
	void setFrequency(double frequency);

	// structure for a lbr iiwa, joint handles, etc
	struct IIWA_device
	{
		// Vector containing the name of the joints - taken from yaml file
		std::vector<std::string> joint_names;

		// joint limits
		std::vector<double>
		joint_lower_limits,
		joint_upper_limits,
		joint_effort_limits;

		// joint state and commands
		std::vector<double>
		joint_position,
		joint_position_prev,
		joint_velocity,
		joint_effort,
		joint_position_command,
		joint_stiffness_command,
		joint_damping_command,
		joint_effort_command;

		// Init vectors
		void init()
		{
			joint_position.resize(IIWA_DOF_JOINTS);
			joint_position_prev.resize(IIWA_DOF_JOINTS);
			joint_velocity.resize(IIWA_DOF_JOINTS);
			joint_effort.resize(IIWA_DOF_JOINTS);
			joint_position_command.resize(IIWA_DOF_JOINTS);
			joint_effort_command.resize(IIWA_DOF_JOINTS);
			joint_stiffness_command.resize(IIWA_DOF_JOINTS);
			joint_damping_command.resize(IIWA_DOF_JOINTS);

			joint_lower_limits.resize(IIWA_DOF_JOINTS);
			joint_upper_limits.resize(IIWA_DOF_JOINTS);
			joint_effort_limits.resize(IIWA_DOF_JOINTS);
		}

		// Reset values of the vectors
		void reset()
		{
			for (int j = 0; j < IIWA_DOF_JOINTS; ++j)
			{
				joint_position[j] = 0.0;
				joint_position_prev[j] = 0.0;
				joint_velocity[j] = 0.0;
				joint_effort[j] = 0.0;
				joint_position_command[j] = 0.0;
				joint_effort_command[j] = 0.0;

				// set default values for these two for now
				joint_stiffness_command[j] = 0.0;
				joint_damping_command[j] = 0.0;
			}
		}
	};

private:

	// Node handle
	ros::NodeHandle nh_;

	// Parameters
	std::string interface_;
	urdf::Model urdf_model_;

	// Interfaces
	hardware_interface::JointStateInterface state_interface_;
	hardware_interface::EffortJointInterface effort_interface_;
	hardware_interface::PositionJointInterface position_interface_;

	// Interfaces for limits
	joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
	joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
	joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
	joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;

	// IIWA_device
	boost::shared_ptr<IIWA_HW::IIWA_device> device_;

	ros::Time timer_;
	ros::Rate* loop_rate_;
	double control_frequency_;

	IIWARos iiwa_ros_;
	IIWA::IIWAMsg current_IIWA_state_message_;

	std::vector<std::string> interface_type_;
};

#endif //LBR_IIWA_HW_H_
