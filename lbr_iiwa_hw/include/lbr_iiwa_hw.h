/** (c) 2015 Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
 *
 * This class implements a bridge between ROS hardware interfaces and a KUKA LBR IIWA Robot,
 * using an IIWARos communication described in the lbr_iiwa_ros package.
 * It is a porting of the work from the Centro E. Piaggio in Pisa : https://github.com/CentroEPiaggio/kuka-lwr
 * for the LBR IIWA.
 * 
 * \author Salvatore Virga
 * \version 1.0.0
 * \date 13/03/2015
 */

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

// IIWAMsg and ROS inteface includes
#include "IIWA/IIWAMsg.h"
#include "IIWARos.h"

#define DEFAULTCONTROLFREQUENCY 1000 // Hz

class IIWA_HW : public hardware_interface::RobotHW
{
public:
	/** 
	 * Constructor
	 */
	IIWA_HW(ros::NodeHandle nh);
	
	/** 
	 * Destructor
	 */
	virtual ~IIWA_HW();

	/** 
	 * \brief Initializes the IIWA device struct and all the hardware and joint limits interfaces needed.
	 * 	
	 * A joint state handle is created and linked to the current joint state of the IIWA robot.
	 * A joint position handle is created and linked  to the command joint position to send to the robot.
	 */
	bool start();
	
	/**
	 * \brief Registers the limits of the joint specified by joint_name and joint_handle. 
	 * 
	 * The limits are retrieved from the urdf_model.
	 * Returns the joint's type, lower position limit, upper position limit, and effort limit.
	 */
	void registerJointLimits(const std::string& joint_name,
			const hardware_interface::JointHandle& joint_handle,
			const urdf::Model *const urdf_model,
			double *const lower_limit, double *const upper_limit,
			double *const effort_limit);

	/**
	 * \brief Reads the current robot state via the IIWARos interfae and sends the values to the IIWA device struct.
	 */
	bool read(ros::Duration period);
	
	/**
	* \brief Sends the command joint position to the robot via IIWARos interface
	 */
	bool write(ros::Duration period);

	/**
	 * \brief Retuns the ros::Rate object to control the receiving/sending rate.
	 */
	ros::Rate* getRate();
	
	/**
	 * \brief Retuns the current frequency used by a ros::Rate object to control the receiving/sending rate.
	 */
	double getFrequency();
	
	/**
	 * \brief Set the frequency to be used by a ros::Rate object to control the receiving/sending rate.
	 */
	void setFrequency(double frequency);

	/** Structure for a lbr iiwa, joint handles, etc */
	struct IIWA_device
	{
		/** Vector containing the name of the joints - taken from yaml file */
		std::vector<std::string> joint_names;

		
		std::vector<double>
		joint_lower_limits, /**< Lower joint limits */
		joint_upper_limits, /**< Upper joint limits */
		joint_effort_limits; /**< Effort joint limits */
		
		/**< Joint state and commands */
		std::vector<double>
		joint_position,
		joint_position_prev,
		joint_velocity,
		joint_effort,
		joint_position_command,
		joint_stiffness_command,
		joint_damping_command,
		joint_effort_command;

		/** 
		 * \brief Initialze vectors
		 */
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

		/** 
		 * \brief Reset values of the vectors
		 */
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

	/* Node handle */
	ros::NodeHandle nh_;

	/* Parameters */
	std::string interface_;
	urdf::Model urdf_model_;
	
	hardware_interface::JointStateInterface state_interface_; /**< Interface for joint state */
	hardware_interface::EffortJointInterface effort_interface_; /**< Interface for joint impedance control */
	hardware_interface::PositionJointInterface position_interface_; /**< Interface for joint position control */

	/** Interfaces for limits */
	joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
	joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
	joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
	joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;

	boost::shared_ptr<IIWA_HW::IIWA_device> device_; /**< IIWA device. */

	/** Objects to control send/receive rate. */
	ros::Time timer_;
	ros::Rate* loop_rate_;
	double control_frequency_;

	IIWARos iiwa_ros_; /**< The IIWARos inteface from lbr_iiwa_ros. */
	
	IIWA::IIWAMsg current_IIWA_state_message_; /**< Contains the current state of the IIWA robot connected. */

	std::vector<std::string> interface_type_; /**< Contains the strings defining the possible hardware interfaces. */
};

#endif //LBR_IIWA_HW_H_
