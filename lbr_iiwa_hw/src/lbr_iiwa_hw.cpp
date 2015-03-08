#include "lbr_iiwa_hw.h"

bool first_run = true;

using namespace std;

IIWA_HW::IIWA_HW(ros::NodeHandle nh)
{
	nh_ = nh;

	timer_ = ros::Time::now();
	control_frequency_ = DEFAULTCONTROLFREQUENCY;
	loop_rate_ = new ros::Rate(control_frequency_);

	interface_type_.push_back("PositionJointInterface");
	interface_type_.push_back("EffortJointInterface");
	interface_type_.push_back("VelocityJointInterface");
}

IIWA_HW::~IIWA_HW() {}

ros::Rate* IIWA_HW::getRate()
{
	return loop_rate_;
}

double IIWA_HW::getFrequency()
{
	return control_frequency_;
}

void IIWA_HW::setFrequency(double frequency)
{
	control_frequency_ = frequency;
	loop_rate_ = new ros::Rate(control_frequency_);
}

bool IIWA_HW::start() {
	// construct a new IIWA device (interface and state storage)
	device_.reset( new IIWA_HW::IIWA_device() );

	// TODO : make use of this
	// get params or give default values
	nh_.param("interface", interface_, std::string("PositionJointInterface") );

	// TODO: use transmission configuration to get names directly from the URDF model
	if( ros::param::get("joints", device_->joint_names) )
	{
		if( !(device_->joint_names.size() == IIWA_DOF_JOINTS) )
		{
			ROS_ERROR("This robot has 7 joints, you must specify 7 names for each one");
		}
	}
	else
	{
		ROS_ERROR("No joints to be handled, ensure you load a yaml file naming the joint names this hardware interface refers to.");
		throw std::runtime_error("No joint name specification");
	}
	if( !(urdf_model_.initParam("/robot_description")) )
	{
		ROS_ERROR("No URDF model in the robot_description parameter, this is required to define the joint limits.");
		throw std::runtime_error("No URDF model available");
	}

	// initialize and set to zero the state and command values
	device_->init();
	device_->reset();

	// general joint to store information
	boost::shared_ptr<const urdf::Joint> joint;

	// create joint handles given the list
	for(int i = 0; i < IIWA_DOF_JOINTS; ++i)
	{
		ROS_INFO_STREAM("Handling joint: " << device_->joint_names[i]);

		// get current joint configuration
		joint = urdf_model_.getJoint(device_->joint_names[i]);
		if(!joint.get())
		{
			ROS_ERROR_STREAM("The specified joint "<< device_->joint_names[i] << " can't be found in the URDF model. Check that you loaded an URDF model in the robot description, or that you spelled correctly the joint name.");
			throw std::runtime_error("Wrong joint name specification");
		}

		// joint state handle
		hardware_interface::JointStateHandle state_handle(device_->joint_names[i],
				&device_->joint_position[i],
				&device_->joint_velocity[i],
				&device_->joint_effort[i]);

		state_interface_.registerHandle(state_handle);

		// position command handle
		hardware_interface::JointHandle position_joint_handle = hardware_interface::JointHandle(
				state_interface_.getHandle(device_->joint_names[i]),
				&device_->joint_position_command[i]);

		position_interface_.registerHandle(position_joint_handle);

		// effort command handle
		hardware_interface::JointHandle joint_handle = hardware_interface::JointHandle(
				state_interface_.getHandle(device_->joint_names[i]),
				&device_->joint_effort_command[i]);

		effort_interface_.registerHandle(joint_handle);

		registerJointLimits(device_->joint_names[i],
				joint_handle,
				&urdf_model_,
				&device_->joint_lower_limits[i],
				&device_->joint_upper_limits[i],
				&device_->joint_effort_limits[i]);
	}

	ROS_INFO("Register state and effort interfaces");

	// TODO: CHECK
	// register ros-controls interfaces
	this->registerInterface(&state_interface_);
	this->registerInterface(&effort_interface_);
	this->registerInterface(&position_interface_);

	return true;
}

// Register the limits of the joint specified by joint_name and joint_handle. The limits are
// retrieved from the urdf_model.
// Return the joint's type, lower position limit, upper position limit, and effort limit.
void IIWA_HW::registerJointLimits(const std::string& joint_name,
		const hardware_interface::JointHandle& joint_handle,
		const urdf::Model *const urdf_model,
		double *const lower_limit, double *const upper_limit,
		double *const effort_limit)
{
	*lower_limit = -std::numeric_limits<double>::max();
	*upper_limit = std::numeric_limits<double>::max();
	*effort_limit = std::numeric_limits<double>::max();

	joint_limits_interface::JointLimits limits;
	bool has_limits = false;
	joint_limits_interface::SoftJointLimits soft_limits;
	bool has_soft_limits = false;

	if (urdf_model != NULL)
	{
		const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);
		if (urdf_joint != NULL)
		{
			// Get limits from the URDF file.
			if (joint_limits_interface::getJointLimits(urdf_joint, limits))
				has_limits = true;
			if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
				has_soft_limits = true;
		}
	}

	if (!has_limits)
		return;

	if (limits.has_position_limits)
	{
		*lower_limit = limits.min_position;
		*upper_limit = limits.max_position;
	}
	if (limits.has_effort_limits)
		*effort_limit = limits.max_effort;

	if (has_soft_limits)
	{
		const joint_limits_interface::EffortJointSoftLimitsHandle limits_handle(joint_handle, limits, soft_limits);
		ej_limits_interface_.registerHandle(limits_handle);
	}
	else
	{
		const joint_limits_interface::EffortJointSaturationHandle sat_handle(joint_handle, limits);
		ej_sat_interface_.registerHandle(sat_handle);
	}
}

bool IIWA_HW::read(ros::Duration period)
{
	ros::Duration delta = ros::Time::now() - timer_;

	if (iiwaRos.read(current_IIWA_state_message_))
	{
		for (int j = 0; j < IIWA_DOF_JOINTS; j++)
		{
			device_->joint_position_prev[j] = device_->joint_position[j];
			device_->joint_position[j] = current_IIWA_state_message_.jointAngles[j];
			device_->joint_effort[j] = current_IIWA_state_message_.jointTorques[j];
			device_->joint_velocity[j] = filters::exponentialSmoothing((device_->joint_position[j]-device_->joint_position_prev[j])/period.toSec(), device_->joint_velocity[j], 0.2);
		}
		return 1;
	}
	else if (delta.toSec() >= 10) {
		cout << "No LBR IIWA is connected. Waiting for the robot to connect ..." << endl;
		timer_ = ros::Time::now();
	}
	return 0;
}

bool IIWA_HW::write(ros::Duration period)
{
	ej_sat_interface_.enforceLimits(period);
	ej_limits_interface_.enforceLimits(period);
	pj_sat_interface_.enforceLimits(period);
	pj_limits_interface_.enforceLimits(period);

	ros::Duration delta = ros::Time::now() - timer_;

	IIWA::IIWAMsg goalState;
	//reading the force/torque values
	if (IIWARos::getRobotIsConnected())
	{
		if (first_run)
		{
			goalState.cartPosition.resize(3);
			goalState.cartPosition = current_IIWA_state_message_.cartPosition;

			goalState.cartOrientation.resize(9);
			goalState.cartOrientation = current_IIWA_state_message_.cartOrientation;

			goalState.cartPositionStiffness.resize(3);
			goalState.cartPositionStiffness.at(0) = 50.0;
			goalState.cartPositionStiffness.at(1) = 50.0;
			goalState.cartPositionStiffness.at(2) = 500.0;

			first_run = false;
		}

		// Joint Position Control
		if (interface_ == interface_type_.at(0)) {
			// You need to set this flag to define the control type
			goalState.isJointControl = true;
			goalState.jointAngles.resize(IIWA_DOF_JOINTS);
			for (int i = 0; i < IIWA_DOF_JOINTS; i++)
			{
				goalState.jointAngles[i] = device_->joint_position_command[i];
			}
		}
		// Joint Impedance Control
		else if (interface_ == interface_type_.at(1)){
			// TODO
		}
		// Joint Velocity Control
		else if (interface_ == interface_type_.at(2)){
			//	TODO
		}

		iiwaRos.write(goalState);
	}
	else if (delta.toSec() >= 10) {
		cout << "No LBR IIWA is connected. Waiting for the robot to connect ..." << endl;
		timer_ = ros::Time::now();
	}
	return 0;
}


