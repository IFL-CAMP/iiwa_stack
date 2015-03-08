#include "lbr_iiwa_hw.h"

// IIWAMsg needed to monitor and command the real robot
IIWA::IIWAMsg receivedIIWAMessage;
IIWA::IIWAMsg goalState;
bool b_firstRun = true;

using namespace std;

IIWA_HW::IIWA_HW(ros::NodeHandle nh)
{
	_nh = nh;

	timer = ros::Time::now();
	controlFrequency = DEFAULTCONTROLFREQUENCY;
	loop_rate = new ros::Rate(controlFrequency);
}

IIWA_HW::~IIWA_HW() {}

ros::Rate* IIWA_HW::getRate()
{
	return loop_rate;
}

double IIWA_HW::getFrequency()
{
	return controlFrequency;
}

void IIWA_HW::setFrequency(double frequency)
{
	controlFrequency = frequency;
	loop_rate = new ros::Rate(controlFrequency);
}

bool IIWA_HW::start() {
	// construct a new IIWA device (interface and state storage)
	_device.reset( new IIWA_HW::IIWA_device() );

	// TODO : make use of this
	// get params or give default values
	_nh.param("interface", _interface, std::string("PositionJointInterface") );

	// TODO: use transmission configuration to get names directly from the URDF model
	if( ros::param::get("joints", _device->joint_names) )
	{
		if( !(_device->joint_names.size() == IIWA_DOF_JOINTS) )
		{
			ROS_ERROR("This robot has 7 joints, you must specify 7 names for each one");
		}
	}
	else
	{
		ROS_ERROR("No joints to be handled, ensure you load a yaml file naming the joint names this hardware interface refers to.");
		throw std::runtime_error("No joint name specification");
	}
	if( !(_urdf_model.initParam("/robot_description")) )
	{
		ROS_ERROR("No URDF model in the robot_description parameter, this is required to define the joint limits.");
		throw std::runtime_error("No URDF model available");
	}

	// initialize and set to zero the state and command values
	_device->init();
	_device->reset();

	// general joint to store information
	boost::shared_ptr<const urdf::Joint> joint;

	// create joint handles given the list
	for(int i = 0; i < IIWA_DOF_JOINTS; ++i)
	{
		ROS_INFO_STREAM("Handling joint: " << _device->joint_names[i]);

		// get current joint configuration
		joint = _urdf_model.getJoint(_device->joint_names[i]);
		if(!joint.get())
		{
			ROS_ERROR_STREAM("The specified joint "<< _device->joint_names[i] << " can't be found in the URDF model. Check that you loaded an URDF model in the robot description, or that you spelled correctly the joint name.");
			throw std::runtime_error("Wrong joint name specification");
		}

		// joint state handle
		hardware_interface::JointStateHandle state_handle(_device->joint_names[i],
				&_device->joint_position[i],
				&_device->joint_velocity[i],
				&_device->joint_effort[i]);

		_state_interface.registerHandle(state_handle);

		// position command handle
		hardware_interface::JointHandle position_joint_handle = hardware_interface::JointHandle(
				_state_interface.getHandle(_device->joint_names[i]),
				&_device->joint_position_command[i]);

		_position_interface.registerHandle(position_joint_handle);

		// effort command handle
		hardware_interface::JointHandle joint_handle = hardware_interface::JointHandle(
				_state_interface.getHandle(_device->joint_names[i]),
				&_device->joint_effort_command[i]);

		_effort_interface.registerHandle(joint_handle);

		registerJointLimits(_device->joint_names[i],
				joint_handle,
				&_urdf_model,
				&_device->joint_lower_limits[i],
				&_device->joint_upper_limits[i],
				&_device->joint_effort_limits[i]);
	}

	ROS_INFO("Register state and effort interfaces");

	// TODO: CHECK
	// register ros-controls interfaces
	this->registerInterface(&_state_interface);
	this->registerInterface(&_effort_interface);
	this->registerInterface(&_position_interface);

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
		_ej_limits_interface.registerHandle(limits_handle);
	}
	else
	{
		const joint_limits_interface::EffortJointSaturationHandle sat_handle(joint_handle, limits);
		_ej_sat_interface.registerHandle(sat_handle);
	}
}

bool IIWA_HW::read(ros::Duration period)
{
	ros::Duration delta = ros::Time::now() - timer;

	if (iiwaRos.read(currentIIWAStateMessage))
	{
		for (int j = 0; j < IIWA_DOF_JOINTS; j++)
		{
			_device->joint_position_prev[j] = _device->joint_position[j];
			_device->joint_position[j] = currentIIWAStateMessage.jointAngles[j];
			_device->joint_effort[j] = currentIIWAStateMessage.jointTorques[j];
			_device->joint_velocity[j] = filters::exponentialSmoothing((_device->joint_position[j]-_device->joint_position_prev[j])/period.toSec(), _device->joint_velocity[j], 0.2);
		}
		return 1;
	}
	else if (delta.toSec() >= 10) {
			cout << "No LBR IIWA is connected. Waiting for the robot to connect ..." << endl;
			timer = ros::Time::now();
	}
	return 0;
}

bool IIWA_HW::write(ros::Duration period)
{
	_ej_sat_interface.enforceLimits(period);
	_ej_limits_interface.enforceLimits(period);
	_pj_sat_interface.enforceLimits(period);
	_pj_limits_interface.enforceLimits(period);

	ros::Duration delta = ros::Time::now() - timer;

	//reading the force/torque values
	if (IIWARos::getRobotIsConnected())
	{
		if (b_firstRun)
		{
			goalState.cartPosition.resize(3);
			goalState.cartPosition = currentIIWAStateMessage.cartPosition;

			goalState.cartOrientation.resize(9);
			goalState.cartOrientation = currentIIWAStateMessage.cartOrientation;

			goalState.cartPositionStiffness.resize(3);
			goalState.cartPositionStiffness.at(0) = 50.0;
			goalState.cartPositionStiffness.at(1) = 50.0;
			goalState.cartPositionStiffness.at(2) = 500.0;

			b_firstRun = false;
		}

		// You need to set this flag to define the control type
		goalState.isJointControl = true;

		for (int i = 0; i < IIWA_DOF_JOINTS; i++)
		{
			goalState.jointAngles[i] = _device->joint_position_command[i];
		}

		iiwaRos.write(goalState);
	}
	else if (delta.toSec() >= 10) {
			cout << "No LBR IIWA is connected. Waiting for the robot to connect ..." << endl;
			timer = ros::Time::now();
	}
	return 0;
}


