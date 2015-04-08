#ifndef IIWAROSCONN_H_
#define IIWAROSCONN_H_

#include "boost/thread.hpp"
#include "time.h"

#include "ros/ros.h"

#include "iiwa_msgs/CartesianPosition.h"
#include "iiwa_msgs/CartesianRotation.h"
#include "iiwa_msgs/CartesianVelocity.h"
#include "iiwa_msgs/CartesianWrench.h"
#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointTorque.h"
#include "iiwa_msgs/JointVelocity.h"

#include <iostream>
#include <string>

#define IIWA_JOINTS 7

class iiwaRosConn {
  public:
	/**
	 * Class constructor
	 */
	iiwaRosConn();
	
	iiwaRosConn(string iiwaInitName);
	
	/**
	 * Class destructor
	 */
	virtual ~iiwaRosConn();
	
	/**
	 * Getters
	 */
	iiwa_msgs::CartesianPosition getCartesianPosition();
	iiwa_msgs::CartesianRotation getCartesianRotation();
	iiwa_msgs::CartesianVelocity getCartesianVelocity();
	iiwa_msgs::CartesianWrench getCartesianWrench();
	
	iiwa_msgs::JointPosition getJointPosition();
	iiwa_msgs::JointTorque getJointTorque();
	iiwa_msgs::JointVelocity getJointVelocity();
	
	/**
	 * Setters
	 */
	void setCartesianPosition(const iiwa_msgs::CartesianPosition& position);
	void setCartesianRotation(const iiwa_msgs::CartesianRotation& rotation);
	void setCartesianVelocity(const iiwa_msgs::CartesianVelocity& velocity);
	void setCartesianWrench(const iiwa_msgs::CartesianWrench& wrench);
	
	void setJointPosition(const iiwa_msgs::JointPosition& position);
	void setJointTorque(const iiwa_msgs::JointTorque& torque);
	void setJointVelocity(const iiwa_msgs::JointVelocity& velocity);
	
	/**
	 * \brief Sends a control state to the connected IIWA robot 
	 */
	bool publish();

	/**
	 * \brief Returns the current connection status of an IIWA robot.
	 */
	bool getRobotIsConnected();

private:
	/**
	 * Callback for the ROS Subscribers
	 */	
	void cartesianPositionCallback(const iiwa_msgs::CartesianPosition& position);
	void cartesianRotationCallback(const iiwa_msgs::CartesianRotation& rotation);
	void cartesianVelocityCallback(const iiwa_msgs::CartesianVelocity& velocity);
	void cartesianWrenchCallback(const iiwa_msgs::CartesianWrench& wrench);
	
	void jointPositionCallback(const iiwa_msgs::JointPosition& position);
	void jointTorqueCallback(const iiwa_msgs::JointTorque& torque);
	void jointVelocityCallback(const iiwa_msgs::JointVelocity& velocity);
	
	template <typename T> 
	bool publishIfSubscriber(const ros::Publisher& p, const T& message);
	
	void init();
	
	/**< ROS Publishers  */
	ros::Publisher cartesian_position_pub_;  
	ros::Publisher cartesian_rotation_pub_;
	ros::Publisher cartesian_velocity_pub_;
	ros::Publisher cartesian_wrench_pub_;
	
	ros::Publisher joint_position_pub_;
	ros::Publisher joint_torque_pub_;
	ros::Publisher joint_velocity_pub_;
	
	/**< ROS Subscribers */
	ros::Subscriber cartesian_position_sub_;
	ros::Subscriber cartesian_rotation_sub_;
	ros::Subscriber cartesian_velocity_sub_;
	ros::Subscriber cartesian_wrench_sub_;
	
	ros::Subscriber joint_position_sub_;
	ros::Subscriber joint_torque_sub_;
	ros::Subscriber joint_velocity_sub_;

	iiwa_msgs::CartesianPosition cartesian_position_;
	iiwa_msgs::CartesianRotation cartesian_rotation_;
	iiwa_msgs::CartesianVelocity cartesian_velocity_;
	iiwa_msgs::CartesianWrench cartesian_wrench_;
	
	iiwa_msgs::JointPosition joint_position_;
	iiwa_msgs::JointTorque joint_torque_;
	iiwa_msgs::JointVelocity joint_velocity_;
		
	bool robot_is_connected_; /**< Stores the current connection state */
	string iiwaName;
};

#endif //IIWAROCONNS_H_