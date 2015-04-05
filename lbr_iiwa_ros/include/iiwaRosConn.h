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

#define IIWA_JOINTS 7

class iiwaRosConn {
  public:
	/**
	 * Class constructor
	 */
	iiwaRosConn();
	
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
	static void setCartesianPosition(const iiwa_msgs::CartesianPosition& position);
	static void setCartesianRotation(const iiwa_msgs::CartesianRotation& rotation);
	static void setCartesianVelocity(const iiwa_msgs::CartesianVelocity& velocity);
	static void setCartesianWrench(const iiwa_msgs::CartesianWrench& wrench);
	
	static void setJointPosition(const iiwa_msgs::JointPosition& position);
	static void setJointTorque(const iiwa_msgs::JointTorque& torque);
	static void setJointVelocity(const iiwa_msgs::JointVelocity& velocity);
	
	/**
	 * Copy
	 
	static void copyCartesianPosition(const iiwa_msgs::CartesianPosition& copy_position, iiwa_msgs::CartesianPosition& paste_position);
	static void copyCartesianRotation(const iiwa_msgs::CartesianRotation& copy_rotation, iiwa_msgs::CartesianRotation& paste_rotation);
	static void copyCartesianVelocity(const iiwa_msgs::CartesianVelocity& copy_velocity, iiwa_msgs::CartesianVelocity& paste_velocity);
	static void copyCartesianWrench(const iiwa_msgs::CartesianWrench& copy_wrench, iiwa_msgs::CartesianWrench& paste_wrench);
	
	static void copyJointPosition(const iiwa_msgs::JointPosition& copy_position, iiwa_msgs::JointPosition& paste_position);
	static void copyJointTorque(const iiwa_msgs::JointTorque& copy_torque, iiwa_msgs::JointTorque& paste_torque);
	static void copyJointVelocity(const iiwa_msgs::JointVelocity& copy_velocity, iiwa_msgs::JointVelocity& paste_velocity);
	*/
	
	/**
	 * Callback for the ROS Subscribers
	 */	
	static void cartesianPositionCallback(const iiwa_msgs::CartesianPosition& position);
	static void cartesianRotationCallback(const iiwa_msgs::CartesianRotation& rotation);
	static void cartesianVelocityCallback(const iiwa_msgs::CartesianVelocity& velocity);
	static void cartesianWrenchCallback(const iiwa_msgs::CartesianWrench& wrench);
	
	static void jointPositionCallback(const iiwa_msgs::JointPosition& position);
	static void jointTorqueCallback(const iiwa_msgs::JointTorque& torque);
	static void jointVelocityCallback(const iiwa_msgs::JointVelocity& velocity);
	
	/**
	 * \brief Sends a control state to the connected IIWA robot 
	 */
	bool publish();

	/**
	 * \brief Returns the current connection status of an IIWA robot.
	 */
	static bool getRobotIsConnected();

private:    
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

	static iiwa_msgs::CartesianPosition cartesian_position_;
	static iiwa_msgs::CartesianRotation cartesian_rotation_;
	static iiwa_msgs::CartesianVelocity cartesian_velocity_;
	static iiwa_msgs::CartesianWrench cartesian_wrench_;
	
	static iiwa_msgs::JointPosition joint_position_;
	static iiwa_msgs::JointTorque joint_torque_;
	static iiwa_msgs::JointVelocity joint_velocity_;
		
	static bool robot_is_connected_; /**< Stores the current connection state */
};

#endif //IIWAROCONNS_H_