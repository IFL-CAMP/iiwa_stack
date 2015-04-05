#include "iiwaRosConn.h"

using namespace std;

bool iiwaRosConn::robot_is_connected_ = false;

iiwa_msgs::CartesianPosition iiwaRosConn::cartesian_position_;
iiwa_msgs::CartesianRotation iiwaRosConn::cartesian_rotation_;
iiwa_msgs::CartesianVelocity iiwaRosConn::cartesian_velocity_;
iiwa_msgs::CartesianWrench iiwaRosConn::cartesian_wrench_;
	
iiwa_msgs::JointPosition iiwaRosConn::joint_position_;
iiwa_msgs::JointTorque iiwaRosConn::joint_torque_;
iiwa_msgs::JointVelocity iiwaRosConn::joint_velocity_;

iiwaRosConn::iiwaRosConn()
{
  ros::NodeHandle node_handle;
    
  cartesian_rotation_.rotation.resize(9);
  cartesian_velocity_.velocity.resize(IIWA_JOINTS);
  joint_position_.position.resize(IIWA_JOINTS);
  joint_torque_.torque.resize(IIWA_JOINTS);
  joint_velocity_.velocity.resize(IIWA_JOINTS);
  
  cartesian_position_pub_ = node_handle.advertise<iiwa_msgs::CartesianPosition>("/iiwa/command/CartesianPosition",1);
  cartesian_rotation_pub_ = node_handle.advertise<iiwa_msgs::CartesianRotation>("/iiwa/command/CartesianRotation",1);
  cartesian_velocity_pub_ = node_handle.advertise<iiwa_msgs::CartesianVelocity>("/iiwa/command/CartesianVelocity",1);
  cartesian_wrench_pub_ = node_handle.advertise<iiwa_msgs::CartesianWrench>("/iiwa/command/CartesianWrench",1);
  
  joint_position_pub_ = node_handle.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition",1);
  joint_torque_pub_ = node_handle.advertise<iiwa_msgs::JointTorque>("/iiwa/command/JointTorque",1);
  joint_velocity_pub_ = node_handle.advertise<iiwa_msgs::JointVelocity>("/iiwa/command/JointVelocity",1);
  
  
  cartesian_position_sub_ = node_handle.subscribe("/iiwa/state/CartesianPosition",1,iiwaRosConn::cartesianPositionCallback);
  cartesian_rotation_sub_ = node_handle.subscribe("/iiwa/state/CartesianRotation",1,iiwaRosConn::cartesianRotationCallback);
  cartesian_velocity_sub_ = node_handle.subscribe("/iiwa/state/CartesianVelocity",1,iiwaRosConn::cartesianVelocityCallback);
  cartesian_wrench_sub_ = node_handle.subscribe("/iiwa/state/CartesianWrench",1,iiwaRosConn::cartesianWrenchCallback);
  
  joint_position_sub_ = node_handle.subscribe("/iiwa/state/JointPosition",1,iiwaRosConn::jointPositionCallback);
  joint_torque_sub_ = node_handle.subscribe("/iiwa/state/JointTorque",1,iiwaRosConn::jointTorqueCallback);
  joint_velocity_sub_ = node_handle.subscribe("/iiwa/state/JointVelocity",1,iiwaRosConn::jointVelocityCallback);
  
}

iiwaRosConn::~iiwaRosConn()
{

}

bool iiwaRosConn::getRobotIsConnected() {
	return robot_is_connected_;
}

iiwa_msgs::CartesianPosition iiwaRosConn::getCartesianPosition() {
  return cartesian_position_;
}
iiwa_msgs::CartesianRotation iiwaRosConn::getCartesianRotation(){
  return cartesian_rotation_;
}
iiwa_msgs::CartesianVelocity iiwaRosConn::getCartesianVelocity(){
  return cartesian_velocity_;
}
iiwa_msgs::CartesianWrench iiwaRosConn::getCartesianWrench(){
  return cartesian_wrench_;
}
iiwa_msgs::JointPosition iiwaRosConn::getJointPosition(){
  return joint_position_;
}
iiwa_msgs::JointTorque iiwaRosConn::getJointTorque(){
  return joint_torque_;
}
iiwa_msgs::JointVelocity iiwaRosConn::getJointVelocity(){
  return joint_velocity_;
}

void iiwaRosConn::setCartesianPosition(const iiwa_msgs::CartesianPosition& position) {
  cartesian_position_.position.point = position.position.point;
  cartesian_position_.position.header = position.position.header;
}
void iiwaRosConn::setCartesianRotation(const iiwa_msgs::CartesianRotation& rotation) {
  for(int i = 0; i < 9; i++)
  cartesian_rotation_.rotation[i] = rotation.rotation[i];
  cartesian_rotation_.header = rotation.header;
}
void iiwaRosConn::setCartesianVelocity(const iiwa_msgs::CartesianVelocity& velocity) {
  for(int i = 0; i < IIWA_JOINTS; i++)
  cartesian_velocity_.velocity[i] = velocity.velocity[i];
  cartesian_velocity_.header = velocity.header;
}
void iiwaRosConn::setCartesianWrench(const iiwa_msgs::CartesianWrench& wrench) {
  cartesian_wrench_.wrench.wrench = wrench.wrench.wrench;
  cartesian_wrench_.wrench.header = wrench.wrench.header;
}
void iiwaRosConn::setJointPosition(const iiwa_msgs::JointPosition& position)  {
  for(int i = 0; i < IIWA_JOINTS; i++)
  joint_position_.position[i] = position.position[i];
  joint_position_.header = position.header;
}
void iiwaRosConn::setJointTorque(const iiwa_msgs::JointTorque& torque)  {
  for(int i = 0; i < IIWA_JOINTS; i++)
  joint_torque_.torque[i] = torque.torque[i];
  joint_torque_.header = torque.header;
}
void iiwaRosConn::setJointVelocity(const iiwa_msgs::JointVelocity& velocity)  {
  for(int i = 0; i < IIWA_JOINTS; i++)
  joint_velocity_.velocity[i] = velocity.velocity[i];
  joint_velocity_.header = velocity.header;
}


void iiwaRosConn::cartesianPositionCallback(const iiwa_msgs::CartesianPosition& position) {
  
  setCartesianPosition(position);
  //copyCartesianPosition(position, cartesian_position_);
  
  if (!robot_is_connected_){
	cout << "IIWA robot is connected." << endl;
	robot_is_connected_ = true;
  }
}
void iiwaRosConn::cartesianRotationCallback(const iiwa_msgs::CartesianRotation& rotation) {
  
  setCartesianRotation(rotation);
  //copyCartesianRotation(rotation, cartesian_rotation_);
  
  if (!robot_is_connected_){
	cout << "IIWA robot is connected." << endl;
	robot_is_connected_ = true;
  }
}
void iiwaRosConn::cartesianVelocityCallback(const iiwa_msgs::CartesianVelocity& velocity) {
  
  setCartesianVelocity(velocity);
  //copyCartesianVelocity(velocity, cartesian_velocity_);
  
  if (!robot_is_connected_){
	cout << "IIWA robot is connected." << endl;
	robot_is_connected_ = true;
  }
}
void iiwaRosConn::cartesianWrenchCallback(const iiwa_msgs::CartesianWrench& wrench) {
  
  setCartesianWrench(wrench);
  //copyCartesianWrench(wrench, cartesian_wrench_);
  
  	if (!robot_is_connected_){
		cout << "IIWA robot is connected." << endl;
		robot_is_connected_ = true;
	}
}
void iiwaRosConn::jointPositionCallback(const iiwa_msgs::JointPosition& position) {
  
  setJointPosition(position);
  //copyJointPosition(position, joint_position_);
  
  if (!robot_is_connected_){
	cout << "IIWA robot is connected." << endl;
	robot_is_connected_ = true;
  }
}
void iiwaRosConn::jointTorqueCallback(const iiwa_msgs::JointTorque& torque) {
  
  setJointTorque(torque);
  //copyJointTorque(torque, joint_torque_);
  
  if (!robot_is_connected_){
	cout << "IIWA robot is connected." << endl;
	robot_is_connected_ = true;
  }
}
void iiwaRosConn::jointVelocityCallback(const iiwa_msgs::JointVelocity& velocity) {
  
  setJointVelocity(velocity);
  //copyJointVelocity(velocity, joint_velocity_);
  
  if (!robot_is_connected_){
	cout << "IIWA robot is connected." << endl;
	robot_is_connected_ = true;
  }
}

bool iiwaRosConn::publish() {
  if (robot_is_connected_)
  {
    if (cartesian_position_pub_.getNumSubscribers())
      cartesian_position_pub_.publish(cartesian_position_);
    
    if (cartesian_rotation_pub_.getNumSubscribers())
      cartesian_rotation_pub_.publish(cartesian_rotation_);
    
    if (cartesian_velocity_pub_.getNumSubscribers())
      cartesian_velocity_pub_.publish(cartesian_velocity_);
    
    if (cartesian_wrench_pub_.getNumSubscribers())
      cartesian_wrench_pub_.publish(cartesian_wrench_);
    
    if (joint_position_pub_.getNumSubscribers())
      joint_position_pub_.publish(joint_position_);
    
    if (joint_torque_pub_.getNumSubscribers())
      joint_torque_pub_.publish(joint_torque_);
    
    if (joint_velocity_pub_.getNumSubscribers())
      joint_velocity_pub_.publish(joint_velocity_);
    
    return 1;
  }
  return 0;
}
