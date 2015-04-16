#include "iiwaRos.h"

using namespace std;

iiwaRos::iiwaRos()
{
  iiwaName_ = "iiwa";
  init();
}

iiwaRos::iiwaRos(const string& iiwaInitName)
{
  iiwaName_ = iiwaInitName;
  init();
}

void iiwaRos::init()
{
  ros::NodeHandle node_handle;
  
  received_cartesian_rotation_.rotation.resize(9);
  received_cartesian_velocity_.velocity.resize(9);
  received_joint_position_.position.resize(IIWA_JOINTS);
  received_joint_torque_.torque.resize(IIWA_JOINTS);
  received_joint_velocity_.velocity.resize(IIWA_JOINTS);
  
  command_cartesian_rotation_.rotation.resize(9);
  command_cartesian_velocity_.velocity.resize(9);
  command_joint_position_.position.resize(IIWA_JOINTS);
  command_joint_torque_.torque.resize(IIWA_JOINTS);
  command_joint_velocity_.velocity.resize(IIWA_JOINTS);
  
  received_cartesian_position_initialized_ = false;
  received_cartesian_rotation_initialized_ = false;
  received_cartesian_velocity_initialized_ = false;
  received_cartesian_wrench_initialized_ = false;
  received_joint_position_initialized_ = false;
  received_joint_torque_initialized_ = false;
  received_joint_velocity_initialized_ = false;
  robot_is_connected_ = false;
  
  cartesian_position_pub_ = node_handle.advertise<iiwa_msgs::CartesianPosition>("/" + iiwaName_ +"/command/CartesianPosition",1);
  cartesian_rotation_pub_ = node_handle.advertise<iiwa_msgs::CartesianRotation>("/" + iiwaName_ +"/command/CartesianRotation",1);
  cartesian_velocity_pub_ = node_handle.advertise<iiwa_msgs::CartesianVelocity>("/" + iiwaName_ +"/command/CartesianVelocity",1);
  cartesian_wrench_pub_ = node_handle.advertise<iiwa_msgs::CartesianWrench>("/" + iiwaName_ +"/command/CartesianWrench",1);
  
  joint_position_pub_ = node_handle.advertise<iiwa_msgs::JointPosition>("/" + iiwaName_ +"/command/JointPosition",1);
  joint_torque_pub_ = node_handle.advertise<iiwa_msgs::JointTorque>("/" + iiwaName_ +"/command/JointTorque",1);
  joint_velocity_pub_ = node_handle.advertise<iiwa_msgs::JointVelocity>("/" + iiwaName_ +"/command/JointVelocity",1);
  
  
  cartesian_position_sub_ = node_handle.subscribe("/" + iiwaName_ +"/state/CartesianPosition", 1, &iiwaRos::cartesianPositionCallback, this);
  cartesian_rotation_sub_ = node_handle.subscribe("/" + iiwaName_ +"/state/CartesianRotation", 1, &iiwaRos::cartesianRotationCallback, this);
  cartesian_velocity_sub_ = node_handle.subscribe("/" + iiwaName_ +"/state/CartesianVelocity", 1, &iiwaRos::cartesianVelocityCallback, this);
  cartesian_wrench_sub_ = node_handle.subscribe("/" + iiwaName_ +"/state/CartesianWrench", 1, &iiwaRos::cartesianWrenchCallback, this);
  
  joint_position_sub_ = node_handle.subscribe("/" + iiwaName_ +"/state/JointPosition", 1, &iiwaRos::jointPositionCallback, this);
  joint_torque_sub_ = node_handle.subscribe("/" + iiwaName_ +"/state/JointTorque", 1, &iiwaRos::jointTorqueCallback, this);
  joint_velocity_sub_ = node_handle.subscribe("/" + iiwaName_ +"/state/JointVelocity", 1, &iiwaRos::jointVelocityCallback, this);
}

void iiwaRos::robotConnected()
{
    if (!robot_is_connected_) {
      if (received_cartesian_position_initialized_
	  && received_cartesian_rotation_initialized_
	  && received_cartesian_velocity_initialized_
	  && received_cartesian_wrench_initialized_
	  && received_joint_position_initialized_
	  && received_joint_torque_initialized_
	  && received_joint_velocity_initialized_)
	{
	  cout << "IIWA robot is connected." << endl;
	  robot_is_connected_ = true;
	}
    }
}


iiwaRos::~iiwaRos()
{
  
}

bool iiwaRos::getRobotIsConnected() {
  return robot_is_connected_;
}

iiwa_msgs::CartesianPosition iiwaRos::getReceivedCartesianPosition() {
  return received_cartesian_position_;
}
iiwa_msgs::CartesianRotation iiwaRos::getReceivedCartesianRotation(){
  return received_cartesian_rotation_;
}
iiwa_msgs::CartesianVelocity iiwaRos::getReceivedCartesianVelocity(){
  return received_cartesian_velocity_;
}
iiwa_msgs::CartesianWrench iiwaRos::getReceivedCartesianWrench(){
  return received_cartesian_wrench_;
}
iiwa_msgs::JointPosition iiwaRos::getReceivedJointPosition(){
  return received_joint_position_;
}
iiwa_msgs::JointTorque iiwaRos::getReceivedJointTorque(){
  return received_joint_torque_;
}
iiwa_msgs::JointVelocity iiwaRos::getReceivedJointVelocity(){
  return received_joint_velocity_;
}

iiwa_msgs::CartesianPosition iiwaRos::getCommandCartesianPosition() {
  return command_cartesian_position_;
}
iiwa_msgs::CartesianRotation iiwaRos::getCommandCartesianRotation(){
  return command_cartesian_rotation_;
}
iiwa_msgs::CartesianVelocity iiwaRos::getCommandCartesianVelocity(){
  return command_cartesian_velocity_;
}
iiwa_msgs::CartesianWrench iiwaRos::getCommandCartesianWrench(){
  return command_cartesian_wrench_;
}
iiwa_msgs::JointPosition iiwaRos::getCommandJointPosition(){
  return command_joint_position_;
}
iiwa_msgs::JointTorque iiwaRos::getCommandJointTorque(){
  return command_joint_torque_;
}
iiwa_msgs::JointVelocity iiwaRos::getCommandJointVelocity(){
  return command_joint_velocity_;
}

/*
 * Setters for command messages - set the message that you want to send
 */
void iiwaRos::setCommandCartesianPosition(const iiwa_msgs::CartesianPosition& position) {
  command_cartesian_position_ = position;
}
void iiwaRos::setCommandCartesianRotation(const iiwa_msgs::CartesianRotation& rotation) {
  command_cartesian_rotation_ = rotation;
}
void iiwaRos::setCommandCartesianVelocity(const iiwa_msgs::CartesianVelocity& velocity) {
  command_cartesian_velocity_ = velocity;
}
void iiwaRos::setCommandCartesianWrench(const iiwa_msgs::CartesianWrench& wrench) {
  command_cartesian_wrench_ = wrench;
}
void iiwaRos::setCommandJointPosition(const iiwa_msgs::JointPosition& position)  {
  command_joint_position_ = position;  
}
void iiwaRos::setCommandJointTorque(const iiwa_msgs::JointTorque& torque)  {
  command_joint_torque_ = torque;
}
void iiwaRos::setCommandJointVelocity(const iiwa_msgs::JointVelocity& velocity)  {
  command_joint_velocity_ = velocity;
}


void iiwaRos::cartesianPositionCallback(const iiwa_msgs::CartesianPosition& position) {
  received_cartesian_position_ = position;
  received_cartesian_position_initialized_ = true;
  robotConnected();
}
void iiwaRos::cartesianRotationCallback(const iiwa_msgs::CartesianRotation& rotation) {
  received_cartesian_rotation_ = rotation;
  received_cartesian_rotation_initialized_ = true;
  robotConnected();
}
void iiwaRos::cartesianVelocityCallback(const iiwa_msgs::CartesianVelocity& velocity) {
  received_cartesian_velocity_ = velocity;
  received_cartesian_velocity_initialized_ = true;
  robotConnected();
}
void iiwaRos::cartesianWrenchCallback(const iiwa_msgs::CartesianWrench& wrench) {
  received_cartesian_wrench_ = wrench;
  received_cartesian_wrench_initialized_ = true;
  robotConnected();
}
void iiwaRos::jointPositionCallback(const iiwa_msgs::JointPosition& position) {
  received_joint_position_ = position;
  received_joint_position_initialized_ = true;
  robotConnected();
}
void iiwaRos::jointTorqueCallback(const iiwa_msgs::JointTorque& torque) {
  received_joint_torque_ = torque;
  received_joint_torque_initialized_ = true;
  robotConnected();
}
void iiwaRos::jointVelocityCallback(const iiwa_msgs::JointVelocity& velocity) {
  received_joint_velocity_ = velocity;
  received_joint_velocity_initialized_ = true;
  robotConnected();
}

bool iiwaRos::publish() {
  if (robot_is_connected_)
  {
    publishIfSubscriber(cartesian_position_pub_, command_cartesian_position_);
    publishIfSubscriber(cartesian_rotation_pub_, command_cartesian_rotation_);
    publishIfSubscriber(cartesian_velocity_pub_, command_cartesian_velocity_);
    publishIfSubscriber(cartesian_wrench_pub_, command_cartesian_wrench_);
    
    publishIfSubscriber(joint_position_pub_, command_joint_position_);
    publishIfSubscriber(joint_torque_pub_, command_joint_torque_);
    publishIfSubscriber(joint_velocity_pub_, command_joint_velocity_);
    
    return 1;
  }
  return 0;
}

template <class T>
bool iiwaRos::publishIfSubscriber(const ros::Publisher& p, const T& message) {
  if (p.getNumSubscribers())
    p.publish(message);
}
