#include "iiwaRos.h"

using namespace std;

iiwaRos::iiwaRos()
{
}

void iiwaRos::init(bool initRos, std::string iiwaName)
{
  // hide ROS away for non-ROS applications
  if (initRos) {
    int argc = 1;
    char *argv[1];
    argv[0] = "iiwaRos";
    ros::init(argc, argv, "iiwaRos");
  }
  
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
  
  new_cartesian_position_ = false;
  new_cartesian_rotation_  = false;
  new_cartesian_velocity_  = false;
  new_cartesian_wrench_  = false;
  new_joint_position_  = false;
  new_joint_torque_  = false;
  new_joint_velocity_  = false;
  robot_is_connected_ = false;
  
  cartesian_position_pub_ = node_handle.advertise<iiwa_msgs::CartesianPosition>("/" + iiwaName +"/command/CartesianPosition",1);
  cartesian_rotation_pub_ = node_handle.advertise<iiwa_msgs::CartesianRotation>("/" + iiwaName +"/command/CartesianRotation",1);
  cartesian_velocity_pub_ = node_handle.advertise<iiwa_msgs::CartesianVelocity>("/" + iiwaName +"/command/CartesianVelocity",1);
  cartesian_wrench_pub_ = node_handle.advertise<iiwa_msgs::CartesianWrench>("/" + iiwaName +"/command/CartesianWrench",1);
  
  joint_position_pub_ = node_handle.advertise<iiwa_msgs::JointPosition>("/" + iiwaName +"/command/JointPosition",1);
  joint_torque_pub_ = node_handle.advertise<iiwa_msgs::JointTorque>("/" + iiwaName +"/command/JointTorque",1);
  joint_velocity_pub_ = node_handle.advertise<iiwa_msgs::JointVelocity>("/" + iiwaName +"/command/JointVelocity",1);
  
  
  cartesian_position_sub_ = node_handle.subscribe("/" + iiwaName +"/state/CartesianPosition", 1, &iiwaRos::cartesianPositionCallback, this);
  cartesian_rotation_sub_ = node_handle.subscribe("/" + iiwaName +"/state/CartesianRotation", 1, &iiwaRos::cartesianRotationCallback, this);
  cartesian_velocity_sub_ = node_handle.subscribe("/" + iiwaName +"/state/CartesianVelocity", 1, &iiwaRos::cartesianVelocityCallback, this);
  cartesian_wrench_sub_ = node_handle.subscribe("/" + iiwaName +"/state/CartesianWrench", 1, &iiwaRos::cartesianWrenchCallback, this);
  
  joint_position_sub_ = node_handle.subscribe("/" + iiwaName +"/state/JointPosition", 1, &iiwaRos::jointPositionCallback, this);
  joint_torque_sub_ = node_handle.subscribe("/" + iiwaName +"/state/JointTorque", 1, &iiwaRos::jointTorqueCallback, this);
  joint_velocity_sub_ = node_handle.subscribe("/" + iiwaName +"/state/JointVelocity", 1, &iiwaRos::jointVelocityCallback, this);
}

void iiwaRos::robotConnected()
{
    if (!robot_is_connected_) {
	{
	  cout << "IIWA robot is connected." << endl;
	  robot_is_connected_ = true;
	}
    }
}

iiwaRos::~iiwaRos()
{
  
}

bool iiwaRos::isCartesianPositionAvailable() {
  return new_cartesian_position_;
}
bool iiwaRos::isCartesianRotationAvailable() {
  return new_cartesian_rotation_;
}
bool iiwaRos::isCartesianVelocityAvailable() {
  return new_cartesian_velocity_;
}
bool iiwaRos::isCartesianWrenchAvailable() {
  return new_cartesian_wrench_;
}
bool iiwaRos::isJointPositionAvailable() {
  return new_joint_position_;
}
bool iiwaRos::isJointTorqueAvailable() {
  return new_joint_torque_;
}
bool iiwaRos::isJointVelocityAvailable() {
  return new_joint_velocity_;
}

bool iiwaRos::getRobotIsConnected() {
  return robot_is_connected_;
}

iiwa_msgs::CartesianPosition iiwaRos::getReceivedCartesianPosition() {
  new_cartesian_position_ = false;
  cp_mutex_.lock();
  iiwa_msgs::CartesianPosition cp = received_cartesian_position_;
  cp_mutex_.unlock();
  return cp;
}
iiwa_msgs::CartesianRotation iiwaRos::getReceivedCartesianRotation(){
  new_cartesian_rotation_  = false;
  cr_mutex_.lock();
  iiwa_msgs::CartesianRotation cr = received_cartesian_rotation_;
  cr_mutex_.unlock();
  return cr;
}
iiwa_msgs::CartesianVelocity iiwaRos::getReceivedCartesianVelocity(){
  new_cartesian_velocity_  = false;
  cv_mutex_.lock();
  iiwa_msgs::CartesianVelocity cv = received_cartesian_velocity_;
  cv_mutex_.unlock();
  return cv;
}
iiwa_msgs::CartesianWrench iiwaRos::getReceivedCartesianWrench(){
  new_cartesian_wrench_  = false;
  cw_mutex_.lock();
  iiwa_msgs::CartesianWrench cw = received_cartesian_wrench_;
  cw_mutex_.unlock();
  return cw;
}
iiwa_msgs::JointPosition iiwaRos::getReceivedJointPosition(){
  new_joint_position_  = false;
  jp_mutex_.lock();
  iiwa_msgs::JointPosition jp = received_joint_position_;
  jp_mutex_.unlock();
  return jp;
}
iiwa_msgs::JointTorque iiwaRos::getReceivedJointTorque(){
  new_joint_torque_  = false;
  jt_mutex_.lock();
  iiwa_msgs::JointTorque jt = received_joint_torque_;
  jt_mutex_.unlock();
  return jt;
}
iiwa_msgs::JointVelocity iiwaRos::getReceivedJointVelocity(){
  new_joint_velocity_  = false;
  jv_mutex_.lock();
  iiwa_msgs::JointVelocity jv = received_joint_velocity_;
  jv_mutex_.unlock();
  return jv;
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
  cp_mutex_.lock();
  received_cartesian_position_ = position;
  cp_mutex_.unlock();
  new_cartesian_position_ = true;
  robotConnected();
}
void iiwaRos::cartesianRotationCallback(const iiwa_msgs::CartesianRotation& rotation) {
  cr_mutex_.lock();
  received_cartesian_rotation_ = rotation;  
  cr_mutex_.unlock();
  new_cartesian_rotation_  = true;
  robotConnected();
}
void iiwaRos::cartesianVelocityCallback(const iiwa_msgs::CartesianVelocity& velocity) {
  cv_mutex_.lock();
  received_cartesian_velocity_ = velocity;
  cv_mutex_.unlock();
  new_cartesian_velocity_  = true;
  robotConnected();
}
void iiwaRos::cartesianWrenchCallback(const iiwa_msgs::CartesianWrench& wrench) {
  cw_mutex_.lock();
  received_cartesian_wrench_ = wrench;
  cw_mutex_.unlock();
  new_cartesian_wrench_  = true;
  robotConnected();
}
void iiwaRos::jointPositionCallback(const iiwa_msgs::JointPosition& position) {
  jp_mutex_.lock();
  received_joint_position_ = position;
  jp_mutex_.unlock();
  new_joint_position_  = true;
  robotConnected();
}
void iiwaRos::jointTorqueCallback(const iiwa_msgs::JointTorque& torque) {
  jt_mutex_.lock();
  received_joint_torque_ = torque;
  jt_mutex_.unlock();
  new_joint_torque_  = true;
  robotConnected();
}
void iiwaRos::jointVelocityCallback(const iiwa_msgs::JointVelocity& velocity) {
  jv_mutex_.lock();
  received_joint_velocity_ = velocity;
  jv_mutex_.unlock();
  new_joint_velocity_  = true;
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
