#include "iiwaRosConn.h"

using namespace std;

iiwaRosConn::iiwaRosConn()
{
  iiwaName_ = "iiwa";
  init();
}

iiwaRosConn::iiwaRosConn(const string& iiwaInitName)
{
  iiwaName_ = iiwaInitName;
  init();
}

void iiwaRosConn::init()
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
  
  robot_is_connected_ = false;
  
  cartesian_position_pub_ = node_handle.advertise<iiwa_msgs::CartesianPosition>("/" + iiwaName_ +"/command/CartesianPosition",1);
  cartesian_rotation_pub_ = node_handle.advertise<iiwa_msgs::CartesianRotation>("/" + iiwaName_ +"/command/CartesianRotation",1);
  cartesian_velocity_pub_ = node_handle.advertise<iiwa_msgs::CartesianVelocity>("/" + iiwaName_ +"/command/CartesianVelocity",1);
  cartesian_wrench_pub_ = node_handle.advertise<iiwa_msgs::CartesianWrench>("/" + iiwaName_ +"/command/CartesianWrench",1);
  
  joint_position_pub_ = node_handle.advertise<iiwa_msgs::JointPosition>("/" + iiwaName_ +"/command/JointPosition",1);
  joint_torque_pub_ = node_handle.advertise<iiwa_msgs::JointTorque>("/" + iiwaName_ +"/command/JointTorque",1);
  joint_velocity_pub_ = node_handle.advertise<iiwa_msgs::JointVelocity>("/" + iiwaName_ +"/command/JointVelocity",1);
  
  
  cartesian_position_sub_ = node_handle.subscribe("/" + iiwaName_ +"/state/CartesianPosition", 1, &iiwaRosConn::cartesianPositionCallback, this);
  cartesian_rotation_sub_ = node_handle.subscribe("/" + iiwaName_ +"/state/CartesianRotation", 1, &iiwaRosConn::cartesianRotationCallback, this);
  cartesian_velocity_sub_ = node_handle.subscribe("/" + iiwaName_ +"/state/CartesianVelocity", 1, &iiwaRosConn::cartesianVelocityCallback, this);
  cartesian_wrench_sub_ = node_handle.subscribe("/" + iiwaName_ +"/state/CartesianWrench", 1, &iiwaRosConn::cartesianWrenchCallback, this);
  
  joint_position_sub_ = node_handle.subscribe("/" + iiwaName_ +"/state/JointPosition", 1, &iiwaRosConn::jointPositionCallback, this);
  joint_torque_sub_ = node_handle.subscribe("/" + iiwaName_ +"/state/JointTorque", 1, &iiwaRosConn::jointTorqueCallback, this);
  joint_velocity_sub_ = node_handle.subscribe("/" + iiwaName_ +"/state/JointVelocity", 1, &iiwaRosConn::jointVelocityCallback, this);
}

void iiwaRosConn::robotConnected()
{
    if (!robot_is_connected_){
    cout << "IIWA robot is connected." << endl;
    robot_is_connected_ = true;
    }
}


iiwaRosConn::~iiwaRosConn()
{
  
}

bool iiwaRosConn::getRobotIsConnected() {
  return robot_is_connected_;
}

iiwa_msgs::CartesianPosition iiwaRosConn::getReceivedCartesianPosition() {
  return received_cartesian_position_;
}
iiwa_msgs::CartesianRotation iiwaRosConn::getReceivedCartesianRotation(){
  return received_cartesian_rotation_;
}
iiwa_msgs::CartesianVelocity iiwaRosConn::getReceivedCartesianVelocity(){
  return received_cartesian_velocity_;
}
iiwa_msgs::CartesianWrench iiwaRosConn::getReceivedCartesianWrench(){
  return received_cartesian_wrench_;
}
iiwa_msgs::JointPosition iiwaRosConn::getReceivedJointPosition(){
  return received_joint_position_;
}
iiwa_msgs::JointTorque iiwaRosConn::getReceivedJointTorque(){
  return received_joint_torque_;
}
iiwa_msgs::JointVelocity iiwaRosConn::getReceivedJointVelocity(){
  return received_joint_velocity_;
}

iiwa_msgs::CartesianPosition iiwaRosConn::getCommandCartesianPosition() {
  return command_cartesian_position_;
}
iiwa_msgs::CartesianRotation iiwaRosConn::getCommandCartesianRotation(){
  return command_cartesian_rotation_;
}
iiwa_msgs::CartesianVelocity iiwaRosConn::getCommandCartesianVelocity(){
  return command_cartesian_velocity_;
}
iiwa_msgs::CartesianWrench iiwaRosConn::getCommandCartesianWrench(){
  return command_cartesian_wrench_;
}
iiwa_msgs::JointPosition iiwaRosConn::getCommandJointPosition(){
  return command_joint_position_;
}
iiwa_msgs::JointTorque iiwaRosConn::getCommandJointTorque(){
  return command_joint_torque_;
}
iiwa_msgs::JointVelocity iiwaRosConn::getCommandJointVelocity(){
  return command_joint_velocity_;
}

/*
 * Setters for command messages - set the message that you want to send
 */
void iiwaRosConn::setCommandCartesianPosition(const iiwa_msgs::CartesianPosition& position) {
  command_cartesian_position_ = position;
}
void iiwaRosConn::setCommandCartesianRotation(const iiwa_msgs::CartesianRotation& rotation) {
  command_cartesian_rotation_ = rotation;
}
void iiwaRosConn::setCommandCartesianVelocity(const iiwa_msgs::CartesianVelocity& velocity) {
  command_cartesian_velocity_ = velocity;
}
void iiwaRosConn::setCommandCartesianWrench(const iiwa_msgs::CartesianWrench& wrench) {
  command_cartesian_wrench_ = wrench;
}
void iiwaRosConn::setCommandJointPosition(const iiwa_msgs::JointPosition& position)  {
  command_joint_position_ = position;  
}
void iiwaRosConn::setCommandJointTorque(const iiwa_msgs::JointTorque& torque)  {
  command_joint_torque_ = torque;
}
void iiwaRosConn::setCommandJointVelocity(const iiwa_msgs::JointVelocity& velocity)  {
  command_joint_velocity_ = velocity;
}


void iiwaRosConn::cartesianPositionCallback(const iiwa_msgs::CartesianPosition& position) {
 
  received_cartesian_position_ = position;
  robotConnected();
}
void iiwaRosConn::cartesianRotationCallback(const iiwa_msgs::CartesianRotation& rotation) {
  
  received_cartesian_rotation_ = rotation;
  robotConnected();
}
void iiwaRosConn::cartesianVelocityCallback(const iiwa_msgs::CartesianVelocity& velocity) {
  
  received_cartesian_velocity_ = velocity;
  robotConnected();
}
void iiwaRosConn::cartesianWrenchCallback(const iiwa_msgs::CartesianWrench& wrench) {
  
  received_cartesian_wrench_ = wrench;
  robotConnected();
}
void iiwaRosConn::jointPositionCallback(const iiwa_msgs::JointPosition& position) {
  
  received_joint_position_ = position;
  robotConnected();
}
void iiwaRosConn::jointTorqueCallback(const iiwa_msgs::JointTorque& torque) {
  
  received_joint_torque_ = torque;
  robotConnected();
}
void iiwaRosConn::jointVelocityCallback(const iiwa_msgs::JointVelocity& velocity) {
  
  received_joint_velocity_ = velocity;
  robotConnected();
}

bool iiwaRosConn::publish() {
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
bool iiwaRosConn::publishIfSubscriber(const ros::Publisher& p, const T& message) {
  if (p.getNumSubscribers())
    p.publish(message);
}
