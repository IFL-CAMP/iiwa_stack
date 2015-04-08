#include "iiwaRosConn.h"

using namespace std;

iiwaRosConn::iiwaRosConn()
{
  iiwaName_ = "iiwa";
  init();
}

iiwaRosConn::iiwaRosConn(string iiwaInitName)
{
  iiwaName_ = iiwaInitName;
  init();
}

void iiwaRosConn::init()
{
  ros::NodeHandle node_handle;
  
  cartesian_rotation_.rotation.resize(9);
  cartesian_velocity_.velocity.resize(9);
  joint_position_.position.resize(IIWA_JOINTS);
  joint_torque_.torque.resize(IIWA_JOINTS);
  joint_velocity_.velocity.resize(IIWA_JOINTS);
  
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
  cartesian_position_ = position;
}
void iiwaRosConn::setCartesianRotation(const iiwa_msgs::CartesianRotation& rotation) {
  cartesian_rotation_ = rotation;
}
void iiwaRosConn::setCartesianVelocity(const iiwa_msgs::CartesianVelocity& velocity) {
  cartesian_velocity_ = velocity;
}
void iiwaRosConn::setCartesianWrench(const iiwa_msgs::CartesianWrench& wrench) {
  cartesian_wrench_ = wrench;
}
void iiwaRosConn::setJointPosition(const iiwa_msgs::JointPosition& position)  {
  joint_position_ = position;
}
void iiwaRosConn::setJointTorque(const iiwa_msgs::JointTorque& torque)  {
  joint_torque_ = torque;
}
void iiwaRosConn::setJointVelocity(const iiwa_msgs::JointVelocity& velocity)  {
  joint_velocity_ = velocity;
}


void iiwaRosConn::cartesianPositionCallback(const iiwa_msgs::CartesianPosition& position) {
  
  setCartesianPosition(position);
  
  if (!robot_is_connected_){
    cout << "IIWA robot is connected." << endl;
    robot_is_connected_ = true;
  }
}
void iiwaRosConn::cartesianRotationCallback(const iiwa_msgs::CartesianRotation& rotation) {
  
  setCartesianRotation(rotation);
  
  if (!robot_is_connected_){
    cout << "IIWA robot is connected." << endl;
    robot_is_connected_ = true;
  }
}
void iiwaRosConn::cartesianVelocityCallback(const iiwa_msgs::CartesianVelocity& velocity) {
  
  setCartesianVelocity(velocity);
  
  if (!robot_is_connected_){
    cout << "IIWA robot is connected." << endl;
    robot_is_connected_ = true;
  }
}
void iiwaRosConn::cartesianWrenchCallback(const iiwa_msgs::CartesianWrench& wrench) {
  
  setCartesianWrench(wrench);
  
  if (!robot_is_connected_){
    cout << "IIWA robot is connected." << endl;
    robot_is_connected_ = true;
  }
}
void iiwaRosConn::jointPositionCallback(const iiwa_msgs::JointPosition& position) {
  
  setJointPosition(position);
  
  if (!robot_is_connected_){
    cout << "IIWA robot is connected." << endl;
    robot_is_connected_ = true;
  }
}
void iiwaRosConn::jointTorqueCallback(const iiwa_msgs::JointTorque& torque) {
  
  setJointTorque(torque);
  
  if (!robot_is_connected_){
    cout << "IIWA robot is connected." << endl;
    robot_is_connected_ = true;
  }
}
void iiwaRosConn::jointVelocityCallback(const iiwa_msgs::JointVelocity& velocity) {
  
  setJointVelocity(velocity);
  
  if (!robot_is_connected_){
    cout << "IIWA robot is connected." << endl;
    robot_is_connected_ = true;
  }
}

bool iiwaRosConn::publish() {
  if (robot_is_connected_)
  {
    publishIfSubscriber(cartesian_position_pub_, cartesian_position_);
    publishIfSubscriber(cartesian_rotation_pub_, cartesian_rotation_);
    publishIfSubscriber(cartesian_velocity_pub_, cartesian_velocity_);
    publishIfSubscriber(cartesian_wrench_pub_, cartesian_wrench_);
    
    publishIfSubscriber(joint_position_pub_, joint_position_);
    publishIfSubscriber(joint_torque_pub_, joint_torque_);
    publishIfSubscriber(joint_velocity_pub_, joint_velocity_);
    
    return 1;
  }
  return 0;
}

template <class T>
bool iiwaRosConn::publishIfSubscriber(const ros::Publisher& p, const T& message) {
  if (p.getNumSubscribers())
    p.publish(message);
}
