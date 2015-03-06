#ifndef LBR_IIWA_HW_H_
#define LBR_IIWA_HW_H_

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

// IIWAMsg include
#include "IIWA/IIWAMsg.h"



// Defines 

// ROS
#define IIWA_LISTEN "/iiwa/command"
#define IIWA_TALK   "/iiwa/state"

// Joint Numbers and Frequency to use
#define IIWA_DOF_JOINTS 7
#define IIWACONTROLFREQUENCY 800 // Hz


  class IIWA_HW : public hardware_interface::RobotHW
  {
  public:
    IIWA_HW(ros::NodeHandle nh);
    ~IIWA_HW();
    
    bool start();
    void registerJointLimits(const std::string& joint_name,
                           const hardware_interface::JointHandle& joint_handle,
                           const urdf::Model *const urdf_model,
                           double *const lower_limit, double *const upper_limit, 
                           double *const effort_limit);
    
    void ResizeIIWAMessage(IIWA::IIWAMsg &msg);
    
    static void CopyIIWAMessage(const IIWA::IIWAMsg &msg_copy, IIWA::IIWAMsg &msg_paste);
    static void IIWASubscriberCallback(const IIWA::IIWAMsg& msg);
    
    bool read(ros::Duration period);
    
    ros::Rate* getRate();

    // structure for a lbr iiwa, joint handles, etc
    struct IIWA_device
    {
      // configuration
      std::vector<std::string> joint_names;

      // limits
      std::vector<double> 
        joint_lower_limits,
        joint_upper_limits,
        joint_effort_limits;

      // state and commands
      std::vector<double>
        joint_position,
        joint_position_prev,
        joint_velocity,
        joint_effort,
        joint_position_command,
        joint_stiffness_command,
        joint_damping_command,
        joint_effort_command;

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

      // reset values
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

    boost::shared_ptr<IIWA_HW::IIWA_device> _device;

  private:

    // Node handle
    ros::NodeHandle _nh;

    // Parameters
    std::string _interface;
    urdf::Model _urdf_model;

    // Interfaces
    hardware_interface::JointStateInterface _state_interface;
    hardware_interface::EffortJointInterface _effort_interface;
    hardware_interface::PositionJointInterface _position_interface;

    joint_limits_interface::EffortJointSaturationInterface   _ej_sat_interface;
    joint_limits_interface::EffortJointSoftLimitsInterface   _ej_limits_interface;
    joint_limits_interface::PositionJointSaturationInterface _pj_sat_interface;
    joint_limits_interface::PositionJointSoftLimitsInterface _pj_limits_interface;
    
  protected:
    
    char static_txt[1025];
    double controlFrequency;
    
    ros::Publisher iiwa_pub;
    ros::Subscriber  iiwa_sub;
    ros::Rate *loop_rate;
    
    IIWA::IIWAMsg currentIIWAStateMessage;

  };


class IIWARobot
{
public:
    virtual void    Controller(const IIWA::IIWAMsg &currentState, IIWA::IIWAMsg &desiredState, ros::Duration period) = 0;
    
    virtual void    read(ros::Duration period) = 0;

    bool            RobotUpdate(ros::Duration period);

};



#endif //LBR_IIWA_HW_H_
