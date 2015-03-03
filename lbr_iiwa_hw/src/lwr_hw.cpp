#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <stdexcept>

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

// fri remote 
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <limits.h>
#include "friudp.h"
#include "friremote.h"

bool g_quit = false;

void quitRequested(int sig) {
  g_quit = true;
}

namespace lwr_ros_control
{ 
  class LWRHW : public hardware_interface::RobotHW
  {
  public:
    LWRHW(ros::NodeHandle nh);
    bool start();
    bool read(ros::Time time, ros::Duration period);
    void write(ros::Time time, ros::Duration period);
    void stop();
    void set_mode();
    void reset();
    void registerJointLimits(const std::string& joint_name,
                           const hardware_interface::JointHandle& joint_handle,
                           const urdf::Model *const urdf_model,
                           double *const lower_limit, double *const upper_limit, 
                           double *const effort_limit);

    // structure for a lwr, joint handles, low lever interface, etc
    struct LWRDevice7
    {
      // low-level interface
      boost::shared_ptr<friRemote> interface;

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

      // FRI values
      FRI_QUALITY lastQuality;
      FRI_CTRL lastCtrlScheme;

      void init()
      {
        joint_position.resize(LBR_MNJ);
        joint_position_prev.resize(LBR_MNJ);
        joint_velocity.resize(LBR_MNJ);
        joint_effort.resize(LBR_MNJ);
        joint_position_command.resize(LBR_MNJ);
        joint_effort_command.resize(LBR_MNJ);
        joint_stiffness_command.resize(LBR_MNJ);
        joint_damping_command.resize(LBR_MNJ);

        joint_lower_limits.resize(LBR_MNJ);
        joint_upper_limits.resize(LBR_MNJ);
        joint_effort_limits.resize(LBR_MNJ);
      }

      // reset values
      void reset() 
      {
        for (int j = 0; j < LBR_MNJ; ++j)
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

    boost::shared_ptr<LWRHW::LWRDevice7> device_;

  private:

    // Node handle
    ros::NodeHandle nh_;

    // Parameters
    int port_;
    std::string hintToRemoteHost_;
    urdf::Model urdf_model_;

    // interfaces
    hardware_interface::JointStateInterface state_interface_;
    hardware_interface::EffortJointInterface effort_interface_;
    hardware_interface::PositionJointInterface position_interface_;

    joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
    joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
    joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
    joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;

  protected:

  };

  LWRHW::LWRHW(ros::NodeHandle nh) :
    nh_(nh)
  {}

  bool LWRHW::start()
  {

    // construct a new lwr device (interface and state storage)
    this->device_.reset( new LWRHW::LWRDevice7() );

    // get params or give default values
    nh_.param("port", port_, 49939);
    nh_.param("ip", hintToRemoteHost_, std::string("192.168.0.10") );

    // TODO: use transmission configuration to get names directly from the URDF model
    if( ros::param::get("joints", this->device_->joint_names) )
    {
      if( !(this->device_->joint_names.size()==LBR_MNJ) )
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

    // construct a low-level lwr
    this->device_->interface.reset( new friRemote( port_, hintToRemoteHost_.c_str() ) );

    // initialize FRI values
    this->device_->lastQuality = FRI_QUALITY_BAD;
    this->device_->lastCtrlScheme = FRI_CTRL_OTHER;

    // initialize and set to zero the state and command values
    this->device_->init();
    this->device_->reset();

    // general joint to store information
    boost::shared_ptr<const urdf::Joint> joint;

    // create joint handles given the list
    for(int i = 0; i < LBR_MNJ; ++i)
    {
      ROS_INFO_STREAM("Handling joint: " << this->device_->joint_names[i]);

      // get current joint configuration
      joint = urdf_model_.getJoint(this->device_->joint_names[i]);
      if(!joint.get())
      {
        ROS_ERROR_STREAM("The specified joint "<< this->device_->joint_names[i] << " can't be found in the URDF model. Check that you loaded an URDF model in the robot description, or that you spelled correctly the joint name.");
        throw std::runtime_error("Wrong joint name specification");
      }

      // joint state handle
      hardware_interface::JointStateHandle state_handle(this->device_->joint_names[i],
          &this->device_->joint_position[i],
          &this->device_->joint_velocity[i],
          &this->device_->joint_effort[i]);

      state_interface_.registerHandle(state_handle);
      
      // position command handle
      hardware_interface::JointHandle position_joint_handle = hardware_interface::JointHandle(
            state_interface_.getHandle(this->device_->joint_names[i]),
            &this->device_->joint_position_command[i]);

      position_interface_.registerHandle(position_joint_handle);

      // effort command handle
      hardware_interface::JointHandle joint_handle = hardware_interface::JointHandle(
            state_interface_.getHandle(this->device_->joint_names[i]),
            &this->device_->joint_effort_command[i]);

      effort_interface_.registerHandle(joint_handle);

      registerJointLimits(this->device_->joint_names[i],
                          joint_handle,
                          &urdf_model_,
                          &this->device_->joint_lower_limits[i],
                          &this->device_->joint_upper_limits[i],
                          &this->device_->joint_effort_limits[i]);
    }

    ROS_INFO("Register state and effort interfaces");

    // register ros-controls interfaces
    this->registerInterface(&state_interface_);
    this->registerInterface(&effort_interface_);
    this->registerInterface(&position_interface_);

    std::cout << "Opening FRI Version " 
      << FRI_MAJOR_VERSION << "." << FRI_SUB_VERSION << "." <<FRI_DATAGRAM_ID_CMD << "." <<FRI_DATAGRAM_ID_MSR 
      << " Interface for LWR ROS server" << std::endl;

    ROS_INFO("Performing handshake to KRL");

    // perform some arbitrary handshake to KRL -- possible in monitor mode already
    // send to krl int a value
    this->device_->interface->setToKRLInt(0,1);
    if ( this->device_->interface->getQuality() >= FRI_QUALITY_OK)
    {
        // send a second marker
        this->device_->interface->setToKRLInt(0,10);
    }

    //
    // just mirror the real value..
    //
    this->device_->interface->setToKRLReal(0,this->device_->interface->getFrmKRLReal(1));

    ROS_INFO_STREAM("LWR Status:\n" << this->device_->interface->getMsrBuf().intf);

    this->device_->interface->doDataExchange();
    ROS_INFO("Done handshake !");

    return true;
  }

  bool LWRHW::read(ros::Time time, ros::Duration period)
    {
      // update the robot positions
      for (int j = 0; j < LBR_MNJ; j++)
      {
      	this->device_->joint_position_prev[j] = this->device_->joint_position[j];
        this->device_->joint_position[j] = this->device_->interface->getMsrMsrJntPosition()[j];
        this->device_->joint_effort[j] = this->device_->interface->getMsrJntTrq()[j];
        this->device_->joint_velocity[j] = filters::exponentialSmoothing((this->device_->joint_position[j]-this->device_->joint_position_prev[j])/period.toSec(), this->device_->joint_velocity[j], 0.2);
      }
      
      //this->device_->interface->doDataExchange();

      return true;
    }

  void LWRHW::write(ros::Time time, ros::Duration period)
    {
      static int warning = 0;

      // enforce limits
      ej_sat_interface_.enforceLimits(period);
      ej_limits_interface_.enforceLimits(period);
      pj_sat_interface_.enforceLimits(period);
      pj_limits_interface_.enforceLimits(period);

      // write to real robot
      float newJntPosition[LBR_MNJ];
      float newJntStiff[LBR_MNJ];
      float newJntDamp[LBR_MNJ];
      float newJntAddTorque[LBR_MNJ];

      if ( this->device_->interface->isPowerOn() )
      { 
        // check control mode
        //if ( this->device_->interface->getState() == FRI_STATE_CMD )
        //{
          // check control scheme
          if( this->device_->interface->getCurrentControlScheme() == FRI_CTRL_POSITION )
          {
            for (int i = 0; i < LBR_MNJ; i++)
            {
                newJntPosition[i] = this->device_->joint_position_command[i];
            }

            // only joint impedance control is performed, since it is the only one that provide access to the joint torque directly
            // note that stiffness and damping are 0, as well as the position, since only effort is allowed to be sent
            // the KRC adds the dynamic terms, such that if zero torque is sent, the robot apply torques necessary to mantain the robot in the current position
            // the only interface is effort, thus any other action you want to do, you have to compute the added torque and send it through a controller
            this->device_->interface->doPositionControl(newJntPosition, true);
          }
          // check control scheme
          if( this->device_->interface->getCurrentControlScheme() == FRI_CTRL_JNT_IMP )
          {
            for (int i = 0; i < LBR_MNJ; i++)
            {
                newJntPosition[i] = this->device_->joint_position[i];
// 		        std::cout << "joint_effort_command " << i << " " << this->device_->joint_effort_command[i] << std::endl;
                newJntAddTorque[i] = this->device_->joint_effort_command[i]; // comes from the controllers
                newJntStiff[i] = this->device_->joint_stiffness_command[i]; // default values for now
                newJntDamp[i] = this->device_->joint_damping_command[i]; // default values for now
            }

            // only joint impedance control is performed, since it is the only one that provide access to the joint torque directly
            // note that stiffness and damping are 0, as well as the position, since only effort is allowed to be sent
            // the KRC adds the dynamic terms, such that if zero torque is sent, the robot apply torques necessary to mantain the robot in the current position
            // the only interface is effort, thus any other action you want to do, you have to compute the added torque and send it through a controller
            this->device_->interface->doJntImpedanceControl(newJntPosition, newJntStiff, newJntDamp, newJntAddTorque, true);
          }
          if( this->device_->interface->getCurrentControlScheme() == FRI_CTRL_OTHER ) // Gravity compensation: just read status, but we have to keep FRI alive
          {
            this->device_->interface->doJntImpedanceControl(NULL, NULL, NULL, NULL, true);
          }
        //}
      }

      // Stop request is issued from the other side
      /*
      if ( this->device_->interface->getFrmKRLInt(0) == -1)
      {
          ROS_INFO(" Stop request issued from the other side");
          this->stop();
      }*/

      // Quality change leads to output of statistics
      // for informational reasons
      //
      /*if ( this->device_->interface->getQuality() != this->device_->lastQuality )
      {
          ROS_INFO_STREAM("Quality change detected "<< this->device_->interface->getQuality()<< " \n");
          ROS_INFO_STREAM("" << this->device_->interface->getMsrBuf().intf);
          this->device_->lastQuality = this->device_->interface->getQuality();
      }*/

      // this is already done in the doJntImpedance Control setting to true the last flag
      // this->device_->interface->doDataExchange();

      return;
    }

  void LWRHW::stop()
  {
    // TODO: decide whether to stop the FRI or just put to idle
  }

  void LWRHW::set_mode()
  {
    // ToDo: just switch between monitor and command mode, no control strategies switch
      return;
  }

  // Register the limits of the joint specified by joint_name and\ joint_handle. The limits are
  // retrieved from the urdf_model.
  // Return the joint's type, lower position limit, upper position limit, and effort limit.
  void LWRHW::registerJointLimits(const std::string& joint_name,
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
}

int main( int argc, char** argv )
{
  // initialize ROS
  ros::init(argc, argv, "lwr_hw_interface", ros::init_options::NoSigintHandler);

  // ros spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // custom signal handlers
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  // construct the lwr
  ros::NodeHandle lwr_nh("");
  lwr_ros_control::LWRHW lwr_robot(lwr_nh);

  // configuration routines
  lwr_robot.start();

  // timer variables
  struct timespec ts = {0, 0};
  ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);
  ros::Duration period(1.0);

  //realtime_tools::RealtimePublisher<std_msgs::Duration> publisher(lwr_nh, "loop_rate", 2);

  //the controller manager
  controller_manager::ControllerManager manager(&lwr_robot, lwr_nh);

  //uint32_t count = 0;

  // run as fast as possible
  while( !g_quit ) 
  {
    // get the time / period
    if (!clock_gettime(CLOCK_REALTIME, &ts)) 
    {
      now.sec = ts.tv_sec;
      now.nsec = ts.tv_nsec;
      period = now - last;
      last = now;
    } 
    else 
    {
      ROS_FATAL("Failed to poll realtime clock!");
      break;
    } 

    // read the state from the lwr
    if(!lwr_robot.read(now, period))
    {
      g_quit = true;
      break;
    }

    // update the controllers
    manager.update(now, period);

    // write the command to the lwr
    lwr_robot.write(now, period);

    // if(count++ > 1000)
    // {
    //   if(publisher.trylock())
    //   {
    //     count = 0;
    //     publisher.msg_.data = period;
    //     publisher.unlockAndPublish();
    //   }
    // }
  }

  //publisher.stop();

  std::cerr<<"Stopping spinner..."<<std::endl;
  spinner.stop();

  std::cerr<<"Stopping LWR..."<<std::endl;
  lwr_robot.stop();

  std::cerr<<"Bye!"<<std::endl;

  return 0;
}
