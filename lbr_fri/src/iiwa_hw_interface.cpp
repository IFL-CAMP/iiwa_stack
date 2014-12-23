#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

#include "iiwa_fri_client.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <iostream>
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"

#define DEFAULT_PORTID 30200
class IIWARobot : public hardware_interface::RobotHW
{
    private:
	IIWAFRIClient client;
	UdpConnection connection;
	ClientApplication app;
	std::string hostname;
	int port;
    public:
	IIWARobot(ros::NodeHandle param_nh): client(),connection(),app(connection,client) 
	{ 
	    param_nh.param<std::string>("hostname",hostname,"192.170.10.2");
	    param_nh.param<int>("port",port,DEFAULT_PORTID);
	    
	    app.connect(port, hostname.c_str());
	    
	    // connect and register the joint state interface
	    hardware_interface::JointStateHandle state_handle_1("lwr_joint_1", &pos[0], &vel[0], &eff[0]);
	    jnt_state_interface.registerHandle(state_handle_1);

	    hardware_interface::JointStateHandle state_handle_2("lwr_joint_2", &pos[1], &vel[1], &eff[1]);
	    jnt_state_interface.registerHandle(state_handle_2);

	    hardware_interface::JointStateHandle state_handle_3("lwr_joint_3", &pos[2], &vel[2], &eff[2]);
	    jnt_state_interface.registerHandle(state_handle_3);

	    hardware_interface::JointStateHandle state_handle_4("lwr_joint_4", &pos[3], &vel[3], &eff[3]);
	    jnt_state_interface.registerHandle(state_handle_4);

	    hardware_interface::JointStateHandle state_handle_5("lwr_joint_5", &pos[4], &vel[4], &eff[4]);
	    jnt_state_interface.registerHandle(state_handle_5);

	    hardware_interface::JointStateHandle state_handle_6("lwr_joint_6", &pos[5], &vel[5], &eff[5]);
	    jnt_state_interface.registerHandle(state_handle_6);

	    hardware_interface::JointStateHandle state_handle_7("lwr_joint_7", &pos[6], &vel[6], &eff[6]);
	    jnt_state_interface.registerHandle(state_handle_7);

	    registerInterface(&jnt_state_interface);

	    // connect and register the joint position interface
	    hardware_interface::JointHandle pos_handle_1(jnt_state_interface.getHandle("lwr_joint_1"), &cmd[0]);
	    jnt_pos_interface.registerHandle(pos_handle_1);

	    hardware_interface::JointHandle pos_handle_2(jnt_state_interface.getHandle("lwr_joint_2"), &cmd[2]);
	    jnt_pos_interface.registerHandle(pos_handle_2);

	    hardware_interface::JointHandle pos_handle_3(jnt_state_interface.getHandle("lwr_joint_3"), &cmd[3]);
	    jnt_pos_interface.registerHandle(pos_handle_3);

	    hardware_interface::JointHandle pos_handle_4(jnt_state_interface.getHandle("lwr_joint_4"), &cmd[4]);
	    jnt_pos_interface.registerHandle(pos_handle_4);

	    hardware_interface::JointHandle pos_handle_5(jnt_state_interface.getHandle("lwr_joint_5"), &cmd[5]);
	    jnt_pos_interface.registerHandle(pos_handle_5);

	    hardware_interface::JointHandle pos_handle_6(jnt_state_interface.getHandle("lwr_joint_6"), &cmd[6]);
	    jnt_pos_interface.registerHandle(pos_handle_6);

	    hardware_interface::JointHandle pos_handle_7(jnt_state_interface.getHandle("lwr_joint_7"), &cmd[7]);
	    jnt_pos_interface.registerHandle(pos_handle_7);

	    registerInterface(&jnt_pos_interface);
	}
	~IIWARobot()
	{
	    app.disconnect();
	}

	void read() 
	{
	    bool success = app.step();
	    client.getJointsRaw(pos,vel,eff);
	}
	void write() 
	{
	    //bool success = app.step();
	    client.setJointTargets(cmd);
	}
	
	//should return current time
	ros::Time get_time() { return client.getTime();  } ;
	//should return duration since last update
	ros::Duration get_period() { return client.getPeriod(); } ;

    private:
	hardware_interface::JointStateInterface jnt_state_interface;
	hardware_interface::PositionJointInterface jnt_pos_interface;
	double cmd[7];
	double pos[7];
	double vel[7];
	double eff[7];
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "iiwa_hw_interface");

    ros::NodeHandle param("~");
    IIWARobot robot(param);

    controller_manager::ControllerManager cm(&robot);

    while (true)
    {
	robot.read();
	cm.update(robot.get_time(), robot.get_period());
	robot.write();
    }
}
