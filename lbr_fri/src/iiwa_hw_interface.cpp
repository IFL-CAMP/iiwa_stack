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
#include <signal.h>

#include <velvet_msgs/VNodeState.h>
#include <velvet_msgs/VNodeTarget.h>

#include <boost/thread/mutex.hpp>

#define DEFAULT_PORTID 30200
//#define bFRI

class IIWARobot : public hardware_interface::RobotHW
{
    public:
#ifdef bFRI
	IIWAFRIClient client;
	UdpConnection connection;
	ClientApplication app;
#else
	IIWAFRIClientNative client;
#endif
	std::string hostname;
	int port;
	ros::NodeHandle n_;
	//ros::Subscriber velvet_state;
	//ros::Publisher velvet_targ;

	//boost::mutex velvet_m;
	std::string velvet_state_topic, velvet_target_topic;
    public:
	IIWARobot(ros::NodeHandle param_nh): client()
#ifdef bFRI
					     ,connection(),app(connection,client) 
#endif
	{ 
	    param_nh.param<std::string>("hostname",hostname,"192.170.10.2");
	    param_nh.param<int>("port",port,DEFAULT_PORTID);
	    param_nh.param<std::string>("velvet_topic_name", velvet_state_topic,"/velvet_node/vnode_state");
	    param_nh.param<std::string>("velvet_target_topic_name", velvet_target_topic,"/velvet_node/vnode_target");

#ifdef bFRI	    
	    app.connect(port, hostname.c_str());
#else
	    client.startThreads();
#endif
	    
	    // connect and register the joint state interface
	    hardware_interface::JointStateHandle state_handle_1("lbr_iiwa_joint_1", &pos[0], &vel[0], &eff[0]);
	    jnt_state_interface.registerHandle(state_handle_1);

	    hardware_interface::JointStateHandle state_handle_2("lbr_iiwa_joint_2", &pos[1], &vel[1], &eff[1]);
	    jnt_state_interface.registerHandle(state_handle_2);

	    hardware_interface::JointStateHandle state_handle_3("lbr_iiwa_joint_3", &pos[2], &vel[2], &eff[2]);
	    jnt_state_interface.registerHandle(state_handle_3);

	    hardware_interface::JointStateHandle state_handle_4("lbr_iiwa_joint_4", &pos[3], &vel[3], &eff[3]);
	    jnt_state_interface.registerHandle(state_handle_4);

	    hardware_interface::JointStateHandle state_handle_5("lbr_iiwa_joint_5", &pos[4], &vel[4], &eff[4]);
	    jnt_state_interface.registerHandle(state_handle_5);

	    hardware_interface::JointStateHandle state_handle_6("lbr_iiwa_joint_6", &pos[5], &vel[5], &eff[5]);
	    jnt_state_interface.registerHandle(state_handle_6);

	    hardware_interface::JointStateHandle state_handle_7("lbr_iiwa_joint_7", &pos[6], &vel[6], &eff[6]);
	    jnt_state_interface.registerHandle(state_handle_7);
	    
	   // hardware_interface::JointStateHandle state_handle_v("velvet_fingers_joint_1", &velvet_pos, &velvet_vel, &velvet_eff);
	   // jnt_state_interface.registerHandle(state_handle_v);

	    registerInterface(&jnt_state_interface);

	    // connect and register the joint position interface
	    hardware_interface::JointHandle vel_handle_1(jnt_state_interface.getHandle("lbr_iiwa_joint_1"), &cmd[0]);
	    jnt_vel_interface.registerHandle(vel_handle_1);

	    hardware_interface::JointHandle vel_handle_2(jnt_state_interface.getHandle("lbr_iiwa_joint_2"), &cmd[1]);
	    jnt_vel_interface.registerHandle(vel_handle_2);

	    hardware_interface::JointHandle vel_handle_3(jnt_state_interface.getHandle("lbr_iiwa_joint_3"), &cmd[2]);
	    jnt_vel_interface.registerHandle(vel_handle_3);                                                       
                                                                                                                  
	    hardware_interface::JointHandle vel_handle_4(jnt_state_interface.getHandle("lbr_iiwa_joint_4"), &cmd[3]);
	    jnt_vel_interface.registerHandle(vel_handle_4);                                                       
                                                                                                                  
	    hardware_interface::JointHandle vel_handle_5(jnt_state_interface.getHandle("lbr_iiwa_joint_5"), &cmd[4]);
	    jnt_vel_interface.registerHandle(vel_handle_5);                                                       
                                                                                                                  
	    hardware_interface::JointHandle vel_handle_6(jnt_state_interface.getHandle("lbr_iiwa_joint_6"), &cmd[5]);
	    jnt_vel_interface.registerHandle(vel_handle_6);                                                       
                                                                                                                  
	    hardware_interface::JointHandle vel_handle_7(jnt_state_interface.getHandle("lbr_iiwa_joint_7"), &cmd[6]);
	    jnt_vel_interface.registerHandle(vel_handle_7);

	    //hardware_interface::JointHandle vel_handle_v(jnt_state_interface.getHandle("velvet_fingers_joint_1"), &velvet_cmd);
	    //jnt_vel_interface.registerHandle(vel_handle_v);
	    
	    registerInterface(&jnt_vel_interface);
	    first_vals = true;

	    velvet_pos =0;
	    velvet_vel =0;
	    velvet_eff =0;
	    n_ = ros::NodeHandle();
	    //velvet_state = n_.subscribe(velvet_state_topic, 1, &IIWARobot::updateVelvetState, this);
	    //velvet_targ = param_nh.advertise<velvet_msgs::VNodeTarget>(velvet_target_topic,10);

#ifndef bFRI
	    bool success = client.waitForSession();
#endif
	}
	~IIWARobot()
	{
#ifdef bFRI
	    app.disconnect();
#else
	    client.step();
#endif
	}

	void updateVelvetState(const velvet_msgs::VNodeStatePtr msg) {
	    //lock mutex and update variables
	    //velvet_m.lock();
	    velvet_pos = msg->joint_pos;
	    velvet_vel = msg->joint_vel;
	    velvet_eff = msg->joint_eff;
	    //velvet_m.unlock();
	}

	void read() 
	{
#ifdef bFRI
	    bool success = app.step();
#endif
	    client.getJointsRaw(pos,vel,eff);
	    if(first_vals) {
		memcpy(prev_pos,pos,sizeof(double)*7);
		first_vals = false;
		return;
	    }
	    for(int i=0; i<7; ++i) {
		vel[i] = (pos[i]-prev_pos[i]) / client.getPeriod().toSec();
	    }
	    memcpy(prev_pos,pos,sizeof(double)*7);
	    //std::cerr<<"read joints: "<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<" "<<pos[3]<<" "<<pos[4]<<" "<<pos[5]<<" "<<pos[6]<<"\n";
	}
	void write() 
	{
	    //std::cerr<<"send joints: "<<cmd[0]<<" "<<cmd[1]<<" "<<cmd[2]<<" "<<cmd[3]<<" "<<cmd[4]<<" "<<cmd[5]<<" "<<cmd[6]<<"\n";
	    client.setJointTargets(cmd);
	    /*
	    velvet_msgs::VNodeTarget vtarg;
	    vtarg.target_vel = velvet_cmd;
	    velvet_targ.publish(vtarg);*/
#ifdef bFRI
	    bool success = app.step();
#endif
	}
	
	//should return current time
	ros::Time get_time() { return client.getTime();  } ;
	//should return duration since last update
	ros::Duration get_period() { return client.getPeriod(); } ;

    private:
	hardware_interface::JointStateInterface jnt_state_interface;
	hardware_interface::VelocityJointInterface jnt_vel_interface;
	double cmd[7];
	double pos[7];
	double prev_pos[7];
	double vel[7];
	double eff[7];
	double velvet_pos, velvet_vel, velvet_cmd, velvet_eff;
	bool first_vals;
};
static IIWARobot *robot;

void handler(int signum) {
    robot->client.step();
    delete robot;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "iiwa_hw_interface");

    ros::NodeHandle param("~");
    robot = new IIWARobot(param);

    signal(SIGTERM, &handler);
    ros::NodeHandle nh;
    controller_manager::ControllerManager cm(robot,nh);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    while (true)
    {
	robot->write();
	robot->read();
	cm.update(robot->get_time(), robot->get_period());
    }

    spinner.stop();
}
