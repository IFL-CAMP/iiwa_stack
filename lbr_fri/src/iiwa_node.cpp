#include <stdlib.h>
#include <stdio.h>
#include <string.h> // strstr
#include "iiwa_fri_client.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <iostream>

#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include <Server.h>
#include <std_srvs/Empty.h>

#define DEFAULT_PORTID 30200
#define DEFAULT_JAVA_PORTID 5010

using namespace KUKA::FRI;


class IIWANode {

    protected:
	// Our NodeHandle
	ros::NodeHandle nh_;
	ros::NodeHandle n_;
        ros::Timer heartbeat_;

	ros::Publisher joints_pub_;
	ros::ServiceServer test_streamer;

	//boost::mutex m, message_m;
	//ros::ServiceServer save_map_;
	std::string hostname;
	int port, port_java;
	int publish_ctr;
	double loop_rate;
	int n_publish_skip;
   
	IIWAFRIClient client;
	UdpConnection connection;
	ClientApplication app;
	Server *target_streamer;
	bool connected;
	bool run_streamer;

    public:
	// Constructor
	IIWANode(ros::NodeHandle param_nh): client(),connection(),app(connection,client) 
	{
	    n_ = ros::NodeHandle("");
	    param_nh.param<std::string>("hostname",hostname,"192.170.10.2");
	    param_nh.param<int>("port",port,DEFAULT_PORTID);
	    param_nh.param<int>("port_java",port_java,DEFAULT_JAVA_PORTID);
	    param_nh.param<double>("loop_rate",loop_rate,0.001);
	    param_nh.param<bool>("run_streamer",run_streamer,false);
	    
	    app.connect(port, hostname.c_str());
	    joints_pub_=n_.advertise<sensor_msgs::JointState>("/joint_states",1000);

	    heartbeat_ = nh_.createTimer(ros::Duration(loop_rate),&IIWANode::do_step,this);
	    publish_ctr = 0;
	    n_publish_skip = 10;
	    connected = false;

	    if(run_streamer) {
		int dataport = -1;
		target_streamer = new Server(port_java,dataport,&connected);
		if(connected) {
		    //wait for incoming connecction
		    target_streamer->Connect();
		    test_streamer = param_nh.advertiseService("test_streamer", &IIWANode::test_streamer_callback, this);;
		}
	    }
	}

	~IIWANode()
	{
	    app.disconnect();
	    if(target_streamer != NULL) delete target_streamer;
	}

	void do_step(const ros::TimerEvent& event) {
	    bool success = app.step();
	    publish_ctr++;
	    if(publish_ctr > n_publish_skip) {
		publish_ctr = 0;
		sensor_msgs::JointState js;
		client.getJointMsg(js);
		js.header.stamp = ros::Time::now();
		joints_pub_.publish(js);
	    }
	}

	bool test_streamer_callback(std_srvs::Empty::Request  &req,
		    std_srvs::Empty::Response &res ) {

	    if(run_streamer && connected) {
		double *jv = new double[7];
		for(int i=0; i<7; ++i) jv[i] = 0;

		target_streamer->SendDoubles(jv, 7);

		delete []jv;
	    }
	    return true;
	}


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "iiwa_node");

    ros::NodeHandle param("~");
    IIWANode nd(param);
    
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
