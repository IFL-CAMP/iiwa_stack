#include <stdlib.h>
#include <stdio.h>
#include <string.h> // strstr
#include "iiwa_fri_client.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <iostream>

#include <ros/ros.h>
#include "sensor_msgs/JointState.h"

#define DEFAULT_PORTID 30200

using namespace KUKA::FRI;


class IIWANode {

    protected:
	// Our NodeHandle
	ros::NodeHandle nh_;
	ros::NodeHandle n_;
        ros::Timer heartbeat_;

	ros::Publisher joints_pub_;

	//boost::mutex m, message_m;
	//ros::ServiceServer save_map_;
	std::string hostname;
	int port;
	int publish_ctr;
	double loop_rate;
	int n_publish_skip;
   
	IIWAFRIClient client;
	UdpConnection connection;
	ClientApplication app;

    public:
	// Constructor
	IIWANode(ros::NodeHandle param_nh): client(),connection(),app(connection,client) 
	{
	    n_ = ros::NodeHandle("");
	    param_nh.param<std::string>("hostname",hostname,"192.170.10.2");
	    param_nh.param<int>("port",port,DEFAULT_PORTID);
	    param_nh.param<double>("loop_rate",loop_rate,0.001);
	    
	    app.connect(port, hostname.c_str());
	    joints_pub_=n_.advertise<sensor_msgs::JointState>("/joint_states",1000);

	    heartbeat_ = nh_.createTimer(ros::Duration(loop_rate),&IIWANode::do_step,this);
	    publish_ctr = 0;
	    n_publish_skip = 10;
	}

	~IIWANode()
	{
	    app.disconnect();
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

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "iiwa_node");

    ros::NodeHandle param("~");
    IIWANode nd(param);

    ros::spin();

    return 0;
}
