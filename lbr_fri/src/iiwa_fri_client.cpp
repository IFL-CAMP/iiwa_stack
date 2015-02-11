#include <iiwa_fri_client.h>

//******************************************************************************
void IIWAFRIClient::onStateChange(ESessionState oldState, ESessionState newState)
{
   LBRClient::onStateChange(oldState, newState);
   switch (newState)
   {
      case MONITORING_READY:
      {
	 //set targets to current joints
	 memcpy(joint_targets, robotState().getMeasuredJointPosition(), 7 * sizeof(double));
         break;
      }
      default:
      {
         break;
      }
   }
}
   
//******************************************************************************
void IIWAFRIClient::monitor()
{
    LBRState current_state = robotState();
    memcpy(joint_pos, current_state.getMeasuredJointPosition(), 7 * sizeof(double));
    memcpy(joint_torques, current_state.getMeasuredTorque(), 7 * sizeof(double));
}
   
void IIWAFRIClient::getJointMsg(sensor_msgs::JointState &msg) {
    std::vector<double> pos (joint_pos, joint_pos+sizeof(joint_pos)/sizeof(double));
    msg.position = pos;

    std::vector<double> torques (joint_torques, joint_torques+sizeof(joint_torques)/sizeof(double));
    msg.effort = torques;
    msg.name = joint_names;
}
   
//******************************************************************************
void IIWAFRIClient::getJointsRaw(double (&pos)[7], double (&vel)[7], double (&eff)[7]) {
    memcpy(pos, joint_pos, 7 * sizeof(double));
    memcpy(eff, joint_torques, 7 * sizeof(double));
}
   
void IIWAFRIClient::setJointTargets(const double (&com)[7]) {
    memcpy(joint_targets, com, 7 * sizeof(double));
    //std::cerr<<"com[6]"<<com[6]<<std::endl;
}

//******************************************************************************
void IIWAFRIClient::command()
{
   // add offset to ipo joint position for all masked joints
    LBRState current_state = robotState();
    //memcpy(joint_pos, current_state.getIpoJointPosition(), 7 * sizeof(double));
    memcpy(joint_pos, current_state.getMeasuredJointPosition(), 7 * sizeof(double));
    memcpy(joint_torques, current_state.getMeasuredTorque(), 7 * sizeof(double));
    //std::cerr<<"joint_targets[6]"<<joint_targets[6]<<std::endl;
    robotCommand().setJointPosition(joint_targets);
    //robotCommand().setJointPosition(joint_pos);
}
   
/**
 * thread that sends joint commands 
 */
void IIWAFRIClientNative::commandThread()  {

    int port = 5011;
    int dataport =-1;
    bool bResult = false;

    command_send = new Server(port, dataport, &bResult);
    if (!bResult)
    {
	printf("Failed to create Server object!\n");
	return;
    }

    fflush(NULL);
    command_send->Connect();
    {
	boost::mutex::scoped_lock lock(session_m);
	send_established = true;
	session_cond_.notify_one();
    }
    printf("SEND Server, got a connection...\n");
    fflush(NULL);

    while(true)
    {
	{
	    boost::mutex::scoped_lock lock(socket_m);
	    while(!doStep) {
		socket_cond_.wait(lock);
	    }
	    break;
	}
    }

    //cleanup
    command_send->Close();
    delete command_send;
    command_send=NULL;

}

/**
 * thread that reads joint values 
 */
void IIWAFRIClientNative::monitorThread() {
    int port = 5010;
    int dataport =-1;
    bool bResult = false;

    joints_recv = new Server(port, dataport, &bResult);
    if (!bResult)
    {
	printf("Failed to create Server object!\n");
	return;
    }
    
    fflush(NULL);
    joints_recv->Connect();
    {
	boost::mutex::scoped_lock lock(session_m);
	recv_established = true;
	session_cond_.notify_one();
    }
    printf("RECV Server, got a connection...\n");
    fflush(NULL);

    while(true) {
//	printf("receiving from  socket\n");
	{
	    boost::mutex::scoped_lock lock(socket_m);
	    while(!doStep) {
		socket_cond_.wait(lock);
	    }
	    break;
	    //doStep=false;
	}
	
    }
    //cleanup
    joints_recv->Close();
    delete joints_recv;
    joints_recv=NULL;
}
   
void IIWAFRIClientNative::getJointMsg(sensor_msgs::JointState &msg) {
    double jp[7], vel[7], eff[7];
    this->getJointsRaw(jp,vel,eff);
    std::vector<double> pos (jp, jp+sizeof(jp)/sizeof(double));
    msg.position = pos;
    msg.name = joint_names;

}

void IIWAFRIClientNative::getJointsRaw(double (&pos)[7], double (&vel)[7], double (&eff)[7]) {
    printf("getting joints\n");
    js_m.lock();
    printf("got mutex\n");
    fflush(NULL);
    joints_recv->RecvDoubles(joint_pos, SIZE);
    js_m.unlock();
    memcpy(pos, joint_pos, SIZE * sizeof(double));
    
    
    t_m.lock();
    double now = getDoubleTime();
    period = now-last_read_time;
    if(last_read_time == 0) period = 0;
    last_read_time = now;
    printf("time : %lf, period %lf\n", now,period);
    t_m.unlock();
    printf("got joints\n");
}

void IIWAFRIClientNative::setJointTargets(double (&com)[7]) {
    printf("setting targets\n");
    jt_m.lock();
    printf("got target mutex\n");
    fflush(NULL);
    command_send->SendDoubles(com, SIZE);
    jt_m.unlock();
    printf("released target mutex\n");
}
