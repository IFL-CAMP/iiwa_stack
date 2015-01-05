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
    std::cerr<<"com[6]"<<com[6]<<std::endl;
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
