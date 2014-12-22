#include <iiwa_fri_client.h>

//******************************************************************************
void IIWAFRIClient::onStateChange(ESessionState oldState, ESessionState newState)
{
   LBRClient::onStateChange(oldState, newState);
   switch (newState)
   {
      case MONITORING_READY:
      {
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
void IIWAFRIClient::command()
{
   // add offset to ipo joint position for all masked joints
    LBRState current_state = robotState();
    memcpy(joint_pos, current_state.getIpoJointPosition(), 7 * sizeof(double));
    memcpy(joint_torques, current_state.getMeasuredTorque(), 7 * sizeof(double));
   //jointPos[0] = -M_PI/4;

   //for (int i=0; i<7; i++)
   //{
       //here set the offset from current joint value
       //jointPos[i] += _offset;
   //}
   robotCommand().setJointPosition(joint_pos);
}
