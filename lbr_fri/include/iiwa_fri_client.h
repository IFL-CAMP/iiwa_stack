#ifndef _IIWA_FRI_CLIENT_H
#define _IIWA_FRI_CLIENT_H

#include "friLBRClient.h"
#include <sensor_msgs/JointState.h>
#include <string>

using namespace KUKA::FRI;

/**
 * \brief Test client that can overlay interpolator joint positions with sine waves.
 */
class IIWAFRIClient : public LBRClient
{
   
public:
   
    IIWAFRIClient () { 
	joint_names.push_back("lbr_iiwa_joint_1");
	joint_names.push_back("lbr_iiwa_joint_2");
	joint_names.push_back("lbr_iiwa_joint_3");
	joint_names.push_back("lbr_iiwa_joint_4");
	joint_names.push_back("lbr_iiwa_joint_5");
	joint_names.push_back("lbr_iiwa_joint_6");
	joint_names.push_back("lbr_iiwa_joint_7");
    }; 
    ~IIWAFRIClient () { };   
   
    /**
    * \brief Callback for FRI state changes.
    * 
    * @param oldState
    * @param newState
    */
   virtual void onStateChange(ESessionState oldState, ESessionState newState);
   
   /**
    * \brief Callback for the FRI state 'Commanding Active'.
    */
   virtual void command();
   
   /**
    * \brief Callback for the FRI state 'Monitoring Active'.
    */
   virtual void monitor();

   void getJointMsg(sensor_msgs::JointState &msg);
   
private:
   double joint_pos[7];
   double joint_torques[7];
   std::vector<std::string> joint_names;
};

#endif // _IIWA_FRI_CLIENT_H
