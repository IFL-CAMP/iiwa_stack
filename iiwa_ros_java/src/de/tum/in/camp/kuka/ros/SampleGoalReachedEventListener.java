package de.tum.in.camp.kuka.ros;

import com.kuka.connectivity.motionModel.smartServo.IServoOnGoalReachedEvent; 
import com.kuka.connectivity.motionModel.smartServo.IServoRuntime;



public class SampleGoalReachedEventListener implements IServoOnGoalReachedEvent{
	IServoRuntime smart_event;
	@Override
	public void onGoalReachedEvent(String state, double[] remainingTime,int[] osTimestamp, int targetId) {
		
		remainingTime[0]= smart_event.getRemainingTime();
		
		if (smart_event.isDestinationReached()){
			targetId = 0;
		}
		else targetId = 1;
	}

}
