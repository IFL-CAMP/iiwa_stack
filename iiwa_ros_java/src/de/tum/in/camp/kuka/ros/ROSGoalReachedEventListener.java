package de.tum.in.camp.kuka.ros;

import com.kuka.connectivity.motionModel.smartServo.IServoOnGoalReachedEvent;

public class ROSGoalReachedEventListener implements IServoOnGoalReachedEvent {

	protected iiwaPublisher publisher_;

	public ROSGoalReachedEventListener(iiwaPublisher publisher) {
		publisher_ = publisher;
	}

	@Override
	public void onGoalReachedEvent(String state, double[] currentAxisPos, int[] osTimestamp, int targetId) {
		if (publisher_ != null) {
		publisher_.publishDestinationReached();
		}
	}
}
