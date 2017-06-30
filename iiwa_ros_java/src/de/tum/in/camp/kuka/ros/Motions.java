package de.tum.in.camp.kuka.ros;

import geometry_msgs.PoseStamped;

import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;

public class Motions {
	
	private LBR robot;

	private JointPosition jp;
	private JointPosition jv;
	private JointPosition jointDisplacement;
	
	private double loopPeriod; // Loop period in s
	private long previousTime; // Timestamp of previous setDestination() in s
	private long currentTime; // Timestamp of last setDestination() in s
	
	public Motions(LBR robot, SmartServo motion) {
		this.robot = robot;
		jp = new JointPosition(robot.getJointCount());
		jv = new JointPosition(robot.getJointCount());
		jointDisplacement = new JointPosition(robot.getJointCount());
		
		// Initialize time stamps
		previousTime = motion.getRuntime().updateWithRealtimeSystem();
		currentTime = motion.getRuntime().updateWithRealtimeSystem();
		loopPeriod = 0.0;
	}
	
	public void cartesianPositionMotion(SmartServo motion, PoseStamped commandPosition) {
		if (commandPosition != null) {
			Frame destinationFrame = Conversions.rosPoseToKukaFrame(commandPosition.getPose());
			if (robot.isReadyToMove()) {
				// try to move to the commanded cartesian pose, it something goes wrong catch the exception.
				motion.getRuntime().setDestination(destinationFrame);
			}
		}
	}
	
	public void jointPositionMotion(SmartServo motion, iiwa_msgs.JointPosition commandPosition) {
		if (commandPosition != null) {
			Conversions.rosJointQuantityToKuka(commandPosition.getPosition(), jp);
			if (robot.isReadyToMove()) {
				motion.getRuntime().setDestination(jp);
			}
		}
	}
	
	public void jointPositionVelocityMotion(SmartServo motion, iiwa_msgs.JointPositionVelocity commandPositionVelocity) {
		if (commandPositionVelocity != null) {
			Conversions.rosJointQuantityToKuka(commandPositionVelocity.getPosition(), jp);
			Conversions.rosJointQuantityToKuka(commandPositionVelocity.getVelocity(), jv);
			if (robot.isReadyToMove()) {
				motion.getRuntime().setDestination(jp, jv);
			}
		}
	}
	
	public void jointVelocityMotion(SmartServo motion, iiwa_msgs.JointVelocity commandVelocity) {
		if (commandVelocity != null) {
			jp = motion.getRuntime().getCurrentJointDestination();
			Conversions.rosJointQuantityToKuka(commandVelocity.getVelocity(), jointDisplacement, loopPeriod); // compute the joint displacement over the current period.
			Conversions.rosJointQuantityToKuka(commandVelocity.getVelocity(), jv);

			for(int i = 0; i < robot.getJointCount(); ++i) { jp.set(i, jp.get(i) + jointDisplacement.get(i)); } //add the displacement to the joint destination.
			previousTime = currentTime;

			if (robot.isReadyToMove()) {
				motion.getRuntime().setDestination(jp, jv);
			}

			currentTime = motion.getRuntime().updateWithRealtimeSystem();
			loopPeriod = (double)(currentTime - previousTime) / 1000.0; // loopPerios is stored in seconds.
		}
	}

}
