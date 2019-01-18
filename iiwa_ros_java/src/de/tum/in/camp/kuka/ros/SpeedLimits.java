package de.tum.in.camp.kuka.ros;

public class SpeedLimits {
	public static double jointVelocity;
	public static double jointAcceleration;
	public static double overrideJointAcceleration;
	
	public static double[] maxTranslationlVelocity = {1000.0, 1000.0, 1000.0};
	public static double[] maxOrientationVelocity = {0.5, 0.5, 0.5};
	
	public static double cartesianAcceleration = 200.0;
	public static double orientationAcceleration = 0.05;
	public static double cartesianJerk = 50;
	
	public static void init(Configuration configuration) {
		jointVelocity = configuration.getDefaultRelativeJointVelocity();
		jointAcceleration = configuration.getDefaultRelativeJointAcceleration();
		overrideJointAcceleration = 1.0;
	}

	public static void setPathParamters(double jointVelocity, double jointAcceleration, double overrideJointAcceleration) {
		SpeedLimits.jointVelocity = jointVelocity;
		SpeedLimits.jointAcceleration = jointAcceleration;
		SpeedLimits.overrideJointAcceleration = overrideJointAcceleration;
	}
}
