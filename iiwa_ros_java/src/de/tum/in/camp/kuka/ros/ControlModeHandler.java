package de.tum.in.camp.kuka.ros;

import iiwa_msgs.CartesianQuantity;
import iiwa_msgs.ConfigureSmartServoRequest;
import iiwa_msgs.JointQuantity;

import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.task.ITaskLogger;

public class ControlModeHandler {

	private LBR robot;
	private Tool tool;
	private ObjectFrame toolFrame;
	private ITaskLogger logger;
	private iiwaPublisher publisher;

	public double jointVelocity;
	public double jointAcceleration;
	public double overrideJointAcceleration;
	
	public double[] maxTranslationlVelocity = {1000.0, 1000.0, 1000.0};
	public double[] maxOrientationVelocity = {0.5, 0.5, 0.5};

	private ConfigureSmartServoRequest lastSmartServoRequest;

	private MessageGenerator helper;
	private GoalReachedEventListener handler;

	public ControlModeHandler(LBR robot, Tool tool, ObjectFrame toolFrame, iiwaPublisher publisher, iiwaActionServer actionServer, Configuration configuration) {
		this.robot = robot;
		this.tool = tool;
		this.toolFrame = toolFrame;
		this.publisher = publisher;
		helper = new MessageGenerator(Configuration.getRobotName(), configuration);
		jointVelocity = configuration.getDefaultRelativeJointVelocity();
		jointAcceleration = configuration.getDefaultRelativeJointAcceleration();
		overrideJointAcceleration = 1.0;
		
		handler = new GoalReachedEventListener(publisher, actionServer);
	}

	public void setLastSmartServoRequest(ConfigureSmartServoRequest request) { this.lastSmartServoRequest = request; }

	public void setPathParamters(double jointVelocity, double jointAcceleration, double overrideJointAcceleration) {
		this.jointVelocity = jointVelocity;
		this.jointAcceleration = jointAcceleration;
		this.overrideJointAcceleration = overrideJointAcceleration;
	}

	/**
	 * Allows to switch control mode on the fly. Kills the given smartServo motion and creates a new one with the given request.
	 * If the given request is null, the last received request will be used. This is the case if only velocity and/or acceleration need(s) to be changed.
	 * @param motion
	 * @param request
	 */
//	@SuppressWarnings("rawtypes")
//	public ServoMotion switchSmartServoMotion(ServoMotion motion, iiwa_msgs.ConfigureSmartServoRequest request) {
//		ServoMotion oldMotion = motion;
//
//		validateForImpedanceMode();
//
//		if (motion instanceof SmartServo) { motion = createSmartServoMotion(); }
//		else if (motion instanceof SmartServoLIN) { motion = createSmartServoLinMotion(); }
//
//		if (request != null) {
//			motion.setMode(buildMotionControlMode(request));
//		} 
//		else if (lastSmartServoRequest != null) {
//			motion.setMode(buildMotionControlMode(lastSmartServoRequest));
//		}
//		else {
//			motion.setMode(new PositionControlMode());
//		}
//
//		switchMotion(motion, oldMotion);
//
//		return motion;
//	}
	
	public SmartServo switchSmartServoMotion(SmartServo motion, iiwa_msgs.ConfigureSmartServoRequest request) {
		SmartServo oldMotion = motion;

		validateForImpedanceMode();

		motion = createSmartServoMotion();

		if (request != null) {
			motion.setMode(buildMotionControlMode(request));
		} 
		else if (lastSmartServoRequest != null) {
			motion.setMode(buildMotionControlMode(lastSmartServoRequest));
		}
		else {
			motion.setMode(new PositionControlMode());
		}

		switchMotion(motion, oldMotion);

		return motion;		
	}
	
	public SmartServoLIN switchSmartServoMotion(SmartServoLIN motion, iiwa_msgs.ConfigureSmartServoRequest request) {
		SmartServoLIN oldMotion = motion;

		validateForImpedanceMode();

		motion = createSmartServoLinMotion();

		if (request != null) {
			motion.setMode(buildMotionControlMode(request));
		} 
		else if (lastSmartServoRequest != null) {
			motion.setMode(buildMotionControlMode(lastSmartServoRequest));
		}
		else {
			motion.setMode(new PositionControlMode());
		}

		switchMotion(motion, oldMotion);

		return motion;		
	}

	/**
	 * Allows to switch control mode on the fly. Kills the given smartServo motion and creates a new one with the given controlMode.
	 * @param motion
	 * @param controlMode
	 * @return
	 */
//	@SuppressWarnings("rawtypes")
//	public ServoMotion switchSmartServoMotion(ServoMotion motion, IMotionControlMode controlMode) {
//		if (controlMode != motion.getMode()) {
//
//			ServoMotion oldMotion = motion;
//
//			if (!(controlMode instanceof PositionControlMode)) {
//				validateForImpedanceMode();
//			}
//
//			if (motion instanceof SmartServo) { motion = createSmartServoMotion(); }
//			else if (motion instanceof SmartServoLIN) { motion = createSmartServoLinMotion(); }
//
//			motion.setMode(controlMode);
//
//			switchMotion(motion, oldMotion);
//		}
//		else {
//			motion.getRuntime().changeControlModeSettings(controlMode);
//		}
//		return motion;
//	}

	public SmartServoLIN switchSmartServoMotion(SmartServo motion, IMotionControlMode controlMode) { 
		SmartServo oldMotion = motion;

		if (!(controlMode instanceof PositionControlMode)) {
			validateForImpedanceMode();
		}

		SmartServoLIN newMotion = createSmartServoLinMotion();
		newMotion.setMode(controlMode);
		switchMotion(newMotion, oldMotion);
		return newMotion; 
	}

	/**
	 * Switches a the SmartServo Motion
	 * @param oldMotion       Current motion
	 * @param newControlMode  new motion mode
	 * @return new SmartServo motion
	 */
	public SmartServo switchSmartServoMotion(SmartServoLIN oldMotion, IMotionControlMode newControlMode) {

		if (!(newControlMode instanceof PositionControlMode)) {
			validateForImpedanceMode();
		}

		SmartServo newMotion = createSmartServoMotion();
		newMotion.setMode(newControlMode);
		switchMotion(newMotion, oldMotion);
		return newMotion;  
	}


	// TODO: doc
	@SuppressWarnings("rawtypes")
	public void switchMotion(ServoMotion motion, ServoMotion oldMotion) {
		toolFrame.moveAsync(motion);
		if (oldMotion != null) {
			oldMotion.getRuntime().stopMotion();
		}
		motion.getRuntime().setGoalReachedEventHandler(handler);
	}

	public SmartServo switchToSmartServo(SmartServo motion, SmartServoLIN linearMotion) {
		System.out.println("Switching to SmartServo motion");
		IMotionControlMode currentMode = motion.getMode();
		if (currentMode == null) { currentMode = new PositionControlMode(); }
		motion = switchSmartServoMotion(linearMotion, currentMode);
		return motion;
	}

	public SmartServoLIN switchToSmartServoLIN(SmartServo motion, SmartServoLIN linearMotion) {
		System.out.println("Switching to SmartServoLIN motion");
		IMotionControlMode currentMode = motion.getMode();
		if (currentMode == null) { currentMode = new PositionControlMode(); }
		linearMotion = switchSmartServoMotion(motion, currentMode);
		return linearMotion;
	}

	/**
	 * Validates the robot for Impedance control, it something fails here Impedance control is not possible.
	 * Reasons might be: 
	 *  - too much force is applied on the robot when the Impedance control mode is selected
	 *  - the robot is in a singularity
	 */
	private void validateForImpedanceMode() {
		if (tool != null) {
			ServoMotion.validateForImpedanceMode(tool);
		}
		else {
			ServoMotion.validateForImpedanceMode(robot);
		}
	}

	/**
	 * Generates a new smartServoMotion with the current parameters.
	 * @return
	 */
	public SmartServo createSmartServoMotion() {
		SmartServo motion = new SmartServo(robot.getCurrentJointPosition());
		motion.setMinimumTrajectoryExecutionTime(0.1); //TODO : parametrize
		motion.setTimeoutAfterGoalReach(3600); //TODO : parametrize
		motion.setJointVelocityRel(jointVelocity);
		motion.setJointAccelerationRel(jointAcceleration);
		motion.overrideJointAcceleration(overrideJointAcceleration);
		return motion;
	}

	public SmartServoLIN createSmartServoLinMotion() {
		System.out.println("toolFrame: "+toolFrame);
		
		SmartServoLIN linearMotion = new SmartServoLIN(robot.getCurrentCartesianPosition(toolFrame));
		linearMotion.setReferenceFrame(World.Current.getRootFrame());
		linearMotion.setMinimumTrajectoryExecutionTime(0.1); //TODO : parametrize
		linearMotion.setTimeoutAfterGoalReach(3600); //TODO : parametrize
		linearMotion.setMaxTranslationVelocity(maxTranslationlVelocity);
		linearMotion.setMaxOrientationVelocity(maxOrientationVelocity);
		//linearMotion.setMaxTranslationAcceleration(value);
		//linearMotion.setMaxNullSpaceAcceleration(value);
		//linearMotion.setMaxNullSpaceVelocity(value);
		//linearMotion.setMaxOrientationAcceleration(value);

		return linearMotion;
	}

	/**
	 * Generates a JointImpedanceControlMode from the given request.
	 * @param request
	 * @return
	 */
	private JointImpedanceControlMode buildJointImpedanceControlMode(iiwa_msgs.ConfigureSmartServoRequest request) {
		JointImpedanceControlMode jcm = new JointImpedanceControlMode(robot.getJointCount());

		JointQuantity stiffness = request.getJointImpedance().getJointStiffness();
		if (helper.isJointQuantityGreaterEqualThan(stiffness, 0)) {
			jcm.setStiffness(Conversions.jointQuantityToVector(stiffness));
		}

		JointQuantity damping = request.getJointImpedance().getJointDamping();
		if (helper.isJointQuantityGreaterEqualThan(damping, 0)) {
			jcm.setDamping(Conversions.jointQuantityToVector(damping));
		}
		return jcm;
	}

	/**
	 * Generates a CartesianImpedanceControlMode from the given request.
	 * @param request
	 * @return
	 */
	private CartesianImpedanceControlMode buildCartesianImpedanceControlMode(iiwa_msgs.ConfigureSmartServoRequest request) {
		CartesianImpedanceControlMode ccm = new CartesianImpedanceControlMode();

		iiwa_msgs.CartesianQuantity stiffness = request.getCartesianImpedance().getCartesianStiffness();
		if (stiffness.getX() >= 0) {
			ccm.parametrize(CartDOF.X).setStiffness(stiffness.getX());
		}
		if (stiffness.getY() >= 0) {
			ccm.parametrize(CartDOF.Y).setStiffness(stiffness.getY()); 
		}
		if (stiffness.getZ() >= 0) {
			ccm.parametrize(CartDOF.Z).setStiffness(stiffness.getZ());
		}
		if (stiffness.getA() >= 0) {
			ccm.parametrize(CartDOF.A).setStiffness(stiffness.getA());
		}
		if (stiffness.getB() >= 0) {
			ccm.parametrize(CartDOF.B).setStiffness(stiffness.getB());
		}
		if (stiffness.getC() >= 0) {
			ccm.parametrize(CartDOF.C).setStiffness(stiffness.getC());
		}

		CartesianQuantity damping = request.getCartesianImpedance().getCartesianDamping();
		if (damping.getX() > 0) {
			ccm.parametrize(CartDOF.X).setDamping(damping.getX());
		}
		if (damping.getY() > 0) {
			ccm.parametrize(CartDOF.Y).setDamping(damping.getY());
		}
		if (damping.getZ() > 0) {
			ccm.parametrize(CartDOF.Z).setDamping(damping.getZ());
		}
		if (damping.getA() > 0) {
			ccm.parametrize(CartDOF.A).setDamping(damping.getA());
		}
		if (damping.getB() > 0) {
			ccm.parametrize(CartDOF.B).setDamping(damping.getB());
		}
		if (damping.getC() > 0) {
			ccm.parametrize(CartDOF.C).setDamping(damping.getC());
		}

		if (request.getCartesianImpedance().getNullspaceStiffness() >= 0) {
			ccm.setNullSpaceStiffness(request.getCartesianImpedance().getNullspaceStiffness());
		}
		if (request.getCartesianImpedance().getNullspaceDamping() > 0) {
			ccm.setNullSpaceDamping(request.getCartesianImpedance().getNullspaceDamping());
		}

		addControlModeLimits(ccm, request.getLimits());
		return ccm;
	}

	/**
	 * Given the parameters from the SmartServo service, it builds up the new control mode to use.
	 * @param request : parameters from the ConfigureSmartServo service
	 * @return resulting control mode
	 */
	public IMotionControlMode buildMotionControlMode(iiwa_msgs.ConfigureSmartServoRequest request) {
		IMotionControlMode cm = null;

		switch (request.getControlMode()) {

		case iiwa_msgs.ControlMode.POSITION_CONTROL: {
			cm = new PositionControlMode(true);
			break;
		}

		case iiwa_msgs.ControlMode.JOINT_IMPEDANCE: {
			cm = buildJointImpedanceControlMode(request);
			break;
		}

		case iiwa_msgs.ControlMode.CARTESIAN_IMPEDANCE: {
			cm = buildCartesianImpedanceControlMode(request);
			break;
		}

		case iiwa_msgs.ControlMode.DESIRED_FORCE : {
			CartesianSineImpedanceControlMode cscm = new CartesianSineImpedanceControlMode();
			CartDOF direction = selectDegreeOfFreedom(request.getDesiredForce().getCartesianDof());

			if (direction != null && request.getDesiredForce().getDesiredStiffness() >= 0) {
				cscm = CartesianSineImpedanceControlMode.createDesiredForce(direction, request.getDesiredForce().getDesiredForce(),  request.getDesiredForce().getDesiredStiffness());
				addControlModeLimits(cscm, request.getLimits());
				cm = cscm;
			}
			break;
		}

		case iiwa_msgs.ControlMode.SINE_PATTERN : {
			CartesianSineImpedanceControlMode cscm = new CartesianSineImpedanceControlMode();
			CartDOF direction = selectDegreeOfFreedom(request.getSinePattern().getCartesianDof());

			if (direction != null && request.getSinePattern().getFrequency() >= 0 && request.getSinePattern().getAmplitude() >= 0 && request.getSinePattern().getStiffness() >= 0) {
				cscm = CartesianSineImpedanceControlMode.createSinePattern(direction, request.getSinePattern().getFrequency(), request.getSinePattern().getAmplitude(), request.getSinePattern().getStiffness());
				addControlModeLimits(cscm, request.getLimits());
				cm = cscm;
			}
			break;
		}

		default:				
			logger.error("Control Mode not supported.");
			throw new UnsupportedControlModeException();  // this should just not happen
		}

		if (cm != null) {
			return cm;
		}
		else {
			throw new UnsupportedControlModeException();
		}
	}

	/**
	 * Transforms a iiwa_msgs.DOF to a KUKA CartDOF object
	 * @param dof
	 * @return
	 */
	private CartDOF selectDegreeOfFreedom(int dof) {
		CartDOF direction = null;
		switch (dof) {
		case iiwa_msgs.DOF.X : {
			direction = CartDOF.X;
			break;
		}
		case iiwa_msgs.DOF.Y : {
			direction = CartDOF.Y;
			break;
		}
		case iiwa_msgs.DOF.Z : {
			direction = CartDOF.Z;
			break;
		}
		default: {
			logger.error("Wrong direction given, use [1,2,3] for directions [X,Y,Z] respectively.");
			break;
		}
		}
		return direction;
	}

	/**
	 * Adds Cartesian limits - maxPathDeviation, maxCartesianVelocity, maxControlForce - to a CartesianImpedanceControlMode
	 * @param controlMode
	 * @param limits
	 */
	private void addControlModeLimits(CartesianImpedanceControlMode controlMode, iiwa_msgs.CartesianControlModeLimits limits) {
		CartesianQuantity maxPathDeviation = limits.getMaxPathDeviation();
		if (helper.isCartesianQuantityGreaterThan(maxPathDeviation, 0)) {
			controlMode.setMaxPathDeviation(maxPathDeviation.getX(), maxPathDeviation.getY(), maxPathDeviation.getZ(), maxPathDeviation.getA(), maxPathDeviation.getB(), maxPathDeviation.getC());
		}

		CartesianQuantity maxControlForce = limits.getMaxControlForce();
		if (helper.isCartesianQuantityGreaterThan(maxControlForce, 0)) {
			controlMode.setMaxControlForce(maxControlForce.getX(), maxControlForce.getY(), maxControlForce.getZ(), maxControlForce.getA(), maxControlForce.getB(), maxControlForce.getC(), limits.getMaxControlForceStop());
		}

		CartesianQuantity maxCartesianVelocity = limits.getMaxCartesianVelocity();
		if (helper.isCartesianQuantityGreaterThan(maxCartesianVelocity, 0)) {
			controlMode.setMaxCartesianVelocity(maxCartesianVelocity.getX(), maxCartesianVelocity.getY(), maxCartesianVelocity.getZ(), maxCartesianVelocity.getA(), maxCartesianVelocity.getB(),maxCartesianVelocity.getC());
		}
	}

	/**
	 * Checks if a SmartServoMode is of the same type as a MotionControlMode from KUKA APIs
	 * @return boolean
	 */
	public boolean isSameControlMode(IMotionControlMode kukacm, int roscm) {
		if (kukacm == null) { return false; }
		String roscmname = null;
		switch (roscm) {
		case iiwa_msgs.ControlMode.POSITION_CONTROL:
			roscmname = "PositionControlMode";
			break;
		case iiwa_msgs.ControlMode.JOINT_IMPEDANCE:
			roscmname = "JointImpedanceControlMode";
			break;
		case iiwa_msgs.ControlMode.CARTESIAN_IMPEDANCE:
			roscmname = "CartesianImpedanceControlMode";
			break;
		case iiwa_msgs.ControlMode.DESIRED_FORCE:
			roscmname = "CartesianSineImpedanceControlMode";
			break;
		case iiwa_msgs.ControlMode.SINE_PATTERN:
			roscmname = "CartesianSineImpedanceControlMode";
			break;
		}
		String kukacmname = kukacm.getClass().getSimpleName();

		return roscmname.equals(kukacmname);
	}
}
