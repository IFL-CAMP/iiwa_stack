/**  
 * Copyright (C) 2016-2017 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
 * Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

package de.tum.in.camp.kuka.ros;

// ROS imports
import geometry_msgs.PoseStamped;
import iiwa_msgs.CartesianQuantity;
import iiwa_msgs.ConfigureSmartServoRequest;
import iiwa_msgs.ConfigureSmartServoResponse;
import iiwa_msgs.JointQuantity;
import iiwa_msgs.TimeToDestinationRequest;
import iiwa_msgs.TimeToDestinationResponse;

import java.net.URI;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.ros.exception.ServiceException;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.service.ServiceResponseBuilder;

import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

/*
 * This application allows to command the robot using SmartServo motions.
 */
public class ROSSmartServo extends ROSBaseApplication {

	private Lock configureSmartServoLock = new ReentrantLock();

	private iiwaMessageGenerator helper; // Helper class to generate iiwa_msgs from current robot state.
	private iiwaSubscriber subscriber; // IIWARos Subscriber.

	// Configuration of the subscriber ROS node.
	private NodeConfiguration nodeConfSubscriber;

	private JointPosition jp; 
	private JointPosition jv;
	private JointPosition jointDisplacement;

	private double loopPeriod; // Loop period in s
	private long previousTime; // Timestamp of previous setDestination() in s
	private long currentTime; // Timestamp of last setDestination() in s
	
	private ConfigureSmartServoRequest latestSmartServoRequest;

	@Override
	protected void configureNodes(URI uri) {
		// Configuration for the Subscriber.
		nodeConfSubscriber = NodeConfiguration.newPublic(iiwaConfiguration.getRobotIp());
		nodeConfSubscriber.setTimeProvider(iiwaConfiguration.getTimeProvider());
		nodeConfSubscriber.setNodeName(iiwaConfiguration.getRobotName() + "/iiwa_subscriber");
		nodeConfSubscriber.setMasterUri(uri);
	}

	@Override
	protected void addNodesToExecutor(NodeMainExecutor nodeMainExecutor) {
		subscriber = new iiwaSubscriber(robot, iiwaConfiguration.getRobotName());

		// Configure the callback for the SmartServo service inside the subscriber class.
		subscriber.setConfigureSmartServoCallback(new ServiceResponseBuilder<iiwa_msgs.ConfigureSmartServoRequest, iiwa_msgs.ConfigureSmartServoResponse>() {
			@Override
			public void build(ConfigureSmartServoRequest req, ConfigureSmartServoResponse res) throws ServiceException {
				try {
					if (isSameControlMode(motion.getMode(), req.getControlMode())) { // We can just change the parameters if the control strategy is the same.
						if (!(motion.getMode() instanceof PositionControlMode)) { // We are in PositioControlMode and the request was for the same mode, there are no parameters to change.
							motion.getRuntime().changeControlModeSettings(buildMotionControlMode(req));
						}
					} else {
						switchSmartServoMotion(req);
					}
				} catch (Exception e) {
					res.setSuccess(false);
					if (e.getMessage() != null) {
						res.setError(e.getClass().getName() + ": " + e.getMessage());
					} else {
						res.setError("because I hate you :)");
					}
					return;
				}
				res.setSuccess(true);
				latestSmartServoRequest = req;

				getLogger().info("Changed SmartServo configuration!");
				getLogger().info("Mode: " + motion.getMode().toString());
			}
		});

		subscriber.setTimeToDestinationCallback(new ServiceResponseBuilder<iiwa_msgs.TimeToDestinationRequest, iiwa_msgs.TimeToDestinationResponse>() {

			@Override
			public void build(TimeToDestinationRequest req, TimeToDestinationResponse res) throws ServiceException {
				try {
					motion.getRuntime().updateWithRealtimeSystem();
					res.setRemainingTime(motion.getRuntime().getRemainingTime());
				}
				catch(Exception e) {
					// An exception should be thrown only if a motion/runtime is not available.
					res.setRemainingTime(-999); 
				}
			}
		});
		
		subscriber.setPathParametersCallback(new ServiceResponseBuilder<iiwa_msgs.SetPathParametersRequest, iiwa_msgs.SetPathParametersResponse>() {
			@Override
			public void build(iiwa_msgs.SetPathParametersRequest req, iiwa_msgs.SetPathParametersResponse res) throws ServiceException {
				try {
					if (req.getJointRelativeVelocity() >= 0) {
						jointVelocity = req.getJointRelativeVelocity();
					}
					if (req.getJointRelativeAcceleration() >= 0) {
						jointAcceleration = req.getJointRelativeAcceleration();
					}
					if (req.getOverrideJointAcceleration() >= 0) {
						overrideJointAcceleration = req.getOverrideJointAcceleration();
					}
					switchSmartServoMotion(null);
					res.setSuccess(true);
				}
				catch(Exception e) {
					res.setError(e.getClass().getName() + ": " + e.getMessage());
					res.setSuccess(false);
				}
			}
		});

		// Execute the subscriber node.
		nodeMainExecutor.execute(subscriber, nodeConfSubscriber);
	}

	@Override
	protected void initializeApp() {
		helper = new iiwaMessageGenerator(iiwaConfiguration.getRobotName());
		jp = new JointPosition(robot.getJointCount());
		jv = new JointPosition(robot.getJointCount());
		jointDisplacement = new JointPosition(robot.getJointCount());
	}

	public static class UnsupportedControlModeException extends RuntimeException {
		private static final long serialVersionUID = 1L;
		public UnsupportedControlModeException() { super(); }
		public UnsupportedControlModeException(String message) { super(message); }
		public UnsupportedControlModeException(String message, Throwable cause) { super(message, cause); }
		public UnsupportedControlModeException(Throwable cause) { super(cause); }
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
	 * Given the parameters from the SmartServo service, it builds up the new control mode to use.
	 * @param request : parameters from the ConfigureSmartServo service
	 * @return resulting control mode
	 */
	public IMotionControlMode buildMotionControlMode(iiwa_msgs.ConfigureSmartServoRequest request) {
		IMotionControlMode cm = null;

		switch (request.getControlMode()) {
		
		case iiwa_msgs.ControlMode.POSITION_CONTROL: {
			PositionControlMode pcm = new PositionControlMode(true);
			cm = pcm;
			break;
		}
		
		case iiwa_msgs.ControlMode.JOINT_IMPEDANCE: {
			JointImpedanceControlMode jcm = new JointImpedanceControlMode(7);

			JointQuantity stiffness = request.getJointImpedance().getJointStiffness();
			if (helper.isJointQuantityGreaterEqualThan(stiffness, 0))
				jcm.setStiffness(helper.jointQuantityToVector(stiffness));

			JointQuantity damping = request.getJointImpedance().getJointDamping();
			if (helper.isJointQuantityGreaterEqualThan(damping, 0))
				jcm.setDamping(helper.jointQuantityToVector(damping));

			cm = jcm;
			break;
		}
		
		case iiwa_msgs.ControlMode.CARTESIAN_IMPEDANCE: {
			CartesianImpedanceControlMode ccm = new CartesianImpedanceControlMode();

			iiwa_msgs.CartesianQuantity stiffness = request.getCartesianImpedance().getCartesianStiffness();
			if (stiffness.getX() >= 0)
				ccm.parametrize(CartDOF.X).setStiffness(stiffness.getX());
			if (stiffness.getY() >= 0)
				ccm.parametrize(CartDOF.Y).setStiffness(stiffness.getY());
			if (stiffness.getZ() >= 0)
				ccm.parametrize(CartDOF.Z).setStiffness(stiffness.getZ());
			if (stiffness.getA() >= 0)
				ccm.parametrize(CartDOF.A).setStiffness(stiffness.getA());
			if (stiffness.getB() >= 0)
				ccm.parametrize(CartDOF.B).setStiffness(stiffness.getB());
			if (stiffness.getC() >= 0)
				ccm.parametrize(CartDOF.C).setStiffness(stiffness.getC());

			CartesianQuantity damping = request.getCartesianImpedance().getCartesianDamping();
			if (damping.getX() > 0)
				ccm.parametrize(CartDOF.X).setDamping(damping.getX());
			if (damping.getY() > 0)
				ccm.parametrize(CartDOF.Y).setDamping(damping.getY());
			if (damping.getZ() > 0)
				ccm.parametrize(CartDOF.Z).setDamping(damping.getZ());
			if (damping.getA() > 0)
				ccm.parametrize(CartDOF.A).setDamping(damping.getA());
			if (damping.getB() > 0)
				ccm.parametrize(CartDOF.B).setDamping(damping.getB());
			if (damping.getC() > 0)
				ccm.parametrize(CartDOF.C).setDamping(damping.getC());

			if (request.getCartesianImpedance().getNullspaceStiffness() >= 0)
				ccm.setNullSpaceStiffness(request.getCartesianImpedance().getNullspaceStiffness());
			if (request.getCartesianImpedance().getNullspaceDamping() > 0)
				ccm.setNullSpaceDamping(request.getCartesianImpedance().getNullspaceDamping());
			
			addControlModeLimits(ccm, request.getLimits());

			cm = ccm;
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
			getLogger().error("Control Mode not supported.");
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
		case iiwa_msgs.DOF.X :
			direction = CartDOF.X;
			break;
		case iiwa_msgs.DOF.Y :
			direction = CartDOF.Y;
			break;
		case iiwa_msgs.DOF.Z :
			direction = CartDOF.Z;
			break;
		default:
			getLogger().error("Wrong direction given, use [1,2,3] for directions [X,Y,Z] respectively.");
			break;
		}
		return direction;
	}

	/**
	 * Generates a new smartServoMotion with the current parameters.
	 * @return
	 */
	private SmartServo createSmartServoMotion() {
		SmartServo mot = new SmartServo(robot.getCurrentJointPosition());
		mot.setMinimumTrajectoryExecutionTime(20e-3); //TODO : parametrize
		mot.setTimeoutAfterGoalReach(300); //TODO : parametrize
		mot.setJointVelocityRel(jointVelocity);
		mot.setJointAccelerationRel(jointAcceleration);
		mot.overrideJointAcceleration(overrideJointAcceleration);
		return mot;
	}	
	
	/**
	 * Allows to switch control mode on the fly. Kills a smartServo motion and creates a new one with the given request.
	 * If null is given as argument, the last received request will be used. This is the case if only velocity and/or acceleration need(s) to be changed.
	 * @param request
	 */
	private void switchSmartServoMotion(iiwa_msgs.ConfigureSmartServoRequest request) {
		configureSmartServoLock.lock();

		SmartServo oldmotion = motion;
		if (tool != null) {
			ServoMotion.validateForImpedanceMode(tool);
		}
		else {
			ServoMotion.validateForImpedanceMode(robot);
		}
		motion = createSmartServoMotion();
		if (request != null) {
			motion.setMode(buildMotionControlMode(request));
		}
		else {
			if (latestSmartServoRequest != null) {
				motion.setMode(buildMotionControlMode(latestSmartServoRequest));
			}
			else {
				motion.setMode(new PositionControlMode());
			}
		}
		toolFrame.moveAsync(motion);
		oldmotion.getRuntime().stopMotion();
		
		motion.getRuntime().activateVelocityPlanning(true); // TODO: parametrize
		motion.getRuntime().setGoalReachedEventHandler(handler);

		configureSmartServoLock.unlock();
	}

	/**
	 * Checks if a SmartServoMode is of the same type as a MotionControlMode from KUKA APIs
	 * @return boolean
	 */
	private boolean isSameControlMode(IMotionControlMode kukacm, int roscm) {
		if (kukacm == null) return false;
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

	@Override
	protected void beforeControlLoop() { 
		motion.getRuntime().activateVelocityPlanning(true);  // TODO: do this whenever appropriate
		motion.getRuntime().setGoalReachedEventHandler(handler);

		// Initialize time stamps
		previousTime = motion.getRuntime().updateWithRealtimeSystem();
		currentTime = motion.getRuntime().updateWithRealtimeSystem();
		loopPeriod = 0.0;
	}

	@Override
	protected void controlLoop() {
		if (subscriber.currentCommandType != null) {
			configureSmartServoLock.lock();

			switch (subscriber.currentCommandType) {
			case CARTESIAN_POSE: {
				/* This will acquire the last received CartesianPose command from the commanding ROS node, if there is any available.
				 * If the robot can move, then it will move to this new position. */
				PoseStamped commandPosition = subscriber.getCartesianPose();
				if (commandPosition != null) {
					Frame destinationFrame = helper.rosPoseToKukaFrame(commandPosition.getPose());
					if (robot.isReadyToMove())
						try {
							// try to move to the commanded cartesian pose, it something goes wrong catch the exception.
							motion.getRuntime().setDestination(destinationFrame);
						}
					catch (Exception e) {
						getLogger().error(e.getClass().getName() + ": " + e.getMessage());
					}	
				}
			}
			break;
			case JOINT_POSITION: {
				/* This will acquire the last received JointPosition command from the commanding ROS node, if there is any available.
				 * If the robot can move, then it will move to this new position. */
				iiwa_msgs.JointPosition commandPosition = subscriber.getJointPosition();
				if (commandPosition != null) {
					helper.rosJointQuantityToKuka(commandPosition.getPosition(), jp);
					if (robot.isReadyToMove())
						motion.getRuntime().setDestination(jp);
				}
			}
			break;
			case JOINT_POSITION_VELOCITY: {
				/* This will acquire the last received JointPositionVelocity command from the commanding ROS node, if there is any available.
				 * If the robot can move, then it will move to this new position. */
				iiwa_msgs.JointPositionVelocity commandPositionVelocity = subscriber.getJointPositionVelocity();
				if (commandPositionVelocity != null) {
					helper.rosJointQuantityToKuka(commandPositionVelocity.getPosition(), jp);
					helper.rosJointQuantityToKuka(commandPositionVelocity.getVelocity(), jv);
					if (robot.isReadyToMove()) 
						motion.getRuntime().setDestination(jp, jv);
				}
			}
			break;
			case JOINT_VELOCITY: {
				/* This will acquire the last received JointVelocity command from the commanding ROS node, if there is any available.
				 * If the robot can move, then it will move to this new position accordingly to the given joint velocity. */
				iiwa_msgs.JointVelocity commandVelocity = subscriber.getJointVelocity();

				if (commandVelocity != null) {
					jp = motion.getRuntime().getCurrentJointDestination();
					helper.rosJointQuantityToKuka(commandVelocity.getVelocity(), jointDisplacement, loopPeriod); // compute the joint displacement over the current period.

					for(int i = 0; i < robot.getJointCount(); ++i) jp.set(i, jp.get(i) + jointDisplacement.get(i)); //add the displacement to the joint destination.
					previousTime = currentTime;

					if (robot.isReadyToMove())
						motion.getRuntime().setDestination(jp);

					currentTime = motion.getRuntime().updateWithRealtimeSystem();
					loopPeriod = (double)(currentTime - previousTime) / 1000.0; // loopPerios is stored in seconds.
				}
			}
			break;

			default:
				throw new UnsupportedControlModeException();
			}

			configureSmartServoLock.unlock();
		}
	}
}