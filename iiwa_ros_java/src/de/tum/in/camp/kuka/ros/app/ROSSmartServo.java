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

package de.tum.in.camp.kuka.ros.app;

import geometry_msgs.PoseStamped;
import iiwa_msgs.ConfigureSmartServoRequest;
import iiwa_msgs.ConfigureSmartServoResponse;
import iiwa_msgs.SetPathParametersLinRequest;
import iiwa_msgs.SetPathParametersLinResponse;
import iiwa_msgs.TimeToDestinationRequest;
import iiwa_msgs.TimeToDestinationResponse;

import java.net.URI;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.ros.address.BindAddress;
import org.ros.exception.ServiceException;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.service.ServiceResponseBuilder;

import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

import de.tum.in.camp.kuka.ros.Conversions;
import de.tum.in.camp.kuka.ros.Logger;
import de.tum.in.camp.kuka.ros.Motions;
import de.tum.in.camp.kuka.ros.UnsupportedControlModeException;
import de.tum.in.camp.kuka.ros.Configuration;
import de.tum.in.camp.kuka.ros.iiwaSubscriber;
import de.tum.in.camp.kuka.ros.iiwaSubscriber.CommandType;

/*
 * This application allows to command the robot using SmartServo motions.
 */
public class ROSSmartServo extends ROSBaseApplication {

	private Lock configureSmartServoLock = new ReentrantLock();

	private iiwaSubscriber subscriber; // IIWARos Subscriber.
	private NodeConfiguration nodeConfSubscriber; // Configuration of the subscriber ROS node.

	private CommandType lastCommandType = CommandType.JOINT_POSITION;
	private Motions motions;

	@Override
	protected void configureNodes(URI uri) {
		// Configuration for the Subscriber.
		nodeConfSubscriber = NodeConfiguration.newPublic(Configuration.getRobotIp());
		nodeConfSubscriber.setTimeProvider(configuration.getTimeProvider());
		nodeConfSubscriber.setNodeName(Configuration.getRobotName() + "/iiwa_subscriber");
		nodeConfSubscriber.setMasterUri(uri);
		nodeConfSubscriber.setTcpRosBindAddress(BindAddress.newPublic(30004));
		nodeConfSubscriber.setXmlRpcBindAddress(BindAddress.newPublic(30005));
	}

	@Override
	protected void addNodesToExecutor(NodeMainExecutor nodeMainExecutor) {
		subscriber = new iiwaSubscriber(robot, Configuration.getRobotName(), configuration);

		// Configure the callback for the SmartServo service inside the subscriber class.
		subscriber.setConfigureSmartServoCallback(new ServiceResponseBuilder<iiwa_msgs.ConfigureSmartServoRequest, iiwa_msgs.ConfigureSmartServoResponse>() {
			@Override
			public void build(ConfigureSmartServoRequest req, ConfigureSmartServoResponse res) throws ServiceException {
				configureSmartServoLock.lock();
				try {
					// TODO: reduce code duplication
					if (lastCommandType == CommandType.CARTESIAN_POSE_LIN) {
						if (controlModeHandler.isSameControlMode(linearMotion.getMode(), req.getControlMode())) { // We can just change the parameters if the control strategy is the same.
							if (!(linearMotion.getMode() instanceof PositionControlMode)) { // We are in PositioControlMode and the request was for the same mode, there are no parameters to change.
								linearMotion.getRuntime().changeControlModeSettings(controlModeHandler.buildMotionControlMode(req));
							}
						} else {
							linearMotion = controlModeHandler.switchSmartServoMotion(linearMotion, req);
						}
					}
					else {
						if (controlModeHandler.isSameControlMode(motion.getMode(), req.getControlMode())) { // We can just change the parameters if the control strategy is the same.
							if (!(motion.getMode() instanceof PositionControlMode)) { // We are in PositioControlMode and the request was for the same mode, there are no parameters to change.
								motion.getRuntime().changeControlModeSettings(controlModeHandler.buildMotionControlMode(req));
							}
						} else {
							motion = controlModeHandler.switchSmartServoMotion(motion, req);
						}
					}

					res.setSuccess(true);
					controlModeHandler.setLastSmartServoRequest(req);
				} catch (Exception e) {
					res.setSuccess(false);
					if (e.getMessage() != null) {
						res.setError(e.getClass().getName() + ": " + e.getMessage());
					} else {
						res.setError("because I hate you :)");
					}
					return;
				}
				finally {
					configureSmartServoLock.unlock();
				}
			}
		});

		// TODO: doc
		subscriber.setTimeToDestinationCallback(new ServiceResponseBuilder<iiwa_msgs.TimeToDestinationRequest, iiwa_msgs.TimeToDestinationResponse>() {

			@Override
			public void build(TimeToDestinationRequest req, TimeToDestinationResponse res) throws ServiceException {
				try {
					if (lastCommandType == CommandType.CARTESIAN_POSE_LIN) { 
						linearMotion.getRuntime().updateWithRealtimeSystem();
						res.setRemainingTime(linearMotion.getRuntime().getRemainingTime());
					}
					else {
						motion.getRuntime().updateWithRealtimeSystem();
						res.setRemainingTime(motion.getRuntime().getRemainingTime());
					}
				}
				catch(Exception e) {
					// An exception should be thrown only if a motion/runtime is not available.
					res.setRemainingTime(-999); 
				}
			}
		});

		// TODO: doc
		subscriber.setPathParametersCallback(new ServiceResponseBuilder<iiwa_msgs.SetPathParametersRequest, iiwa_msgs.SetPathParametersResponse>() {
			@Override
			public void build(iiwa_msgs.SetPathParametersRequest req, iiwa_msgs.SetPathParametersResponse res) throws ServiceException {
				configureSmartServoLock.lock();
				try {
					if (req.getJointRelativeVelocity() >= 0) {
						controlModeHandler.jointVelocity = req.getJointRelativeVelocity();
					}
					if (req.getJointRelativeAcceleration() >= 0) {
						controlModeHandler.jointAcceleration = req.getJointRelativeAcceleration();
					}
					if (req.getOverrideJointAcceleration() >= 0) {
						controlModeHandler.overrideJointAcceleration = req.getOverrideJointAcceleration();
					}
					if (lastCommandType != CommandType.CARTESIAN_POSE_LIN) {
						iiwa_msgs.ConfigureSmartServoRequest request = null;
						motion = controlModeHandler.switchSmartServoMotion(motion, request);
					}

					res.setSuccess(true);
				}
				catch(Exception e) {
					res.setError(e.getClass().getName() + ": " + e.getMessage());
					res.setSuccess(false);
				}
				finally {
					configureSmartServoLock.unlock();
				}
			}
		});

		subscriber.setPathParametersLinCallback(new ServiceResponseBuilder<iiwa_msgs.SetPathParametersLinRequest, iiwa_msgs.SetPathParametersLinResponse>() {
			@Override
			public void build(SetPathParametersLinRequest req, SetPathParametersLinResponse res) throws ServiceException {
				configureSmartServoLock.lock();
				try {
					if (isVector3GreaterThan(req.getMaxCartesianVelocity().getLinear(), 0)) { // TODO: this just works for linear velocity atm
						controlModeHandler.maxTranslationlVelocity = Conversions.rosVectorToArray(req.getMaxCartesianVelocity().getLinear());
					}
					if (lastCommandType == CommandType.CARTESIAN_POSE_LIN) {
						iiwa_msgs.ConfigureSmartServoRequest request = null;
						linearMotion = controlModeHandler.switchSmartServoMotion(linearMotion, request);
					}
					res.setSuccess(true);
				}
				catch (Exception e) {
					res.setError(e.getClass().getName() + ": " + e.getMessage());
					res.setSuccess(false);
				}
				finally {
					configureSmartServoLock.unlock();
				}
			}
		});

		// Execute the subscriber node.
		nodeMainExecutor.execute(subscriber, nodeConfSubscriber);
	}

	// TODO move this somewhere else.
	private boolean isTwistGreaterThan(geometry_msgs.Twist twist, double value) {
		return (twist.getLinear().getX() > value && 
				twist.getLinear().getY() > value &&
				twist.getLinear().getZ() > value &&
				twist.getAngular().getX() > value &&
				twist.getAngular().getY() > value &&
				twist.getAngular().getZ() > value);
	}

	private boolean isVector3GreaterThan(geometry_msgs.Vector3 vector, double value) {
		return (vector.getX() > value &&
				vector.getY() > value &&
				vector.getZ() > value);
	}

	@Override
	protected void initializeApp() {}

	@Override
	protected void beforeControlLoop() { 
		motions = new Motions(robot, motion);
	}

	/**
	 * TODO: doc, take something from
	 * This will acquire the last received CartesianPose command from the commanding ROS node, if there is any available.
	 * If the robot can move, then it will move to this new position.
	 */
	private void moveRobot() {
		if (subscriber.currentCommandType != null) {
			try {
				switch (subscriber.currentCommandType) {
				case CARTESIAN_POSE: {
					if (lastCommandType == CommandType.CARTESIAN_POSE_LIN) { 
						motion = controlModeHandler.switchToSmartServo(motion, linearMotion);
					}
					PoseStamped commandPosition = subscriber.getCartesianPose();
					commandPosition = subscriber.transformPose(commandPosition, robotBaseFrameID);
					if (commandPosition != null) {
						motions.cartesianPositionMotion(motion, commandPosition);
					}
					break;
				}
				case CARTESIAN_POSE_LIN: {
					if (lastCommandType != CommandType.CARTESIAN_POSE_LIN) {
						linearMotion = controlModeHandler.switchToSmartServoLIN(motion, linearMotion);
					}
					PoseStamped commandPosition = subscriber.getCartesianPoseLin();
					commandPosition = subscriber.transformPose(commandPosition, robotBaseFrameID);
					if (commandPosition != null) {
						motions.cartesianPositionLinMotion(linearMotion, commandPosition);
					}
					break;
				}
				case CARTESIAN_VELOCITY: {
					geometry_msgs.TwistStamped commandVelocity = subscriber.getCartesianVelocity();
					motions.cartesianVelocityMotion(motion, commandVelocity, toolFrame);
					break;
				}
				case JOINT_POSITION: {
					if (lastCommandType == CommandType.CARTESIAN_POSE_LIN) { 
						motion = controlModeHandler.switchToSmartServo(motion, linearMotion);
					}
					iiwa_msgs.JointPosition commandPosition = subscriber.getJointPosition();
					motions.jointPositionMotion(motion, commandPosition);
					break;
				}
				case JOINT_POSITION_VELOCITY: {
					if (lastCommandType == CommandType.CARTESIAN_POSE_LIN) { 
						motion = controlModeHandler.switchToSmartServo(motion, linearMotion);
					}
					iiwa_msgs.JointPositionVelocity commandPositionVelocity = subscriber.getJointPositionVelocity();
					motions.jointPositionVelocityMotion(motion, commandPositionVelocity);
					break;
				}
				case JOINT_VELOCITY: {
					if (lastCommandType == CommandType.CARTESIAN_POSE_LIN) { 
						motion = controlModeHandler.switchToSmartServo(motion, linearMotion);
					}
					/* This will acquire the last received JointVelocity command from the commanding ROS node, if there is any available.
					 * If the robot can move, then it will move to this new position accordingly to the given joint velocity. */
					motion.getRuntime().activateVelocityPlanning(true);
					motion.setSpeedTimeoutAfterGoalReach(0.1);
					iiwa_msgs.JointVelocity commandVelocity = subscriber.getJointVelocity();
					motions.jointVelocityMotion(motion, commandVelocity);
					break;
				}
				default:
					throw new UnsupportedControlModeException();
				}
			}
			catch (Exception e) {
				e.printStackTrace();
				Logger.error(e.getClass().getName() + ": " + e.getMessage());
				subscriber.currentCommandType = null;
			}
		}
		lastCommandType = subscriber.currentCommandType;
	}

	@Override
	protected void controlLoop() {
		configureSmartServoLock.lock();
		moveRobot();
		configureSmartServoLock.unlock();
	}	
}