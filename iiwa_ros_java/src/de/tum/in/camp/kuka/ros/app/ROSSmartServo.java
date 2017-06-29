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

// ROS imports
import geometry_msgs.PoseStamped;
import iiwa_msgs.ConfigureSmartServoRequest;
import iiwa_msgs.ConfigureSmartServoResponse;
import iiwa_msgs.TimeToDestinationRequest;
import iiwa_msgs.TimeToDestinationResponse;

import java.net.URI;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.ros.exception.ServiceException;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.service.ServiceResponseBuilder;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;

import de.tum.in.camp.kuka.ros.ROSBaseApplication;
import de.tum.in.camp.kuka.ros.UnsupportedControlModeException;
import de.tum.in.camp.kuka.ros.iiwaConfiguration;
import de.tum.in.camp.kuka.ros.iiwaMessageGenerator;
import de.tum.in.camp.kuka.ros.iiwaSubscriber;

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

	protected JointImpedanceControlMode gravCompControlMode; 
	private IUserKeyBar gravcompKeybar;
	private IUserKey gravCompKey;
	private IUserKeyListener gravCompKeyList;
	private boolean gravCompEnabled = false;
	private boolean gravCompSwitched = false;

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
				configureSmartServoLock.lock();
				try {
					if (controlModeHandler.isSameControlMode(motion.getMode(), req.getControlMode())) { // We can just change the parameters if the control strategy is the same.
						if (!(motion.getMode() instanceof PositionControlMode)) { // We are in PositioControlMode and the request was for the same mode, there are no parameters to change.
							motion.getRuntime().changeControlModeSettings(controlModeHandler.buildMotionControlMode(req));
						}
					} else {
						// TODO: TEST THIS!!!
						motion = controlModeHandler.switchSmartServoMotion(motion, req);
					}

					res.setSuccess(true);
					controlModeHandler.setLastSmartServoRequest(req);

					getLogger().info("Changed SmartServo configuration!");
					getLogger().info("Mode: " + motion.getMode().toString());

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
					motion.getRuntime().updateWithRealtimeSystem();
					res.setRemainingTime(motion.getRuntime().getRemainingTime());
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
						jointVelocity = req.getJointRelativeVelocity();
					}
					if (req.getJointRelativeAcceleration() >= 0) {
						jointAcceleration = req.getJointRelativeAcceleration();
					}
					if (req.getOverrideJointAcceleration() >= 0) {
						overrideJointAcceleration = req.getOverrideJointAcceleration();
					}
					// TODO: test me!!!
					iiwa_msgs.ConfigureSmartServoRequest request = null;
					motion = controlModeHandler.switchSmartServoMotion(motion, request);
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

		// Execute the subscriber node.
		nodeMainExecutor.execute(subscriber, nodeConfSubscriber);
	}

	@Override
	protected void initializeApp() {
		helper = new iiwaMessageGenerator(iiwaConfiguration.getRobotName());
		jp = new JointPosition(robot.getJointCount());
		jv = new JointPosition(robot.getJointCount());
		jointDisplacement = new JointPosition(robot.getJointCount());

		gravcompKeybar = getApplicationUI().createUserKeyBar("Gravcomp");
		gravCompKeyList = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent event) {
				configureSmartServoLock.lock();
				if (event == UserKeyEvent.FirstKeyDown) {
					gravCompEnabled = true;
					gravCompSwitched = true;
				} else if (event == UserKeyEvent.SecondKeyDown) {
					gravCompEnabled = false;
					gravCompSwitched = true;
				}
				configureSmartServoLock.unlock();
			};
		};

		gravCompKey = gravcompKeybar.addDoubleUserKey(0, gravCompKeyList, true);
		gravCompKey.setText(UserKeyAlignment.TopMiddle, "ON");
		gravCompKey.setText(UserKeyAlignment.BottomMiddle, "OFF");
		gravcompKeybar.publish();
	}

	@Override
	protected void beforeControlLoop() { 
		motion.getRuntime().activateVelocityPlanning(true);  // TODO: do this whenever appropriate
		motion.getRuntime().setGoalReachedEventHandler(handler);
		gravCompControlMode = new JointImpedanceControlMode(robot.getJointCount());

		// Initialize time stamps
		previousTime = motion.getRuntime().updateWithRealtimeSystem();
		currentTime = motion.getRuntime().updateWithRealtimeSystem();
		loopPeriod = 0.0;
	}

	// TODO: doc
	private void gravityCompensationMode() {
		if (gravCompEnabled) {
			if (gravCompSwitched) {
				gravCompSwitched = false;
				getLogger().warn("Enabling gravity compensation");
				gravCompControlMode.setStiffness(10, 10, 10, 10, 10, 0, 0);
				gravCompControlMode.setDampingForAllJoints(0.7);
				motion = controlModeHandler.switchSmartServoMotion(motion, gravCompControlMode);
			}
		} else {
			if (gravCompSwitched) {
				gravCompSwitched = false;
				getLogger().warn("Disabling gravity compensation");
				motion = controlModeHandler.switchSmartServoMotion(motion, new PositionControlMode(true));
			}
		}
	}

	private void moveRobot() {

		if (subscriber.currentCommandType != null) {
			try {
				switch (subscriber.currentCommandType) {
				case CARTESIAN_POSE: {
					/* This will acquire the last received CartesianPose command from the commanding ROS node, if there is any available.
					 * If the robot can move, then it will move to this new position. */
					PoseStamped commandPosition = subscriber.getCartesianPose();
					if (commandPosition != null) {
						Frame destinationFrame = helper.rosPoseToKukaFrame(commandPosition.getPose());
						if (robot.isReadyToMove()) {
							// try to move to the commanded cartesian pose, it something goes wrong catch the exception.
							motion.getRuntime().setDestination(destinationFrame);
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
						if (robot.isReadyToMove()) {
							motion.getRuntime().setDestination(jp);
						}
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
						if (robot.isReadyToMove()) {
							motion.getRuntime().setDestination(jp, jv);
						}
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

						for(int i = 0; i < robot.getJointCount(); ++i) { jp.set(i, jp.get(i) + jointDisplacement.get(i)); } //add the displacement to the joint destination.
						previousTime = currentTime;

						if (robot.isReadyToMove()) {
							motion.getRuntime().setDestination(jp);
						}

						currentTime = motion.getRuntime().updateWithRealtimeSystem();
						loopPeriod = (double)(currentTime - previousTime) / 1000.0; // loopPerios is stored in seconds.
					}
				}
				break;

				default:
					throw new UnsupportedControlModeException();
				}
			}
			catch (Exception e) {
				getLogger().error(e.getClass().getName() + ": " + e.getMessage());
			}
		}
	}

	@Override
	protected void controlLoop() {

		configureSmartServoLock.lock();

		gravityCompensationMode();

		if(!gravCompEnabled && !gravCompSwitched) {
			moveRobot();
		}

		configureSmartServoLock.unlock();
	}
}