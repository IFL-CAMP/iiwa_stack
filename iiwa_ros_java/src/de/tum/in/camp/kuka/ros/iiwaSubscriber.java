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

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.Subscriber;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;

/**
 * This class provides ROS subscribers for ROS messages defined in the iiwa_msgs ROS package.
 * It allows to received messages of that type from ROS topics named with the following convention :
 * <robot name>/command/<iiwa message type> (e.g. MyIIWA/command/JointPosition)
 */
public class iiwaSubscriber extends AbstractNodeMain {

	public enum CommandType {
		CARTESIAN_POSE,
		CARTESIAN_POSE_LIN,
		CARTESIAN_VELOCITY,
		JOINT_POSITION,
		JOINT_POSITION_VELOCITY,
		JOINT_VELOCITY
	}

	private ConnectedNode node = null;

	// Service for reconfiguring control mode
	@SuppressWarnings("unused")
	private ServiceServer<iiwa_msgs.ConfigureSmartServoRequest, iiwa_msgs.ConfigureSmartServoResponse> configureSmartServoServer = null;
	private ServiceResponseBuilder<iiwa_msgs.ConfigureSmartServoRequest, iiwa_msgs.ConfigureSmartServoResponse> configureSmartServoCallback = null;

	@SuppressWarnings("unused")
	private ServiceServer<iiwa_msgs.TimeToDestinationRequest, iiwa_msgs.TimeToDestinationResponse> timeToDestinationServer = null;
	private ServiceResponseBuilder<iiwa_msgs.TimeToDestinationRequest, iiwa_msgs.TimeToDestinationResponse> timeToDestinationCallback = null;

	@SuppressWarnings("unused")
	private ServiceServer<iiwa_msgs.SetPathParametersRequest, iiwa_msgs.SetPathParametersResponse> setPathParametersServer = null;
	private ServiceResponseBuilder<iiwa_msgs.SetPathParametersRequest, iiwa_msgs.SetPathParametersResponse> setPathParametersCallback = null;
	
	@SuppressWarnings("unused")
	private ServiceServer<iiwa_msgs.SetPathParametersLinRequest, iiwa_msgs.SetPathParametersLinResponse> setPathParametersLinServer = null;
	private ServiceResponseBuilder<iiwa_msgs.SetPathParametersLinRequest, iiwa_msgs.SetPathParametersLinResponse> setPathParametersLinCallback = null;

	// ROSJava Subscribers for iiwa_msgs
	private Subscriber<geometry_msgs.PoseStamped> cartesianPoseSubscriber;
	private Subscriber<geometry_msgs.PoseStamped> cartesianPoseLinSubscriber;
	private Subscriber<geometry_msgs.TwistStamped> cartesianVelocitySubscriber;
	private Subscriber<iiwa_msgs.JointPosition> jointPositionSubscriber;
	private Subscriber<iiwa_msgs.JointPositionVelocity> jointPositionVelocitySubscriber;
	private Subscriber<iiwa_msgs.JointVelocity> jointVelocitySubscriber;


	// Object to easily build iiwa_msgs from the current robot state
	private MessageGenerator helper;

	// Local iiwa_msgs to store received messages 
	private geometry_msgs.PoseStamped cp;
	private geometry_msgs.PoseStamped cp_lin;
	private geometry_msgs.TwistStamped cv;
	private iiwa_msgs.JointPosition jp;
	private iiwa_msgs.JointPositionVelocity jpv;
	private iiwa_msgs.JointVelocity jv;

	private Boolean new_jp = new Boolean("false");
	private Boolean new_cp = new Boolean("false");
	private Boolean new_cp_lin = new Boolean("false");
	private Boolean new_jpv = new Boolean("false");

	// Current control strategy
	public CommandType currentCommandType = null;

	// Name to use to build the name of the ROS topics
	private String iiwaName = "iiwa";

	/**
	 * Constructs a series of ROS subscribers for messages defined by the iiwa_msgs ROS package. <p>
	 * While no messages are received, the initial values are set to the state of the robot at the moment of this call.
	 * For Cartesian messages, the initial values will refer to the frame of the robot's Flange.
	 * @param robot: an iiwa Robot, its current state is used to set up initial values for the messages.
	 * @param robotName: name of the robot, it will be used for the topic names with this format : <robot name>/command/<iiwa message type>
	 */
	public iiwaSubscriber(LBR robot, String robotName, Configuration configuration) {
		this(robot, robot.getFlange(), robotName, configuration);
	}

	/**
	 * Constructs a series of ROS subscribers for messages defined by the iiwa_msgs ROS package.<p>
	 * While no messages are received, the initial values are set to the state of the robot at the moment of this call.<br>
	 * For Cartesian messages, the initial values will refer to the given frame.
	 * @param robot: an iiwa Robot, its current state is used to set up initial values for the messages.
	 * @param frame: reference frame to set the values of the Cartesian position.
	 * @param robotName : name of the robot, it will be used for the topic names with this format : <robot name>/command/<iiwa message type>
	 */
	public iiwaSubscriber(LBR robot, ObjectFrame frame, String robotName, Configuration configuration) {
		iiwaName = robotName;
		helper = new MessageGenerator(iiwaName, configuration);

		cp = helper.buildMessage(geometry_msgs.PoseStamped._TYPE);
		cp_lin = helper.buildMessage(geometry_msgs.PoseStamped._TYPE);
		cv = helper.buildMessage(geometry_msgs.TwistStamped._TYPE);
		jp = helper.buildMessage(iiwa_msgs.JointPosition._TYPE);
		jpv = helper.buildMessage(iiwa_msgs.JointPositionVelocity._TYPE);
		jv = helper.buildMessage(iiwa_msgs.JointVelocity._TYPE);
	}

	/**
	 * Add a callback to the SmartServo service
	 */
	public void setConfigureSmartServoCallback(ServiceResponseBuilder<iiwa_msgs.ConfigureSmartServoRequest, iiwa_msgs.ConfigureSmartServoResponse> callback) {
		configureSmartServoCallback = callback;
	}

	/**
	 * Add a callback to the TimeToDestination service
	 */
	public void setTimeToDestinationCallback(ServiceResponseBuilder<iiwa_msgs.TimeToDestinationRequest, iiwa_msgs.TimeToDestinationResponse> callback) {
		timeToDestinationCallback = callback;
	}

	/**
	 * Add a callback to the SetPathParameters service
	 */
	public void setPathParametersCallback(ServiceResponseBuilder<iiwa_msgs.SetPathParametersRequest, iiwa_msgs.SetPathParametersResponse> callback) {
		setPathParametersCallback = callback;
	}
	
	/**
	 * Add a callback to the SetPathParametersLin service
	 */
	public void setPathParametersLinCallback(ServiceResponseBuilder<iiwa_msgs.SetPathParametersLinRequest, iiwa_msgs.SetPathParametersLinResponse> callback) {
		setPathParametersLinCallback = callback;
	}

	/**
	 * Returns the last PoseStamped message received from the /command/CartesianPose topic. Returns null if no new message is available.<p>
	 * @return the received PoseStamped message.
	 */
	public geometry_msgs.PoseStamped getCartesianPose() {
		synchronized(new_cp) {
			if (new_cp) {
				new_cp = false;
				return cp;
			} else {
				
				return null;
			}
		}	
	}

	/**
	 * Returns the last PoseStamped message received from the /command/CartesianPoseLin topic. Returns null if no new message is available.<p>
	 * @return the received PoseStamped message.
	 */
	public geometry_msgs.PoseStamped getCartesianPoseLin() {
		synchronized(new_cp_lin) {
			if (new_cp_lin) {
				new_cp_lin = false;
				return cp_lin;
			} else {
				return null;
			}
		}	
	}

	/**
	 * TODO
	 * @return the received PoseStamped message.
	 */
	public geometry_msgs.TwistStamped getCartesianVelocity() {
		return cv;
	}

	/**
	 * Returns the last received Joint Position message. Returns null if no new message is available.<p>
	 * @return the received Joint Position message.
	 */
	public iiwa_msgs.JointPosition getJointPosition() {
		synchronized(new_jp) {
			if (new_jp) {
				new_jp = false;
				return jp;
			} else {
				return null;
			}
		}
	}

	/**
	 * Returns the last received Joint Position-Velocity message. Returns null if no new message is available.<p>
	 * @return the received Joint Position-Velocity message.
	 */
	public iiwa_msgs.JointPositionVelocity getJointPositionVelocity() {
		synchronized(new_jpv) {
			if (new_jpv) {
				new_jpv = false;
				return jpv;
			} else {
				return null;
			}
		}	
	}

	/**
	 * Returns the last received Joint Velocity message. Returns null if no new message is available.<p>
	 * @return the received Joint Velocity message.
	 */
	public iiwa_msgs.JointVelocity getJointVelocity() {
		return jv;
	}

	/**
	 * @see org.ros.node.NodeMain#getDefaultNodeName()
	 */
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(iiwaName + "/subscriber");
	}

	/**
	 * This method is called when the <i>execute</i> method from a <i>nodeMainExecutor</i> is called.<br>
	 * Do <b>NOT</b> manually call this. <p> 
	 * @see org.ros.node.AbstractNodeMain#onStart(org.ros.node.ConnectedNode)
	 */
	@Override
	public void onStart(ConnectedNode connectedNode) {

		node = connectedNode;

		// Creating the subscribers
		cartesianPoseSubscriber = connectedNode.newSubscriber(iiwaName + "/command/CartesianPose", geometry_msgs.PoseStamped._TYPE);
		cartesianPoseLinSubscriber = connectedNode.newSubscriber(iiwaName + "/command/CartesianPoseLin", geometry_msgs.PoseStamped._TYPE);
		cartesianVelocitySubscriber = connectedNode.newSubscriber(iiwaName + "/command/CartesianVelocity", geometry_msgs.TwistStamped._TYPE);
		jointPositionSubscriber = connectedNode.newSubscriber(iiwaName + "/command/JointPosition", iiwa_msgs.JointPosition._TYPE);
		jointPositionVelocitySubscriber = connectedNode.newSubscriber(iiwaName + "/command/JointPositionVelocity", iiwa_msgs.JointPositionVelocity._TYPE);
		jointVelocitySubscriber = connectedNode.newSubscriber(iiwaName + "/command/JointVelocity", iiwa_msgs.JointVelocity._TYPE);

		// Subscribers' callbacks
		cartesianPoseSubscriber.addMessageListener(new MessageListener<geometry_msgs.PoseStamped>() {
			@Override
			public void onNewMessage(geometry_msgs.PoseStamped position) {
				// accept only incrementing sequence numbers (unless the sender is forgetting to set it)
				if ((position.getHeader().getSeq() == 0 && cp.getHeader().getSeq() == 0) 
						|| position.getHeader().getSeq() > cp.getHeader().getSeq()) {
					synchronized (new_cp) {
						cp = position;
						currentCommandType = CommandType.CARTESIAN_POSE;
						new_cp = true;
					}
				}
			}
		});

		cartesianVelocitySubscriber.addMessageListener(new MessageListener<geometry_msgs.TwistStamped>() {
			@Override
			public void onNewMessage(geometry_msgs.TwistStamped velocity) {
				// accept only incrementing sequence numbers (unless the sender is forgetting to set it)
				if ((velocity.getHeader().getSeq() == 0 && cv.getHeader().getSeq() == 0) 
						|| velocity.getHeader().getSeq() > cv.getHeader().getSeq()) {
					cv = velocity;
					currentCommandType = CommandType.CARTESIAN_VELOCITY;
				}
			}
		});

		cartesianPoseLinSubscriber.addMessageListener(new MessageListener<geometry_msgs.PoseStamped>() {
			@Override
			public void onNewMessage(geometry_msgs.PoseStamped position) {
				synchronized (new_cp_lin) {
					cp_lin = position;
					currentCommandType = CommandType.CARTESIAN_POSE_LIN;
					new_cp_lin = true;
				}
			}
		});

		jointPositionSubscriber.addMessageListener(new MessageListener<iiwa_msgs.JointPosition>() {
			@Override
			public void onNewMessage(iiwa_msgs.JointPosition position) {
				// accept only incrementing sequence numbers (unless the sender is forgetting to set it)
				if ((position.getHeader().getSeq() == 0 && jp.getHeader().getSeq() == 0) 
						|| position.getHeader().getSeq() > jp.getHeader().getSeq()) {
					synchronized (new_jp) {
						jp = position;
						currentCommandType = CommandType.JOINT_POSITION;
						new_jp = true;
					}
				}
			}
		});

		jointPositionVelocitySubscriber.addMessageListener(new MessageListener<iiwa_msgs.JointPositionVelocity>() {
			@Override
			public void onNewMessage(iiwa_msgs.JointPositionVelocity positionVelocity) {
				// accept only incrementing sequence numbers (unless the sender is forgetting to set it)
				if ((positionVelocity.getHeader().getSeq() == 0 && jpv.getHeader().getSeq() == 0) 
						|| positionVelocity.getHeader().getSeq() > jpv.getHeader().getSeq()) {
					synchronized (new_jpv) {
						jpv = positionVelocity;
						currentCommandType = CommandType.JOINT_POSITION_VELOCITY;
						new_jpv = true;
					}
				}
			}
		});

		jointVelocitySubscriber.addMessageListener(new MessageListener<iiwa_msgs.JointVelocity>() {
			@Override
			public void onNewMessage(iiwa_msgs.JointVelocity velocity) {
				// accept only incrementing sequence numbers (unless the sender is forgetting to set it)
				if ((velocity.getHeader().getSeq() == 0 && jv.getHeader().getSeq() == 0) 
						|| velocity.getHeader().getSeq() > jv.getHeader().getSeq()) {
					jv = velocity;
					currentCommandType = CommandType.JOINT_VELOCITY;
				}
			}
		});

		// Creating SmartServo service if a callback has been defined.
		if (configureSmartServoCallback != null) {
			configureSmartServoServer = node.newServiceServer(
					iiwaName + "/configuration/configureSmartServo", 
					"iiwa_msgs/ConfigureSmartServo", 
					configureSmartServoCallback);
		}

		// Creating TimeToDestination service if a callback has been defined.
		if (timeToDestinationCallback != null) {
			timeToDestinationServer = node.newServiceServer(
					iiwaName + "/state/timeToDestination", 
					"iiwa_msgs/TimeToDestination", 
					timeToDestinationCallback);
		}

		// Creating TimeToDestination service if a callback has been defined.
		if (setPathParametersCallback != null) {
			setPathParametersServer = node.newServiceServer(
					iiwaName + "/configuration/pathParameters", 
					"iiwa_msgs/SetPathParameters",
					setPathParametersCallback);
		}
		
		// Creating TimeToDestination service if a callback has been defined.
		if (setPathParametersLinCallback != null) {
			setPathParametersLinServer = node.newServiceServer(
					iiwaName + "/configuration/pathParametersLin", 
					"iiwa_msgs/SetPathParametersLin",
					setPathParametersLinCallback);
		}
	}
}
