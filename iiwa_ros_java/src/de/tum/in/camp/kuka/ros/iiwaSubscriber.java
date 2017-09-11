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

	// ROSJava Subscribers for iiwa_msgs
	// Cartesian Message Subscribers
	private Subscriber<geometry_msgs.PoseStamped> cartesianPoseSubscriber;
	private Subscriber<iiwa_msgs.JointPosition> jointPositionSubscriber;
	private Subscriber<iiwa_msgs.JointPositionVelocity> jointPositionVelocitySubscriber;
	private Subscriber<iiwa_msgs.JointVelocity> jointVelocitySubscriber;


	// Object to easily build iiwa_msgs from the current robot state
	private iiwaMessageGenerator helper;

	// Local iiwa_msgs to store received messages 
	private geometry_msgs.PoseStamped cp;
	private iiwa_msgs.JointPosition jp;
	private iiwa_msgs.JointPositionVelocity jpv;
	private iiwa_msgs.JointVelocity jv;

	private Boolean new_jp = new Boolean("false");
	private Boolean new_cp = new Boolean("false");
	private Boolean new_jpv = new Boolean("false");
	private Boolean new_jv = new Boolean("false");

	// Current control strategy
	CommandType currentCommandType = null;

	// Name to use to build the name of the ROS topics
	private String iiwaName = "iiwa";

	/**
	 * Constructs a series of ROS subscribers for messages defined by the iiwa_msgs ROS package. <p>
	 * While no messages are received, the initial values are set to the state of the robot at the moment of this call.
	 * For Cartesian messages, the initial values will refer to the frame of the robot's Flange.
	 * @param robot: an iiwa Robot, its current state is used to set up initial values for the messages.
	 * @param robotName: name of the robot, it will be used for the topic names with this format : <robot name>/command/<iiwa message type>
	 */
	public iiwaSubscriber(LBR robot, String robotName) {
		this(robot, robot.getFlange(), robotName);
	}

	/**
	 * Constructs a series of ROS subscribers for messages defined by the iiwa_msgs ROS package.<p>
	 * While no messages are received, the initial values are set to the state of the robot at the moment of this call.<br>
	 * For Cartesian messages, the initial values will refer to the given frame.
	 * @param robot: an iiwa Robot, its current state is used to set up initial values for the messages.
	 * @param frame: reference frame to set the values of the Cartesian position.
	 * @param robotName : name of the robot, it will be used for the topic names with this format : <robot name>/command/<iiwa message type>
	 */
	public iiwaSubscriber(LBR robot, ObjectFrame frame, String robotName) {
		iiwaName = robotName;
		helper = new iiwaMessageGenerator(iiwaName);

		cp = helper.buildMessage(geometry_msgs.PoseStamped._TYPE);
		jp = helper.buildMessage(iiwa_msgs.JointPosition._TYPE);
		jpv = helper.buildMessage(iiwa_msgs.JointPositionVelocity._TYPE);
		jv = helper.buildMessage(iiwa_msgs.JointVelocity._TYPE);

		helper.getCurrentCartesianPose(cp, robot, frame);
		helper.getCurrentJointPosition(jp, robot);
		helper.getCurrentJointPositionVelocity(jpv, robot);
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
	 * Get the last received PoseStamped message. Returns null if no new message is available.<p>
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
		}	}

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
		jointPositionSubscriber = connectedNode.newSubscriber(iiwaName + "/command/JointPosition", iiwa_msgs.JointPosition._TYPE);
		jointPositionVelocitySubscriber = connectedNode.newSubscriber(iiwaName + "/command/JointPositionVelocity", iiwa_msgs.JointPositionVelocity._TYPE);
		jointVelocitySubscriber = connectedNode.newSubscriber(iiwaName + "/command/JointVelocity", iiwa_msgs.JointVelocity._TYPE);


		// Subscribers' callbacks
		cartesianPoseSubscriber.addMessageListener(new MessageListener<geometry_msgs.PoseStamped>() {
			@Override
			public void onNewMessage(geometry_msgs.PoseStamped position) {
				synchronized (new_cp) {
					cp = position;
					currentCommandType = CommandType.CARTESIAN_POSE;
					new_cp = true;
				}
			}
		});

		jointPositionSubscriber.addMessageListener(new MessageListener<iiwa_msgs.JointPosition>() {
			@Override
			public void onNewMessage(iiwa_msgs.JointPosition position){
				synchronized (new_jp) {
					jp = position;
					currentCommandType = CommandType.JOINT_POSITION;
					new_jp = true;
				}
			}
		});

		jointPositionVelocitySubscriber.addMessageListener(new MessageListener<iiwa_msgs.JointPositionVelocity>() {
			@Override
			public void onNewMessage(iiwa_msgs.JointPositionVelocity positionVelocity){
				synchronized (new_jpv) {
					jpv = positionVelocity;
					currentCommandType = CommandType.JOINT_POSITION_VELOCITY;
					new_jpv = true;
				}
			}
		});

		jointVelocitySubscriber.addMessageListener(new MessageListener<iiwa_msgs.JointVelocity>() {
			@Override
			public void onNewMessage(iiwa_msgs.JointVelocity velocity){
				synchronized(new_jv) {
					jv = velocity;
					currentCommandType = CommandType.JOINT_VELOCITY;
					new_jv = true;
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
	}
}
