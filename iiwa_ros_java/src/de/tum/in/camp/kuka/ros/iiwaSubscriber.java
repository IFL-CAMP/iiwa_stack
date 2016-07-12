 /**  
 * Copyright (C) 2016 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
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
import geometry_msgs.WrenchStamped;
import iiwa_msgs.CartesianVelocity;
import iiwa_msgs.ConfigureSmartServoRequest;
import iiwa_msgs.ConfigureSmartServoResponse;
import iiwa_msgs.JointPosition;
import iiwa_msgs.JointPositionVelocity;
import iiwa_msgs.JointStiffness;
import iiwa_msgs.JointTorque;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.Subscriber;

// KUKA imports
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;


/**
 * This class provides ROS subscribers for ROS messages defined in the iiwa_msgs ROS package.
 * It allows to received messages of that type from ROS topics named with the following convention :
 * <robot name>/command/<iiwa message type> (e.g. MyIIWA/command/CartesianPosition)
 */
public class iiwaSubscriber extends AbstractNodeMain {
	
	public enum CommandType {
		CARTESIAN_POSE,
		CARTESIAN_VELOCITY,
		JOINT_POSITION,
		JOINT_VELOCITY,
		JOINT_POSITION_VELOCITY
	}
	
	private ConnectedNode node = null;
	
	// Service for reconfiguring control mode
	@SuppressWarnings("unused")
	private ServiceServer<iiwa_msgs.ConfigureSmartServoRequest, iiwa_msgs.ConfigureSmartServoResponse> configureSmartServoServer = null;
	private ServiceResponseBuilder<ConfigureSmartServoRequest, ConfigureSmartServoResponse> configureSmartServoCallback = null;

	// ROSJava Subscribers for iiwa_msgs
	// Cartesian Message Subscribers
	private Subscriber<geometry_msgs.PoseStamped> cartesianPoseSubscriber;
//	private Subscriber<iiwa_msgs.CartesianVelocity> cartesianVelocitySubscriber;
	private Subscriber<geometry_msgs.WrenchStamped> cartesianWrenchSubscriber;
	// Joint Message Publishers
	private Subscriber<iiwa_msgs.JointPosition> jointPositionSubscriber;
	private Subscriber<iiwa_msgs.JointPositionVelocity> jointPositionVelocitySubscriber;
	private Subscriber<iiwa_msgs.JointStiffness> jointStiffnessSubscriber;
	private Subscriber<iiwa_msgs.JointTorque> jointTorqueSubscriber;
//	private Subscriber<iiwa_msgs.JointVelocity> jointVelocitySubscriber;

	// Object to easily build iiwa_msgs from the current robot state
	private iiwaMessageGenerator helper = new iiwaMessageGenerator();
		
	// Local iiwa_msgs to store received messages 
	// Cartesian Messages
	private geometry_msgs.PoseStamped cp = helper.buildMessage(PoseStamped._TYPE);
	private iiwa_msgs.CartesianVelocity cv = helper.buildMessage(CartesianVelocity._TYPE);
	private geometry_msgs.WrenchStamped cw = helper.buildMessage(WrenchStamped._TYPE);
	// Joint Messages
	private iiwa_msgs.JointPosition jp = helper.buildMessage(JointPosition._TYPE);
	private iiwa_msgs.JointPositionVelocity jpv = helper.buildMessage(JointPositionVelocity._TYPE);
	private iiwa_msgs.JointStiffness js = helper.buildMessage(JointStiffness._TYPE);
	private iiwa_msgs.JointTorque jt = helper.buildMessage(JointTorque._TYPE);
//	private iiwa_msgs.JointVelocity jv;
	
	// current control strategy TODO: set this with a service; for now it is the last message arrived
	CommandType currentCommandType = CommandType.JOINT_POSITION;

	

	// Name to use to build the name of the ROS topics
	private String iiwaName = "iiwa";

	/**
	 * Constructs a series of ROS subscribers for messages defined by the iiwa_msgs ROS package. <p>
	 * While no messages are received, the initial values are set to the state of the robot at the moment of this call.
	 * Except for Velocity messages, those will be initialized to zero.<br>
	 * For Cartesian messages, the initial values will refer to the frame of the robot's Flange.
	 * @param robot : an iiwa Robot, its current state is used to set up initial values for the messages.
	 */
	public iiwaSubscriber(LBR robot, String robotName) {
		this(robot, robot.getFlange(), robotName);
	}

	/**
	 * Constructs a series of ROS subscribers for messages defined by the iiwa_msgs ROS package.<p>
	 * While no messages are received, the initial values are set to the state of the robot at the moment of this call.
	 * Except for Velocity messages, those will be initialized to zero.<br>
	 * For Cartesian messages, the initial values will refer to the given frame.
	 * @param robot : an iiwa Robot, its current state is used to set up initial values for the messages.
	 * @param frame : reference frame to use to set up initial values for Cartesian messages.
	 */
	public iiwaSubscriber(LBR robot, ObjectFrame frame, String robotName) {
		helper.getCurrentCartesianPose(cp, robot, frame);
//		cv = helper.buildCartesianVelocity(robot, frame);
		helper.getCurrentCartesianWrench(cw, robot, frame);

		helper.getCurrentJointPosition(jp, robot);
		helper.getCurrentJointStiffness(js, robot, null);
		helper.getCurrentJointTorque(jt, robot);
//		jv = helper.buildJointVelocity(robot);
		
		iiwaName = robotName;
	}
	
	public void setConfigureSmartServoCallback(ServiceResponseBuilder<ConfigureSmartServoRequest, ConfigureSmartServoResponse> callback) {
		configureSmartServoCallback = callback;
	}
	
	public geometry_msgs.PoseStamped getCartesianPose() {
		return cp;
	}

	/**
	 * Returns the last received Cartesian Velocity message. <p>
	 * If no messages have been received yet, it returns a message filled with initial values created in the class constructor.
	 * @return the received Cartesian Velocity message.
	 */
	public iiwa_msgs.CartesianVelocity getCartesianVelocity() {
		return cv;
	}

	/**
	 * Returns the last received Cartesian Wrench message. <p>
	 * If no messages have been received yet, it returns a message filled with initial values created in the class constructor.
	 * @return the received Cartesian Wrench message.
	 */
	public geometry_msgs.WrenchStamped getCartesianWrench() {
		return cw;
	}

	/**
	 * Returns the last received Joint Position message. <p>
	 * If no messages have been received yet, it returns a message filled with initial values created in the class constructor.
	 * @return the received Joint Position message.
	 */
	public iiwa_msgs.JointPosition getJointPosition() {
		return jp;
	}
	
	/**
	 * Returns the last received Joint Position-Velocity message. <p>
	 * If no messages have been received yet, it returns a message filled with initial values created in the class constructor.
	 * @return the received Joint Position-Velocity message.
	 */
	public iiwa_msgs.JointPositionVelocity getJointPositionVelocity() {
		return jpv;
	}
	
	/**
	 * Returns the last received Joint Stiffness message. <p>
	 * If no messages have been received yet, it returns a message filled with initial values created in the class constructor.
	 * @return the received Joint Stiffness message.
	 */
	public iiwa_msgs.JointStiffness getJointStiffness() {
		return js;
	}
	
	/**
	 * Returns the last received Joint Torque message. <p>
	 * If no messages have been received yet, it returns a message filled with initial values created in the class constructor.
	 * @return the received Joint Torque message.
	 */
	public iiwa_msgs.JointTorque getJointTorque() {
		return jt;
	}
	/**
	 * Returns the last received Joint Velocity message. <p>
	 * If no messages have been received yet, it returns a message filled with initial values created in the class constructor.
	 * @return the received Joint Velocity message.
	 */
//	public iiwa_msgs.JointVelocity getJointVelocity() {
//		return jv;
//	}

	/**
	 * Set the name to use to compose the ROS topics' names for the subscribers. <p>
	 * e.g. setIIWAName("dummy"), the topics names will be "dummy/command/...". <br>
	 * The creation of the nodes is performed when the <i>execute</i> method from a <i>nodeMainExecutor</i> is called.
	 * @param newName : the new name to use for ROS topics.
	 */
	public void setIIWAName(String newName) {
		iiwaName = newName;
	}

	/**
	 * Returns the current name used to compose the ROS topics' names for the subscribers. <p>
	 * e.g. returning "dummy" means that the topics' names will be "dummy/command/...". <br>
	 * The creation of the nodes is performed when the <i>execute</i> method from a <i>nodeMainExecutor</i> is called.
	 * @return the current name to use for ROS topics.
	 */
	public String getIIWAName() {
		return iiwaName;
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

		cartesianPoseSubscriber = connectedNode.newSubscriber(iiwaName + "/command/CartesianPose", geometry_msgs.PoseStamped._TYPE);
//		cartesianVelocitySubscriber = connectedNode.newSubscriber(iiwaName + "/command/CartesianVelocity", iiwa_msgs.CartesianVelocity._TYPE);
		cartesianWrenchSubscriber = connectedNode.newSubscriber(iiwaName + "/command/CartesianWrench", geometry_msgs.WrenchStamped._TYPE);

		jointPositionSubscriber = connectedNode.newSubscriber(iiwaName + "/command/JointPosition", iiwa_msgs.JointPosition._TYPE);
		jointPositionVelocitySubscriber = connectedNode.newSubscriber(iiwaName + "/command/JointPositionVelocity", iiwa_msgs.JointPositionVelocity._TYPE);
		jointStiffnessSubscriber = connectedNode.newSubscriber(iiwaName + "/command/JointStiffness", iiwa_msgs.JointStiffness._TYPE);
		jointTorqueSubscriber = connectedNode.newSubscriber(iiwaName + "/commmand/JointTorque", iiwa_msgs.JointTorque._TYPE);
//		jointVelocitySubscriber = connectedNode.newSubscriber(iiwaName + "/command/JointVelocity", iiwa_msgs.JointVelocity._TYPE);

		cartesianPoseSubscriber.addMessageListener(new MessageListener<geometry_msgs.PoseStamped>() {
			@Override
			public void onNewMessage(geometry_msgs.PoseStamped position) {
				cp = position;
				currentCommandType = CommandType.CARTESIAN_POSE;
			}
		});

//		cartesianVelocitySubscriber.addMessageListener(new MessageListener<iiwa_msgs.CartesianVelocity>() {
//			@Override
//			public void onNewMessage(iiwa_msgs.CartesianVelocity velocity) {
//				cv = velocity;
//				currentCommandType = CommandType.CARTESIAN_VELOCITY;
//			}
//		});

		cartesianWrenchSubscriber.addMessageListener(new MessageListener<geometry_msgs.WrenchStamped>() {
			@Override
			public void onNewMessage(geometry_msgs.WrenchStamped wrench) {
				cw = wrench;
			}
		});

		jointPositionSubscriber.addMessageListener(new MessageListener<iiwa_msgs.JointPosition>() {
			@Override
			public void onNewMessage(iiwa_msgs.JointPosition position){
				jp = position;
				currentCommandType = CommandType.JOINT_POSITION;
			}
		});
		
		jointPositionVelocitySubscriber.addMessageListener(new MessageListener<iiwa_msgs.JointPositionVelocity>() {
			@Override
			public void onNewMessage(iiwa_msgs.JointPositionVelocity positionVelocity){
				jpv = positionVelocity;
				currentCommandType = CommandType.JOINT_POSITION_VELOCITY;
			}
		});
		
		jointStiffnessSubscriber.addMessageListener(new MessageListener<iiwa_msgs.JointStiffness>() {
			@Override
			public void onNewMessage(iiwa_msgs.JointStiffness stiffness){
				js = stiffness;
			}
		});

		jointTorqueSubscriber.addMessageListener(new MessageListener<iiwa_msgs.JointTorque>() {
			@Override
			public void onNewMessage(iiwa_msgs.JointTorque torque) {
				jt = torque;
			}
		});

//		jointVelocitySubscriber.addMessageListener(new MessageListener<iiwa_msgs.JointVelocity>() {
//			@Override
//			public void onNewMessage(iiwa_msgs.JointVelocity velocity){
//				jv = velocity;
//				currentCommandType = CommandType.JOINT_VELOCITY;
//			}
//		});
		
		if (configureSmartServoCallback != null) {
			configureSmartServoServer = node.newServiceServer(
					iiwaName + "/configuration/configureSmartServo", 
					"iiwa_msgs/ConfigureSmartServo", 
					configureSmartServoCallback);
		}
	}
}
