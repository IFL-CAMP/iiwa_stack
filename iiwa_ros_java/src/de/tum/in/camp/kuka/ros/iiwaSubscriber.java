/** Copyright (C) 2015 Salvatore Virga - salvo.virga@tum.de
 * Technische Universitaet Muenchen
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultaet fuer Informatik / I16, Boltzmannstrasse 3, 85748 Garching bei Muenchen, Germany
 * http://campar.in.tum.de
 * 
 * LICENSE :
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * @author Salvatore Virga
 * 
 */

package de.tum.in.camp.kuka.ros;

// ROS imports
import iiwa_msgs.ConfigureSmartServoRequest;
import iiwa_msgs.ConfigureSmartServoResponse;

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
		JOINT_VELOCITY
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
	private Subscriber<iiwa_msgs.JointStiffness> jointStiffnessSubscriber;
	private Subscriber<iiwa_msgs.JointTorque> jointTorqueSubscriber;
//	private Subscriber<iiwa_msgs.JointVelocity> jointVelocitySubscriber;

	// Local iiwa_msgs to store received messages 
	// Cartesian Messages
	private geometry_msgs.PoseStamped cp;
	private iiwa_msgs.CartesianVelocity cv;
	private geometry_msgs.WrenchStamped cw;
	// Joint Messages
	private iiwa_msgs.JointPosition jp;
	private iiwa_msgs.JointStiffness js;
	private iiwa_msgs.JointTorque jt;
//	private iiwa_msgs.JointVelocity jv;
	
	// current control strategy TODO: set this with a service; for now it is the last message arrived
	CommandType currentCommandType;

	// Object to easily build iiwa_msgs from the current robot state
	private iiwaMessageGenerator helper = new iiwaMessageGenerator();

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
		cp = helper.buildCartesianPose(robot);
//		cv = helper.buildCartesianVelocity(robot);
		cw = helper.buildCartesianWrench(robot);

		jp = helper.buildJointPosition(robot);
		js = helper.buildJointStiffness(robot, null);
		jt = helper.buildJointTorque(robot);
//		jv = helper.buildJointVelocity(robot);
		
		iiwaName = robotName;
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
		cp = helper.buildCartesianPose(robot, frame);
//		cv = helper.buildCartesianVelocity(robot, frame);
		cw = helper.buildCartesianWrench(robot, frame);

		jp = helper.buildJointPosition(robot);
		js = helper.buildJointStiffness(robot, null);
		jt = helper.buildJointTorque(robot);
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
		jointStiffnessSubscriber = connectedNode.newSubscriber(iiwaName + "/command/JointStiffness", iiwa_msgs.JointStiffness._TYPE);
		jointTorqueSubscriber = connectedNode.newSubscriber(iiwaName + "/commmand/JointTorque", iiwa_msgs.JointTorque._TYPE);
//		jointVelocitySubscriber = connectedNode.newSubscriber(iiwaName + "/command/JointVelocity", iiwa_msgs.JointVelocity._TYPE);

		cartesianPoseSubscriber.addMessageListener(new MessageListener<geometry_msgs.PoseStamped>() {
			@Override
			public void onNewMessage(geometry_msgs.PoseStamped position) {
				// TODO: check if it works
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
					"configureSmartServo", 
					"iiwa_msgs/ConfigureSmartServo", 
					configureSmartServoCallback);
		}
	}
}
