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

import geometry_msgs.PoseStamped;
import geometry_msgs.WrenchStamped;
import iiwa_msgs.JointPosition;
import iiwa_msgs.JointStiffness;
import iiwa_msgs.JointTorque;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;

/**
 * This class provides ROS subscribers for ROS messages defined in the iiwa_msgs ROS package.
 * It allows to send messages of that type via ROS topics named with the following convention :
 * <robot name>/state/<iiwa message type> (e.g. MyIIWA/state/CartesianPosition)
 */
public class iiwaPublisher extends AbstractNodeMain {

	// ROSJava Publishers for iiwa_msgs
	// Cartesian Message Publishers
	private Publisher<geometry_msgs.PoseStamped> cartesianPosePublisher;
//	private Publisher<iiwa_msgs.CartesianVelocity> cartesianVelocityPublisher;
	private Publisher<geometry_msgs.WrenchStamped> cartesianWrenchPublisher;
	// Joint Message Publishers
	private Publisher<iiwa_msgs.JointPosition> jointPositionPublisher;
	private Publisher<iiwa_msgs.JointStiffness> jointStiffnessPublisher;
	private Publisher<iiwa_msgs.JointDamping> jointDampingPublisher;
	private Publisher<iiwa_msgs.JointTorque> jointTorquePublisher;
//	private Publisher<iiwa_msgs.JointVelocity> jointVelocityPublisher;
	// UserKey Event Publisher
	private Publisher<std_msgs.String> iiwaButtonPublisher; // TODO: iiwa_msgs.ButtonEvent
	// JointState publisher (optional)
	private Publisher<sensor_msgs.JointState> jointStatesPublisher;
	private boolean publishJointState = false;
	
	// Object to easily build iiwa_msgs from the current robot state
	private iiwaMessageGenerator helper = new iiwaMessageGenerator();
	
	// Cache objects
	private geometry_msgs.PoseStamped cp = helper.buildMessage(PoseStamped._TYPE);
//	private iiwa_msgs.CartesianVelocity cv = helper.buildMessage(CartesianVelocity._TYPE);
	private geometry_msgs.WrenchStamped cw = helper.buildMessage(WrenchStamped._TYPE);
	private iiwa_msgs.JointPosition jp = helper.buildMessage(JointPosition._TYPE);
	private iiwa_msgs.JointStiffness jst = helper.buildMessage(JointStiffness._TYPE);
	private iiwa_msgs.JointDamping jd = helper.buildMessage(iiwa_msgs.JointDamping._TYPE);
	private iiwa_msgs.JointTorque jt = helper.buildMessage(JointTorque._TYPE);
	private sensor_msgs.JointState js = helper.buildMessage(sensor_msgs.JointState._TYPE);

	// Name to use to build the name of the ROS topics
	private String iiwaName = "iiwa";

	/**
	 * Constructs a series of ROS publishers for messages defined by the iiwa_msgs ROS package. <p>
	 * The initial values of the messages are set using the current state of the iiwa robot passed.
	 * Except for Velocity messages, those will be initialized to zero.<br>
	 * For Cartesian messages, the initial values will refer to the frame of the robot's Flange.
	 * @param robot : an iiwa Robot, its current state is used to set up initial values for the messages.
	 */
	public iiwaPublisher(String robotName) {
		iiwaName = robotName;
	}
	
	public void setPublishJointStates(boolean b) {
		publishJointState = b;
	}

	/**
	 * Set the name to use to compose the ROS topics' names for the publishers. <p>
	 * e.g. setIIWAName("dummy"), the topics' names will be "dummy/state/...". <br>
	 * The creation of the nodes is performed when the <i>execute</i> method from a <i>nodeMainExecutor</i> is called.
	 * @param newName : the new name to use for ROS topics.
	 */
	public void setIIWAName(String newName) {
		iiwaName = newName;
	}

	/**
	 * Returns the current name used to compose the ROS topics' names for the publishers. <p>
	 * e.g. returning "dummy" means that the topics' names will be "dummy/state/...". <br>
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
		return GraphName.of(iiwaName + "/publisher");
	}

	/**
	 * This method is called when the <i>execute</i> method from a <i>nodeMainExecutor</i> is called.<br>
	 * Do <b>NOT</b> manually call this. <p> 
	 * @see org.ros.node.AbstractNodeMain#onStart(org.ros.node.ConnectedNode)
	 */
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		cartesianPosePublisher = connectedNode.newPublisher(iiwaName + "/state/CartesianPose", geometry_msgs.PoseStamped._TYPE);
//		cartesianVelocityPublisher = connectedNode.newPublisher(iiwaName + "/state/CartesianVelocity", iiwa_msgs.CartesianVelocity._TYPE);
		cartesianWrenchPublisher = connectedNode.newPublisher(iiwaName + "/state/CartesianWrench", geometry_msgs.WrenchStamped._TYPE);

		jointPositionPublisher = connectedNode.newPublisher(iiwaName + "/state/JointPosition", iiwa_msgs.JointPosition._TYPE);
		jointStiffnessPublisher = connectedNode.newPublisher(iiwaName + "/state/JointStiffness", iiwa_msgs.JointStiffness._TYPE);
		jointDampingPublisher = connectedNode.newPublisher(iiwaName + "/state/JointDamping", iiwa_msgs.JointDamping._TYPE);
		jointTorquePublisher = connectedNode.newPublisher(iiwaName + "/state/JointTorque", iiwa_msgs.JointTorque._TYPE);
//		jointVelocityPublisher = connectedNode.newPublisher(iiwaName + "/state/JointVelocity", iiwa_msgs.JointVelocity._TYPE);
		
		iiwaButtonPublisher = connectedNode.newPublisher(iiwaName + "/state/buttonEvent", std_msgs.String._TYPE);
		jointStatesPublisher = connectedNode.newPublisher(iiwaName + "/joint_states", sensor_msgs.JointState._TYPE);
	}
	
	public void publishCurrentState(LBR robot, SmartServo motion) throws InterruptedException {
		publishCurrentState(robot, motion, robot.getFlange());
	}
	
	/**
	 * Publishes to the respective topics all the iiwa_msgs with the values they are currently set to.<p>
	 * To set the messages to be published use their set methods.<br>
	 * Only the nodes that currently have subscribers will publish the messages.<br>
	 * @throws InterruptedException
	 */
	public void publishCurrentState(LBR robot, SmartServo motion, ObjectFrame frame) throws InterruptedException {
		if (cartesianPosePublisher.getNumberOfSubscribers() > 0) {
			helper.getCurrentCartesianPose(cp, robot, frame);
			helper.incrementSeqNumber(cp.getHeader());
			cartesianPosePublisher.publish(cp);
		}
		if (cartesianWrenchPublisher.getNumberOfSubscribers() > 0) {
			helper.getCurrentCartesianWrench(cw, robot, frame);
			helper.incrementSeqNumber(cw.getHeader());
			cartesianWrenchPublisher.publish(cw);
		}
		if (jointPositionPublisher.getNumberOfSubscribers() > 0) {
			helper.getCurrentJointPosition(jp, robot);
			helper.incrementSeqNumber(jp.getHeader());
			jointPositionPublisher.publish(jp);
		}
		if (jointTorquePublisher.getNumberOfSubscribers() > 0) {
			helper.getCurrentJointTorque(jt, robot);
			helper.incrementSeqNumber(jt.getHeader());
			jointTorquePublisher.publish(jt);
		}
		if (jointStiffnessPublisher.getNumberOfSubscribers() > 0) {
			helper.getCurrentJointStiffness(jst, robot, motion);
			helper.incrementSeqNumber(jst.getHeader());
			jointStiffnessPublisher.publish(jst);
		}
		if (jointDampingPublisher.getNumberOfSubscribers() > 0) {
			helper.getCurrentJointDamping(jd, robot, motion);
			helper.incrementSeqNumber(jp.getHeader());
			jointDampingPublisher.publish(jd);
		}
		if (publishJointState && jointStatesPublisher.getNumberOfSubscribers() > 0) {
			helper.getCurrentJointState(js, robot, motion);
			helper.incrementSeqNumber(js.getHeader());
			jointStatesPublisher.publish(js);
		}
	}
	
	public void publishButtonPressed(String name) {
		final std_msgs.String msg = iiwaButtonPublisher.newMessage();
		msg.setData(name + "_pressed");
		iiwaButtonPublisher.publish(msg);
	}
	
	public void publishButtonReleased(String name) {
		final std_msgs.String msg = iiwaButtonPublisher.newMessage();
		msg.setData(name + "_released");
		iiwaButtonPublisher.publish(msg);
	}
}
