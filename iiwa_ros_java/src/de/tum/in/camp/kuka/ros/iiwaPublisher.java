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

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;

/**
 * This class implements a ROS Node that publishes the current state of the robot. <br>
 * Messages will be send via topics in this format : <robot name>/state/<iiwa_msgs type> (e.g. MyIIWA/state/CartesianPosition)
 */
public class iiwaPublisher extends AbstractNodeMain {

	// ROSJava Publishers for iiwa_msgs
	// Cartesian Message Publishers
	private Publisher<geometry_msgs.PoseStamped> cartesianPosePublisher;
	private Publisher<geometry_msgs.WrenchStamped> cartesianWrenchPublisher;
	// Joint Message Publishers
	private Publisher<iiwa_msgs.JointPosition> jointPositionPublisher;
	private Publisher<iiwa_msgs.JointPositionVelocity> jointPositionVelocityPublisher;
	private Publisher<iiwa_msgs.JointStiffness> jointStiffnessPublisher;
	private Publisher<iiwa_msgs.JointDamping> jointDampingPublisher;
	private Publisher<iiwa_msgs.JointTorque> jointTorquePublisher;
	private Publisher<iiwa_msgs.JointVelocity> jointVelocityPublisher;
	// UserKey Event Publisher
	private Publisher<std_msgs.String> iiwaButtonPublisher; // TODO: iiwa_msgs.ButtonEvent
	// JointState publisher (optional)
	private Publisher<sensor_msgs.JointState> jointStatesPublisher;
	private boolean publishJointState = false;
	//DestinationReachedFlag publisher
	private Publisher<std_msgs.Time> destinationReachedPublisher;
	// Name to use to build the name of the ROS topics
	private String iiwaName = "iiwa";

	// Object to easily build iiwa_msgs from the current robot state
	private iiwaMessageGenerator helper;
	
	private ConnectedNode node = null;

	// Cache objects
	private geometry_msgs.PoseStamped cp;
	private geometry_msgs.WrenchStamped cw;
	private iiwa_msgs.JointPosition jp;
	private iiwa_msgs.JointPositionVelocity jpv;
	private iiwa_msgs.JointStiffness jst;
	private iiwa_msgs.JointDamping jd;
	private iiwa_msgs.JointTorque jt;
	private sensor_msgs.JointState js;
	private iiwa_msgs.JointVelocity jv;
	private std_msgs.Time t;

	/**
	 * Create a ROS node with publishers for a robot state. <br>
	 * Node will be running when the <i>execute</i> method from a <i>nodeMainExecutor</i> is called.<br>
	 * 
	 * @param robotName : name of the robot, topics will be created accordingly : <robot name>/state/<iiwa_msgs type> (e.g. MyIIWA/state/CartesianPosition)
	 */
	public iiwaPublisher(String robotName) {
		iiwaName = robotName;
		helper = new iiwaMessageGenerator(iiwaName);

		cp = helper.buildMessage(geometry_msgs.PoseStamped._TYPE);
		cw = helper.buildMessage(geometry_msgs.WrenchStamped._TYPE);
		jp = helper.buildMessage(iiwa_msgs.JointPosition._TYPE);
		jpv = helper.buildMessage(iiwa_msgs.JointPositionVelocity._TYPE);
		jst = helper.buildMessage(iiwa_msgs.JointStiffness._TYPE);
		jd = helper.buildMessage(iiwa_msgs.JointDamping._TYPE);
		jt = helper.buildMessage(iiwa_msgs.JointTorque._TYPE);
		jv = helper.buildMessage(iiwa_msgs.JointVelocity._TYPE);
		js = helper.buildMessage(sensor_msgs.JointState._TYPE);
		t = helper.buildMessage(std_msgs.Time._TYPE);
	}

	/**
	 * Set if also joint_states should be published
	 * @param publishJointState
	 */
	public void setPublishJointStates(boolean publishJointState) {
		this.publishJointState = publishJointState;
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
		node = connectedNode;
		
		cartesianPosePublisher = connectedNode.newPublisher(iiwaName + "/state/CartesianPose", geometry_msgs.PoseStamped._TYPE);
		cartesianWrenchPublisher = connectedNode.newPublisher(iiwaName + "/state/CartesianWrench", geometry_msgs.WrenchStamped._TYPE);

		jointPositionPublisher = connectedNode.newPublisher(iiwaName + "/state/JointPosition", iiwa_msgs.JointPosition._TYPE);
		jointPositionVelocityPublisher = connectedNode.newPublisher(iiwaName + "/state/JointPositionVelocity", iiwa_msgs.JointPositionVelocity._TYPE);
		jointStiffnessPublisher = connectedNode.newPublisher(iiwaName + "/state/JointStiffness", iiwa_msgs.JointStiffness._TYPE);
		jointDampingPublisher = connectedNode.newPublisher(iiwaName + "/state/JointDamping", iiwa_msgs.JointDamping._TYPE);
		jointTorquePublisher = connectedNode.newPublisher(iiwaName + "/state/JointTorque", iiwa_msgs.JointTorque._TYPE);
		jointVelocityPublisher = connectedNode.newPublisher(iiwaName + "/state/JointVelocity",  iiwa_msgs.JointVelocity._TYPE);

		iiwaButtonPublisher = connectedNode.newPublisher(iiwaName + "/state/buttonEvent", std_msgs.String._TYPE);
		jointStatesPublisher = connectedNode.newPublisher(iiwaName + "/joint_states", sensor_msgs.JointState._TYPE);

		destinationReachedPublisher = connectedNode.newPublisher(iiwaName + "/state/DestinationReached", std_msgs.Time._TYPE);
	}

	/**
	 * Publishes to the respective topics all the iiwa_msgs with the values they are currently set to.<p>
	 * Only the nodes that currently have subscribers will publish the messages.<br>
	 * <b>Cartesian information published will be relative to the robot's flange</b>
	 * 
	 * @param robot : the state of this robot will be published
	 * @param motion : the dynamic of this motion will be published
	 * @throws InterruptedException
	 */
	public void publishCurrentState(LBR robot, SmartServo motion) throws InterruptedException {
		publishCurrentState(robot, motion, robot.getFlange());
	}

	/**
	 * Publishes to the respective topics all the iiwa_msgs with the values they are currently set to.<p>
	 * Only the nodes that currently have subscribers will publish the messages.<br>
	 * 
	 * @param robot : the state of this robot will be published
	 * @param motion : the dynamic of this motion will be published
	 * @param frame : the Cartesian information published will be relative to this frame
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
		if (jointPositionVelocityPublisher.getNumberOfSubscribers() > 0) {
			helper.getCurrentJointPositionVelocity(jpv, robot);
			helper.incrementSeqNumber(jpv.getHeader());
			jointPositionVelocityPublisher.publish(jpv);
		}
		if (jointVelocityPublisher.getNumberOfSubscribers() > 0) {
			helper.getCurrentJointVelocity(jv, robot);
			helper.incrementSeqNumber(jv.getHeader());
			jointVelocityPublisher.publish(jv);
		}
		if (jointTorquePublisher.getNumberOfSubscribers() > 0) {
			helper.getCurrentJointTorque(jt, robot);
			helper.incrementSeqNumber(jt.getHeader());
			jointTorquePublisher.publish(jt);
		}
		
		if (motion != null) {
			if (jointStiffnessPublisher.getNumberOfSubscribers() > 0 && motion.getMode() instanceof JointImpedanceControlMode) {
				helper.getCurrentJointStiffness(jst, robot, motion);
				helper.incrementSeqNumber(jst.getHeader());
				jointStiffnessPublisher.publish(jst);
			}
			if (jointDampingPublisher.getNumberOfSubscribers() > 0  && motion.getMode() instanceof JointImpedanceControlMode) {
				helper.getCurrentJointDamping(jd, robot, motion);
				helper.incrementSeqNumber(jp.getHeader());
				jointDampingPublisher.publish(jd);
			}
		}
		
		if (publishJointState && jointStatesPublisher.getNumberOfSubscribers() > 0) {
			helper.getCurrentJointState(js, robot);
			helper.incrementSeqNumber(js.getHeader());
			jointStatesPublisher.publish(js);
		}		
	}

	/**
	 * Publishes the current timestamp on the destinationReached topic.
	 */
	public void publishDestinationReached() {
		if (destinationReachedPublisher.getNumberOfSubscribers() > 0) {
			t.setData(node.getCurrentTime());
			destinationReachedPublisher.publish(t);
		}
	}

	/**
	 * Publishes the event of a button on the SmartPad toolbar being <b>pressed</b>
	 * @param name : name of the button
	 */
	public void publishButtonPressed(String name) {
		final std_msgs.String msg = iiwaButtonPublisher.newMessage();
		msg.setData(name + "_pressed");
		iiwaButtonPublisher.publish(msg);
	}

	/**
	 * Publishes the event of a button on the SmartPad toolbar being <b>released</b>
	 * @param name : name of the button
	 */
	public void publishButtonReleased(String name) {
		final std_msgs.String msg = iiwaButtonPublisher.newMessage();
		msg.setData(name + "_released");
		iiwaButtonPublisher.publish(msg);
	}
}
