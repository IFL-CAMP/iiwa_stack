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
import java.net.URI;
import java.util.ArrayList;
import java.util.List;

import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;

/*
 * This example shows how to monitor and change the state of the robot.
 * The current state is published into a (monitor) ROS node, while another (command) node receives
 * messages containing the a new position for the robot.
 * The example uses a SmartServo motion to move the robot to the new commanded position.
 * 
 * Only the Joint Position of the robot is published (current position)
 * and received (new position) in this example,
 * but any other of its property included in the iiwa_msgs ROS package can be published
 * and received in the same way.
 */
public class ROSSmartServo extends RoboticsAPIApplication {

	private LBR robot;
	private Tool tool;
	private SmartServo motion;
	private ISmartServoRuntime runtime;
	
	private boolean debug = false;
	
	private iiwaMessageGenerator helper; //< Helper class to generate iiwa_msgs from current robot state.
	private iiwaPublisher publisher; //< IIWARos Publisher.
	private iiwaSubscriber subscriber; //< IIWARos Subscriber.
	private iiwaConfiguration configuration; //< Configuration via parameters and services.

	// TODO: change the following IP addresses according to your setup.
	private String masterUri = "http://160.69.69.100:11311";
	private String host = "160.69.69.69";

	// ROS Configuration and Node execution objects. Two different configurations are needed
	// for the Publisher and the Subscriber.
	private URI uri;
	private NodeConfiguration nodeConfPublisher;
	private NodeConfiguration nodeConfSubscriber;
	private NodeConfiguration nodeConfConfiguration;
	private NodeMainExecutor nodeMainExecutor;

	// iiwa_msgs to Publish and to receive.
	private iiwa_msgs.JointPosition currentPosition;
	private iiwa_msgs.JointPosition commandPosition;

	// configurable toolbars
	private List<IUserKeyBar> generalKeyBars = new ArrayList<IUserKeyBar>();
	private List<IUserKey> generalKeys = new ArrayList<IUserKey>();
	private List<IUserKeyListener> generalKeyLists = new ArrayList<IUserKeyListener>();
	
	// TODO: create iiwa_msgs.SmartServoParameters (name is negotiable)
	public static class SmartServoParameters {
		public static enum Mode {
			JOINT_IMPEDANCE,
			CARTESIAN_IMPEDANCE
		}
		
		public Mode mode;
		
		// both modes
		public float nullspace_stiffness;
		public float nullspace_damping;
		
		// joint space control
		public double[] joint_stiffness;
		public double[] joint_damping;
		
		// cartesian control
		public float[] cartesian_stiffness;
		public float[] cartesian_damping;
	}
	
	public static class UnsupportedControlModeException extends RuntimeException {
		private static final long serialVersionUID = 1L;
		public UnsupportedControlModeException() { super(); }
		public UnsupportedControlModeException(String message) { super(message); }
		public UnsupportedControlModeException(String message, Throwable cause) { super(message, cause); }
		public UnsupportedControlModeException(Throwable cause) { super(cause); }
	}
	
	public IMotionControlMode configureSmartServo(SmartServoParameters params) {
		IMotionControlMode cm;
		
		switch (params.mode) {
			case CARTESIAN_IMPEDANCE: {
				CartesianImpedanceControlMode ccm = new CartesianImpedanceControlMode();
				
				switch (params.cartesian_stiffness.length) {
				case 1: {
					ccm.parametrize(CartDOF.ALL).setStiffness(params.cartesian_stiffness[0]);
					break;
				}
				case 2: {
					ccm.parametrize(CartDOF.TRANSL).setStiffness(params.cartesian_stiffness[0]);
					ccm.parametrize(CartDOF.ROT).setStiffness(params.cartesian_stiffness[1]);
					break;
				}
				case 6: {
					ccm.parametrize(CartDOF.X).setStiffness(params.cartesian_stiffness[0]);
					ccm.parametrize(CartDOF.Y).setStiffness(params.cartesian_stiffness[1]);
					ccm.parametrize(CartDOF.Z).setStiffness(params.cartesian_stiffness[2]);
					ccm.parametrize(CartDOF.A).setStiffness(params.cartesian_stiffness[3]);
					ccm.parametrize(CartDOF.B).setStiffness(params.cartesian_stiffness[4]);
					ccm.parametrize(CartDOF.C).setStiffness(params.cartesian_stiffness[5]);
					break;
				}
				default: {
					throw new java.lang.IndexOutOfBoundsException("Illegal length "+params.cartesian_damping.length+" of cartesian stiffness array");
				}
				}
				
				switch (params.cartesian_damping.length) {
				case 1: {
					ccm.parametrize(CartDOF.ALL).setDamping(params.cartesian_stiffness[0]);
					break;
				}
				case 2: {
					ccm.parametrize(CartDOF.TRANSL).setDamping(params.cartesian_stiffness[0]);
					ccm.parametrize(CartDOF.ROT).setDamping(params.cartesian_stiffness[1]);
					break;
				}
				case 6: {
					ccm.parametrize(CartDOF.X).setDamping(params.cartesian_stiffness[0]);
					ccm.parametrize(CartDOF.Y).setDamping(params.cartesian_stiffness[1]);
					ccm.parametrize(CartDOF.Z).setDamping(params.cartesian_stiffness[2]);
					ccm.parametrize(CartDOF.A).setDamping(params.cartesian_stiffness[3]);
					ccm.parametrize(CartDOF.B).setDamping(params.cartesian_stiffness[4]);
					ccm.parametrize(CartDOF.C).setDamping(params.cartesian_stiffness[5]);
					break;
				}
				default: {
					throw new java.lang.IndexOutOfBoundsException("Illegal length "+params.cartesian_damping.length+" of cartesian damping array");
				}
				}
				
				ccm.setNullSpaceStiffness(params.nullspace_stiffness);
				ccm.setNullSpaceDamping(params.nullspace_damping);
				
				cm = ccm;
				break;
			}
			
			case JOINT_IMPEDANCE: {
				JointImpedanceControlMode jcm = new JointImpedanceControlMode();
				
				switch (params.joint_stiffness.length) {
				case 1: {
					jcm.setStiffnessForAllJoints(params.joint_stiffness[0]);
					break;
				}
				case 7: {
					jcm.setStiffness(params.joint_stiffness);
					break;
				}
				default: {
					throw new java.lang.IndexOutOfBoundsException("Illegal length "+params.cartesian_damping.length+" of joint stiffness array");
				}
				}
				
				switch (params.joint_damping.length) {
				case 1: {
					jcm.setDampingForAllJoints(params.joint_damping[0]);
					break;
				}
				case 7: {
					jcm.setDamping(params.joint_damping);
					break;
				}
				default: {
					throw new java.lang.IndexOutOfBoundsException("Illegal length "+params.cartesian_damping.length+" of joint stiffness array");
				}
				}
				
				cm = jcm;
				break;
			}
			
			default: {
				throw new UnsupportedControlModeException();  // this should just not happen
			}
		}
		
		return cm;
	}
	
	public void initialize() {
		robot = getContext().getDeviceFromType(LBR.class);
		helper = new iiwaMessageGenerator();
		publisher = new iiwaPublisher(robot,"iiwa");
		subscriber = new iiwaSubscriber(robot,"iiwa");
		configuration = new iiwaConfiguration("iiwa");

		try {
			// Set the configuration parameters of the ROS nodes to create.
			uri = new URI(masterUri);

			// Configuration for the Publisher.
			nodeConfPublisher = NodeConfiguration.newPublic(host);
			nodeConfPublisher.setNodeName("publisherNode");
			nodeConfPublisher.setMasterUri(uri);

			// Configuration for the Subscriber.
			nodeConfSubscriber = NodeConfiguration.newPublic(host);
			nodeConfSubscriber.setNodeName("subscriberNode");
			nodeConfSubscriber.setMasterUri(uri);
			
			nodeConfConfiguration = NodeConfiguration.newPublic(host);
			nodeConfConfiguration.setNodeName("/iiwa/iiwa_configuration");
			nodeConfConfiguration.setMasterUri(uri);

			// Publisher and Subscriber nodes are executed. Their onStart method is called here.
			nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
			nodeMainExecutor.execute(publisher, nodeConfPublisher);
			nodeMainExecutor.execute(subscriber, nodeConfSubscriber);
			nodeMainExecutor.execute(configuration, nodeConfConfiguration);
		}
		catch (Exception e) {
			if (debug) getLogger().info("Node Configuration failed.");
			getLogger().info(e.toString());
		}

		if (debug) getLogger().info("ROS Nodes initialized.");
	}

	public void run() {

		motion = new SmartServo(robot.getCurrentJointPosition());
		motion.setMinimumTrajectoryExecutionTime(8e-3);
		motion.setJointVelocityRel(0.2);
		
		try {
			configuration.waitForInitialization();
		} catch (InterruptedException e1) {
			e1.printStackTrace();
			return;
		}
		
		// configurable toolbars to publish events on topics
		configuration.setupToolbars(getApplicationUI(), publisher, generalKeys, generalKeyLists, generalKeyBars);
		
		String toolFromConfig = configuration.getToolName();
		if (toolFromConfig != null && toolFromConfig != "") {
			getLogger().info("attaching tool " + toolFromConfig);
			tool = (Tool)getApplicationData().createFromTemplate(toolFromConfig);
			tool.attachTo(robot.getFlange());
		} else {
			getLogger().info("no tool attached");
		}

		robot.moveAsync(motion);
		runtime = motion.getRuntime();

		// The run loop
		getLogger().info("Starting the ROS Command loop...");
		try {   
			while (true) {

				/*
				 * This will build a JointPosition message with the current robot state.
				 * Set that message to be published and then publish it if there's a subscriber listening.
				 * Any other of the set methods for iiwa_msgs included in the published can be used at the same time,
				 * one just needs to build the message and set it to the publisher.
				 */
				currentPosition = helper.buildJointPosition(robot);
				publisher.setJointPosition(currentPosition);
				publisher.publish();

				/*
				 * This will acquire the last received JointPosition command from the commanding ROS node.
				 * If the robot can move, then it will move to this new position.
				 */
				commandPosition = subscriber.getJointPosition();
				if (robot.isReadyToMove()) 
					runtime.setDestination(new JointPosition(commandPosition.getPosition()));
			}
		} catch (Exception ex) {
			getLogger().info("ROS loop aborted. " + ex.toString());
		}
		finally {
			// The ROS nodes are killed.
			if (nodeMainExecutor != null) {
				nodeMainExecutor.shutdownNodeMain(publisher);
				nodeMainExecutor.shutdownNodeMain(subscriber);
				nodeMainExecutor.shutdownNodeMain(configuration);
			}
			runtime.stopMotion();
			if (debug)getLogger().info("ROS Nodea terminated.");
		}
		getLogger().info("ROS loop has ended. Application terminated.");
	}

	@Override
	public void dispose() {
		// The ROS nodes are killed.
		if (nodeMainExecutor != null && publisher != null && subscriber != null) {
			nodeMainExecutor.shutdownNodeMain(publisher);
			nodeMainExecutor.shutdownNodeMain(subscriber);
			nodeMainExecutor.shutdownNodeMain(configuration);
			getLogger().info("ROS nodes have been terminated by Garbage Collection.");
		}
		super.dispose();
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		ROSSmartServo app = new ROSSmartServo();
		app.runApplication();
	}
}