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
import geometry_msgs.PoseStamped;
import iiwa_msgs.CartesianQuantity;
import iiwa_msgs.ConfigureSmartServoRequest;
import iiwa_msgs.ConfigureSmartServoResponse;
import iiwa_msgs.JointQuantity;
import iiwa_msgs.SmartServoMode;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.net.URI;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.ros.exception.ServiceException;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.time.NtpTimeProvider;

import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
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
	private String toolFrameID;
	private ObjectFrame toolFrame;
	private SmartServo motion;
	private Lock configureSmartServoLock = new ReentrantLock();

	private boolean initSuccessful = false;
	private boolean debug = false;

	private iiwaMessageGenerator helper; //< Helper class to generate iiwa_msgs from current robot state.
	private iiwaPublisher publisher; //< IIWARos Publisher.
	private iiwaSubscriber subscriber; //< IIWARos Subscriber.
	private iiwaConfiguration configuration; //< Configuration via parameters and services.

	// ROS Configuration and Node execution objects. Two different configurations are needed
	// for the Publisher and the Subscriber.
	private NodeConfiguration nodeConfPublisher;
	private NodeConfiguration nodeConfSubscriber;
	private NodeConfiguration nodeConfConfiguration;
	private NodeMainExecutor nodeMainExecutor;

	// configurable toolbars
	private List<IUserKeyBar> generalKeyBars = new ArrayList<IUserKeyBar>();
	private List<IUserKey> generalKeys = new ArrayList<IUserKey>();
	private List<IUserKeyListener> generalKeyLists = new ArrayList<IUserKeyListener>();

	public static class UnsupportedControlModeException extends RuntimeException {
		private static final long serialVersionUID = 1L;
		public UnsupportedControlModeException() { super(); }
		public UnsupportedControlModeException(String message) { super(message); }
		public UnsupportedControlModeException(String message, Throwable cause) { super(message, cause); }
		public UnsupportedControlModeException(Throwable cause) { super(cause); }
	}

	public IMotionControlMode buildMotionControlMode(iiwa_msgs.SmartServoMode params) {
		IMotionControlMode cm;

		switch (params.getMode()) {
		case iiwa_msgs.SmartServoMode.CARTESIAN_IMPEDANCE: {
			CartesianImpedanceControlMode ccm = new CartesianImpedanceControlMode();

			CartesianQuantity stiffness = params.getCartesianStiffness().getStiffness();
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

			CartesianQuantity damping = params.getCartesianDamping().getDamping();
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
			
			// TODO: add stiffness along axis
			if (params.getNullspaceStiffness() >= 0)
				ccm.setNullSpaceStiffness(params.getNullspaceStiffness());
			if (params.getNullspaceDamping() > 0)
				ccm.setNullSpaceDamping(params.getNullspaceDamping());

			cm = ccm;
			break;
		}

		case iiwa_msgs.SmartServoMode.JOINT_IMPEDANCE: {
			JointImpedanceControlMode jcm = new JointImpedanceControlMode(7);

			JointQuantity stiffness = params.getJointStiffness().getStiffness();
			jcm.setStiffness(helper.jointQuantityToVector(stiffness));

			JointQuantity damping = params.getJointDamping().getDamping();
			if (damping.getA1() > 0 && damping.getA2() > 0 && damping.getA3() > 0 && damping.getA4() > 0
					&& damping.getA5() > 0 && damping.getA6() > 0 && damping.getA7() > 0)
				jcm.setDamping(helper.jointQuantityToVector(damping));

			cm = jcm;
			break;
		}

		default: {
			throw new UnsupportedControlModeException();  // this should just not happen
		}
		}

		return cm;
	}

	public SmartServo configureSmartServoMotion(iiwa_msgs.SmartServoMode ssm) {
		SmartServo mot = new SmartServo(robot.getCurrentJointPosition());
		mot.setMinimumTrajectoryExecutionTime(0.1);
		mot.setTimeoutAfterGoalReach(300);
		
		configureSmartServoMotion(ssm, mot);
		return mot;
	}

	public void configureSmartServoMotion(iiwa_msgs.SmartServoMode ssm, SmartServo mot) {
		if (mot == null)
			return; // TODO: exception?

		if (ssm.getRelativeVelocity() > 0)
			mot.setJointVelocityRel(ssm.getRelativeVelocity());
		mot.setMode(buildMotionControlMode(ssm));
	}
	
	public boolean isSameControlMode(IMotionControlMode kukacm, SmartServoMode roscm) {		
		String roscmname = null;
		switch (roscm.getMode()) {
		case SmartServoMode.CARTESIAN_IMPEDANCE:
			roscmname = "CartesianImpedanceControlMode";
			break;
		case SmartServoMode.JOINT_IMPEDANCE:
			roscmname = "JointImpedanceControlMode";
			break;
		}
		String kukacmname = kukacm.getClass().getSimpleName();
		
		return roscmname.equals(kukacmname);
	}

	public void initialize() {
		robot = getContext().getDeviceFromType(LBR.class);
		helper = new iiwaMessageGenerator();
		configuration = new iiwaConfiguration();
		publisher = new iiwaPublisher(iiwaConfiguration.getRobotName());
		subscriber = new iiwaSubscriber(robot, iiwaConfiguration.getRobotName());

		// SmartServo configuration service callback
		subscriber.setConfigureSmartServoCallback(new ServiceResponseBuilder<iiwa_msgs.ConfigureSmartServoRequest, iiwa_msgs.ConfigureSmartServoResponse>() {

			@Override
			public void build(ConfigureSmartServoRequest req,
					ConfigureSmartServoResponse resp) throws ServiceException {
				// we can change the parameters if it is the same type of control strategy
				// otherwise we have to stop the motion, replace it and start it again
				try {
					if (req.getMode().getMode() == -1
							|| (motion.getMode() != null	&& isSameControlMode(motion.getMode(), req.getMode()))) {
						if (req.getMode().getMode() != -1)
							motion.getRuntime().changeControlModeSettings(buildMotionControlMode(req.getMode()));
						if (req.getMode().getRelativeVelocity() > 0)
							motion.setJointVelocityRel(req.getMode().getRelativeVelocity());
					} else {
						configureSmartServoLock.lock();
						
						SmartServo oldmotion = motion;
						ServoMotion.validateForImpedanceMode(robot);
						motion = configureSmartServoMotion(req.getMode());
						robot.moveAsync(motion);
						oldmotion.getRuntime().stopMotion();
						
						configureSmartServoLock.unlock();
					}
				} catch (Exception e) {
					resp.setSuccess(false);
					if (e.getMessage() != null) {
						StringWriter sw = new StringWriter();
						PrintWriter pw = new PrintWriter(sw);
						e.printStackTrace(pw);
						resp.setError(e.getClass().getName() + ": " + e.getMessage() + ", " + sw.toString());
					} else {
						resp.setError("because I hate you :)");
					}
					return;
				}
				resp.setSuccess(true);
			}
		});

		try {
			// Set the configuration parameters of the ROS nodes to create.
			URI uri = new URI(iiwaConfiguration.getMasterURI());

			nodeConfConfiguration = NodeConfiguration.newPublic(iiwaConfiguration.getRobotIp());
			nodeConfConfiguration.setTimeProvider(iiwaConfiguration.getTimeProvider());
			nodeConfConfiguration.setNodeName(iiwaConfiguration.getRobotName() + "/iiwa_configuration");
			nodeConfConfiguration.setMasterUri(uri);
			
			// Configuration for the Publisher.
			nodeConfPublisher = NodeConfiguration.newPublic(iiwaConfiguration.getRobotIp());
			nodeConfPublisher.setTimeProvider(iiwaConfiguration.getTimeProvider());
			nodeConfPublisher.setNodeName(iiwaConfiguration.getRobotName() + "/iiwa_publisher");
			nodeConfPublisher.setMasterUri(uri);
			

			// Configuration for the Subscriber.
			nodeConfSubscriber = NodeConfiguration.newPublic(iiwaConfiguration.getRobotIp());
			nodeConfSubscriber.setTimeProvider(iiwaConfiguration.getTimeProvider());
			nodeConfSubscriber.setNodeName(iiwaConfiguration.getRobotName() + "/iiwa_subscriber");
			nodeConfSubscriber.setMasterUri(uri);

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

		if (debug) 
			getLogger().info("ROS Nodes initialized.");

		initSuccessful = true;
	}

	public void run() {
		if (!initSuccessful) {
			throw new RuntimeException("Could not init the RoboticApplication successfully");
		}
			
		getLogger().info("using time provider: " + iiwaConfiguration.getTimeProvider().getClass().getSimpleName());

		try {
			configuration.waitForInitialization();
		} catch (InterruptedException e1) {
			e1.printStackTrace();
			return;
		}
		
		motion = new SmartServo(robot.getCurrentJointPosition());
		motion.setMinimumTrajectoryExecutionTime(8e-3);
		motion.setJointVelocityRel(configuration.getDefaultRelativeJointSpeed());
		motion.setTimeoutAfterGoalReach(300);

		// configurable toolbars to publish events on topics
		configuration.setupToolbars(getApplicationUI(), publisher, generalKeys, generalKeyLists, generalKeyBars);
		
		// Tool to attach
		String toolFromConfig = configuration.getToolName();
		if (toolFromConfig != "") {
			getLogger().info("attaching tool " + toolFromConfig);
			tool = (Tool)getApplicationData().createFromTemplate(toolFromConfig);
			tool.attachTo(robot.getFlange());
			toolFrameID = toolFromConfig + "_link_ee_kuka";
			toolFrame = tool.getFrame("/" + toolFrameID);
		} else {
			getLogger().info("no tool attached");
			toolFrameID = "iiwa_link_ee_kuka";
			toolFrame = robot.getFlange();
		}
		
		// publish joint state?
		publisher.setPublishJointStates(configuration.getPublishJointStates());

		toolFrame.moveAsync(motion);

		// The run loop
		getLogger().info("Starting the ROS Command loop...");
		try {   
			while (true) {

				if (iiwaConfiguration.getTimeProvider() instanceof org.ros.time.NtpTimeProvider) {
					((NtpTimeProvider) iiwaConfiguration.getTimeProvider()).updateTime();
				}
				
				motion.getRuntime().activateVelocityPlanning(true);
				
				/*
				 * This will build a JointPosition message with the current robot state.
				 * Set that message to be published and then publish it if there's a subscriber listening.
				 * Any other of the set methods for iiwa_msgs included in the published can be used at the same time,
				 * one just needs to build the message and set it to the publisher.
				 */
				publisher.publishCurrentState(robot, motion, toolFrame);

				if (subscriber.currentCommandType != null) {
					configureSmartServoLock.lock(); // the service could stop the motion and restart it
					
					switch (subscriber.currentCommandType) {
					case CARTESIAN_POSE: {
						PoseStamped commandPosition = subscriber.getCartesianPose(); // TODO: check that frame_id is consistent
						Transformation tr = helper.rosPoseToKukaTransformation(commandPosition.getPose());

						if (robot.isReadyToMove()) 
							motion.getRuntime().setDestination(tr);
					}
					break;
					case JOINT_POSITION: {
						/*
						 * This will acquire the last received JointPosition command from the commanding ROS node.
						 * If the robot can move, then it will move to this new position.
						 */
						iiwa_msgs.JointPosition commandPosition = subscriber.getJointPosition();
						JointPosition jp = helper.rosJointQuantityToKuka(commandPosition.getPosition());

						if (robot.isReadyToMove()) 
							motion.getRuntime().setDestination(jp);
					}
					break;
					case JOINT_POSITION_VELOCITY: {
						/*
						 * This will acquire the last received JointPositionVelocity command from the commanding ROS node.
						 * If the robot can move, then it will move to this new position.
						 */
						iiwa_msgs.JointPositionVelocity commandPositionVelocity = subscriber.getJointPositionVelocity();
						JointPosition jp = helper.rosJointQuantityToKuka(commandPositionVelocity.getPosition());
						JointPosition jv = helper.rosJointQuantityToKuka(commandPositionVelocity.getVelocity());

						if (robot.isReadyToMove()) 
							motion.getRuntime().setDestination(jp, jv);
					}
					break;

					default:
						throw new UnsupportedControlModeException();
					}

					configureSmartServoLock.unlock();
				}

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
			motion.getRuntime().stopMotion();
			if (debug)getLogger().info("ROS Node terminated.");
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
}