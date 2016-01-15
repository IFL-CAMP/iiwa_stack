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

//ROS imports
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.URI;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;

/*
 * This example shows how to monitor the state of the robot, publishing it into ROS nodes.
 * Only the Joint Position of the robot is published in this example,
 * but any other of its property included in the iiwa_msgs ROS package can be published in the same way.
 */
public class ROSMonitor extends RoboticsAPIApplication {

	private LBR robot;
	private Tool tool;
	private SmartServo motion;
	private ISmartServoRuntime runtime;
	
	private boolean debug = false;
	
	private iiwaMessageGenerator helper; //< Helper class to generate iiwa_msgs from current robot state.
	private iiwaPublisher publisher; //< IIWARos Publisher.
	private iiwaConfiguration configuration; //< Configuration via parameters and services.

	// TODO: change the following IP addresses according to your setup.
	private String masterIp = null;
	private String masterPort = null;
	private String masterUri = null; //< IP address of ROS core to talk to.
	private String localhostIp = null;
	
	private boolean configSuccessful = false;

	// ROS Configuration and Node execution objects.
	private URI uri;
	private NodeConfiguration nodeConfPublisher;
	private NodeConfiguration nodeConfConfiguration;
	private NodeMainExecutor nodeExecutor;

	// Message to publish.
	private iiwa_msgs.JointPosition currentPosition;

	// configurable toolbars
	private List<IUserKeyBar> generalKeyBars = new ArrayList<IUserKeyBar>();
	private List<IUserKey> generalKeys = new ArrayList<IUserKey>();
	private List<IUserKeyListener> generalKeyLists = new ArrayList<IUserKeyListener>();
	
	// gravity compensation stuff
	private IUserKeyBar gravcompKeybar;
	private IUserKey gravCompKey;
	private IUserKeyListener gravCompKeyList;
	private boolean gravCompEnabled = false;
	private boolean gravCompSwitched = false;
	
	public void initialize() {
		robot = getContext().getDeviceFromType(LBR.class);
		
		// standard stuff
		helper = new iiwaMessageGenerator();
		publisher = new iiwaPublisher(robot,"iiwa");
		configuration = new iiwaConfiguration("iiwa");
		
		// gravity compensation - only in ROSMonitor for safety
		gravcompKeybar = getApplicationUI().createUserKeyBar("Gravcomp");
		gravCompKeyList = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent event) {
				if (event == UserKeyEvent.FirstKeyDown) {
					gravCompEnabled = true;
					gravCompSwitched = true;
				} else if (event == UserKeyEvent.SecondKeyDown) {
					gravCompEnabled = false;
					gravCompSwitched = true;
				}
			}
		};
		gravCompKey = gravcompKeybar.addDoubleUserKey(0, gravCompKeyList, true);
		gravCompKey.setText(UserKeyAlignment.TopMiddle, "ON");
		gravCompKey.setText(UserKeyAlignment.BottomMiddle, "OFF");
		gravcompKeybar.publish();
		
		// network configuration
		BufferedReader br = new BufferedReader(new InputStreamReader(getClass().getResourceAsStream("config.txt")));
		try {
			String line = null;
			while((line = br.readLine()) != null) {
				String[] lineComponents = line.split(":");
				if (lineComponents.length == 0)
					continue;
				if (lineComponents[0].equals("master_ip")) {
					masterIp = lineComponents[1].trim();
				}
				if (lineComponents[0].equals("master_port")) {
					masterPort = lineComponents[1].trim();
				}
			}
		} catch (IOException e2) {
			// TODO Auto-generated catch block
			e2.printStackTrace();
		}
		
		if (masterIp == null) {
			System.out.println("Could not find ROS master ip in config file!");
			return;
		}
		
		if (masterPort == null) {
			System.out.println("Could not find ROS master port in config file!");
			return;
		}
		
		masterUri = "http://" + masterIp + ":" + masterPort;
		
		String[] master_components = masterIp.split("\\.");
		Enumeration<NetworkInterface> ifaces = null;
		try {
			ifaces = NetworkInterface.getNetworkInterfaces();
		} catch (SocketException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		boolean localhostIpFound = false;
		while(!localhostIpFound && ifaces.hasMoreElements())
		{
		    NetworkInterface n = (NetworkInterface) ifaces.nextElement();
		    Enumeration<InetAddress> ee = n.getInetAddresses();
		    while (ee.hasMoreElements())
		    {
		        localhostIp = ((InetAddress) ee.nextElement()).getHostAddress();
		        String[] components = localhostIp.split("\\.");
				
				boolean matches = components[0].equals(master_components[0])
						&& components[1].equals(master_components[1])
						&& components[2].equals(master_components[2]);
				if (matches) {
					localhostIpFound = true;
					break;
				}
		    }
		}
		
		// ROS initialization

		try {
			// Set the configuration parameters of the ROS node to create.
			uri = new URI(masterUri);
			nodeConfPublisher = NodeConfiguration.newPublic(localhostIp);
			nodeConfPublisher.setNodeName("/iiwa/iiwa_publisher");
			nodeConfPublisher.setMasterUri(uri);
			nodeConfConfiguration = NodeConfiguration.newPublic(localhostIp);
			nodeConfConfiguration.setNodeName("/iiwa/iiwa_configuration");
			nodeConfConfiguration.setMasterUri(uri);
		}
		catch (Exception e) {
			if (debug) getLogger().info("Node Configuration failed; please check the ROS master IP in the Sunrise app source code");
			getLogger().info(e.toString());
			return;
		}

		try {
			// Start the Publisher node with the set up configuration.
			nodeExecutor = DefaultNodeMainExecutor.newDefault();
			nodeExecutor.execute(publisher, nodeConfPublisher);
			nodeExecutor.execute(configuration, nodeConfConfiguration);
			if (debug) getLogger().info("ROS Node initialized.");
		}
		catch(Exception e) {
			if (debug) getLogger().info("Node Executor failed.");
			getLogger().info(e.toString());
			return;
		}
		
		configSuccessful = true;
	}

	public void run() {
		if (!configSuccessful) {
			throw new RuntimeException("Could not configure successfully");
		}
		
		motion = new SmartServo(robot.getCurrentJointPosition());
		motion.setMinimumTrajectoryExecutionTime(8e-3);
		motion.setJointVelocityRel(0.2);
		motion.setTimeoutAfterGoalReach(300);
		
		try {
			configuration.waitForInitialization();
		} catch (InterruptedException e1) {
			e1.printStackTrace();
			return;
		}
		
		// configurable toolbars to publish events on topics
		configuration.setupToolbars(getApplicationUI(), publisher, generalKeys, generalKeyLists, generalKeyBars);
		
		// Tool to attach
		String toolFromConfig = configuration.getToolName();
		if (toolFromConfig == null) 
			getLogger().error("no Sunrise tool name specified!");
		if (toolFromConfig != "") {
			getLogger().info("attaching tool " + toolFromConfig);
			tool = (Tool)getApplicationData().createFromTemplate(toolFromConfig);
			tool.attachTo(robot.getFlange());
		} else {
			getLogger().info("no tool attached");
		}
		
		if (!SmartServo.validateForImpedanceMode(robot))
			getLogger().error("Too much external torque on the robot! Is it a singular position?");
		
		JointImpedanceControlMode controlMode = new JointImpedanceControlMode(7); // TODO!!
		robot.moveAsync(motion.setMode(controlMode));
		runtime = motion.getRuntime();
		
		// The run loop
		getLogger().info("Starting the ROS Monitor loop...");
		try {
			while(true) { 

				/*
				 * This will build a JointPosition message with the current robot state.
				 * Set that message to be published and then publish it if there's a subscriber listening.
				 * Any other of the set methods for iiwa_msgs included in the published can be used at the same time,
				 * one just needs to build the message and set it to the publisher.
				 */
				currentPosition = helper.buildJointPosition(robot);
				publisher.setJointPosition(currentPosition);
				//published.setJointTorque(aJointTorqueMessage);
				//published.setCartesianRotation(aCartesianRotationMessage);
				publisher.publish();
				
				if (gravCompEnabled) {
					if (gravCompSwitched) {
						gravCompSwitched = false;
						getLogger().warn("Enabling gravity compensation");
						controlMode.setStiffnessForAllJoints(0);
						controlMode.setDampingForAllJoints(0.7);
						runtime.changeControlModeSettings(controlMode);
					}
					runtime.setDestination(robot.getCurrentJointPosition());
				} else {
					if (gravCompSwitched) {
						gravCompSwitched = false;
						getLogger().warn("Disabling gravity compensation");
						controlMode.setStiffnessForAllJoints(1500);
						runtime.changeControlModeSettings(controlMode);
						runtime.setDestination(robot.getCurrentJointPosition());
						robot.moveAsync(motion);
					}
				}
			} 
		}
		catch (InterruptedException e) {
			getLogger().info("ROS loop aborted. " + e.toString());
		} finally {
			if (nodeExecutor != null) {
				nodeExecutor.shutdownNodeMain(publisher);
				nodeExecutor.shutdownNodeMain(configuration);
				if (debug)getLogger().info("ROS Node terminated.");
			}
			getLogger().info("ROS loop has ended. Application terminated.");
		}
	}

	@Override
	public void dispose() {
		// The Publisher node is killed.
		if (nodeExecutor != null) {
			nodeExecutor.shutdownNodeMain(publisher);
			nodeExecutor.shutdownNodeMain(configuration);
			getLogger().info("ROS nodes have been terminated by Garbage Collection.");
		}
		super.dispose();
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		ROSMonitor app = new ROSMonitor();
		app.runApplication();
	}
}
