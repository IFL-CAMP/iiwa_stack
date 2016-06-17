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
import java.net.URI;
import java.util.ArrayList;
import java.util.List;

import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.time.NtpTimeProvider;

import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplicationState;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;

/*
 * Base application for all ROS-Sunrise applications. 
 * Manages lifetime of ROS Nodes, NTP synchronization, loading the configuration from the ROS parameter server,
 * attaching the Sunrise tool specified in the configuration, and publishing the current state of the robot.
 */
public abstract class ROSBaseApplication extends RoboticsAPIApplication {

	protected LBR robot;
	protected Tool tool;
	protected String toolFrameID;
	protected ObjectFrame toolFrame;
	protected SmartServo motion;
	
	protected boolean initSuccessful = false;
	protected boolean debug = false;
	protected boolean running = true;
	
	protected iiwaPublisher publisher; //< IIWARos Publisher.
	protected iiwaConfiguration configuration; //< Configuration via parameters and services.

	// ROS Configuration and Node execution objects.
	protected NodeConfiguration nodeConfPublisher;
	protected NodeConfiguration nodeConfConfiguration;
	protected NodeMainExecutor nodeMainExecutor;

	// configurable toolbars
	protected List<IUserKeyBar> generalKeyBars = new ArrayList<IUserKeyBar>();
	protected List<IUserKey> generalKeys = new ArrayList<IUserKey>();
	protected List<IUserKeyListener> generalKeyLists = new ArrayList<IUserKeyListener>();
	
	protected abstract void configureNodes(URI uri);
	protected abstract void addNodesToExecutor(NodeMainExecutor nodeExecutor);
	protected abstract void initializeApp();
	protected abstract void beforeControlLoop();
	protected abstract void controlLoop();
	
	/*
	 * Performing NTP synchronization and SmartServo control makes the control loop very slow
	 * These variables are used to run them every *decimation* times, 
	 * In order to balance the load, they alternate at *decimationCounter* % *decimation* == 0 and
	 * *decimationCounter* % *decimation* == *decimation* / 2
	 */
	 
	protected int decimationCounter = 0; 
	protected int controlDecimation = 8;
	protected int ntpDecimation = 1024;
	
	
	public void initialize() {
		robot = getContext().getDeviceFromType(LBR.class);
		
		// Standard stuff
		configuration = new iiwaConfiguration();
		publisher = new iiwaPublisher(iiwaConfiguration.getRobotName());
	
		// ROS initialization
		try {
			URI uri = new URI(iiwaConfiguration.getMasterURI());
			
			nodeConfConfiguration = NodeConfiguration.newPublic(iiwaConfiguration.getRobotIp());
			nodeConfConfiguration.setTimeProvider(iiwaConfiguration.getTimeProvider());
			nodeConfConfiguration.setNodeName(iiwaConfiguration.getRobotName() + "/iiwa_configuration");
			nodeConfConfiguration.setMasterUri(uri);
			
			nodeConfPublisher = NodeConfiguration.newPublic(iiwaConfiguration.getRobotIp());
			nodeConfPublisher.setTimeProvider(iiwaConfiguration.getTimeProvider());
			nodeConfPublisher.setNodeName(iiwaConfiguration.getRobotName() + "/iiwa_publisher");
			nodeConfPublisher.setMasterUri(uri);
			
			configureNodes(uri);
		}
		catch (Exception e) {
			if (debug) getLogger().info("Node Configuration failed; "
					+ "please check the ROS master IP in the Sunrise app source code");
			getLogger().info(e.toString());
			return;
		}

		try {
			// Start the Publisher node with the set up configuration.
			nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
			nodeMainExecutor.execute(publisher, nodeConfPublisher);
			nodeMainExecutor.execute(configuration, nodeConfConfiguration);
			
			addNodesToExecutor(nodeMainExecutor);  // add Nodes from subclass
			
			if (debug) 
				getLogger().info("ROS Node Executor initialized.");
		}
		catch(Exception e) {
			if (debug) 
				getLogger().info("ROS Node Executor initialization failed.");
			
			getLogger().info(e.toString());
			return;
		}
		
		initializeApp();  // further initialization as specified by subclass
		
		initSuccessful = true;  // we cannot throw here
	}

	public void run() {
		if (!initSuccessful) {
			throw new RuntimeException("Could not init the RoboticApplication successfully");
		}
		
		try {
			getLogger().info("Waiting for configuration... ");
			configuration.waitForInitialization();
			getLogger().info("done!");
		} catch (InterruptedException e1) {
			e1.printStackTrace();
			return;
		}
		
		getLogger().info("using time provider: " + iiwaConfiguration.getTimeProvider().getClass().getSimpleName());

		motion = new SmartServo(robot.getCurrentJointPosition());
		motion.setMinimumTrajectoryExecutionTime(20e-3);
		motion.setJointVelocityRel(configuration.getDefaultRelativeJointSpeed());
		motion.setTimeoutAfterGoalReach(300);
		
		// Configurable toolbars to publish events on topics
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
		
		// Publish joint state?
		publisher.setPublishJointStates(configuration.getPublishJointStates());
		
		if (!SmartServo.validateForImpedanceMode(robot))
			getLogger().error("Too much external torque on the robot! Is it a singular position?");
		
		robot.moveAsync(motion);
		
		beforeControlLoop();
		
		// The run loop
		getLogger().info("Starting the ROS control loop...");
		try {
			while(running) { 
				decimationCounter++;
				
				if ((decimationCounter % ntpDecimation) == (int)(ntpDecimation/2) 
						&& iiwaConfiguration.getTimeProvider() instanceof org.ros.time.NtpTimeProvider) {
					((NtpTimeProvider) iiwaConfiguration.getTimeProvider()).updateTime();
				}
				
				// This will publish the current robot state on the various ROS topics.
				publisher.publishCurrentState(robot, motion, toolFrame);
				
				if ((decimationCounter % controlDecimation) == 0)
					controlLoop();  // Perform control loop specified by subclass
			} 
		}
		catch (Exception e) {
			getLogger().info("ROS control loop aborted. " + e.toString());
		} finally {
			cleanup();
			getLogger().info("ROS control loop has ended. Application terminated.");
		}
	}
	
	@Override 
	public void onApplicationStateChanged(RoboticsAPIApplicationState state) {
		if (state == RoboticsAPIApplicationState.STOPPING) {
			running = false;
		}
		super.onApplicationStateChanged(state);
	};
	
	void cleanup() {
		running = false;
		if (nodeMainExecutor != null) {
			getLogger().info("Stopping ROS nodes");
			nodeMainExecutor.shutdown();	
			nodeMainExecutor.getScheduledExecutorService().shutdownNow();
		}
		getLogger().info("Stopped ROS nodes");
	}
}
