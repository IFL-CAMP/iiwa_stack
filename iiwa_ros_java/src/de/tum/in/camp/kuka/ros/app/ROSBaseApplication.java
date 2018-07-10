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

package de.tum.in.camp.kuka.ros.app;

import java.io.IOException;
import java.net.URI;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import org.ros.address.BindAddress;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.time.NtpTimeProvider;

import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplicationState;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;

import de.tum.in.camp.kuka.ros.ControlModeHandler;
import de.tum.in.camp.kuka.ros.GoalReachedEventListener;
import de.tum.in.camp.kuka.ros.Configuration;
import de.tum.in.camp.kuka.ros.iiwaPublisher;
import de.tum.in.camp.kuka.ros.configuration.FileBasedConfigurationProviderFactory;
import de.tum.in.camp.kuka.ros.configuration.IConfigurationProvider;
import de.tum.in.camp.kuka.ros.Logger;

/*
 * Base application for all ROS-Sunrise applications. 
 * Manages lifetime of ROS Nodes, NTP synchronization, loading the configuration from the ROS parameter server,
 * attaching the Sunrise tool specified in the configuration, and publishing the current state of the robot.
 */
public abstract class ROSBaseApplication extends RoboticsAPIApplication {

	protected LBR robot;
	protected Tool tool;
	protected String toolFrameID;
	protected static final String toolFrameIDSuffix = "_link_ee";
	protected ObjectFrame toolFrame;
	protected SmartServo motion;
	protected SmartServoLIN linearMotion;
	protected ControlModeHandler controlModeHandler;
	protected GoalReachedEventListener handler;

	protected boolean initSuccessful;
	protected boolean debug;
	protected boolean running;

	protected iiwaPublisher publisher;
	protected Configuration configuration;

	// ROS Configuration and Node execution objects.
	protected NodeConfiguration nodeConfPublisher;
	protected NodeConfiguration nodeConfConfiguration;
	protected NodeMainExecutor nodeMainExecutor;

	// Configurable Toolbars.
	protected List<IUserKeyBar> generalKeyBars = new ArrayList<IUserKeyBar>();
	protected List<IUserKey> generalKeys = new ArrayList<IUserKey>();
	protected List<IUserKeyListener> generalKeyLists = new ArrayList<IUserKeyListener>();

	protected abstract void configureNodes(URI uri);
	protected abstract void addNodesToExecutor(NodeMainExecutor nodeExecutor);
	protected abstract void initializeApp();
	protected abstract void beforeControlLoop();
	protected abstract void controlLoop();

	/*
	 * SmartServo control makes the control loop very slow
	 * These variables are used to run them every *decimation* times, 
	 * In order to balance the load, they alternate at *decimationCounter* % *decimation* == 0 and
	 * *decimationCounter* % *decimation* == *decimation* / 2
	 */

	// TODO : in config.txt or processData
	protected int decimationCounter = 0; 
	protected int controlDecimation = 8;

	/**
	 * Returns the configuration provider which should be used for the application.
	 * 
	 * The base implementation returns a configuration provider using the default config.txt  
	 * 
	 * @return Configuration provider
	 */
	protected IConfigurationProvider getConfigurationProvider() {
		try {
			return FileBasedConfigurationProviderFactory.createFromDefaultConfigFile();
		} catch (IOException e) {
			throw new RuntimeException("Could not create default configuration provider.", e);
		}
	}
	
	public void initialize() {
		Logger.setSunriseLogger(getLogger());
		
		robot = getContext().getDeviceFromType(LBR.class);

		// Standard configuration.
		Configuration.setConfigurationProvider(getConfigurationProvider());
		configuration = new Configuration();
		publisher = new iiwaPublisher(Configuration.getRobotName(), configuration);

		// ROS initialization.
		try {
			URI uri = new URI(Configuration.getMasterURI());

			nodeConfConfiguration = NodeConfiguration.newPublic(Configuration.getRobotIp());
			nodeConfConfiguration.setTimeProvider(configuration.getTimeProvider());
			nodeConfConfiguration.setNodeName(Configuration.getRobotName() + "/iiwa_configuration");
			nodeConfConfiguration.setMasterUri(uri);			
			nodeConfConfiguration.setTcpRosBindAddress(BindAddress.newPublic(30000));
			nodeConfConfiguration.setXmlRpcBindAddress(BindAddress.newPublic(30001));			
			
			nodeConfPublisher = NodeConfiguration.newPublic(Configuration.getRobotIp());
			nodeConfPublisher.setTimeProvider(configuration.getTimeProvider());
			nodeConfPublisher.setNodeName(Configuration.getRobotName() + "/iiwa_publisher");
			nodeConfPublisher.setMasterUri(uri);
			nodeConfPublisher.setTcpRosBindAddress(BindAddress.newPublic(30002));
			nodeConfPublisher.setXmlRpcBindAddress(BindAddress.newPublic(30003));

			// Additional configuration needed in subclasses.
			configureNodes(uri);
		}
		catch (Exception e) {
			if (debug) 
				Logger.info("Node Configuration failed. " + "Please check the ROS master IP in the Sunrise configuration.");
			Logger.info(e.toString());
			return;
		}

		try {
			// Start the Publisher node with the set up configuration.
			nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
			nodeMainExecutor.execute(publisher, nodeConfPublisher);
			nodeMainExecutor.execute(configuration, nodeConfConfiguration);

			// Additional Nodes from subclasses.
			addNodesToExecutor(nodeMainExecutor); 

			if (debug) 
				Logger.info("ROS Node Executor initialized.");
		}
		catch(Exception e) {
			if (debug) 
				Logger.info("ROS Node Executor initialization failed.");
			Logger.info(e.toString());
			return;
		}
		// END of ROS initialization.


		// Additional initialization from subclasses.
		initializeApp();

		initSuccessful = true;  // We cannot throw here.
	}
	
	public void run() {
		if (!initSuccessful) {
			throw new RuntimeException("Could not init the RoboticApplication successfully");
		}
		try {
			Logger.info("Waiting for ROS Master to connect... ");
			configuration.waitForInitialization();
			Logger.info("ROS Master is connected!");
		} catch (InterruptedException e1) {
			e1.printStackTrace();
			return;
		}

		Logger.info("Using time provider: " + configuration.getTimeProvider().getClass().getSimpleName());

		// Configurable toolbars to publish events on topics.
		configuration.setupToolbars(getApplicationUI(), publisher, generalKeys, generalKeyLists, generalKeyBars);

		// Tool to attach, robot's flange will be used if no tool has been defined.
		String toolFromConfig = configuration.getToolName();
		if (toolFromConfig != "") {
			Logger.info("Attaching tool " + toolFromConfig);
			tool = (Tool)getApplicationData().createFromTemplate(toolFromConfig);
			tool.attachTo(robot.getFlange());
			toolFrameID = toolFromConfig + toolFrameIDSuffix;
			toolFrame = tool.getFrame("/" + toolFrameID);
		} else {
			Logger.info("No tool attached. Using flange.");
			toolFrameID = Configuration.getRobotName() + toolFrameIDSuffix;
			toolFrame = robot.getFlange();
		}

		controlModeHandler = new ControlModeHandler(robot, tool, toolFrame, publisher, configuration);
		motion = controlModeHandler.createSmartServoMotion();
		// Publish joint state?
		publisher.setPublishJointStates(configuration.getPublishJointStates());

		// Initialize motion.
		toolFrame.moveAsync(motion);
		// Hook the GoalReachedEventHandler
		motion.getRuntime().setGoalReachedEventHandler(handler);

		if (configuration.getTimeProvider() instanceof org.ros.time.NtpTimeProvider) {
			((NtpTimeProvider) configuration.getTimeProvider()).startPeriodicUpdates(100, TimeUnit.MILLISECONDS); // TODO: update time as param
		}

		// Run what is needed before the control loop in the subclasses.
		beforeControlLoop();

		running = true;

		// The run loop
		Logger.info("Starting the ROS control loop...");
		try {
			while(running) { 
				decimationCounter++;

				// This will publish the current robot state on the various ROS topics.
				publisher.publishCurrentState(robot, motion, toolFrame);

				if ((decimationCounter % controlDecimation) == 0)
					controlLoop();  // Perform control loop specified by subclass
			} 
		}
		catch (Exception e) {
			Logger.info("ROS control loop aborted. " + e.toString());
		} finally {
			cleanup();
			Logger.info("ROS control loop has ended. Application terminated.");
		}
	}
	
	@Override
	public void dispose() {
		configuration.cleanup();
		cleanup();
		super.dispose();
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
			Logger.info("Stopping ROS nodes");
			nodeMainExecutor.shutdown();	
			nodeMainExecutor.getScheduledExecutorService().shutdownNow();
		}
		Logger.info("Stopped ROS nodes");
	}
}
