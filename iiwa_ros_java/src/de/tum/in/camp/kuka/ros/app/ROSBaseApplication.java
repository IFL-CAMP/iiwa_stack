/**  
 * Copyright (C) 2016-2017 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
 * Technische Universit�t M�nchen
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakult�t f�r Informatik / I16, Boltzmannstra�e 3, 85748 Garching bei M�nchen, Germany
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

import java.net.URI;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import javax.annotation.PostConstruct;
import javax.inject.Inject;

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
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;

import de.tum.in.camp.kuka.ros.AddressGeneration;
import de.tum.in.camp.kuka.ros.ControlModeHandler;
import de.tum.in.camp.kuka.ros.GoalReachedEventListener;
import de.tum.in.camp.kuka.ros.Configuration;
import de.tum.in.camp.kuka.ros.ROSTool;
import de.tum.in.camp.kuka.ros.SpeedLimits;
import de.tum.in.camp.kuka.ros.iiwaActionServer;
import de.tum.in.camp.kuka.ros.iiwaPublisher;
import de.tum.in.camp.kuka.ros.Logger;

/*
 * Base application for all ROS-Sunrise applications. 
 * Manages lifetime of ROS Nodes, NTP synchronization, loading the configuration from the ROS parameter server,
 * attaching the Sunrise tool specified in the configuration, and publishing the current state of the robot.
 */
public abstract class ROSBaseApplication extends RoboticsAPIApplication {

	protected LBR robot;
	protected Tool tool;
	protected String robotBaseFrameID;
	protected static final String robotBaseFrameIDSuffix = "_link_0";
	protected String toolFrameID;
	protected static final String toolFrameIDSuffix = "_link_ee";

	protected SmartServo motion;
	protected SmartServoLIN linearMotion;
	protected ControlModeHandler controlModeHandler;
	protected GoalReachedEventListener handler;
	
	// Robot base
	protected ObjectFrame worldFrame;
	// Flange
	protected ObjectFrame flangeFrame;
	// Tool
	protected ObjectFrame toolFrame;
	// Active frame for sending and receiving Cartesian positions
	protected ObjectFrame endpointFrame;

	protected boolean initSuccessful;
	protected boolean debug;
	protected boolean running;

	protected iiwaPublisher publisher;
	protected iiwaActionServer actionServer;
	protected Configuration configuration;
	
	// Tool
	protected ROSTool rosTool = null;
	// TODO: Replace this with the tool you are using, e.g.:
	// @Inject protected SchunkEGN100 rosTool;

	// ROS Configuration and Node execution objects.
	protected NodeConfiguration nodeConfPublisher;
	protected NodeConfiguration nodeConfActionServer;
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

	@PostConstruct
	public void initialize() {
		Logger.setSunriseLogger(getLogger());
		
		robot = getContext().getDeviceFromType(LBR.class);

		// Standard configuration.
		configuration = new Configuration();
		publisher = new iiwaPublisher(Configuration.getRobotName(), configuration);
		actionServer = new iiwaActionServer(robot, Configuration.getRobotName(), configuration);
		robotBaseFrameID = Configuration.getRobotName()+robotBaseFrameIDSuffix;

		// ROS initialization.
		try {
			URI uri = new URI(Configuration.getMasterURI());

			nodeConfConfiguration = NodeConfiguration.newPublic(Configuration.getRobotIp());
			nodeConfConfiguration.setTimeProvider(configuration.getTimeProvider());
			nodeConfConfiguration.setNodeName(Configuration.getRobotName() + "/iiwa_configuration");
			nodeConfConfiguration.setMasterUri(uri);			
			nodeConfConfiguration.setTcpRosBindAddress(BindAddress.newPublic(AddressGeneration.getNewAddress()));
			nodeConfConfiguration.setXmlRpcBindAddress(BindAddress.newPublic(AddressGeneration.getNewAddress()));

			nodeConfActionServer = NodeConfiguration.newPublic(Configuration.getRobotIp());
			nodeConfActionServer.setTimeProvider(configuration.getTimeProvider());
			nodeConfActionServer.setNodeName(Configuration.getRobotName() + "/iiwa_action_server");
			nodeConfActionServer.setMasterUri(uri);	
			nodeConfActionServer.setTcpRosBindAddress(BindAddress.newPublic(AddressGeneration.getNewAddress()));
			nodeConfActionServer.setXmlRpcBindAddress(BindAddress.newPublic(AddressGeneration.getNewAddress()));
			
			nodeConfPublisher = NodeConfiguration.newPublic(Configuration.getRobotIp());
			nodeConfPublisher.setTimeProvider(configuration.getTimeProvider());
			nodeConfPublisher.setNodeName(Configuration.getRobotName() + "/iiwa_publisher");
			nodeConfPublisher.setMasterUri(uri);
			nodeConfPublisher.setTcpRosBindAddress(BindAddress.newPublic(AddressGeneration.getNewAddress()));
			nodeConfPublisher.setXmlRpcBindAddress(BindAddress.newPublic(AddressGeneration.getNewAddress()));

			// Additional configuration needed in subclasses.
			configureNodes(uri);
		}
		catch (Exception e) {
			if (debug) 
				Logger.info("Node Configuration failed. " + "Please check the ROS master IP in the Sunrise configuration.");
			Logger.error(e.toString());
			e.printStackTrace();
			return;
		}

		try {
			nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
		
			if (debug) {
				Logger.info("Initializing ROS tool.");
			}
			if (rosTool != null) {
				rosTool.initialize(configuration, nodeMainExecutor);
			}
			
			// Start the Publisher node with the set up configuration.
			nodeMainExecutor.execute(actionServer, nodeConfActionServer);
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
			Logger.error(e.toString());
			e.printStackTrace();
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
			Logger.info("Waiting for ROS Master to connect at " + Configuration.getMasterIp());
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
		worldFrame = World.Current.getRootFrame();
		World.Current.getFrame("");
		flangeFrame = robot.getFlange();
		String toolFromConfig = configuration.getToolName();
		String endpointFrameFromConfig = configuration.getEndpointFrame();
		
		if (!toolFromConfig.isEmpty()) {
			Logger.info("Attaching tool " + toolFromConfig);
			tool = (Tool)getApplicationData().createFromTemplate(toolFromConfig);
			tool.attachTo(robot.getFlange());
			toolFrameID = toolFromConfig + toolFrameIDSuffix;
			toolFrame = tool.getFrame("/" + toolFrameID);
		} else {
			Logger.info("No tool attached. Using flange.");
			toolFrameID = Configuration.getRobotName() + toolFrameIDSuffix;
			toolFrame = flangeFrame;
		}
		
		if (endpointFrameFromConfig.isEmpty()) {
			endpointFrame = toolFrame;
		}
		else if (endpointFrameFromConfig.equals(Configuration.getRobotName()+toolFrameIDSuffix)) {
			endpointFrame = flangeFrame;
		}
		else {
			Logger.info("Setting endpoint frame " + endpointFrameFromConfig);
			endpointFrame = tool.getFrame(endpointFrameFromConfig);
		}

		// Load speed limits from configuration
		SpeedLimits.init(configuration);
		
		controlModeHandler = new ControlModeHandler(robot, tool, endpointFrame, publisher, actionServer, configuration);

		motion = controlModeHandler.createSmartServoMotion();

		// Initialize motion.
		endpointFrame.moveAsync(motion);
	
		// Hook the GoalReachedEventHandler
		motion.getRuntime().setGoalReachedEventHandler(handler);

		// Publish joint state?
		publisher.setPublishJointStates(configuration.getPublishJointStates());
		
    // Initialize the ntp updates if necessary.
    if (configuration.getTimeProvider() instanceof org.ros.time.NtpTimeProvider) {
      // TODO: update time as parameter.
      ((NtpTimeProvider) configuration.getTimeProvider()).startPeriodicUpdates(100, TimeUnit.MILLISECONDS);
    }

    // Run what is needed before the control loop in the subclasses.
    beforeControlLoop();

    running = true;

    // The run loop
    Logger.info("Starting the ROS control loop...");
    try {

      publisherThread = new PublisherThread(publisher, endpointFrame);
      publisherTimer = new Timer();
      publisherTimer.scheduleAtFixedRate(publisherThread, 0, 1);

      while (running) {
        decimationCounter++;
        if (rosTool != null) {
          rosTool.publishCurrentState();
        }
        actionServer.publishCurrentState();

        // Perform control loop specified by subclass.
        if ((decimationCounter % controlDecimation) == 0) controlLoop();
      }
    }
    catch (Exception e) {
      dispose();
      Logger.info("ROS control loop aborted. " + e.toString());
      e.printStackTrace();
    }
    finally {
      Logger.info("ROS control loop has ended. The application will be terminated.");
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
    if (publisherTimer != null) {
      publisherTimer.cancel();
      publisherTimer.purge();
    }
    if (nodeMainExecutor != null) {
      Logger.info("Stopping ROS nodes...");
      nodeMainExecutor.shutdown();
      nodeMainExecutor.getScheduledExecutorService().shutdownNow();
    }
    Logger.info("Stopped ROS nodes.");
  }
	
	public boolean isRunning() {
		return running;
	}
}
