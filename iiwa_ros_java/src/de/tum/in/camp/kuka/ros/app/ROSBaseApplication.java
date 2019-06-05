/**
 * Copyright (C) 2016 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
 * Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided
 * that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
 * following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
 * the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package de.tum.in.camp.kuka.ros.app;

import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import javax.annotation.PostConstruct;
// import javax.inject.Inject;

import org.ros.address.BindAddress;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.time.NtpTimeProvider;

import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
// import com.kuka.generated.ioAccess.MediaFlangeIOGroup; // MEDIAFLANGEIO
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplicationState;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;

import de.tum.in.camp.kuka.ros.ActionServerThread;
import de.tum.in.camp.kuka.ros.ActiveToolThread;
import de.tum.in.camp.kuka.ros.AddressGenerator;
import de.tum.in.camp.kuka.ros.CommandTypes.CommandType;
import de.tum.in.camp.kuka.ros.Logger.Level;
import de.tum.in.camp.kuka.ros.ControlModeHandler;
import de.tum.in.camp.kuka.ros.GoalReachedEventListener;
import de.tum.in.camp.kuka.ros.Configuration;
import de.tum.in.camp.kuka.ros.MoveAsyncErrorHandler;
import de.tum.in.camp.kuka.ros.PublisherThread;
import de.tum.in.camp.kuka.ros.ActiveTool;
import de.tum.in.camp.kuka.ros.SpeedLimits;
import de.tum.in.camp.kuka.ros.iiwaActionServer;
import de.tum.in.camp.kuka.ros.iiwaPublisher;
import de.tum.in.camp.kuka.ros.Logger;

/*
 * Base application for all ROS-Sunrise applications. Manages lifetime of ROS Nodes, NTP synchronization,
 * loading the configuration from the ROS parameter server, attaching the Sunrise tool specified in the
 * configuration, and publishing the current state of the robot.
 */
public abstract class ROSBaseApplication extends RoboticsAPIApplication {

  protected LBR robot = null;
  protected Tool tool = null;
  protected String toolFrameID = "";
  protected static final String toolFrameIDSuffix = "_link_ee";

  protected SmartServo motion = null;
  protected SmartServoLIN linearMotion = null;
  protected ControlModeHandler controlModeHandler = null;
  protected Lock controlModeLock = new ReentrantLock();
  protected CommandType lastCommandType = CommandType.SMART_SERVO_JOINT_POSITION;
  protected GoalReachedEventListener handler = null;

  // Tool frame.
  protected ObjectFrame toolFrame = null;
  // Active frame for sending and receiving Cartesian positions.
  protected ObjectFrame endpointFrame = null;

  protected boolean initSuccessful = false;
  protected boolean running = true;

  // ROS Nodes.
  protected Configuration configuration = null;

  protected AddressGenerator addressGenerator = new AddressGenerator();

  // MEDIAFLANGEIO
  // @Inject
  // protected MediaFlangeIOGroup mediaFlange;

  protected iiwaPublisher publisher = null;
  PublisherThread publisherThread = null;
  Timer publisherTimer = null;

  protected iiwaActionServer actionServer = null;
  ActionServerThread actionServerThread = null;
  Timer actionServerTimer = null;

  // Active tool, you can replace this with a tool you are using that supports ROS messages.
  protected ActiveTool rosTool = null;
  // Example available at https://github.com/exo-core/iiwa_stack_tools
  // @Inject protected ROSZimmerR840 rosTool;

  ActiveToolThread activeToolThread = null;
  Timer activeToolTimer = null;

  // ROS Configuration and Node execution objects.
  protected NodeConfiguration configurationNodeConfiguration = null;
  protected NodeConfiguration publisherNodeConfiguration = null;
  protected NodeConfiguration actionServerNodeConfiguration = null;
  protected NodeMainExecutor nodeMainExecutor = null;

  // Configurable Toolbars.
  protected List<IUserKeyBar> generalKeyBars = new ArrayList<IUserKeyBar>();
  protected List<IUserKey> generalKeys = new ArrayList<IUserKey>();
  protected List<IUserKeyListener> generalKeyLists = new ArrayList<IUserKeyListener>();

  protected JointImpedanceControlMode handGuidanceControlMode;
  private IUserKeyBar handGuidanceKeybar;
  private IUserKey handGuidanceKey;
  private IUserKeyListener handGuidanceKeyList;
  private boolean handGuidanceEnabled = false;
  private boolean handGuidanceSwitched = false;

  protected abstract void configureNodes();

  protected abstract void addNodesToExecutor(NodeMainExecutor nodeExecutor);

  protected abstract void initializeApp();

  protected abstract void beforeControlLoop();

  protected abstract void controlLoop();

  @PostConstruct
  public void initialize() {
    // Get the robot instance.
    robot = getContext().getDeviceFromType(LBR.class);

    // Standard configuration.
    configuration = new Configuration(getApplicationData());
    publisher = new iiwaPublisher(robot, configuration.getRobotName(), configuration.getTimeProvider());
    actionServer = new iiwaActionServer(robot, configuration);

    // Get the Sunrise Logger and set its log level.
    Logger.setSunriseLogger(getLogger());
    if (configuration.getDebugOutputEnabled()) {
      Logger.setLogLevel(Level.DEBUG);
    }
    else {
      Logger.setLogLevel(Level.INFO);
    }

    // ROS initialization.
    try {
      configurationNodeConfiguration = configureNode("/iiwa_configuration", addressGenerator.getNewAddress(),
          addressGenerator.getNewAddress());
      publisherNodeConfiguration = configureNode("/iiwa_publisher", addressGenerator.getNewAddress(),
          addressGenerator.getNewAddress());
      actionServerNodeConfiguration = configureNode("/iiwa_action_server", addressGenerator.getNewAddress(),
          addressGenerator.getNewAddress());

      // Additional configuration needed in subclasses.
      configureNodes();
    }
    catch (Exception e) {
      Logger.error("Node Configuration failed. Please check the ROS Master IP in the Sunrise Process Data.");
      Logger.error(e.toString());
      e.printStackTrace();
      return;
    }

    try {
      nodeMainExecutor = DefaultNodeMainExecutor.newDefault();

      Logger.debug("Initializing ROS tool.");
      if (rosTool != null) {
        rosTool.initialize(configuration, nodeMainExecutor);
      }

      // Start the Publisher node with the set up configuration.
      nodeMainExecutor.execute(configuration, configurationNodeConfiguration);
      nodeMainExecutor.execute(publisher, publisherNodeConfiguration);
      nodeMainExecutor.execute(actionServer, actionServerNodeConfiguration);

      // Additional Nodes from subclasses.
      addNodesToExecutor(nodeMainExecutor);

      Logger.debug("ROS Node Executor initialized.");
    }
    catch (Exception e) {
      Logger.error("ROS Node Executor initialization failed.");
      Logger.error(e.toString());
      e.printStackTrace();
      return;
    }

    // END of ROS initialization.

    handGuidanceControlMode = new JointImpedanceControlMode(robot.getJointCount());
    handGuidanceKeybar = getApplicationUI().createUserKeyBar("Hand Guide");
    handGuidanceKeyList = new IUserKeyListener() {
      @Override
      public void onKeyEvent(IUserKey key, com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent event) {
        controlModeLock.lock();
        if (event == UserKeyEvent.FirstKeyDown) {
          handGuidanceEnabled = true;
          handGuidanceSwitched = true;
        }
        else if (event == UserKeyEvent.SecondKeyDown) {
          handGuidanceEnabled = false;
          handGuidanceSwitched = true;
        }
        controlModeLock.unlock();
      };
    };

    handGuidanceKey = handGuidanceKeybar.addDoubleUserKey(0, handGuidanceKeyList, true);
    handGuidanceKey.setText(UserKeyAlignment.TopMiddle, "ON");
    handGuidanceKey.setText(UserKeyAlignment.BottomMiddle, "OFF");
    handGuidanceKeybar.publish();

    // Register MoveAsyncErrorHandler
    getApplicationControl().registerMoveAsyncErrorHandler(new MoveAsyncErrorHandler(publisher, actionServer));

    // Additional initialization from subclasses.
    initializeApp();

    initSuccessful = true; // We cannot throw here.
  }

  public void run() {
    if (!initSuccessful) { throw new RuntimeException("Could not initialize the RoboticApplication successfully."); }

    try {
      Logger.info("Waiting for ROS Master to connect at " + configuration.getMasterIp());
      configuration.waitForInitialization();
      Logger.info("ROS Master is connected!");
    }
    catch (InterruptedException e1) {
      e1.printStackTrace();
      return;
    }

    Logger.info("Using time provider: " + configuration.getTimeProvider().getClass().getSimpleName());

    // Configurable toolbars to publish events on topics.
    configuration.setupToolbars(getApplicationUI(), publisher, generalKeys, generalKeyLists, generalKeyBars);

    // Tool to attach, robot's flange will be used if no tool has been defined.
    String toolFromConfig = configuration.getToolName();
    if (!toolFromConfig.isEmpty()) {
      Logger.info("Attaching tool " + toolFromConfig);
      tool = (Tool) getApplicationData().createFromTemplate(toolFromConfig);
      tool.attachTo(robot.getFlange());
      toolFrameID = toolFromConfig + toolFrameIDSuffix;
      toolFrame = tool.getFrame("/" + toolFrameID);
    }
    else {
      Logger.info("No tool attached. Using robot's flange.");
      toolFrameID = configuration.getRobotName() + toolFrameIDSuffix;
      toolFrame = robot.getFlange();
    }

    String endpointFrameFromConfig = configuration.getEndpointFrame();
    if (endpointFrameFromConfig.isEmpty()) {
      endpointFrame = toolFrame;
    }
    else if (endpointFrameFromConfig.equals(configuration.getRobotName() + toolFrameIDSuffix)) {
      endpointFrame = robot.getFlange();
    }
    else {
      try {
        Logger.info("Setting endpoint frame " + endpointFrameFromConfig);
        endpointFrame = tool.getFrame(endpointFrameFromConfig);
      }
      catch (Exception e) {
        Logger.error("Error while setting endpoint frame to \"" + endpointFrameFromConfig + "\": " + e.getMessage());
      }
    }

    // Load speed limits from configuration.
    SpeedLimits.init(configuration, getApplicationControl());

    // TODO: check this.
    controlModeHandler = new ControlModeHandler(robot, tool, endpointFrame, publisher, actionServer, configuration);

    motion = controlModeHandler.createSmartServoMotion();

    // Initialize motion.
    endpointFrame.moveAsync(motion);

    // Hook the GoalReachedEventHandler.
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

      actionServerThread = new ActionServerThread(actionServer);
      actionServerTimer = new Timer();
      actionServerTimer.scheduleAtFixedRate(actionServerThread, 0, 500);

      if (rosTool != null) {
        activeToolThread = new ActiveToolThread(rosTool);
        activeToolTimer = new Timer();
        activeToolTimer.scheduleAtFixedRate(activeToolThread, 0, 100);
      }

      while (running) {
        controlModeLock.lock();
        fakeHandGuidanceMode();
        if (!handGuidanceEnabled && !handGuidanceSwitched) {
          // Perform control loop specified by subclass.
          controlLoop();
        }
        controlModeLock.unlock();
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

  private void cleanup() {
    running = false;
    disposeTimer(publisherTimer);
    disposeTimer(actionServerTimer);
    disposeTimer(activeToolTimer);
    if (nodeMainExecutor != null) {
      Logger.info("Stopping ROS nodes...");
      nodeMainExecutor.shutdown();
      nodeMainExecutor.getScheduledExecutorService().shutdownNow();
    }
    Logger.info("Stopped ROS nodes.");
  }

  private void disposeTimer(Timer timer) {
    if (timer != null) {
      timer.cancel();
      timer.purge();
    }
  }

  /**
   * Returns if the control loop of the application is currently active.
   */
  public boolean isRunning() {
    return running;
  }

  /**
   * Utility function to create the configuration of a ROS node.
   * 
   * @param nodeName name of the ROS node.
   * @param tcpPort port to use for the ROS TCP connection - cannot be the same as the XML one.
   * @param xmlPort port to use for the ROS XML connection - cannot be the same as the TCP one.
   * @return
   * @throws URISyntaxException
   */
  protected NodeConfiguration configureNode(String nodeName, int tcpPort, int xmlPort) throws URISyntaxException {
    NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(configuration.getRobotIp());
    nodeConfiguration.setTimeProvider(configuration.getTimeProvider());
    nodeConfiguration.setNodeName(configuration.getRobotName() + nodeName);
    nodeConfiguration.setMasterUri(configuration.getMasterURI());
    nodeConfiguration.setTcpRosBindAddress(BindAddress.newPublic(tcpPort));
    nodeConfiguration.setXmlRpcBindAddress(BindAddress.newPublic(xmlPort));
    return nodeConfiguration;
  }

  /**
   * 
   */
  private void fakeHandGuidanceMode() {
    if (handGuidanceEnabled) {
      if (handGuidanceSwitched) {
        handGuidanceSwitched = false;
        Logger.warn("Enabling Fake Hand Guiding Mode.");
        handGuidanceControlMode.setStiffness(2, 2, 2, 2, 2, 0, 0);
        handGuidanceControlMode.setDampingForAllJoints(0.7);
        if (lastCommandType == CommandType.SMART_SERVO_CARTESIAN_POSE_LIN) {
          linearMotion = controlModeHandler.changeSmartServoControlMode(linearMotion, handGuidanceControlMode);
        }
        else {
          motion = controlModeHandler.changeSmartServoControlMode(motion, handGuidanceControlMode);
        }
      }
    }
    else {
      if (handGuidanceSwitched) {
        handGuidanceSwitched = false;
        Logger.warn("Disabling Fake Hand Guiding Mode.");
        if (lastCommandType == CommandType.SMART_SERVO_CARTESIAN_POSE_LIN) {
          linearMotion = controlModeHandler.changeSmartServoControlMode(linearMotion, new PositionControlMode(true));
        }
        else {
          motion = controlModeHandler.changeSmartServoControlMode(motion, new PositionControlMode(true));
        }
      }
    }
  }
}