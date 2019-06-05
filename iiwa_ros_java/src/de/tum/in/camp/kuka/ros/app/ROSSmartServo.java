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

import geometry_msgs.PoseStamped;
import iiwa_msgs.ConfigureControlModeRequest;
import iiwa_msgs.ConfigureControlModeResponse;
import iiwa_msgs.JointPosition;
import iiwa_msgs.MoveAlongSplineActionGoal;
import iiwa_msgs.MoveToCartesianPoseActionGoal;
import iiwa_msgs.MoveToJointPositionActionGoal;
import iiwa_msgs.RedundancyInformation;
import iiwa_msgs.SetSmartServoLinSpeedLimitsRequest;
import iiwa_msgs.SetSmartServoLinSpeedLimitsResponse;
import iiwa_msgs.SetWorkpieceRequest;
import iiwa_msgs.SetWorkpieceResponse;
import iiwa_msgs.SetEndpointFrameRequest;
import iiwa_msgs.SetEndpointFrameResponse;
import iiwa_msgs.Spline;
import iiwa_msgs.TimeToDestinationRequest;
import iiwa_msgs.TimeToDestinationResponse;

import java.net.URISyntaxException;
import java.util.List;

import org.ros.exception.ServiceException;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.service.ServiceResponseBuilder;

import com.kuka.roboticsAPI.geometricModel.SceneGraphObject;
import com.kuka.roboticsAPI.geometricModel.Workpiece;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

import de.tum.in.camp.kuka.ros.CommandTypes.CommandType;
import de.tum.in.camp.kuka.ros.Logger;
import de.tum.in.camp.kuka.ros.Motions;
import de.tum.in.camp.kuka.ros.SpeedLimits;
import de.tum.in.camp.kuka.ros.UnsupportedControlModeException;
import de.tum.in.camp.kuka.ros.iiwaActionServer.Goal;
import de.tum.in.camp.kuka.ros.iiwaSubscriber;

/*
 * This application allows to command the robot using SmartServo motions.
 */
public class ROSSmartServo extends ROSBaseApplication {

  // IIWA ROS Subscriber.
  private iiwaSubscriber subscriber;
  // Configuration of the subscriber ROS node.
  private NodeConfiguration subscriberNodeConfiguration;

  private Motions motions;
  private String robotBaseFrameID = "";
  private static final String robotBaseFrameIDSuffix = "_link_0";

  @Override
  protected void configureNodes() {
    // Configuration for the Subscriber.
    try {
      subscriberNodeConfiguration = configureNode("/iiwa_subscriber", addressGenerator.getNewAddress(),
          addressGenerator.getNewAddress());
    }
    catch (URISyntaxException e) {
      Logger.error(e.toString());
    }
  }

  @Override
  protected void addNodesToExecutor(NodeMainExecutor nodeMainExecutor) {
    subscriber = new iiwaSubscriber(robot, configuration.getRobotName(), configuration.getTimeProvider(),
        configuration.getEnforceMessageSequence());

    // Configure the callback for the SmartServo service inside the subscriber
    // class.
    subscriber
        .setConfigureControlModeCallback(new ServiceResponseBuilder<iiwa_msgs.ConfigureControlModeRequest, iiwa_msgs.ConfigureControlModeResponse>() {
          @Override
          public void build(ConfigureControlModeRequest req, ConfigureControlModeResponse res) throws ServiceException {
            controlModeLock.lock();
            try {
              // TODO: reduce code duplication
              if (lastCommandType == CommandType.SMART_SERVO_CARTESIAN_POSE_LIN) {
                // We can just change the parameters if the control strategy is the same.
                if (controlModeHandler.isSameControlMode(linearMotion.getMode(), req.getControlMode())) {
                  // If the request was for PositionControlMode and we are already there, do nothing.
                  if (!(linearMotion.getMode() instanceof PositionControlMode)) {
                    linearMotion.getRuntime().changeControlModeSettings(controlModeHandler.buildMotionControlMode(req));
                  }
                }
                else {
                  linearMotion = controlModeHandler.changeSmartServoControlMode(linearMotion, req);
                }
              }
              else {
                // We can just change the parameters if the control strategy is the same.
                if (controlModeHandler.isSameControlMode(motion.getMode(), req.getControlMode())) {
                  // If the request was for PositionControlMode and we are already there, do nothing.
                  if (!(motion.getMode() instanceof PositionControlMode)) {
                    motion.getRuntime().changeControlModeSettings(controlModeHandler.buildMotionControlMode(req));
                  }
                }
                else {
                  motion = controlModeHandler.changeSmartServoControlMode(motion, req);
                }
              }

              res.setSuccess(true);
              controlModeHandler.setLastSmartServoRequest(req);
            }
            catch (Exception e) {
              res.setSuccess(false);
              if (e.getMessage() != null) {
                res.setError(e.getClass().getName() + ": " + e.getMessage());
              }
              else {
                res.setError("because I hate you :)");
              }
              return;
            }
            finally {
              controlModeLock.unlock();
            }
          }
        });

    // TODO: doc
    subscriber
        .setTimeToDestinationCallback(new ServiceResponseBuilder<iiwa_msgs.TimeToDestinationRequest, iiwa_msgs.TimeToDestinationResponse>() {

          @Override
          public void build(TimeToDestinationRequest req, TimeToDestinationResponse res) throws ServiceException {
            try {
              if (lastCommandType == CommandType.SMART_SERVO_CARTESIAN_POSE_LIN) {
                linearMotion.getRuntime().updateWithRealtimeSystem();
                res.setRemainingTime(linearMotion.getRuntime().getRemainingTime());
              }
              else {
                motion.getRuntime().updateWithRealtimeSystem();
                res.setRemainingTime(motion.getRuntime().getRemainingTime());
              }
            }
            catch (Exception e) {
              // An exception should be thrown only if a motion/runtime is not available.
              res.setRemainingTime(-999);
            }
          }
        });

    // TODO: doc
    subscriber
        .setSpeedOverrideCallback(new ServiceResponseBuilder<iiwa_msgs.SetSpeedOverrideRequest, iiwa_msgs.SetSpeedOverrideResponse>() {
          @Override
          public void build(iiwa_msgs.SetSpeedOverrideRequest req, iiwa_msgs.SetSpeedOverrideResponse res)
              throws ServiceException {
            controlModeLock.lock();
            try {
              SpeedLimits.setOverrideRecution(req.getOverrideReduction(), true);
              res.setSuccess(true);
            }
            catch (Exception e) {
              res.setError(e.getClass().getName() + ": " + e.getMessage());
              res.setSuccess(false);
            }
            finally {
              controlModeLock.unlock();
            }
          }
        });

    // TODO: doc
    subscriber
        .setPTPCartesianLimitsCallback(new ServiceResponseBuilder<iiwa_msgs.SetPTPCartesianSpeedLimitsRequest, iiwa_msgs.SetPTPCartesianSpeedLimitsResponse>() {
          @Override
          public void build(iiwa_msgs.SetPTPCartesianSpeedLimitsRequest req,
              iiwa_msgs.SetPTPCartesianSpeedLimitsResponse res) throws ServiceException {
            controlModeLock.lock();
            try {
              SpeedLimits.setPTPCartesianSpeedLimits(req);
              res.setSuccess(true);
            }
            catch (Exception e) {
              res.setError(e.getClass().getName() + ": " + e.getMessage());
              res.setSuccess(false);
            }
            finally {
              controlModeLock.unlock();
            }
          }
        });

    // TODO: doc
    subscriber
        .setPTPJointLimitsCallback(new ServiceResponseBuilder<iiwa_msgs.SetPTPJointSpeedLimitsRequest, iiwa_msgs.SetPTPJointSpeedLimitsResponse>() {
          @Override
          public void build(iiwa_msgs.SetPTPJointSpeedLimitsRequest req, iiwa_msgs.SetPTPJointSpeedLimitsResponse res)
              throws ServiceException {
            controlModeLock.lock();
            try {
              SpeedLimits.setPTPJointSpeedLimits(req);
              res.setSuccess(true);
            }
            catch (Exception e) {
              res.setError(e.getClass().getName() + ": " + e.getMessage());
              res.setSuccess(false);
            }
            finally {
              controlModeLock.unlock();
            }
          }
        });

    // TODO: doc
    subscriber
        .setSmartServoLimitsCallback(new ServiceResponseBuilder<iiwa_msgs.SetSmartServoJointSpeedLimitsRequest, iiwa_msgs.SetSmartServoJointSpeedLimitsResponse>() {
          @Override
          public void build(iiwa_msgs.SetSmartServoJointSpeedLimitsRequest req,
              iiwa_msgs.SetSmartServoJointSpeedLimitsResponse res) throws ServiceException {
            controlModeLock.lock();
            try {
              SpeedLimits.setSmartServoJointSpeedLimits(req);

              if (lastCommandType != CommandType.SMART_SERVO_CARTESIAN_POSE_LIN) {
                iiwa_msgs.ConfigureControlModeRequest request = null;
                motion = controlModeHandler.changeSmartServoControlMode(motion, request);
              }

              res.setSuccess(true);
            }
            catch (Exception e) {
              res.setError(e.getClass().getName() + ": " + e.getMessage());
              res.setSuccess(false);
            }
            finally {
              controlModeLock.unlock();
            }
          }
        });

    // TODO: doc
    subscriber
        .setSmartServoLinLimitsCallback(new ServiceResponseBuilder<iiwa_msgs.SetSmartServoLinSpeedLimitsRequest, iiwa_msgs.SetSmartServoLinSpeedLimitsResponse>() {
          @Override
          public void build(SetSmartServoLinSpeedLimitsRequest req, SetSmartServoLinSpeedLimitsResponse res)
              throws ServiceException {
            controlModeLock.lock();
            try {
              SpeedLimits.setSmartServoLinSpeedLimits(req);

              if (lastCommandType == CommandType.SMART_SERVO_CARTESIAN_POSE_LIN) {
                iiwa_msgs.ConfigureControlModeRequest request = null;
                linearMotion = controlModeHandler.changeSmartServoControlMode(linearMotion, request);

              }
              res.setSuccess(true);
            }
            catch (Exception e) {
              res.setError(e.getClass().getName() + ": " + e.getMessage());
              res.setSuccess(false);
            }
            finally {
              controlModeLock.unlock();
            }
          }
        });

    // TODO: doc
    subscriber
        .setWorkpieceCallback(new ServiceResponseBuilder<iiwa_msgs.SetWorkpieceRequest, iiwa_msgs.SetWorkpieceResponse>() {
          @Override
          public void build(SetWorkpieceRequest req, SetWorkpieceResponse res) throws ServiceException {
            try {
              List<SceneGraphObject> oldWorkpieces;
              if (tool != null) {
                oldWorkpieces = tool.getChildren();
              }
              else {
                oldWorkpieces = robot.getChildren();
              }

              for (SceneGraphObject oldObject : oldWorkpieces) {
                if (oldObject instanceof Workpiece) {
                  ((Workpiece) oldObject).detach();
                }
              }

              robot.setSafetyWorkpiece(null);
              controlModeHandler.setWorkpiece(null);

              if (req.getWorkpieceId() != null && !req.getWorkpieceId().isEmpty()) {
                Workpiece workpiece = getApplicationData().createFromTemplate(req.getWorkpieceId());
                workpiece.attachTo(toolFrame);
                robot.setSafetyWorkpiece(workpiece);
                controlModeHandler.setWorkpiece(workpiece);
              }

              res.setSuccess(true);
            }
            catch (Exception e) {
              Logger.error(e.getClass().getName() + ": " + e.getMessage());
              e.printStackTrace();

              res.setError(e.getClass().getName() + ": " + e.getMessage());
              res.setSuccess(false);
            }
          }
        });

    // TODO: doc
    subscriber
        .setEndpointFrameCallback(new ServiceResponseBuilder<iiwa_msgs.SetEndpointFrameRequest, iiwa_msgs.SetEndpointFrameResponse>() {
          @Override
          public void build(SetEndpointFrameRequest req, SetEndpointFrameResponse res) throws ServiceException {
            try {
              if (req.getFrameId().isEmpty()) {
                endpointFrame = toolFrame;
              }
              else if (req.getFrameId().equals(configuration.getRobotName() + toolFrameIDSuffix)) {
                endpointFrame = robot.getFlange();
              }
              else {
                endpointFrame = tool.getFrame(req.getFrameId());
              }

              motions.setEnpointFrame(endpointFrame);
              controlModeHandler.setEndpointFrame(endpointFrame);

              // update motion
              if (lastCommandType == CommandType.SMART_SERVO_CARTESIAN_POSE_LIN) {
                activateMotionMode(CommandType.SMART_SERVO_JOINT_POSITION);
                activateMotionMode(CommandType.SMART_SERVO_CARTESIAN_POSE_LIN);
              }
              else if (lastCommandType == CommandType.SMART_SERVO_CARTESIAN_POSE
                  || lastCommandType == CommandType.SMART_SERVO_CARTESIAN_VELOCITY
                  || lastCommandType == CommandType.SMART_SERVO_JOINT_POSITION
                  || lastCommandType == CommandType.SMART_SERVO_JOINT_POSITION_VELOCITY
                  || lastCommandType == CommandType.SMART_SERVO_JOINT_VELOCITY) {
                activateMotionMode(CommandType.SMART_SERVO_JOINT_POSITION);
                CommandType currentCommandType = lastCommandType;
                activateMotionMode(CommandType.SMART_SERVO_CARTESIAN_POSE_LIN);
                activateMotionMode(currentCommandType);
              }

              res.setSuccess(true);
            }
            catch (Exception e) {
              Logger.error("Error while setting endpoint frame to \"" + req.getFrameId() + "\": " + e.getMessage());
              res.setError(e.getMessage());
            }
          }
        });

    // Execute the subscriber node.
    nodeMainExecutor.execute(subscriber, subscriberNodeConfiguration);
  }

  @Override
  protected void initializeApp() {
    robotBaseFrameID = configuration.getRobotName() + robotBaseFrameIDSuffix;
  }

  @Override
  protected void beforeControlLoop() {
    motions = new Motions(robot, robotBaseFrameID, motion, endpointFrame, publisher, actionServer);
    subscriber.resetSequenceIds();
  }

  /**
   * TODO: doc, take something from This will acquire the last received CartesianPose command from the
   * commanding ROS node, if there is any available. If the robot can move, then it will move to this new
   * position.
   */
  private void moveRobot() {
    try {
      if (actionServer.newGoalAvailable()) {
        while (actionServer.newGoalAvailable()) {
          actionServer.markCurrentGoalFailed("Received new goal. Dropping old task.");
          actionServer.acceptNewGoal();
        }

        Goal<?> actionGoal = actionServer.getCurrentGoal();
        switch (actionGoal.goalType) {
          case POINT_TO_POINT_CARTESIAN_POSE: {
            movePointToPointCartesian(((MoveToCartesianPoseActionGoal) actionGoal.goal).getGoal().getCartesianPose()
                .getPoseStamped(), ((MoveToCartesianPoseActionGoal) actionGoal.goal).getGoal().getCartesianPose()
                .getRedundancy());
            break;
          }
          case POINT_TO_POINT_CARTESIAN_POSE_LIN: {
            movePointToPointCartesianLin(((MoveToCartesianPoseActionGoal) actionGoal.goal).getGoal().getCartesianPose()
                .getPoseStamped(), ((MoveToCartesianPoseActionGoal) actionGoal.goal).getGoal().getCartesianPose()
                .getRedundancy());
            break;
          }
          case POINT_TO_POINT_CARTESIAN_SPLINE: {
            movePointToPointCartesianSpline(((MoveAlongSplineActionGoal) actionGoal.goal).getGoal().getSpline());
            break;
          }
          case POINT_TO_POINT_JOINT_POSITION: {
            movePointToPointJointPosition(((MoveToJointPositionActionGoal) actionGoal.goal).getGoal()
                .getJointPosition());
            break;
          }
          default: {
            throw new UnsupportedControlModeException("goalType: " + actionGoal.goalType);
          }
        }
      }
      else if (subscriber.currentCommandType != null) {
        if (actionServer.hasCurrentGoal()) {
          actionServer.markCurrentGoalFailed("Received new Action command. Dropping old task.");
        }

        // TODO: ask Arne: Why the need to set this to null?
        // the methods to get the last commands already check if a new one has arrived, with the exception of
        // the velocity commands.
        // This was the velocity commands will only run for 1 control period.
        CommandType copy = subscriber.currentCommandType;
        subscriber.currentCommandType = null;

        switch (copy) {
          case SMART_SERVO_CARTESIAN_POSE: {
            moveToCartesianPose(subscriber.getCartesianPose(), null);
            break;
          }
          case SMART_SERVO_CARTESIAN_POSE_LIN: {
            moveToCartesianPoseLin(subscriber.getCartesianPoseLin(), null);
            break;
          }
          case SMART_SERVO_CARTESIAN_VELOCITY: {
            moveByCartesianVelocity(subscriber.getCartesianVelocity());
            break;
          }
          case SMART_SERVO_JOINT_POSITION: {
            moveToJointPosition(subscriber.getJointPosition());
            break;
          }
          case SMART_SERVO_JOINT_POSITION_VELOCITY: {
            moveByJointPositionVelocity(subscriber.getJointPositionVelocity());
            break;
          }
          case SMART_SERVO_JOINT_VELOCITY: {
            moveByJointVelocity(subscriber.getJointVelocity());
            break;
          }
          default: {
            throw new UnsupportedControlModeException("commandType: " + copy);
          }
        }
      }
    }
    catch (Exception e) {
      Logger.error(e.getClass().getName() + ": " + e.getMessage());
      e.printStackTrace();
    }
  }

  @Override
  protected void controlLoop() {
    moveRobot();
    if (rosTool != null) {
      rosTool.moveTool();
    }
  }

  /**
   * Checks what kind of command has been executed at last and changes the controller type if necessary.
   * 
   * @param commandType
   */
  protected void activateMotionMode(CommandType commandType) {
    if (commandType == lastCommandType) {
      if (commandType == CommandType.POINT_TO_POINT_CARTESIAN_SPLINE) {
        // For some reason the application gets stuck when executing two spline motions
        // in a row. Switching the control mode to SmartServo and back in between
        // resolves the issue.
        // TODO: Find a cleaner way of solving this issue
        activateMotionMode(CommandType.SMART_SERVO_CARTESIAN_POSE_LIN);
        activateMotionMode(commandType);
      }
      return;
    }

    Logger.debug("Switching control mode from " + lastCommandType + " to " + commandType);

    if (commandType == CommandType.SMART_SERVO_CARTESIAN_POSE
        || commandType == CommandType.SMART_SERVO_CARTESIAN_VELOCITY
        || commandType == CommandType.SMART_SERVO_JOINT_POSITION
        || commandType == CommandType.SMART_SERVO_JOINT_POSITION_VELOCITY
        || commandType == CommandType.SMART_SERVO_JOINT_VELOCITY) {
      if (lastCommandType == CommandType.SMART_SERVO_CARTESIAN_POSE_LIN || lastCommandType == null) {
        motion = controlModeHandler.switchToSmartServo(linearMotion);
      }
      else if (lastCommandType == CommandType.POINT_TO_POINT_CARTESIAN_POSE
          || lastCommandType == CommandType.POINT_TO_POINT_CARTESIAN_POSE_LIN
          || lastCommandType == CommandType.POINT_TO_POINT_CARTESIAN_SPLINE
          || lastCommandType == CommandType.POINT_TO_POINT_JOINT_POSITION) {
        motion = controlModeHandler.enableSmartServo(motion);
      }
    }
    else if (commandType == CommandType.SMART_SERVO_CARTESIAN_POSE_LIN) {
      if (lastCommandType != CommandType.SMART_SERVO_CARTESIAN_POSE_LIN || lastCommandType == null) {
        linearMotion = controlModeHandler.switchToSmartServoLIN(motion);
      }
      else if (lastCommandType == CommandType.POINT_TO_POINT_CARTESIAN_POSE
          || lastCommandType == CommandType.POINT_TO_POINT_CARTESIAN_POSE_LIN
          || lastCommandType == CommandType.POINT_TO_POINT_CARTESIAN_SPLINE
          || lastCommandType == CommandType.POINT_TO_POINT_JOINT_POSITION) {
        linearMotion = controlModeHandler.enableSmartServo(linearMotion);
      }
    }
    else if (commandType == CommandType.POINT_TO_POINT_CARTESIAN_POSE
        || commandType == CommandType.POINT_TO_POINT_CARTESIAN_POSE_LIN
        || commandType == CommandType.POINT_TO_POINT_CARTESIAN_SPLINE
        || commandType == CommandType.POINT_TO_POINT_JOINT_POSITION) {
      if (lastCommandType == CommandType.SMART_SERVO_CARTESIAN_POSE
          || lastCommandType == CommandType.SMART_SERVO_JOINT_POSITION
          || lastCommandType == CommandType.SMART_SERVO_JOINT_POSITION_VELOCITY
          || lastCommandType == CommandType.SMART_SERVO_JOINT_VELOCITY
          || lastCommandType == CommandType.SMART_SERVO_CARTESIAN_VELOCITY) {
        controlModeHandler.disableSmartServo(motion);
      }
      else if (lastCommandType == CommandType.SMART_SERVO_CARTESIAN_POSE_LIN) {
        controlModeHandler.disableSmartServo(linearMotion);
      }
      else if (lastCommandType == null) {
        // For some reason the application gets stuck when executing two spline motions
        // in a row. Switching the control mode to SmartServo and back in between
        // resolves the issue.
        // TODO: Find a cleaner way of solving this issue
        activateMotionMode(CommandType.SMART_SERVO_CARTESIAN_POSE_LIN);
        activateMotionMode(commandType);
      }
    }
    else {
      Logger.error("Received an unknown command type.");
    }

    lastCommandType = commandType;
  }

  protected void moveToJointPosition(iiwa_msgs.JointPosition commandPosition) {
    activateMotionMode(CommandType.SMART_SERVO_JOINT_POSITION);
    motions.jointPositionMotion(motion, commandPosition);
  }

  protected void moveToCartesianPose(PoseStamped commandPosition, RedundancyInformation redundancy) {
    activateMotionMode(CommandType.SMART_SERVO_CARTESIAN_POSE);
    commandPosition = subscriber.transformPose(commandPosition, robotBaseFrameID);
    if (commandPosition != null) {
      motions.cartesianPositionMotion(motion, commandPosition, redundancy);
    }
    else {
      Logger.warn("Invalid motion target pose");
    }
  }

  protected void moveToCartesianPoseLin(PoseStamped commandPosition, RedundancyInformation redundancy) {
    activateMotionMode(CommandType.SMART_SERVO_CARTESIAN_POSE_LIN);
    commandPosition = subscriber.transformPose(commandPosition, robotBaseFrameID);
    if (commandPosition != null) {
      motions.cartesianPositionLinMotion(linearMotion, commandPosition, redundancy);
    }
    else {
      Logger.warn("Invalid motion target pose");
    }
  }

  protected void movePointToPointJointPosition(JointPosition commandPosition) {
    activateMotionMode(CommandType.POINT_TO_POINT_JOINT_POSITION);
    motions.pointToPointJointPositionMotion(controlModeHandler.getControlMode(), commandPosition);
  }

  protected void movePointToPointCartesian(PoseStamped commandPosition, RedundancyInformation redundancy) {
    activateMotionMode(CommandType.POINT_TO_POINT_CARTESIAN_POSE);
    commandPosition = subscriber.transformPose(commandPosition, robotBaseFrameID);

    if (commandPosition != null) {
      motions.pointToPointCartesianMotion(controlModeHandler.getControlMode(), commandPosition, redundancy);
    }
    else {
      Logger.warn("Invalid motion target pose");
    }
  }

  protected void movePointToPointCartesianLin(PoseStamped commandPosition, RedundancyInformation redundancy) {
    activateMotionMode(CommandType.POINT_TO_POINT_CARTESIAN_POSE_LIN);
    commandPosition = subscriber.transformPose(commandPosition, robotBaseFrameID);

    if (commandPosition != null) {
      motions.pointToPointLinearCartesianMotion(controlModeHandler.getControlMode(), commandPosition, redundancy);
    }
    else {
      Logger.warn("Invalid motion target pose");
    }
  }

  protected void movePointToPointCartesianSpline(Spline spline) {
    activateMotionMode(CommandType.POINT_TO_POINT_CARTESIAN_SPLINE);
    boolean success = motions
        .pointToPointCartesianSplineMotion(controlModeHandler.getControlMode(), spline, subscriber);

    if (!success && actionServer.hasCurrentGoal()) {
      actionServer.markCurrentGoalFailed("Invalid spline.");
    }
  }

  protected void moveByJointPositionVelocity(iiwa_msgs.JointPositionVelocity commandPositionVelocity) {
    activateMotionMode(CommandType.SMART_SERVO_JOINT_POSITION_VELOCITY);
    motions.jointPositionVelocityMotion(motion, commandPositionVelocity);
  }

  protected void moveByJointVelocity(iiwa_msgs.JointVelocity commandVelocity) {
    activateMotionMode(CommandType.SMART_SERVO_JOINT_VELOCITY);

    /*
     * This will acquire the last received JointVelocity command from the commanding ROS node, if there is any
     * available. If the robot can move, then it will move to this new position accordingly to the given joint
     * velocity.
     */
    motion.getRuntime().activateVelocityPlanning(true);
    motion.setSpeedTimeoutAfterGoalReach(0.1);
    motions.jointVelocityMotion(motion, commandVelocity);
  }

  protected void moveByCartesianVelocity(geometry_msgs.TwistStamped commandVelocity) {
    activateMotionMode(CommandType.SMART_SERVO_CARTESIAN_VELOCITY);
    motions.cartesianVelocityMotion(motion, commandVelocity, endpointFrame);
  }
}
