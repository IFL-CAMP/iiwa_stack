/**
 * Copyright (C) 2018 Arne Peters - arne.peters@tum.de 
 * Technische Universität München
 * Chair for Robotics, Artificial Intelligence and Embedded Systems 
 * Fakultät für Informatik / I6, Boltzmannstraße 3, 85748 Garching bei München, Germany 
 * http://www6.in.tum.de 
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

package de.tum.in.camp.kuka.ros;

import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;

import actionlib_msgs.GoalID;
import actionlib_msgs.GoalStatus;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;

import com.github.rosjava_actionlib.ActionServer;
import com.github.rosjava_actionlib.ActionServerListener;

import de.tum.in.camp.kuka.ros.CommandTypes.CommandType;

import iiwa_msgs.MoveAlongSplineActionFeedback;
import iiwa_msgs.MoveAlongSplineActionGoal;
import iiwa_msgs.MoveAlongSplineActionResult;
import iiwa_msgs.MoveToCartesianPoseActionGoal;
import iiwa_msgs.MoveToCartesianPoseActionResult;
import iiwa_msgs.MoveToCartesianPoseActionFeedback;
import iiwa_msgs.MoveToJointPositionActionGoal;
import iiwa_msgs.MoveToJointPositionActionResult;
import iiwa_msgs.MoveToJointPositionActionFeedback;
import org.ros.internal.message.Message;

public class iiwaActionServer extends AbstractNodeMain {

  public class Goal<T_ACTION_GOAL extends Message> {

    public CommandType goalType = null;
    public T_ACTION_GOAL goal = null;
    public String goalId = null;

    public Goal(CommandType goalType, T_ACTION_GOAL goal, String goalId) {
      this.goalType = goalType;
      this.goal = goal;
      this.goalId = goalId;
    }
  }

  public abstract class iiwaActionServerListener<T_ACTION_GOAL extends Message> implements ActionServerListener<T_ACTION_GOAL> {
    private iiwaActionServer server = null;
    private CommandType goalType = null;

    public iiwaActionServerListener(iiwaActionServer server, CommandType goalType) {
      this.server = server;
      this.goalType = goalType;
    }

    /**
     * Gets called after a new has been received. We initially accept all goals kill the old ones afterwards.
     */
    @Override
    public boolean acceptGoal(T_ACTION_GOAL arg0) {
      return true;
    }

    /**
     * Goal got canceled by remote callback.
     */
    @Override
    public void cancelReceived(GoalID arg0) {
      server.markCurrentGoalFailed("Goal execution canceled by client.");
    }

    /**
     * Goal received callback. Adds the received goal to the goal queue
     */
    @Override
    public void goalReceived(T_ACTION_GOAL goal) {
      synchronized (server) {
        server.goalQueue.add(new Goal<T_ACTION_GOAL>(goalType, goal, this.getGoalId(goal)));
      }
    }

    public abstract String getGoalId(T_ACTION_GOAL goal);
  }

  private ActionServer<MoveToCartesianPoseActionGoal, MoveToCartesianPoseActionFeedback, MoveToCartesianPoseActionResult> cartesianPoseServer = null;
  private ActionServer<MoveToCartesianPoseActionGoal, MoveToCartesianPoseActionFeedback, MoveToCartesianPoseActionResult> cartesianPoseLinServer = null;
  private ActionServer<MoveAlongSplineActionGoal, MoveAlongSplineActionFeedback, MoveAlongSplineActionResult> moveAlongSplineServer = null;
  private ActionServer<MoveToJointPositionActionGoal, MoveToJointPositionActionFeedback, MoveToJointPositionActionResult> jointPositionServer = null;
  Queue<Goal<?>> goalQueue;
  Goal<?> currentGoal;

  private ConnectedNode node = null;

  // Name to use to build the name of the ROS topics
  private String iiwaName = "iiwa";

  public iiwaActionServer(LBR robot, Configuration configuration) {
    this(robot, robot.getFlange(), configuration);
  }

  public iiwaActionServer(LBR robot, ObjectFrame frame, Configuration configuration) {
    iiwaName = configuration.getRobotName();
    goalQueue = new LinkedBlockingQueue<iiwaActionServer.Goal<?>>();
  }

  @Override
  public synchronized void onStart(ConnectedNode connectedNode) {
    node = connectedNode;
    goalQueue.clear();

    cartesianPoseServer = new ActionServer<MoveToCartesianPoseActionGoal, MoveToCartesianPoseActionFeedback, MoveToCartesianPoseActionResult>(node, iiwaName
        + "/action/move_to_cartesian_pose", MoveToCartesianPoseActionGoal._TYPE, MoveToCartesianPoseActionFeedback._TYPE, MoveToCartesianPoseActionResult._TYPE);
    cartesianPoseServer.attachListener(new iiwaActionServerListener<MoveToCartesianPoseActionGoal>(this, CommandType.POINT_TO_POINT_CARTESIAN_POSE) {
      @Override
      public String getGoalId(MoveToCartesianPoseActionGoal goal) {
        return goal.getGoalId().getId();
      }
    });

    cartesianPoseLinServer = new ActionServer<MoveToCartesianPoseActionGoal, MoveToCartesianPoseActionFeedback, MoveToCartesianPoseActionResult>(node, iiwaName
        + "/action/move_to_cartesian_pose_lin", MoveToCartesianPoseActionGoal._TYPE, MoveToCartesianPoseActionFeedback._TYPE, MoveToCartesianPoseActionResult._TYPE);
    cartesianPoseLinServer.attachListener(new iiwaActionServerListener<MoveToCartesianPoseActionGoal>(this, CommandType.POINT_TO_POINT_CARTESIAN_POSE_LIN) {
      @Override
      public String getGoalId(MoveToCartesianPoseActionGoal goal) {
        return goal.getGoalId().getId();
      }
    });

    moveAlongSplineServer = new ActionServer<MoveAlongSplineActionGoal, MoveAlongSplineActionFeedback, MoveAlongSplineActionResult>(node, iiwaName + "/action/move_along_spline",
        MoveAlongSplineActionGoal._TYPE, MoveAlongSplineActionFeedback._TYPE, MoveAlongSplineActionResult._TYPE);
    moveAlongSplineServer.attachListener(new iiwaActionServerListener<MoveAlongSplineActionGoal>(this, CommandType.POINT_TO_POINT_CARTESIAN_SPLINE) {
      @Override
      public String getGoalId(MoveAlongSplineActionGoal goal) {
        return goal.getGoalId().getId();
      }
    });

    jointPositionServer = new ActionServer<MoveToJointPositionActionGoal, MoveToJointPositionActionFeedback, MoveToJointPositionActionResult>(node, iiwaName
        + "/action/move_to_joint_position", MoveToJointPositionActionGoal._TYPE, MoveToJointPositionActionFeedback._TYPE, MoveToJointPositionActionResult._TYPE);
    jointPositionServer.attachListener(new iiwaActionServerListener<MoveToJointPositionActionGoal>(this, CommandType.POINT_TO_POINT_JOINT_POSITION) {
      @Override
      public String getGoalId(MoveToJointPositionActionGoal goal) {
        return goal.getGoalId().getId();
      }
    });
  }

  @Override
  public void onShutdown(Node node) {
    cartesianPoseServer.finish();
    cartesianPoseLinServer.finish();
    moveAlongSplineServer.finish();
    jointPositionServer.finish();
    goalQueue.clear();
    goalQueue = null;
  }

  public Boolean isActive() {
    return (cartesianPoseServer != null && cartesianPoseLinServer != null && jointPositionServer != null);
  }

  /**
   * @see org.ros.node.NodeMain#getDefaultNodeName()
   */
  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of(iiwaName + "/action_server");
  }

  /**
   * Sets current goal to succeeded and publishes result message
   */
  public void markCurrentGoalReached() {
    if (hasCurrentGoal()) {
      Logger.debug("Publishing current " + currentGoal.goalType + " goal reached: " + currentGoal.goalId);
    }
    markCurrentGoal(true, "");
  }

  /**
   * Sets current goal to aborted and publishes result message
   */
  public void markCurrentGoalFailed(String error_msg) {
    if (hasCurrentGoal()) {
      Logger.debug("Publishing current " + currentGoal.goalType + " goal failed: " + currentGoal.goalId);
    }
    markCurrentGoal(false, error_msg);
  }

  private synchronized void markCurrentGoal(boolean succeeded, String error_msg) {
    if (hasCurrentGoal()) {
      switch (currentGoal.goalType) {
      // TODO: Reduce similar code
        case POINT_TO_POINT_CARTESIAN_POSE: {
          MoveToCartesianPoseActionResult result = cartesianPoseServer.newResultMessage();
          result.getResult().setSuccess(succeeded);
          result.getResult().setError(error_msg);
          result.getStatus().getGoalId().setId(currentGoal.goalId);
          if (succeeded) {
            result.getStatus().setStatus(GoalStatus.SUCCEEDED);
            cartesianPoseServer.setSucceed(currentGoal.goalId);
          }
          else {
            result.getStatus().setStatus(GoalStatus.ABORTED);
            cartesianPoseServer.setAborted(currentGoal.goalId);
          }
          cartesianPoseServer.sendResult(result);
          cartesianPoseServer.setGoalStatus(result.getStatus(), currentGoal.goalId);
          break;
        }
        case POINT_TO_POINT_CARTESIAN_POSE_LIN: {
          MoveToCartesianPoseActionResult result = cartesianPoseLinServer.newResultMessage();
          result.getResult().setSuccess(succeeded);
          result.getResult().setError(error_msg);
          result.getStatus().getGoalId().setId(currentGoal.goalId);
          if (succeeded) {
            result.getStatus().setStatus(GoalStatus.SUCCEEDED);
            cartesianPoseLinServer.setSucceed(currentGoal.goalId);
          }
          else {
            result.getStatus().setStatus(GoalStatus.ABORTED);
            cartesianPoseLinServer.setAborted(currentGoal.goalId);
          }
          cartesianPoseLinServer.sendResult(result);
          cartesianPoseLinServer.setGoalStatus(result.getStatus(), currentGoal.goalId);
          break;
        }
        case POINT_TO_POINT_CARTESIAN_SPLINE: {
          MoveAlongSplineActionResult result = moveAlongSplineServer.newResultMessage();
          result.getResult().setSuccess(succeeded);
          result.getResult().setError(error_msg);
          result.getStatus().getGoalId().setId(currentGoal.goalId);
          if (succeeded) {
            result.getStatus().setStatus(GoalStatus.SUCCEEDED);
            moveAlongSplineServer.setSucceed(currentGoal.goalId);
          }
          else {
            result.getStatus().setStatus(GoalStatus.ABORTED);
            moveAlongSplineServer.setAborted(currentGoal.goalId);
          }
          moveAlongSplineServer.sendResult(result);
          moveAlongSplineServer.setGoalStatus(result.getStatus(), currentGoal.goalId);
          break;
        }
        case POINT_TO_POINT_JOINT_POSITION: {
          MoveToJointPositionActionResult result = jointPositionServer.newResultMessage();
          result.getResult().setSuccess(succeeded);
          result.getResult().setError(error_msg);
          result.getStatus().getGoalId().setId(currentGoal.goalId);
          if (succeeded) {
            result.getStatus().setStatus(GoalStatus.SUCCEEDED);
            jointPositionServer.setSucceed(currentGoal.goalId);
          }
          else {
            result.getStatus().setStatus(GoalStatus.ABORTED);
            jointPositionServer.setAborted(currentGoal.goalId);
          }
          jointPositionServer.sendResult(result);
          jointPositionServer.setGoalStatus(result.getStatus(), currentGoal.goalId);
          break;
        }
        default:
          System.out.println("Unknown goal type: " + currentGoal.goalType);
          break;
      }

      currentGoal = null;
    }
  }

  /**
   * True if a new goal has been received
   * 
   * @return
   */
  public synchronized boolean newGoalAvailable() {
    if (goalQueue != null && isActive()) {
      return !goalQueue.isEmpty();
    }
    else {
      return false;
    }
  }

  /**
   * True if a goal is active at the moment
   * 
   * @return
   */
  public synchronized boolean hasCurrentGoal() {
    return currentGoal != null;
  }

  public synchronized Goal<?> getCurrentGoal() {
    return currentGoal;
  }

  public synchronized Goal<?> getNextGoal() {
    return goalQueue.peek();
  }

  public synchronized Goal<?> acceptNewGoal() {
    currentGoal = goalQueue.poll();
    return currentGoal;
  }

  /**
   * Send heartbeat to action clients
   */
  public synchronized void publishCurrentState() {
    if (hasCurrentGoal()) {
      switch (currentGoal.goalType) {
        case POINT_TO_POINT_CARTESIAN_POSE:
          cartesianPoseServer.sendStatusTick();
          break;
        case POINT_TO_POINT_CARTESIAN_POSE_LIN:
          cartesianPoseLinServer.sendStatusTick();
          break;
        case POINT_TO_POINT_CARTESIAN_SPLINE:
          moveAlongSplineServer.sendStatusTick();
          break;
        case POINT_TO_POINT_JOINT_POSITION:
          jointPositionServer.sendStatusTick();
          break;
        default:
          break;
      }
    }
  }
}
